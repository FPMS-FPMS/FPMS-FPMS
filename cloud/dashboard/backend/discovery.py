"""Real device discovery for the FPMS dashboard.

What this does:
  1. Enumerate the local machine's IPv4 subnets (from psutil).
  2. TCP-scan a subnet for the ports that matter for onboarding an Orange Pi
     5B: SSH (22), MQTT (1883), HTTP (80/8080), HTTPS (443).
  3. Fingerprint each responder — banner grab, hostname lookup — and guess
     whether it looks like an Orange Pi.
  4. Optionally attempt SSH login with credentials the operator supplies.
  5. Provision: SSH in and drop a small MQTT publisher unit that streams
     LiDAR / camera / thermal to AWS IoT Core (LocalStack).

Everything is opt-in from the UI. The operator has to explicitly choose a
subnet, click Scan, then click SSH per device with their own credentials.
Nothing scans until asked.
"""
from __future__ import annotations

import asyncio
import contextlib
import ipaddress
import logging
import socket
import time
from dataclasses import asdict, dataclass
from typing import Any

import paramiko
import psutil

log = logging.getLogger("fpms.discovery")

DEFAULT_PORTS = [22, 1883, 80, 8080, 443]
SCAN_CONCURRENCY = 128
SCAN_TIMEOUT_S = 0.6
BANNER_TIMEOUT_S = 1.2
SSH_TIMEOUT_S = 5


# ---- interface enumeration -------------------------------------------------

@dataclass
class Interface:
    name: str
    ip: str
    netmask: str
    cidr: str
    is_up: bool
    is_loopback: bool


def list_interfaces() -> list[dict[str, Any]]:
    out: list[Interface] = []
    stats = psutil.net_if_stats()
    for name, addrs in psutil.net_if_addrs().items():
        st = stats.get(name)
        for a in addrs:
            if a.family != socket.AF_INET:
                continue
            try:
                net = ipaddress.IPv4Network(f"{a.address}/{a.netmask}", strict=False)
            except Exception:  # noqa: BLE001
                continue
            out.append(Interface(
                name=name,
                ip=a.address,
                netmask=a.netmask,
                cidr=str(net),
                is_up=bool(st and st.isup),
                is_loopback=a.address.startswith("127."),
            ))
    # Prefer non-loopback, up interfaces first
    out.sort(key=lambda i: (i.is_loopback, not i.is_up, i.name))
    return [asdict(i) for i in out]


# ---- TCP scan --------------------------------------------------------------

@dataclass
class HostResult:
    ip: str
    hostname: str | None
    open_ports: list[int]
    ssh_banner: str | None
    guess: str


async def _check_port(ip: str, port: int) -> bool:
    try:
        fut = asyncio.open_connection(ip, port)
        reader, writer = await asyncio.wait_for(fut, timeout=SCAN_TIMEOUT_S)
        writer.close()
        with contextlib.suppress(Exception):
            await writer.wait_closed()
        return True
    except (OSError, asyncio.TimeoutError):
        return False


async def _grab_ssh_banner(ip: str) -> str | None:
    """Read the first line an SSH server sends. Non-invasive."""
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(ip, 22),
            timeout=BANNER_TIMEOUT_S,
        )
        line = await asyncio.wait_for(reader.readline(), timeout=BANNER_TIMEOUT_S)
        writer.close()
        with contextlib.suppress(Exception):
            await writer.wait_closed()
        return line.decode("utf-8", errors="replace").strip()
    except Exception:  # noqa: BLE001
        return None


def _reverse_lookup(ip: str) -> str | None:
    try:
        return socket.gethostbyaddr(ip)[0]
    except Exception:  # noqa: BLE001
        return None


def _guess(hostname: str | None, banner: str | None) -> str:
    h = (hostname or "").lower()
    b = (banner or "").lower()
    if "orangepi" in h or "orange-pi" in h or "opi" in h:
        return "Orange Pi (hostname match)"
    if "raspberrypi" in h or "raspi" in h or "rpi" in h:
        return "Raspberry Pi (hostname match)"
    if "ubuntu" in b or "debian" in b:
        return "Linux SSH server"
    if banner and "openssh" in b:
        return "SSH host"
    if hostname:
        return "Named host"
    return "Unknown"


async def scan_subnet(cidr: str, ports: list[int] | None = None) -> list[dict[str, Any]]:
    """TCP connect scan of an IPv4 subnet on the given ports.

    Only /24-or-smaller subnets are accepted to keep scan cost sane.
    """
    net = ipaddress.IPv4Network(cidr, strict=False)
    if net.num_addresses > 512:
        raise ValueError("subnet too large; use a /23 or smaller")
    ports = ports or DEFAULT_PORTS
    sem = asyncio.Semaphore(SCAN_CONCURRENCY)
    hosts = [str(a) for a in net.hosts()]
    results: dict[str, HostResult] = {}

    async def probe_host(ip: str) -> None:
        async with sem:
            open_ports: list[int] = []
            for p in ports:
                if await _check_port(ip, p):
                    open_ports.append(p)
            if not open_ports:
                return
            banner = await _grab_ssh_banner(ip) if 22 in open_ports else None
            hostname = _reverse_lookup(ip)
            results[ip] = HostResult(
                ip=ip,
                hostname=hostname,
                open_ports=open_ports,
                ssh_banner=banner,
                guess=_guess(hostname, banner),
            )

    t0 = time.time()
    await asyncio.gather(*(probe_host(ip) for ip in hosts))
    log.info("scanned %s in %.1fs — %d responders", cidr, time.time() - t0, len(results))
    # Prefer likely-Pi devices first
    ordered = sorted(
        results.values(),
        key=lambda r: (0 if "Pi" in r.guess else 1, r.ip),
    )
    return [asdict(r) for r in ordered]


# ---- SSH probe -------------------------------------------------------------

@dataclass
class SshResult:
    ok: bool
    ip: str
    username: str
    error: str | None
    hostname: str | None
    os_release: str | None
    kernel: str | None
    is_orange_pi: bool
    uptime: str | None


def ssh_probe(ip: str, username: str, password: str | None = None,
              key_path: str | None = None, port: int = 22) -> dict[str, Any]:
    """SSH into `ip` and pull identifying facts. No writes.

    Returns a dict with everything we know about the device.
    """
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        kwargs: dict[str, Any] = {
            "hostname": ip, "port": port, "username": username,
            "timeout": SSH_TIMEOUT_S, "banner_timeout": SSH_TIMEOUT_S,
            "auth_timeout": SSH_TIMEOUT_S, "look_for_keys": False,
            "allow_agent": False,
        }
        if key_path:
            kwargs["key_filename"] = key_path
        elif password:
            kwargs["password"] = password
        client.connect(**kwargs)
    except paramiko.AuthenticationException as e:
        return asdict(SshResult(False, ip, username, f"auth failed: {e}", None, None, None, False, None))
    except (paramiko.SSHException, OSError) as e:
        return asdict(SshResult(False, ip, username, f"connection failed: {e}", None, None, None, False, None))

    def run(cmd: str) -> str:
        try:
            _, out, _ = client.exec_command(cmd, timeout=5)
            return out.read().decode("utf-8", errors="replace").strip()
        except Exception as e:  # noqa: BLE001
            return f"<err: {e}>"

    hostname = run("hostname")
    os_release = run("cat /etc/os-release 2>/dev/null | grep -E '^(NAME|VERSION)=' | head -3")
    kernel = run("uname -a")
    uptime = run("uptime -p")
    model = run("cat /proc/device-tree/model 2>/dev/null || true").strip("\x00").strip()

    is_pi = "orange" in (hostname + model).lower() or "orangepi" in os_release.lower() or "rk3588" in kernel.lower()

    client.close()
    return asdict(SshResult(
        ok=True, ip=ip, username=username, error=None,
        hostname=hostname, os_release=os_release, kernel=kernel,
        is_orange_pi=is_pi, uptime=uptime,
    ))


# ---- Provisioning ----------------------------------------------------------

def build_provisioner_script(thing_name: str, iot_endpoint: str,
                             cert_pem: str | None = None,
                             private_key: str | None = None,
                             use_tls: bool = True,
                             mqtt_port: int | None = None,
                             mqtt_username: str = "",
                             mqtt_password: str = "",
                             publish_interval: float = 5.0,
                             ca_cert: str | None = None) -> str:
    """A single bash script that, when run on the Pi, installs a persistent
    MQTT publisher streaming LiDAR / camera / thermal telemetry.

    Two transports, because there are two deployments:

    * ``use_tls=True``  — AWS IoT Core on 8883 with X.509 client certs.
    * ``use_tls=True`` + ``ca_cert`` — TLS on 8883 to the FPMS cloud broker,
      verifying it against our own CA and authenticating with username/password.
      This is the field/cellular setup.
    * ``use_tls=False`` — plain MQTT on 1883 to the Mosquitto broker running on
      the HQ laptop. This is the LAN setup, and it is what the dashboard
      subscribes to by default. ``iot_endpoint`` is then the laptop's LAN IP.

    The publisher is a real systemd service — not a one-shot. When the Pi
    boots, it reconnects and resumes publishing.
    """
    port = mqtt_port or (8883 if use_tls else 1883)
    if use_tls and cert_pem and not private_key:
        raise ValueError("client certificate supplied without a private key")

    # Broker credentials, when the LAN broker has allow_anonymous false.
    if mqtt_username:
        auth_config = f"FPMS_MQTT_USER={mqtt_username}\nFPMS_MQTT_PASS={mqtt_password}"
        auth_call = 'c.username_pw_set(CFG["FPMS_MQTT_USER"], CFG.get("FPMS_MQTT_PASS") or None)'
    else:
        auth_config = ""
        auth_call = "# broker accepts anonymous connections"

    if use_tls and cert_pem and private_key:
        # Mutual TLS with X.509 client certs — AWS IoT Core.
        creds_block = f"""cat >/tmp/fpms-cert.pem <<'CERT'
{cert_pem}
CERT
cat >/tmp/fpms-key.pem <<'KEY'
{private_key}
KEY
sudo mv /tmp/fpms-cert.pem /etc/fpms/cert.pem
sudo mv /tmp/fpms-key.pem  /etc/fpms/key.pem
sudo chmod 600 /etc/fpms/*.pem
"""
        cert_config = "FPMS_CERT=/etc/fpms/cert.pem\nFPMS_KEY=/etc/fpms/key.pem"
        tls_call = 'c.tls_set(certfile=CFG["FPMS_CERT"], keyfile=CFG["FPMS_KEY"])'
    elif use_tls and ca_cert:
        # Server-verified TLS against our own CA, with username/password auth —
        # the cloud broker. Pinning our CA is deliberate: the broker cert is
        # self-signed and long-lived, so there's no public chain to trust and no
        # renewal that could silently break the whole fleet mid-season.
        creds_block = f"""cat >/tmp/fpms-ca.crt <<'CACERT'
{ca_cert}
CACERT
sudo mv /tmp/fpms-ca.crt /etc/fpms/ca.crt
sudo chmod 644 /etc/fpms/ca.crt
"""
        cert_config = "FPMS_CA=/etc/fpms/ca.crt"
        tls_call = 'c.tls_set(ca_certs=CFG["FPMS_CA"])'
    elif use_tls:
        # TLS against a publicly trusted broker certificate.
        creds_block = "# TLS using the system CA bundle.\n"
        cert_config = ""
        tls_call = "c.tls_set()"
    else:
        creds_block = "# Plain MQTT on the LAN — no certificates needed.\n"
        cert_config = ""
        tls_call = "# plain MQTT — no TLS on the LAN broker"

    return f"""#!/usr/bin/env bash
set -euo pipefail
sudo mkdir -p /etc/fpms
{creds_block}
sudo tee /etc/fpms/config.env >/dev/null <<CFG
FPMS_THING_NAME={thing_name}
FPMS_IOT_ENDPOINT={iot_endpoint}
FPMS_MQTT_PORT={port}
FPMS_PUBLISH_INTERVAL={publish_interval}
{cert_config}
{auth_config}
CFG
sudo chmod 600 /etc/fpms/config.env

sudo tee /usr/local/bin/fpms-publisher >/dev/null <<'PUB'
#!/usr/bin/env python3
# FPMS rover publisher: reads sensors, publishes telemetry to the broker.
import json, os, time
from paho.mqtt.client import Client, CallbackAPIVersion
CFG = {{}}
with open("/etc/fpms/config.env") as f:
    for line in f:
        if "=" in line:
            k, v = line.strip().split("=", 1); CFG[k] = v
thing = CFG["FPMS_THING_NAME"]
port = int(CFG.get("FPMS_MQTT_PORT", "1883"))
# Seconds between pose publishes. On a metered cloud broker the old 0.5s loop
# meant ~10M messages/month for two rovers; 5s cuts that ~90% and loses nothing,
# since detections are event-driven rather than polled.
interval = float(CFG.get("FPMS_PUBLISH_INTERVAL", "5"))
c = Client(client_id=thing, callback_api_version=CallbackAPIVersion.VERSION2)
{auth_call}
{tls_call}
# Keep retrying: the Pi often boots before the HQ laptop is reachable.
while True:
    try:
        c.connect(CFG["FPMS_IOT_ENDPOINT"], port, 30)
        break
    except Exception as e:
        print("connect failed (%s), retrying in 5s" % e, flush=True)
        time.sleep(5)
c.loop_start()
# Announce ourselves so the dashboard's Devices tab lights up immediately.
# qos=1 for events: an "online" notice or a fire alert must not be silently
# dropped the way qos=0 allows. Telemetry stays qos=0 - losing one pose sample
# between 5-second updates costs nothing, and qos=1 on a firehose is expensive.
c.publish("fpms/%s/events/online" % thing,
          json.dumps({{"ts": time.time(), "thing": thing, "status": "online"}}), qos=1)
# Replace these with real driver reads from your ldlidar / thermal / camera stack.
# Publish detections/alerts to fpms/<thing>/events/... with qos=1.
while True:
    payload = {{"ts": time.time(), "thing": thing, "note": "hook up your ldlidar_ros2 topic here"}}
    c.publish("fpms/%s/telemetry/pose" % thing, json.dumps(payload), qos=0)
    time.sleep(interval)
PUB
sudo chmod +x /usr/local/bin/fpms-publisher

sudo apt-get update -y
sudo apt-get install -y python3-paho-mqtt || pip3 install paho-mqtt

sudo tee /etc/systemd/system/fpms-publisher.service >/dev/null <<'SVC'
[Unit]
Description=FPMS rover MQTT publisher
After=network-online.target
Wants=network-online.target
[Service]
ExecStart=/usr/local/bin/fpms-publisher
Restart=always
RestartSec=3
User=root
[Install]
WantedBy=multi-user.target
SVC

sudo systemctl daemon-reload
sudo systemctl enable --now fpms-publisher
sudo systemctl status fpms-publisher --no-pager | tail -n 20
echo "FPMS publisher installed as thing '{thing_name}'."
"""


def build_cloud_http_provisioner_script(thing_name: str, ingest_url: str,
                                        ingest_token: str,
                                        publish_interval: float = 5.0) -> str:
    """Publisher for the Cloudflare cloud app: HTTPS POST instead of MQTT.

    Chosen for field rovers on cellular because it removes every moving part
    that tends to fail out there — no broker to keep alive, no client
    certificates to provision or rotate, no inbound ports, and no tunnel.
    Carrier NAT and captive portals pass ordinary HTTPS.

    Uses only the Python standard library, so there is nothing to pip install on
    a Pi that may be on a slow or metered link.

    Readings are queued on disk when the network is down and flushed on the next
    success, so an alert raised in a dead zone still arrives. MQTT QoS 1 would
    have given delivery guarantees only once connected; this survives being
    offline entirely.
    """
    return f"""#!/usr/bin/env bash
set -euo pipefail
sudo mkdir -p /etc/fpms /var/lib/fpms

sudo tee /etc/fpms/config.env >/dev/null <<CFG
FPMS_THING_NAME={thing_name}
FPMS_INGEST_URL={ingest_url}
FPMS_INGEST_TOKEN={ingest_token}
FPMS_PUBLISH_INTERVAL={publish_interval}
CFG
sudo chmod 600 /etc/fpms/config.env

sudo tee /usr/local/bin/fpms-publisher >/dev/null <<'PUB'
#!/usr/bin/env python3
# FPMS rover publisher — HTTPS to the FPMS cloud app. Standard library only.
import json, os, time, urllib.error, urllib.request

CFG = {{}}
with open("/etc/fpms/config.env") as f:
    for line in f:
        if "=" in line and not line.startswith("#"):
            k, v = line.strip().split("=", 1)
            CFG[k] = v

THING = CFG["FPMS_THING_NAME"]
URL = CFG["FPMS_INGEST_URL"]
TOKEN = CFG["FPMS_INGEST_TOKEN"]
INTERVAL = float(CFG.get("FPMS_PUBLISH_INTERVAL", "5"))
SPOOL = "/var/lib/fpms/spool.jsonl"
MAX_SPOOL = 5000          # ~ hours of backlog; bounded so a long outage can't fill the disk
BATCH = 50                # the cloud app accepts up to 100 per request


def post(items):
    req = urllib.request.Request(
        URL,
        data=json.dumps(items).encode(),
        method="POST",
        headers={{"Content-Type": "application/json",
                 "Authorization": "Bearer " + TOKEN,
                 "User-Agent": "fpms-rover/1.0"}},
    )
    with urllib.request.urlopen(req, timeout=15) as r:
        return 200 <= r.status < 300


def spool(item):
    try:
        with open(SPOOL, "a") as f:
            f.write(json.dumps(item) + "\\n")
        # Trim from the front: newest readings matter more than oldest.
        with open(SPOOL) as f:
            lines = f.readlines()
        if len(lines) > MAX_SPOOL:
            with open(SPOOL, "w") as f:
                f.writelines(lines[-MAX_SPOOL:])
    except Exception as e:
        print("spool failed: %s" % e, flush=True)


def drain():
    if not os.path.exists(SPOOL):
        return
    try:
        with open(SPOOL) as f:
            lines = [l for l in f.read().splitlines() if l.strip()]
        if not lines:
            os.remove(SPOOL)
            return
        while lines:
            chunk, lines = lines[:BATCH], lines[BATCH:]
            if not post([json.loads(l) for l in chunk]):
                # Put the unsent remainder back and try again next tick.
                with open(SPOOL, "w") as f:
                    f.write("\\n".join(chunk + lines) + "\\n")
                return
        os.remove(SPOOL)
        print("spool drained", flush=True)
    except Exception as e:
        print("drain failed: %s" % e, flush=True)


def send(item):
    try:
        if post([item]):
            drain()
            return
    except Exception as e:
        print("post failed (%s); spooling" % e, flush=True)
    spool(item)


send({{"thing": THING, "subtype": "online", "kind": "events",
      "data": {{"ts": time.time(), "thing": THING, "status": "online"}}}})

# Replace the body below with real reads from your ldlidar / thermal / camera
# stack. Detections belong on kind="events" so they are never dropped.
while True:
    send({{"thing": THING, "subtype": "pose", "kind": "telemetry",
          "data": {{"ts": time.time(), "thing": THING,
                   "note": "hook up your ldlidar_ros2 topic here"}}}})
    time.sleep(INTERVAL)
PUB
sudo chmod +x /usr/local/bin/fpms-publisher

sudo tee /etc/systemd/system/fpms-publisher.service >/dev/null <<'SVC'
[Unit]
Description=FPMS rover publisher (HTTPS to FPMS cloud)
After=network-online.target
Wants=network-online.target
[Service]
ExecStart=/usr/local/bin/fpms-publisher
Restart=always
RestartSec=5
User=root
[Install]
WantedBy=multi-user.target
SVC

sudo systemctl daemon-reload
sudo systemctl enable --now fpms-publisher
sudo systemctl status fpms-publisher --no-pager | tail -n 20
echo "FPMS cloud publisher installed as thing '{thing_name}'."
"""


def ssh_provision(ip: str, username: str, script: str,
                  password: str | None = None, key_path: str | None = None,
                  port: int = 22) -> dict[str, Any]:
    """SCP the script to /tmp on the device and run it with sudo."""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        kwargs: dict[str, Any] = {
            "hostname": ip, "port": port, "username": username,
            "timeout": SSH_TIMEOUT_S, "look_for_keys": False, "allow_agent": False,
        }
        if key_path: kwargs["key_filename"] = key_path
        elif password: kwargs["password"] = password
        client.connect(**kwargs)
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"ssh connect failed: {e}"}
    try:
        sftp = client.open_sftp()
        with sftp.file("/tmp/fpms-provision.sh", "w") as f:
            f.write(script)
        sftp.chmod("/tmp/fpms-provision.sh", 0o755)
        sftp.close()
        cmd = "bash /tmp/fpms-provision.sh"
        _, stdout, stderr = client.exec_command(cmd, timeout=180)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        rc = stdout.channel.recv_exit_status()
        return {"ok": rc == 0, "exit_code": rc, "stdout": out[-4000:], "stderr": err[-2000:]}
    finally:
        client.close()
