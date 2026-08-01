"""FPMS dashboard FastAPI app.

Runs on the operator's laptop. Anyone on the LAN can hit http://<laptop-ip>:5173
(the Vite dev server) which talks to this backend on :8000. Nothing runs on a
remote server.

AWS surface used (all against LocalStack — same API shape as real AWS):
    - AWS IoT Core           : device telemetry over MQTT (Mosquitto broker
                               stands in for the IoT Core MQTT endpoint,
                               which LocalStack Community does not ship)
    - AWS IoT Data plane     : boto3 `iot-data` publish for dashboard→rover
                               commands (identical to production AWS)
    - Amazon S3              : `fpms-archive/events/...` — recent events strip
    - AWS Lambda + SNS       : event routing runs in LocalStack via the
                               existing `cloud/lambda/` pipeline
"""
from __future__ import annotations

import asyncio
import json
import logging
import re
from contextlib import asynccontextmanager
from typing import Any

import boto3
from botocore.config import Config as BotoConfig
from botocore.exceptions import BotoCoreError, ClientError
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response as FastResponse
from fastapi.staticfiles import StaticFiles

from pydantic import BaseModel

from . import analyst, aws, auth, cloud_forwarder, discovery, downloads, email_alerts, network as net_mod, settings_store, terminal as term_mod
from .config import settings
from .hub import hub
from .mqtt_bridge import get_bridge, start_bridge

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
log = logging.getLogger("fpms.dashboard")


@asynccontextmanager
async def lifespan(_app: FastAPI):
    loop = asyncio.get_running_loop()
    start_bridge(loop)
    log.info("MQTT bridge started; dashboard ready on %s:%s", settings.bind_host, settings.bind_port)
    yield
    bridge = get_bridge()
    if bridge:
        bridge.stop()


app = FastAPI(title="FPMS Dashboard", version="1.0.0", lifespan=lifespan)

# Password gate. No-op unless FPMS_PASSWORD is set.
app.add_middleware(auth.PasswordAuthMiddleware)

# Safe mode: machine-control routes are refused to public visitors even with a
# valid password, so a guessed password can't become a shell on this laptop.
app.add_middleware(auth.PublicSafeModeMiddleware)

# Wide-open CORS — the dashboard runs on the LAN and Vite serves on a
# different port than the API. Cookie-based auth needs credentialed CORS
# when the frontend origin differs from the API origin (dev only).
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---- Auth --------------------------------------------------------------

class LoginBody(BaseModel):
    password: str


@app.get("/api/auth-status")
def auth_status(request: Request) -> dict[str, Any]:
    tok = request.cookies.get("fpms_auth")
    is_remote = auth.request_is_remote(request)
    return {
        "auth_required": auth.auth_enabled(),
        "authenticated": bool(tok and auth.valid_token(tok)),
        "is_lan": auth.is_lan(request.client.host if request.client else None),
        "client_host": request.client.host if request.client else None,
        # Drives which tabs the UI offers. Purely cosmetic — the middleware
        # enforces this regardless of what the client chooses to render.
        "is_remote": is_remote,
        "safe_mode": settings.public_safe_mode,
        "role": settings.role,
        # Cloud deployments are always data-only; the edge app only locks down
        # when the visitor arrived over the public tunnel.
        "controls_disabled": settings.is_cloud or (settings.public_safe_mode and is_remote),
    }


@app.post("/api/login")
def login(body: LoginBody, response: Response) -> dict[str, Any]:
    if not auth.check_password(body.password):
        raise HTTPException(401, "invalid password")
    token = auth.make_token()
    auth.issue_cookie(response, token)
    return {"ok": True, "token": token}


@app.post("/api/logout")
def logout(response: Response) -> dict[str, Any]:
    auth.clear_cookie(response)
    return {"ok": True}


# ---- REST -----------------------------------------------------------------

@app.get("/api/health")
def health() -> dict[str, Any]:
    bridge = get_bridge()
    mqtt = bridge.status() if bridge else {
        "connected": False,
        "problem": "MQTT bridge never started — check launch.log for an import "
                   "error during startup.",
    }
    # "ok" used to be a hardcoded True, which made it useless: the dashboard
    # reported ok:true while the broker was refusing every connection and no
    # rover data could possibly arrive.
    return {
        "ok": bool(mqtt.get("connected")),
        "problems": [p for p in (mqtt.get("problem"),) if p],
        "mqtt": mqtt,
        "frontend": _frontend_status(),
        "channels": hub.channels(),
        "aws": {
            # effective_mode reflects real credentials when they exist, so the
            # Overview pill can't disagree with the AWS tab.
            "mode": aws.effective_mode(),
            "endpoint": settings.aws_endpoint_url or "aws-real",
            "region": settings.aws_region,
            # Cached TCP probe. This used to call S3 list_buckets, which took
            # ~8.5s whenever LocalStack was down — on an endpoint the Overview
            # page polls every 2.5s, so requests queued up faster than they
            # completed and every status pill lagged.
            "reachable": aws.endpoint_reachable(),
        },
        "cloud": cloud_forwarder.forwarder.status(),
    }


@app.get("/api/info")
def info() -> dict[str, Any]:
    """Structured mission info for the Intro page.

    Mirrors github.com/FPMS-FPMS/README.md. Kept as data (not fetched at
    runtime) so the page renders even when offline.
    """
    return FPMS_INFO


@app.get("/api/events")
def recent_events(limit: int = 25) -> dict[str, Any]:
    """Recent events from the LocalStack S3 archive.

    Empty list is fine — LocalStack may not be up, or nothing has fired yet.
    """
    try:
        s3 = _s3_client()
        resp = s3.list_objects_v2(Bucket=settings.archive_bucket, Prefix="events/", MaxKeys=200)
        objs = sorted(
            resp.get("Contents", []),
            key=lambda o: o["LastModified"],
            reverse=True,
        )[:limit]
        out = []
        for o in objs:
            body = s3.get_object(Bucket=settings.archive_bucket, Key=o["Key"])["Body"].read()
            try:
                out.append({"key": o["Key"], "modified": o["LastModified"].isoformat(),
                            "event": json.loads(body)})
            except Exception:  # noqa: BLE001
                out.append({"key": o["Key"], "modified": o["LastModified"].isoformat(),
                            "event": {"raw": body.decode("utf-8", errors="replace")}})
        return {"bucket": settings.archive_bucket, "count": len(out), "events": out}
    except (BotoCoreError, ClientError, OSError) as e:
        return {"bucket": settings.archive_bucket, "count": 0, "events": [], "note": str(e)}


@app.get("/api/aws/services")
def aws_services() -> dict[str, Any]:
    """Live inventory of every AWS service in play (via LocalStack)."""
    return aws.inventory()


@app.get("/api/aws/things")
def aws_things() -> dict[str, Any]:
    return {"things": aws.iot_things()}


@app.post("/api/aws/verify")
def aws_verify() -> dict[str, Any]:
    """Run an end-to-end smoke test across every AWS service in play."""
    return aws.verify_all()


class RegisterThingBody(BaseModel):
    thing_name: str
    hostname: str | None = None
    ip: str | None = None


@app.post("/api/aws/register-thing")
def aws_register_thing(body: RegisterThingBody) -> dict[str, Any]:
    """Create an AWS IoT Thing with certs + policy, ready to ship to a device."""
    try:
        return aws.iot_register_thing(
            body.thing_name,
            attributes={k: v for k, v in {
                "hostname": body.hostname or "",
                "ip": body.ip or "",
            }.items() if v},
        )
    except Exception as e:  # noqa: BLE001
        raise HTTPException(500, f"IoT register failed: {e}") from e


@app.get("/api/discovery/interfaces")
def discovery_interfaces() -> dict[str, Any]:
    return {"interfaces": discovery.list_interfaces()}


class ScanBody(BaseModel):
    cidr: str
    ports: list[int] | None = None


@app.post("/api/discovery/scan")
async def discovery_scan(body: ScanBody) -> dict[str, Any]:
    try:
        results = await discovery.scan_subnet(body.cidr, body.ports)
    except ValueError as e:
        raise HTTPException(400, str(e)) from e
    return {"cidr": body.cidr, "count": len(results), "hosts": results}


class SshBody(BaseModel):
    ip: str
    username: str
    password: str | None = None
    key_path: str | None = None
    port: int = 22


@app.post("/api/discovery/ssh")
def discovery_ssh(body: SshBody) -> dict[str, Any]:
    return discovery.ssh_probe(
        body.ip, body.username,
        password=body.password, key_path=body.key_path, port=body.port,
    )


class ProvisionBody(BaseModel):
    ip: str
    username: str
    password: str | None = None
    key_path: str | None = None
    port: int = 22
    thing_name: str
    # Where the rover should publish. Defaults to this laptop's Mosquitto,
    # which is what the dashboard subscribes to. Set use_aws to route through
    # AWS IoT Core with TLS instead.
    broker_host: str | None = None
    use_aws: bool | None = None
    # Field/cellular rovers: point them at the cloud broker over TLS. Supply the
    # broker's CA certificate (deploy/mosquitto/certs/ca.crt) so the rover can
    # verify it. Without TLS a cellular rover would send credentials in clear.
    broker_tls: bool = False
    broker_ca_cert: str | None = None
    # Cloudflare cloud app: publish over HTTPS instead of MQTT. No broker, no
    # certificates on the device — the practical choice for cellular rovers.
    cloud_ingest_url: str | None = None
    cloud_ingest_token: str | None = None


@app.post("/api/discovery/provision")
def discovery_provision(body: ProvisionBody) -> dict[str, Any]:
    """End-to-end: work out the transport, then SSH in and install the
    publisher unit on the device."""
    use_aws = body.use_aws if body.use_aws is not None else (settings.aws_mode == "cloud")

    # Cloudflare cloud app takes priority when an ingest URL is given: it's the
    # only transport that needs neither a broker nor certificates on the rover.
    if body.cloud_ingest_url:
        if not body.cloud_ingest_token:
            raise HTTPException(400, "cloud_ingest_url requires cloud_ingest_token")
        script = discovery.build_cloud_http_provisioner_script(
            thing_name=body.thing_name,
            ingest_url=body.cloud_ingest_url,
            ingest_token=body.cloud_ingest_token,
        )
        result = discovery.ssh_provision(
            body.ip, body.username, script,
            password=body.password, key_path=body.key_path, port=body.port,
        )
        return {
            "thing": body.thing_name,
            "transport": "cloudflare-https",
            "iot_endpoint": body.cloud_ingest_url,
            "mqtt_port": None,
            "install": result,
        }

    if use_aws:
        try:
            bundle = aws.iot_register_thing(body.thing_name, attributes={"ip": body.ip})
        except Exception as e:  # noqa: BLE001
            raise HTTPException(500, f"IoT register failed: {e}") from e
        thing = bundle["thing_name"]
        endpoint = bundle["iot_endpoint"]
        script = discovery.build_provisioner_script(
            thing_name=thing,
            iot_endpoint=endpoint,
            cert_pem=bundle["certificate_pem"],
            private_key=bundle["private_key"],
            use_tls=True,
        )
    else:
        thing = body.thing_name
        if body.broker_tls:
            # Field/cellular rover -> cloud broker. A LAN IP would be
            # unreachable, so the caller must name the broker explicitly.
            if not body.broker_host:
                raise HTTPException(
                    400,
                    "broker_tls needs broker_host, e.g. mqtt.yourdomain.com. A LAN "
                    "address is unreachable from a rover on cellular.",
                )
            if not body.broker_ca_cert:
                raise HTTPException(
                    400,
                    "broker_tls requires broker_ca_cert so the rover can verify the "
                    "broker. Use deploy/mosquitto/certs/ca.crt from the cloud host.",
                )
            endpoint = body.broker_host
        else:
            # "localhost" on the Pi would mean the Pi, so resolve our LAN address.
            endpoint = body.broker_host or net_mod.primary_lan_ip()
            if not endpoint:
                raise HTTPException(
                    503,
                    "No LAN address found for this laptop, so the rover has nowhere "
                    "to publish. Connect to WiFi, or pass broker_host explicitly.",
                )
        script = discovery.build_provisioner_script(
            thing_name=thing,
            iot_endpoint=endpoint,
            use_tls=body.broker_tls,
            ca_cert=body.broker_ca_cert,
            mqtt_port=8883 if body.broker_tls else settings.mqtt_port,
            # The rover authenticates with the same broker account this
            # dashboard uses, so a secured broker doesn't silently reject it.
            mqtt_username=settings.mqtt_username,
            mqtt_password=settings.mqtt_password,
        )

    result = discovery.ssh_provision(
        body.ip, body.username, script,
        password=body.password, key_path=body.key_path, port=body.port,
    )
    if use_aws:
        transport, port = "aws-iot-tls", 8883
    elif body.broker_tls:
        transport, port = "cloud-mqtt-tls", 8883
    else:
        transport, port = "lan-mqtt", settings.mqtt_port
    return {
        "thing": thing,
        "transport": transport,
        "iot_endpoint": endpoint,
        "mqtt_port": port,
        "install": result,
    }


@app.get("/api/alerts/status")
def alerts_status() -> dict[str, Any]:
    return email_alerts.status()


class TestEmailBody(BaseModel):
    to: str | None = None


@app.post("/api/alerts/test")
def alerts_test(body: TestEmailBody | None = None) -> dict[str, Any]:
    return email_alerts.send_test(to=body.to if body else None)


class SmtpCredsBody(BaseModel):
    user: str
    pw: str
    to: str | None = None
    host: str | None = None
    port: int | None = None


@app.post("/api/alerts/signin")
def alerts_signin(body: SmtpCredsBody) -> dict[str, Any]:
    """Save the SMTP credentials the user entered from the UI."""
    return email_alerts.save_smtp_credentials(
        user=body.user, pw=body.pw, to=body.to, host=body.host, port=body.port,
    )


class ResendCredsBody(BaseModel):
    api_key: str
    to: str | None = None
    from_: str | None = None


@app.post("/api/alerts/signin-resend")
def alerts_signin_resend(body: ResendCredsBody) -> dict[str, Any]:
    """Save Resend credentials (alternative when Gmail App Passwords are blocked)."""
    return email_alerts.save_resend_credentials(
        api_key=body.api_key, to=body.to, from_=body.from_,
    )


@app.post("/api/alerts/signout")
def alerts_signout() -> dict[str, Any]:
    """Forget saved SMTP credentials."""
    return email_alerts.clear_credentials()


@app.get("/api/analyst/status")
def analyst_status() -> dict[str, Any]:
    return analyst.status()


@app.get("/api/analyst/report")
def analyst_report(thing: str = "rover1") -> dict[str, Any]:
    return analyst.report(thing)


class ChatBody(BaseModel):
    question: str
    thing: str = "rover1"


@app.post("/api/analyst/chat")
def analyst_chat(body: ChatBody) -> dict[str, Any]:
    return analyst.chat(body.question, body.thing)


@app.get("/api/network")
def network_status() -> dict[str, Any]:
    return {
        "urls": net_mod.reachable_urls(),
        "firewall": net_mod.firewall_status(),
        "tunnel": net_mod.tunnel_status(),
    }


@app.post("/api/network/firewall/allow")
def network_firewall_allow() -> dict[str, Any]:
    return net_mod.firewall_add_elevated()


@app.post("/api/network/tunnel/start")
def network_tunnel_start() -> dict[str, Any]:
    # Supervised: a tunnel started from the UI should survive a cloudflared
    # crash too, not just one started at boot.
    return net_mod.start_tunnel_supervised()


@app.post("/api/network/tunnel/stop")
def network_tunnel_stop() -> dict[str, Any]:
    return net_mod.stop_tunnel()


class GatewayBody(BaseModel):
    gateway_url: str = ""
    gateway_secret: str = ""


@app.get("/api/network/gateway")
def network_gateway_get() -> dict[str, Any]:
    """Gateway config with the secret withheld — the UI only needs to know
    whether one is stored, never its value."""
    cfg = net_mod.gateway_config()
    return {"gateway_url": cfg["url"], "secret_set": bool(cfg["secret"])}


@app.post("/api/network/gateway")
def network_gateway_set(body: GatewayBody) -> dict[str, Any]:
    url = body.gateway_url.strip().rstrip("/")
    if url and not url.startswith("https://"):
        raise HTTPException(400, "gateway_url must start with https://")

    updates: dict[str, Any] = {"gateway_url": url}
    # An empty secret means "keep what's stored", so the UI can edit the URL
    # without having to round-trip a secret it was never given.
    if body.gateway_secret.strip():
        updates["gateway_secret"] = body.gateway_secret.strip()
    settings_store.set_many(updates)

    return {"ok": True, **net_mod.tunnel_status()}


@app.get("/qr")
def qr_code(data: str) -> FastResponse:
    if len(data) > 512:
        raise HTTPException(400, "data too long")
    png = net_mod.qr_png(data)
    return FastResponse(content=png, media_type="image/png",
                        headers={"Cache-Control": "public, max-age=300"})


@app.get("/api/downloads")
def downloads_catalog(request: Request) -> dict[str, Any]:
    host = request.headers.get("host", "localhost:8000")
    return downloads.catalog(host)


@app.get("/download/windows")
def download_windows() -> FileResponse:
    p = downloads.windows_download_path()
    if not p:
        raise HTTPException(404, "Windows build not available on this host")
    return FileResponse(
        p,
        media_type="application/vnd.microsoft.portable-executable",
        filename=p.name,
    )


@app.post("/api/rover/{thing}/{action}")
def rover_command(thing: str, action: str) -> dict[str, Any]:
    """Publish a control-plane command to a real Orange Pi 5B via AWS IoT Core.

    Uses the AWS `iot-data` API (boto3 → LocalStack) so this is the exact same
    call the app would make against production AWS. Identical code, identical
    IAM shape — only the endpoint URL changes.
    """
    if action not in {"connect", "disconnect"}:
        raise HTTPException(400, "action must be connect or disconnect")
    topic = f"fpms/{thing}/commands/{action}"

    # Mosquitto first: LocalStack's iot-data accepts the publish and no rover
    # ever hears it, because the rovers are subscribed to the broker this
    # bridge is already connected to. The boto3 call stays as the fallback —
    # it is the real production-AWS shape, and it still works if paho is down.
    bridge = get_bridge()
    if bridge is not None:
        sent = bridge.publish_command(thing, action)
        if sent.get("ok"):
            return {"ok": True, "topic": topic, "via": sent["via"]}
        log.warning("bridge publish of %s failed (%s); trying iot-data",
                    topic, sent.get("error") or sent.get("rc"))

    try:
        client = _iot_data_client()
        client.publish(
            topic=topic,
            qos=1,
            payload=json.dumps({"source": "dashboard"}).encode("utf-8"),
        )
    except Exception as e:  # noqa: BLE001
        raise HTTPException(502, f"IoT Core publish failed: {e}") from e
    return {"ok": True, "topic": topic, "via": "aws-iot-data"}


# `thing` is interpolated straight into an MQTT topic, so an unvalidated value
# ("+", "../", a whole path) could publish anywhere in the fpms tree.
THING_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")

# ALLOWLIST: the full set of actions the rover fleet supports. `action` is
# interpolated into an MQTT topic just like `thing` (see THING_RE above), so
# an action not in this set must 400 here rather than being published and
# left to be nacked on the far end.
#
# On the rover, no single process owns this whole list. The authoritative
# source for each half is on the rover itself:
#
#   fpms_teleop.py    TELEOP_ACTIONS  — motion, actuators, the board-side
#                     diagnostics, and its own drive_* verbs. It publishes this
#                     dict verbatim on events/online and in every drive_status
#                     reply, precisely so the dashboard does not have to guess.
#   fpms_missions.py  "mission"       — the mission executor, which also ACTS
#                     on stop/estop/auto_off/set_coordinate without answering.
#   fpms_rover_agent  ping, status, connect, disconnect, restart.
#
# A command's arrival here does not imply a single receiver on the far end, and
# an action being in this set is NOT a claim that anything answers it — see
# `auto_on` below.
#
# THIS SET MUST BE A SUPERSET OF THE ROVER'S. It is only a topic-injection
# guard: a verb the rover supports but this set omits is refused with a 400
# before it is ever published, which looks to the operator exactly like a
# dashboard bug and cannot be diagnosed from the rover side at all. That is what
# happened to the three drive_* verbs below — fpms-teleop subscribed and
# advertised them, and every attempt to send one 400'd here.
CONTROL_ACTIONS = {
    # Lifecycle / connectivity — fpms-rover-agent.
    "connect", "disconnect", "restart",
    # Safety / autonomy toggles.
    #
    # `auto_on` is deliberately kept even though NOTHING on the rover subscribes
    # it: fpms-rover-agent answers it with a nack that names where autonomy
    # actually lives (the mission executor). Dropping it from this set would
    # replace that useful refusal with a 400 that says nothing.
    "stop", "estop", "auto_on", "auto_off",
    # Manual driving (Drive page): a streamed analog jog, bounded steps, a
    # closed-loop turn, named missions, and a pose correction.
    "jog", "nudge", "turn", "mission", "set_coordinate",
    # Diagnostics / sensing.
    "test_motors", "read_encoders", "ping", "status",
    # fpms-teleop's OWN status and stream verbs. Distinctly named so they do not
    # shadow fpms-rover-agent's ping/status/connect/disconnect — the two
    # services answer different questions and both must stay reachable, which is
    # the whole reason teleop chose separate names.
    #
    # `drive_status` is the richest read on the rover (motion envelope, deadband
    # floors, micro-ROS link, and the live TELEOP_ACTIONS list itself) and it
    # commands no motion, so it is also the dashboard's capability-refresh path.
    "drive_status", "drive_connect", "drive_disconnect",
    # Actuators / misc.
    "beep", "servo", "set_speed",
}


class CommandBody(BaseModel):
    params: dict[str, Any] = {}


@app.post("/api/control/{thing}/{action}")
def control_command(thing: str, action: str,
                    body: CommandBody = CommandBody()) -> dict[str, Any]:
    """Rover control plane — motion, autonomy and actuator commands.

    HQ-only: `/api/control/` is in auth._PRIVILEGED_PREFIXES, so the cloud role
    and public-tunnel visitors are refused before they reach this handler.
    """
    if not THING_RE.match(thing):
        raise HTTPException(400, "invalid thing name")
    if action not in CONTROL_ACTIONS:
        raise HTTPException(400, f"action must be one of {sorted(CONTROL_ACTIONS)}")

    bridge = get_bridge()
    if bridge is None:
        raise HTTPException(502, "MQTT bridge is not running")
    sent = bridge.publish_command(thing, action, body.params)
    if not sent.get("ok"):
        detail = sent.get("error") or f"broker returned rc={sent.get('rc')}"
        raise HTTPException(502, f"command publish failed: {detail}")
    return {"ok": True, "thing": thing, "action": action,
            "topic": sent["topic"], "via": sent["via"]}


# ---- Terminal WebSocket ---------------------------------------------------

@app.websocket("/ws/term/ssh")
async def ws_terminal_ssh(ws: WebSocket) -> None:
    """SSH terminal session. Auth-checked via cookie during handshake by
    the middleware; credentials arrive as query params on the WS URL."""
    await ws.accept()
    q = ws.query_params
    host = q.get("host", "").strip()
    if not host:
        await ws.send_text("\r\nMissing 'host' query param.\r\n")
        await ws.close()
        return
    await term_mod.run_ssh_session(
        ws,
        host=host,
        port=int(q.get("port", "22")),
        username=q.get("username", "orangepi"),
        password=q.get("password") or None,
        key_path=q.get("key_path") or None,
    )
    try:
        await ws.close()
    except Exception:  # noqa: BLE001
        pass


@app.websocket("/ws/term/local")
async def ws_terminal_local(ws: WebSocket) -> None:
    """Local PowerShell — LAN clients only."""
    client_host = ws.client.host if ws.client else None
    if not auth.is_lan(client_host):
        await ws.accept()
        await ws.send_text(
            f"\r\n\x1b[31m✗ Local shell blocked. Client {client_host} is not on the LAN.\x1b[0m\r\n"
        )
        await ws.close()
        return
    await ws.accept()
    await term_mod.run_local_shell(ws)
    try:
        await ws.close()
    except Exception:  # noqa: BLE001
        pass


# ---- Telemetry WebSocket --------------------------------------------------

@app.websocket("/ws/{channel}")
async def stream(ws: WebSocket, channel: str) -> None:
    await hub.connect(channel, ws)
    try:
        while True:
            # We never expect a client message, but reading keeps the socket
            # honest and lets us detect a browser tab close promptly.
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await hub.disconnect(channel, ws)


# ---- Static frontend ------------------------------------------------------
# In production the React app is built to frontend/dist/ and served from
# the same origin as the API. In dev, `npm run dev` on :5173 proxies /api
# and /ws here (see frontend/vite.config.ts). The PyInstaller launcher sets
# FPMS_FRONTEND_DIST to the extracted bundle path when running as a frozen exe.

import os

# FPMS_FRONTEND_DIST lets a packaged build serve a NEWER UI than the one frozen
# into it — the exe bundles frontend/dist (see build/fpms.spec), so without an
# override the installed app shows whatever UI it was built with, no matter how
# many times the frontend is rebuilt.
#
# The override only wins if it actually exists. Taking it blindly would mean a
# moved or deleted directory serves NO interface at all: every tab gone, which
# is the exact failure the override was added to prevent.
_dist_override = os.environ.get("FPMS_FRONTEND_DIST")
_bundled_dist = Path(__file__).resolve().parent.parent / "frontend" / "dist"
if _dist_override and Path(_dist_override).is_dir():
    FRONTEND_DIST = Path(_dist_override)
    FRONTEND_DIST_SOURCE = "FPMS_FRONTEND_DIST override"
else:
    if _dist_override:
        log.error("FPMS_FRONTEND_DIST=%r is not a directory; falling back to the "
                  "bundled UI. Fix or clear that variable — the UI you are "
                  "looking at is NOT the one you pointed at.", _dist_override)
        FRONTEND_DIST_SOURCE = "bundled (override path does not exist)"
    else:
        FRONTEND_DIST_SOURCE = "bundled"
    FRONTEND_DIST = _bundled_dist

log.info("serving frontend from %s (%s)", FRONTEND_DIST, FRONTEND_DIST_SOURCE)
if not FRONTEND_DIST.is_dir():
    log.error("No frontend build at %s — the API works but every page will be "
              "missing. Run `npm run build` in cloud/dashboard/frontend.",
              FRONTEND_DIST)


def _frontend_status() -> dict[str, Any]:
    """Which UI this process is actually serving.

    Serving a stale or missing dist is invisible from the browser — the app just
    looks old or blank — so /api/health has to name the exact directory and where
    that choice came from.
    """
    index = FRONTEND_DIST / "index.html"
    st: dict[str, Any] = {
        "dist": str(FRONTEND_DIST),
        "source": FRONTEND_DIST_SOURCE,
        "override_env": _dist_override or None,
        "override_honoured": FRONTEND_DIST_SOURCE.startswith("FPMS_FRONTEND_DIST"),
        "exists": FRONTEND_DIST.is_dir(),
        "index_html": index.is_file(),
        "built_at": None,
    }
    try:
        if index.is_file():
            st["built_at"] = index.stat().st_mtime
    except OSError:
        pass
    return st

if FRONTEND_DIST.is_dir():
    app.mount(
        "/assets",
        StaticFiles(directory=str(FRONTEND_DIST / "assets")),
        name="assets",
    )

    @app.get("/favicon.svg")
    def favicon() -> FileResponse:
        return FileResponse(FRONTEND_DIST / "favicon.svg")

    # Browsers are strict about these two: a manifest served as text/html is
    # ignored, and the app becomes non-installable.
    _ROOT_FILE_TYPES = {
        ".webmanifest": "application/manifest+json",
        ".js": "text/javascript",
    }

    @app.get("/", include_in_schema=False)
    @app.get("/{full_path:path}", include_in_schema=False)
    def spa(full_path: str = "") -> FileResponse:
        # Anything not caught by /api or /ws or /assets falls back to
        # index.html so React Router owns client-side routes.
        if full_path.startswith(("api/", "ws/", "assets/")):
            raise HTTPException(404, "not found")

        # ...but a real file sitting at the web root must be served as itself.
        # manifest.webmanifest, sw.js and the pwa-*.png icons all live here, and
        # handing back index.html for them silently breaks "Add to Home Screen"
        # and the service worker.
        if full_path:
            root = FRONTEND_DIST.resolve()
            candidate = (root / full_path).resolve()
            # Confine to the dist directory so "../" can't escape it.
            if candidate.is_file() and candidate.is_relative_to(root):
                return FileResponse(
                    candidate,
                    media_type=_ROOT_FILE_TYPES.get(candidate.suffix.lower()),
                )

        # Never cache the SPA shell. Asset filenames are content-hashed so they
        # can be cached forever, but a stale index.html points at an old bundle
        # — which is exactly how the app kept showing a previous build's UI
        # after an update, in both Edge and the embedded WebView2.
        return FileResponse(
            FRONTEND_DIST / "index.html",
            headers={"Cache-Control": "no-store, must-revalidate"},
        )
else:
    @app.get("/")
    def root() -> JSONResponse:
        return JSONResponse({
            "name": "FPMS Dashboard API",
            "note": "frontend not built yet; run `npm run build` in frontend/",
            "health": "/api/health",
        })


# ---- helpers --------------------------------------------------------------

def _boto_kwargs(_service: str) -> dict[str, Any]:
    k: dict[str, Any] = {
        "region_name": settings.aws_region,
        "config": BotoConfig(retries={"max_attempts": 1}, connect_timeout=2, read_timeout=5),
    }
    if settings.aws_mode == "local":
        k["endpoint_url"] = settings.aws_endpoint_url
        k["aws_access_key_id"] = settings.aws_access_key
        k["aws_secret_access_key"] = settings.aws_secret_key
    return k


def _s3_client():
    return boto3.client("s3", **_boto_kwargs("s3"))


def _iot_data_client():
    return boto3.client("iot-data", **_boto_kwargs("iot-data"))


def _localstack_reachable() -> bool:
    try:
        _s3_client().list_buckets()
        return True
    except Exception:  # noqa: BLE001
        return False


# Snapshot of README-derived info, served to the Intro page. Keeping this in
# code (not fetched) means the page works even with no internet.
FPMS_INFO: dict[str, Any] = {
    "name": "FPMS",
    "tagline": "Protecting the past with the power of the present.",
    "summary": (
        "Two small autonomous rovers keeping quiet watch over land where cultural "
        "heritage sites meet the growing risk of wildfire."
    ),
    "team": [
        {"name": "Aryan Wadhawan", "role": "Systems, programming, cultural outreach lead"},
        {"name": "Alex Tang", "role": "Hardware, mechanical design, integration lead"},
    ],
    "school": "David Leeder Middle School, Toronto (Grade 8)",
    "achievements": [
        "🥇 Gold, WRO Canada Nationals 2026 — Montreal",
        "🌎 Advancing to WRO International Finals, San Juan, Puerto Rico — December 2026",
    ],
    "mission": {
        "reactive": {
            "title": "Reactive — When something is already burning",
            "body": (
                "Thermal + vision AI detect a hotspot. Both cameras must agree before "
                "the rover acts. It drives to the source, releases water from a small "
                "onboard pump, and logs every decision to the cloud."
            ),
        },
        "proactive": {
            "title": "Proactive — Before anything is burning",
            "body": (
                "The rover scans vegetation for dry brush, ground temperature, and "
                "fuel buildup — building a live risk map that can support Indigenous "
                "cultural burning practices. The community always makes the burn decisions."
            ),
        },
    },
    "how_it_sees": [
        {"camera": "RGB", "sees": "Visible light", "good_at": "Recognizing shapes (YOLO)"},
        {"camera": "Thermal", "sees": "Infrared heat", "good_at": "Embers under leaves, hot ground before flames"},
    ],
    "hardware_rover1": [
        {"component": "Radxa ROCK 5B+ (RK3588)", "purpose": "Main compute — Nav2, SLAM, YOLO26 on 6 TOPS NPU"},
        {"component": "Yahboom V3.0 motor board (STM32F103)", "purpose": "Motor control, encoder/IMU fusion"},
        {"component": "4× Yahboom 520 motors, 86mm off-road wheels", "purpose": "Locomotion"},
        {"component": "LDROBOT D500 LiDAR", "purpose": "2D SLAM mapping"},
        {"component": "HBV RGB camera", "purpose": "YOLO vision"},
        {"component": "Waveshare thermal camera (LWIR)", "purpose": "Heat detection"},
        {"component": "MG90S servo", "purpose": "Shared camera pan"},
        {"component": "R385 pump", "purpose": "Water suppression"},
        {"component": "Yahboom 9600mAh 12V Li-ion", "purpose": "Main power"},
    ],
    "stack": [
        "Ubuntu 22.04 LTS", "ROS2 Humble + Cyclone DDS", "Nav2 + SLAM Toolbox",
        "YOLO26 via Rockchip RKNN (FP16, 15+ FPS measured)", "EKF via robot_localization",
        "AWS IoT Core (MQTT/TLS), S3, Lambda + SNS", "FastAPI dashboard",
    ],
    "principles": [
        "Cross-validated perception (RGB + thermal must agree)",
        "Event-based, not streaming — video stays on the rover",
        "Everything the rover does is public by default",
        "Technology supports Indigenous fire stewardship — it does not replace it",
    ],
    "repo": "https://github.com/FPMS-FPMS",
}
