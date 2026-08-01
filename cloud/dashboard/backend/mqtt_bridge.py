"""MQTT → WebSocket bridge.

Subscribes to the sensor telemetry topics the rovers publish on Mosquitto
(the same broker the existing `iot-rule-bridge` uses) and fans each message
out through the WebSocket hub. Also runs the thermal analyzer on every
thermal frame and re-broadcasts the analysis on its own channel.

Topic map:
    fpms/<thing>/telemetry/lidar   → channel "lidar:<thing>"
    fpms/<thing>/telemetry/camera  → channel "camera:<thing>"
    fpms/<thing>/telemetry/thermal → channel "thermal:<thing>"
                                     + analysis on  "thermal-analysis:<thing>"
    fpms/<thing>/telemetry/pose    → channel "pose:<thing>"
    fpms/+/events/#                → channel "events"
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any

import paho.mqtt.client as mqtt

from . import cloud_forwarder
from .config import settings
from .hub import hub
from .thermal_analysis import analyzer

log = logging.getLogger("fpms.mqtt")

TOPIC_FILTERS = [
    ("fpms/+/telemetry/lidar", 0),
    ("fpms/+/telemetry/camera", 0),
    ("fpms/+/telemetry/thermal", 0),
    ("fpms/+/telemetry/pose", 0),
    # Drive/teleop health from fpms_teleop.py on the rover: battery volts, pose,
    # measured topic rates and micro-ROS link state. The Drive page's health
    # panel and its motion lockout both key off this, so without the
    # subscription the panel reads "no telemetry" forever and the operator gets
    # no warning that the link is down.
    ("fpms/+/telemetry/drive", 0),
    # Mission progress and planned route, from fpms_missions.py. The Drive page
    # already subscribed the "mission:<thing>" channel these produce, but nothing
    # subscribed the MQTT topics behind it — so the mission card could never have
    # shown anything. It would sit at "no mission" through an entire run, with no
    # error anywhere to explain why.
    ("fpms/+/telemetry/mission", 0),
    # qos=1 for the plan: unlike the others it is published once per preview
    # rather than on a heartbeat, so a dropped message is not corrected a moment
    # later by the next one. It just leaves a map with no route on it.
    ("fpms/+/telemetry/mission_plan", 1),
    ("fpms/+/events/#", 1),
]

# Every WebSocket channel the frontend opens is produced by one of the filters
# above (channel name = <subtype>:<thing>, or "events"). Keep this list in step
# with frontend/src — a channel with no matching filter is not an error anywhere,
# it is a panel that says "waiting…" forever:
#
#   lidar:<thing>            <- fpms/+/telemetry/lidar
#   camera:<thing>           <- fpms/+/telemetry/camera
#   thermal:<thing>          <- fpms/+/telemetry/thermal
#   thermal-analysis:<thing> <- derived locally from thermal frames
#   pose:<thing>             <- fpms/+/telemetry/pose
#   drive:<thing>            <- fpms/+/telemetry/drive
#   mission:<thing>          <- fpms/+/telemetry/mission
#   mission_plan:<thing>     <- fpms/+/telemetry/mission_plan
#   events                   <- fpms/+/events/#

# Broker reason codes that mean "your credentials were refused", as opposed to
# "the broker is down". The distinction matters because the remedies are
# completely different and the symptom — a dashboard where every panel waits
# forever — is identical.
_AUTH_FAILURE_CODES = {
    4,    # MQTT 3.1.1 CONNACK: bad user name or password
    5,    # MQTT 3.1.1 CONNACK: not authorized
    134,  # MQTT 5 : bad user name or password
    135,  # MQTT 5 : not authorised
}

_AUTH_HELP = (
    "The broker REFUSED our credentials. The dashboard will keep retrying and "
    "every rover panel will sit at 'waiting' with no other error. Fix it with "
    "one of:\n"
    "  1. Set credentials for this user, then restart the app:\n"
    "       [Environment]::SetEnvironmentVariable('FPMS_MQTT_USERNAME','fpms','User')\n"
    "       [Environment]::SetEnvironmentVariable('FPMS_MQTT_PASSWORD','<broker password>','User')\n"
    "  2. Or re-run the broker setup to (re)create that account:\n"
    "       powershell -ExecutionPolicy Bypass -File scripts\\Setup-Mosquitto.ps1 -Password '<broker password>'\n"
    # Deliberately ASCII-only: this text is redirected to service.log through a
    # cmd pipe, and a non-ASCII dash comes out as mojibake there.
    "Never commit the password - it belongs in the User environment only."
)

_NO_CREDS_HELP = (
    "No MQTT credentials are configured (FPMS_MQTT_USERNAME / FPMS_MQTT_PASSWORD "
    "are empty). That is fine ONLY if the broker runs with allow_anonymous true. "
    "If it does not, the connection will be refused with 'Not authorized' and "
    "every rover panel will wait forever with no visible error."
)


class Bridge:
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.connected = False
        self.last_message_at: float | None = None
        self.messages_seen = 0
        self.things_seen: set[str] = set()
        # Why we are not connected, in words the operator can act on. Surfaced
        # through /api/health so a broken broker link is diagnosable without
        # reading launch.log.
        self.last_error: str | None = None
        self.last_error_at: float | None = None
        self.auth_failed = False
        self.connect_attempts = 0
        self.subscribed: list[str] = []
        self._last_auth_log_at = 0.0

        self.client = mqtt.Client(
            client_id=settings.mqtt_client_id,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        # Username/password for a broker with allow_anonymous false. Must be set
        # before connect() — paho sends it as part of the CONNECT packet.
        if settings.mqtt_username:
            self.client.username_pw_set(settings.mqtt_username,
                                        settings.mqtt_password or None)
            log.info("MQTT auth enabled for user %r", settings.mqtt_username)
            if not settings.mqtt_password:
                log.warning(
                    "FPMS_MQTT_USERNAME is set to %r but FPMS_MQTT_PASSWORD is "
                    "EMPTY — a password-protected broker will refuse this.",
                    settings.mqtt_username,
                )
        else:
            # Starting with no credentials at all is the failure that has bitten
            # this project before: the app looks healthy, the broker answers
            # "Not authorized", and paho retries in a loop nobody reads.
            log.warning("MQTT: %s", _NO_CREDS_HELP)

        # TLS + X.509 for real AWS IoT Core, if configured.
        if settings.mqtt_tls or settings.aws_mode == "cloud":
            self._configure_tls()

    def _configure_tls(self) -> None:
        """Enable TLS. For real AWS IoT Core, the client cert + private key
        must correspond to a Thing with a policy that allows Connect/Subscribe/
        Publish on the FPMS topic tree."""
        import ssl
        ca = settings.mqtt_ca_cert or None
        cert = settings.mqtt_client_cert or None
        key = settings.mqtt_client_key or None
        try:
            self.client.tls_set(
                ca_certs=ca,
                certfile=cert,
                keyfile=key,
                tls_version=ssl.PROTOCOL_TLSv1_2,
            )
            self.client.tls_insecure_set(False)
            log.info("MQTT TLS configured (ca=%s cert=%s key=%s)",
                     bool(ca), bool(cert), bool(key))
        except Exception as e:  # noqa: BLE001
            log.error("MQTT TLS setup failed: %s", e)

    def start(self) -> None:
        # AWS IoT Core defaults to 8883 (mqtts). If cloud mode is on and
        # the operator forgot to change the port, do the right thing.
        port = self.effective_port()
        log.info("connecting to MQTT %s:%s (tls=%s, user=%s, password=%s)",
                 settings.mqtt_host, port,
                 settings.mqtt_tls or settings.aws_mode == "cloud",
                 settings.mqtt_username or "<none>",
                 "set" if settings.mqtt_password else "NOT SET")
        try:
            self.client.connect(settings.mqtt_host, port, keepalive=30)
        except OSError as e:
            self.last_error = f"could not reach broker {settings.mqtt_host}:{port}: {e}"
            self.last_error_at = time.time()
            log.error("initial MQTT connect to %s:%s failed (%s); paho will keep "
                      "retrying. Until it succeeds every rover panel stays empty.",
                      settings.mqtt_host, port, e)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.loop_stop()
        try:
            self.client.disconnect()
        except Exception:  # noqa: BLE001
            pass

    def effective_port(self) -> int:
        """The port we actually dial — see start(), which promotes 1883→8883
        in cloud mode. status() used to recompute this inline and the two could
        disagree."""
        if settings.aws_mode == "cloud" and settings.mqtt_port == 1883:
            return 8883
        return settings.mqtt_port

    def status(self) -> dict[str, Any]:
        creds_ok = bool(settings.mqtt_username and settings.mqtt_password)
        st: dict[str, Any] = {
            "connected": self.connected,
            "host": settings.mqtt_host,
            "port": self.effective_port(),
            "tls": settings.mqtt_tls or settings.aws_mode == "cloud",
            "mode": settings.aws_mode,
            "client_id": settings.mqtt_client_id,
            "messages_seen": self.messages_seen,
            "things_seen": sorted(self.things_seen),
            "last_message_at": self.last_message_at,
            "connect_attempts": self.connect_attempts,
            "subscribed": list(self.subscribed),
            # Presence only — the password itself is never returned. /api/health
            # is reachable over the public tunnel once logged in.
            "credentials": {
                "username": settings.mqtt_username or None,
                "username_set": bool(settings.mqtt_username),
                "password_set": bool(settings.mqtt_password),
            },
            "auth_failed": self.auth_failed,
            "last_error": self.last_error,
            "last_error_at": self.last_error_at,
        }
        # A single field the UI (and a human reading curl output) can act on.
        if self.connected:
            st["problem"] = None
        elif self.auth_failed:
            st["problem"] = (
                f"MQTT authentication failed against {settings.mqtt_host}:"
                f"{self.effective_port()} as "
                f"{settings.mqtt_username or '<no username configured>'}. "
                "Set FPMS_MQTT_USERNAME / FPMS_MQTT_PASSWORD (User scope) and "
                "restart, or re-run scripts\\Setup-Mosquitto.ps1."
            )
        elif not creds_ok:
            st["problem"] = (
                f"Not connected to {settings.mqtt_host}:{self.effective_port()}, "
                "and no MQTT credentials are configured. If the broker requires "
                "auth, set FPMS_MQTT_USERNAME / FPMS_MQTT_PASSWORD and restart."
            )
        else:
            st["problem"] = (
                f"Not connected to {settings.mqtt_host}:{self.effective_port()}"
                + (f" — {self.last_error}" if self.last_error else
                   " — broker unreachable. Is the mosquitto service running?")
            )
        return st

    def publish_command(self, thing: str, action: str,
                        payload: dict[str, Any] | None = None) -> dict[str, Any]:
        """Send a dashboard→rover command on the bridge's own connection.

        The rovers subscribe on Mosquitto, so a command has to go out over the
        same connection that already receives their telemetry — a publish to
        any other broker is accepted and then heard by nobody.
        """
        topic = f"fpms/{thing}/commands/{action}"
        body = dict(payload or {})
        body.setdefault("source", "dashboard")
        body.setdefault("ts", time.time())

        # A publish while paho is down is queued, not delivered, and would come
        # back rc=0 — so report the disconnect instead of a false success.
        if self.client is None or not self.connected:
            log.warning("command %s not sent: MQTT bridge is not connected", topic)
            return {"ok": False, "topic": topic, "via": "mosquitto-bridge",
                    "error": "MQTT bridge is not connected"}
        try:
            info = self.client.publish(topic, json.dumps(body), qos=1)
        except Exception as e:  # noqa: BLE001
            log.exception("command publish failed on %s", topic)
            return {"ok": False, "topic": topic, "via": "mosquitto-bridge",
                    "error": str(e)}
        return {"ok": bool(info.rc == 0), "topic": topic,
                "via": "mosquitto-bridge", "rc": int(info.rc)}

    # ---- paho callbacks (run on paho's thread) ---------------------------
    def _on_connect(self, client, _userdata, _flags, reason_code, _props=None) -> None:
        # paho v2 gives a ReasonCode object (not an int); use .is_failure.
        ok = not getattr(reason_code, "is_failure", bool(reason_code))
        self.connected = ok
        self.connect_attempts += 1

        if ok:
            self.last_error = None
            self.auth_failed = False
            log.info("MQTT connected to %s:%s as %s", settings.mqtt_host,
                     self.effective_port(), settings.mqtt_username or "<anonymous>")
            self.subscribed = []
            for topic, qos in TOPIC_FILTERS:
                client.subscribe(topic, qos=qos)
                self.subscribed.append(topic)
            log.info("MQTT subscribed to %d topic filters: %s",
                     len(self.subscribed), ", ".join(self.subscribed))
            return

        # --- refused -------------------------------------------------------
        code = int(getattr(reason_code, "value", reason_code) or 0)
        self.last_error = f"broker refused the connection: {reason_code}"
        self.last_error_at = time.time()
        self.auth_failed = code in _AUTH_FAILURE_CODES

        if self.auth_failed:
            # paho retries on a timer, so this fires repeatedly. Log the full
            # remedy the first time and once a minute after that: loud enough to
            # find, quiet enough that it doesn't bury everything else.
            now = time.time()
            if self.connect_attempts == 1 or now - self._last_auth_log_at > 60:
                self._last_auth_log_at = now
                log.error(
                    "MQTT AUTHENTICATION FAILED against %s:%s as user %r "
                    "(reason: %s, attempt %d).\n%s",
                    settings.mqtt_host, self.effective_port(),
                    settings.mqtt_username or "<anonymous — none configured>",
                    reason_code, self.connect_attempts,
                    _AUTH_HELP if settings.mqtt_username else
                    _NO_CREDS_HELP + "\n" + _AUTH_HELP,
                )
        else:
            log.error("MQTT connection refused by %s:%s — %s (attempt %d)",
                      settings.mqtt_host, self.effective_port(), reason_code,
                      self.connect_attempts)

    def _on_disconnect(self, _client, _userdata, _flags, reason_code, _props=None) -> None:
        self.connected = False
        self.subscribed = []
        if not self.auth_failed:
            self.last_error = f"disconnected from broker: {reason_code}"
            self.last_error_at = time.time()
        log.warning("MQTT disconnected from %s: %s", settings.mqtt_host, reason_code)

    def _on_message(self, _client, _userdata, msg: mqtt.MQTTMessage) -> None:
        self.messages_seen += 1
        self.last_message_at = time.time()
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:  # noqa: BLE001
            log.exception("bad payload on %s", msg.topic)
            return
        parts = msg.topic.split("/")
        if len(parts) < 4 or parts[0] != "fpms":
            return
        thing = parts[1]
        channel_kind = parts[2]
        subtype = parts[3]
        self.things_seen.add(thing)

        if channel_kind == "telemetry":
            self._dispatch_telemetry(thing, subtype, payload)
        elif channel_kind == "events":
            self._dispatch_event(thing, subtype, payload)

    # ---- routing helpers -------------------------------------------------
    def _dispatch_telemetry(self, thing: str, subtype: str, payload: dict[str, Any]) -> None:
        channel = f"{subtype}:{thing}"
        envelope = {"thing": thing, "subtype": subtype, "ts": self.last_message_at, "data": payload}
        asyncio.run_coroutine_threadsafe(hub.broadcast(channel, envelope), self.loop)
        # Throttled copy to the cloud app. The local dashboard always sees the
        # full rate; only the uplink is rate-limited.
        cloud_forwarder.forwarder.offer(thing, subtype, "telemetry", payload)

        if subtype == "thermal" and isinstance(payload.get("grid"), list):
            try:
                analysis = analyzer.analyze(payload["grid"])
            except Exception:  # noqa: BLE001
                log.exception("thermal analysis failed")
                return
            analysis_envelope = {
                "thing": thing,
                "subtype": "thermal-analysis",
                "ts": self.last_message_at,
                "data": analysis,
            }
            asyncio.run_coroutine_threadsafe(
                hub.broadcast(f"thermal-analysis:{thing}", analysis_envelope),
                self.loop,
            )

    def _dispatch_event(self, thing: str, subtype: str, payload: dict[str, Any]) -> None:
        # Events bypass the throttle entirely — an alert must not wait.
        cloud_forwarder.forwarder.offer(thing, subtype, "events", payload)
        envelope = {
            "thing": thing,
            "subtype": subtype,
            "ts": self.last_message_at,
            "data": payload,
        }
        asyncio.run_coroutine_threadsafe(hub.broadcast("events", envelope), self.loop)
        # Fire an HQ email alert for fire/alert-class events. No-op if SMTP
        # isn't configured. Runs in the paho thread but SMTP is fast enough
        # and this is a one-shot per event.
        try:
            from . import email_alerts as _ea  # local import to keep bridge cold-start light
            _ea.on_event(thing, subtype, payload)
        except Exception:  # noqa: BLE001
            log.exception("email alert hook failed")


_bridge: Bridge | None = None


def start_bridge(loop: asyncio.AbstractEventLoop) -> Bridge:
    global _bridge
    if _bridge is None:
        _bridge = Bridge(loop)
        _bridge.start()
        if cloud_forwarder.forwarder.enabled:
            cloud_forwarder.forwarder.start()
        else:
            log.info("cloud forwarding disabled (no cloud_ingest_url/token configured)")
    return _bridge


def get_bridge() -> Bridge | None:
    return _bridge
