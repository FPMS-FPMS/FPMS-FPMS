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
    ("fpms/+/events/#", 1),
]


class Bridge:
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.connected = False
        self.last_message_at: float | None = None
        self.messages_seen = 0
        self.things_seen: set[str] = set()

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
        port = settings.mqtt_port
        if settings.aws_mode == "cloud" and port == 1883:
            port = 8883
        log.info("connecting to MQTT %s:%s (tls=%s)", settings.mqtt_host, port,
                 settings.mqtt_tls or settings.aws_mode == "cloud")
        try:
            self.client.connect(settings.mqtt_host, port, keepalive=30)
        except OSError as e:
            log.warning("initial MQTT connect failed (%s); paho will retry", e)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.loop_stop()
        try:
            self.client.disconnect()
        except Exception:  # noqa: BLE001
            pass

    def status(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "host": settings.mqtt_host,
            "port": settings.mqtt_port if not (settings.aws_mode == "cloud" and settings.mqtt_port == 1883) else 8883,
            "tls": settings.mqtt_tls or settings.aws_mode == "cloud",
            "mode": settings.aws_mode,
            "messages_seen": self.messages_seen,
            "things_seen": sorted(self.things_seen),
            "last_message_at": self.last_message_at,
        }

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
        log.info("MQTT connected: %s", reason_code)
        if ok:
            for topic, qos in TOPIC_FILTERS:
                client.subscribe(topic, qos=qos)

    def _on_disconnect(self, _client, _userdata, _flags, reason_code, _props=None) -> None:
        self.connected = False
        log.warning("MQTT disconnected: %s", reason_code)

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
