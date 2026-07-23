"""FPMS Docker-free local pipeline (Windows / macOS / Linux — no containers).

One Python process runs all three things:
  - moto (a Python S3/SNS mock) in a background thread on http://127.0.0.1:4566
  - MQTT subscriber that listens on fpms/+/events/# via a local Mosquitto
  - The archive-and-alert logic that the Lambdas run in the Docker version

The topic taxonomy, event schema, S3 key layout, and log messages are all
identical to the Docker/LocalStack path — this is just the same pipeline
without needing Docker installed.

Requires:  pip install boto3 paho-mqtt "moto[server]"
Requires:  a running local Mosquitto broker on port 1883.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from datetime import datetime, timezone

import boto3
import paho.mqtt.client as mqtt
from moto.server import ThreadedMotoServer

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
log = logging.getLogger("fpms")

MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
S3_PORT = int(os.environ.get("S3_PORT", "4566"))
S3_ENDPOINT = f"http://127.0.0.1:{S3_PORT}"

ARCHIVE_BUCKET = "fpms-archive"
BUCKETS = (ARCHIVE_BUCKET, "fpms-heritage", "fpms-media")


def start_moto() -> ThreadedMotoServer:
    server = ThreadedMotoServer(port=S3_PORT, ip_address="127.0.0.1")
    server.start()
    log.info("moto (mock AWS) listening on %s", S3_ENDPOINT)
    return server


def make_s3_client():
    return boto3.client(
        "s3",
        endpoint_url=S3_ENDPOINT,
        aws_access_key_id="test",
        aws_secret_access_key="test",
        region_name="us-east-1",
    )


def ensure_buckets(s3) -> None:
    for name in BUCKETS:
        try:
            s3.create_bucket(Bucket=name)
            log.info("created bucket %s", name)
        except s3.exceptions.BucketAlreadyOwnedByYou:
            pass
    s3.put_bucket_versioning(
        Bucket="fpms-heritage",
        VersioningConfiguration={"Status": "Enabled"},
    )


def archive_key(thing: str, event_type: str, ts: datetime, event_id: str) -> str:
    return (
        f"events/thing={thing}/type={event_type}/"
        f"year={ts.year:04d}/month={ts.month:02d}/day={ts.day:02d}/"
        f"{ts.strftime('%H%M%S')}-{event_id}.json"
    )


def archive_and_alert(s3, topic: str, payload: dict) -> None:
    parts = topic.split("/")
    if len(parts) < 4 or parts[0] != "fpms" or parts[2] != "events":
        log.warning("skipping unmatched topic: %s", topic)
        return
    thing, event_type = parts[1], parts[3]

    ts_raw = payload.get("timestamp")
    try:
        ts = (
            datetime.fromisoformat(ts_raw.replace("Z", "+00:00"))
            if ts_raw
            else datetime.now(timezone.utc)
        )
    except (AttributeError, ValueError):
        ts = datetime.now(timezone.utc)
    event_id = payload.get("event_id", f"evt-{int(ts.timestamp() * 1000)}")

    key = archive_key(thing, event_type, ts, event_id)
    body = json.dumps({"topic": topic, "payload": payload}, sort_keys=True).encode("utf-8")

    s3.put_object(
        Bucket=ARCHIVE_BUCKET,
        Key=key,
        Body=body,
        ContentType="application/json",
        Metadata={"thing": thing, "event-type": event_type},
    )
    log.info("archived s3://%s/%s (%d bytes)", ARCHIVE_BUCKET, key, len(body))

    if event_type == "fire-detected":
        loc = payload.get("location", {}) or {}
        log.warning(
            "ALERT [%s] fire-detected severity=%s at lat=%s lon=%s (archive: %s)",
            thing,
            payload.get("severity", "unknown"),
            loc.get("lat"),
            loc.get("lon"),
            key,
        )


def main() -> int:
    moto = start_moto()
    time.sleep(0.5)
    s3 = make_s3_client()
    ensure_buckets(s3)

    client = mqtt.Client(client_id="fpms-local-worker", clean_session=True)
    client.on_connect = lambda *_: log.info(
        "connected to mqtt://%s:%s", MQTT_HOST, MQTT_PORT
    )
    client.on_disconnect = lambda *_: log.warning("mqtt disconnected — paho will retry")

    def on_message(_c, _u, msg: mqtt.MQTTMessage) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            log.error("dropping non-JSON message on %s", msg.topic)
            return
        archive_and_alert(s3, msg.topic, payload)

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.subscribe("fpms/+/events/#", qos=1)
    log.info("subscribed to fpms/+/events/# — ctrl-c to stop")

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        log.info("shutting down")
    finally:
        client.disconnect()
        moto.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
