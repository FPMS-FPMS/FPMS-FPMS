"""FPMS event router — archives every rover event to S3.

Invoked by the IoT rule bridge (or, in real AWS, by an IoT Rule) whenever a
rover publishes to ``fpms/+/events/#``. Writes the raw payload to the archive
bucket under a partitioned key, then re-publishes fire-detected events onto
the alert SNS topic so the alert_dispatcher Lambda can fan them out.

Contract in ``event.schema.json``. Keep this file dependency-light so the
Lambda zip stays under 5 MB — boto3 is provided by the Lambda runtime.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime, timezone

import boto3

log = logging.getLogger()
log.setLevel(os.environ.get("LOG_LEVEL", "INFO"))

ARCHIVE_BUCKET = os.environ["ARCHIVE_BUCKET"]
ALERT_TOPIC_ARN = os.environ["ALERT_TOPIC_ARN"]
AWS_ENDPOINT_URL = os.environ.get("AWS_ENDPOINT_URL")  # LocalStack sets this; real AWS does not.

_s3 = boto3.client("s3", endpoint_url=AWS_ENDPOINT_URL)
_sns = boto3.client("sns", endpoint_url=AWS_ENDPOINT_URL)


def _archive_key(thing: str, event_type: str, ts: datetime, event_id: str) -> str:
    # Hive-style partitioning so Athena/downstream tools can prune by day.
    return (
        f"events/thing={thing}/type={event_type}/"
        f"year={ts.year:04d}/month={ts.month:02d}/day={ts.day:02d}/"
        f"{ts.strftime('%H%M%S')}-{event_id}.json"
    )


def handler(event, _context):
    # The bridge wraps each MQTT message as {"topic": "...", "payload": {...}}.
    topic = event["topic"]
    payload = event["payload"]

    parts = topic.split("/")
    if len(parts) < 4 or parts[0] != "fpms" or parts[2] != "events":
        # Topic didn't match fpms/<thing>/events/<type> — refuse rather than mis-archive.
        raise ValueError(f"unexpected topic shape: {topic!r}")
    thing, event_type = parts[1], parts[3]

    ts_raw = payload.get("timestamp")
    ts = datetime.fromisoformat(ts_raw.replace("Z", "+00:00")) if ts_raw else datetime.now(timezone.utc)
    event_id = payload.get("event_id", f"evt-{int(ts.timestamp() * 1000)}")

    key = _archive_key(thing, event_type, ts, event_id)
    body = json.dumps({"topic": topic, "payload": payload}, sort_keys=True).encode("utf-8")

    _s3.put_object(
        Bucket=ARCHIVE_BUCKET,
        Key=key,
        Body=body,
        ContentType="application/json",
        Metadata={"thing": thing, "event-type": event_type},
    )
    log.info("archived s3://%s/%s (%d bytes)", ARCHIVE_BUCKET, key, len(body))

    if event_type == "fire-detected":
        _sns.publish(
            TopicArn=ALERT_TOPIC_ARN,
            Subject=f"FPMS fire detected — {thing}",
            Message=json.dumps({"thing": thing, "s3_key": key, "payload": payload}),
            MessageAttributes={
                "thing": {"DataType": "String", "StringValue": thing},
                "severity": {"DataType": "String", "StringValue": payload.get("severity", "unknown")},
            },
        )
        log.info("fanned fire-detected to %s", ALERT_TOPIC_ARN)

    return {"status": "ok", "s3_key": key}
