"""FPMS alert dispatcher — turns SNS fire-alerts into human-visible notices.

Subscribed to the ``fpms-fire-alerts`` SNS topic by the init scripts. In
LocalStack we just log the alert (there is no real SMS/email); in production
this Lambda would push to SNS SMS/email subscriptions and to the dashboard's
WebSocket channel.

Kept intentionally small — one place to add real channels later.
"""

from __future__ import annotations

import json
import logging
import os

log = logging.getLogger()
log.setLevel(os.environ.get("LOG_LEVEL", "INFO"))


def handler(event, _context):
    dispatched = 0
    for record in event.get("Records", []):
        sns = record.get("Sns") or {}
        subject = sns.get("Subject", "(no subject)")
        try:
            body = json.loads(sns.get("Message", "{}"))
        except json.JSONDecodeError:
            # Non-JSON messages should never reach us — surface loudly.
            log.error("non-JSON SNS message: %r", sns.get("Message"))
            continue

        thing = body.get("thing", "unknown")
        severity = body.get("payload", {}).get("severity", "unknown")
        loc = body.get("payload", {}).get("location", {})
        log.warning(
            "ALERT [%s] %s — severity=%s at lat=%s lon=%s — archived at s3://%s",
            thing,
            subject,
            severity,
            loc.get("lat"),
            loc.get("lon"),
            body.get("s3_key"),
        )
        dispatched += 1
    return {"dispatched": dispatched}
