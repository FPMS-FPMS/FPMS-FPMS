"""IoT rule bridge — subscribes to Mosquitto, invokes LocalStack Lambdas.

In production AWS, IoT Rules on ``fpms/+/events/#`` would call the Lambda for
us. LocalStack Community edition does not include the IoT Rules engine, so we
run this tiny bridge instead. The rover code stays identical either way.

Rule table below is intentionally small; add more rules as you grow the topic
tree. Everything is fail-loud — if Lambda invocation raises, we log and keep
serving (one bad message must not break the pipeline).
"""

from __future__ import annotations

import json
import logging
import os
import sys
from dataclasses import dataclass
from typing import Callable

import boto3
import paho.mqtt.client as mqtt

MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
AWS_ENDPOINT_URL = os.environ.get("AWS_ENDPOINT_URL", "http://localhost:4566")
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO")

logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s %(levelname)s [bridge] %(message)s",
)
log = logging.getLogger("fpms.bridge")


@dataclass(frozen=True)
class Rule:
    topic_filter: str          # MQTT wildcard filter, e.g. "fpms/+/events/#"
    lambda_name: str           # Target Lambda function name
    invocation: str = "Event"  # "Event" (async) or "RequestResponse" (sync)


RULES: list[Rule] = [
    Rule(topic_filter="fpms/+/events/#", lambda_name="fpms-event-router", invocation="Event"),
]


def _matches(rule_filter: str, topic: str) -> bool:
    """MQTT topic wildcard match (+ is single level, # is multi and only trailing)."""
    filter_parts = rule_filter.split("/")
    topic_parts = topic.split("/")
    for i, fp in enumerate(filter_parts):
        if fp == "#":
            return i == len(filter_parts) - 1
        if i >= len(topic_parts):
            return False
        if fp == "+":
            continue
        if fp != topic_parts[i]:
            return False
    return len(filter_parts) == len(topic_parts)


def _make_dispatcher(lambda_client) -> Callable[[mqtt.Client, object, mqtt.MQTTMessage], None]:
    def on_message(_client, _userdata, msg: mqtt.MQTTMessage) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            log.error("dropping non-JSON message on %s: %r", msg.topic, msg.payload[:80])
            return

        event = {"topic": msg.topic, "payload": payload}
        wire = json.dumps(event).encode("utf-8")

        for rule in RULES:
            if not _matches(rule.topic_filter, msg.topic):
                continue
            try:
                lambda_client.invoke(
                    FunctionName=rule.lambda_name,
                    InvocationType=rule.invocation,
                    Payload=wire,
                )
                log.info("routed %s -> %s", msg.topic, rule.lambda_name)
            except Exception:
                log.exception("failed to invoke %s for %s", rule.lambda_name, msg.topic)

    return on_message


def main() -> int:
    lambda_client = boto3.client("lambda", endpoint_url=AWS_ENDPOINT_URL)

    client = mqtt.Client(client_id="fpms-rule-bridge", clean_session=False)
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    def _on_connect(_c, _u, _f, rc):
        if rc != 0:
            log.error("mqtt connect failed rc=%s", rc)
            return
        log.info("connected to mqtt://%s:%s", MQTT_HOST, MQTT_PORT)
        for rule in RULES:
            client.subscribe(rule.topic_filter, qos=1)
            log.info("subscribed to %s", rule.topic_filter)

    client.on_connect = _on_connect
    client.on_disconnect = lambda *_: log.warning("mqtt disconnected — reconnecting")
    client.on_message = _make_dispatcher(lambda_client)

    client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    try:
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        log.info("shutting down")
        client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
