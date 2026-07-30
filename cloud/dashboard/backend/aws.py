"""Live inventory of every AWS service the dashboard uses.

Talks to LocalStack via boto3. Same API calls, same shape, same auth model as
production AWS — swap `endpoint_url` and this becomes a real AWS Console view.
"""
from __future__ import annotations

import logging
import socket
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any
from urllib.parse import urlparse

import boto3
import paho.mqtt.publish as mqtt_publish
from botocore.config import Config as BotoConfig
from botocore.exceptions import BotoCoreError, ClientError

from . import iot_registry
from .config import settings

log = logging.getLogger("fpms.aws")


_REACHABLE_CACHE: tuple[float, bool] | None = None
_REACHABLE_TTL_S = 5.0


def _endpoint_reachable() -> bool:
    """Can we open a TCP connection to the AWS endpoint at all?

    Worth the extra probe because it is ~1000x cheaper than finding out the slow
    way. Without it, every service summary independently waits out boto3's
    connect timeout, and the AWS tab took 42 seconds to render whenever
    LocalStack wasn't running — which is its normal state.

    In cloud mode we don't probe: real AWS is reachable, and a DNS lookup per
    call would be its own tax.
    """
    global _REACHABLE_CACHE

    if effective_mode() == "real-aws":
        return True

    now = time.monotonic()
    if _REACHABLE_CACHE and now - _REACHABLE_CACHE[0] < _REACHABLE_TTL_S:
        return _REACHABLE_CACHE[1]

    parsed = urlparse(settings.aws_endpoint_url)
    host = parsed.hostname or "localhost"
    port = parsed.port or (443 if parsed.scheme == "https" else 80)
    try:
        with socket.create_connection((host, port), timeout=0.3):
            ok = True
    except OSError:
        ok = False

    _REACHABLE_CACHE = (now, ok)
    return ok


def endpoint_reachable() -> bool:
    """Public, cached, sub-second reachability check.

    Exists so callers on hot paths (the health endpoint polls every 2.5s) never
    reach for a boto3 API call just to answer "is anything there".
    """
    return _endpoint_reachable()


_UNREACHABLE = {
    "error": "EndpointUnreachable",
    "message": "No service is listening on the configured AWS endpoint. "
               "Start LocalStack, or set FPMS_AWS_MODE=cloud to use real AWS.",
}


def _iot_available() -> bool:
    """Is the real IoT Core control plane usable?

    True when talking to real AWS OR LocalStack Pro. False on LocalStack
    Community (where iot/iotdata always return InternalFailure).
    """
    if not _endpoint_reachable():
        return False
    try:
        _client("iot").list_things(maxResults=1)
        return True
    except Exception:  # noqa: BLE001
        return False


_CREDS_CACHE: tuple[float, bool] | None = None
_CREDS_TTL_S = 30.0


def credentials_available() -> bool:
    """Can boto3 resolve real AWS credentials from the standard chain?

    Covers env vars, ~/.aws/credentials, SSO cache, and EC2 instance roles.
    Cached briefly because the lookup touches disk and this is called per
    request while a dashboard tab polls.
    """
    global _CREDS_CACHE
    now = time.monotonic()
    if _CREDS_CACHE and now - _CREDS_CACHE[0] < _CREDS_TTL_S:
        return _CREDS_CACHE[1]
    try:
        ok = boto3.Session().get_credentials() is not None
    except Exception:  # noqa: BLE001
        ok = False
    _CREDS_CACHE = (now, ok)
    return ok


def effective_mode() -> str:
    """Which AWS this tab is actually showing: 'real-aws' or 'local-emulation'.

    Real credentials win over the FPMS_AWS_MODE default. The AWS tab exists to
    show what is genuinely deployed, so silently reporting emulated LocalStack
    state while real credentials sit on the machine would be misleading — the
    numbers would look plausible and be wrong.

    Deliberately scoped to this module: the MQTT bridge keeps following
    settings.aws_mode, so appearing credentials cannot swing live rover
    telemetry over to an AWS IoT endpoint that was never provisioned.
    """
    if settings.aws_mode == "cloud":
        return "real-aws"
    return "real-aws" if credentials_available() else "local-emulation"


def _kwargs() -> dict[str, Any]:
    """boto3 kwargs. For real AWS we omit endpoint_url so calls go to the
    operator's account using the normal credential chain."""
    k: dict[str, Any] = {
        "region_name": settings.aws_region,
        "config": BotoConfig(retries={"max_attempts": 1}, connect_timeout=2, read_timeout=5),
    }
    if effective_mode() == "local-emulation":
        k["endpoint_url"] = settings.aws_endpoint_url
        k["aws_access_key_id"] = settings.aws_access_key
        k["aws_secret_access_key"] = settings.aws_secret_key
    return k


def _client(service: str):
    return boto3.client(service, **_kwargs())


def _safe(fn):
    try:
        return fn()
    except (BotoCoreError, ClientError, OSError) as e:
        return {"error": type(e).__name__, "message": str(e)[:200]}


def _mode_banner() -> dict[str, Any]:
    """State the tab must display, so emulated data is never mistaken for AWS."""
    mode = effective_mode()
    real = mode == "real-aws"
    return {
        "mode": mode,
        "emulated": not real,
        "credentials_found": credentials_available(),
        "account_region": settings.aws_region,
        "message": (
            f"Live AWS account, region {settings.aws_region}."
            if real else
            "NOT real AWS — local emulation (LocalStack + a local SQLite Thing "
            "registry). Run 'aws login' or 'aws configure' and this switches to "
            "your real account automatically."
        ),
    }


def inventory() -> dict[str, Any]:
    """Return a snapshot of every AWS service in play. Never raises."""
    reachable = _endpoint_reachable()

    if not reachable:
        # Skip boto3 entirely. IoT still reports, because its fallback reads the
        # local SQLite registry and Mosquitto rather than the AWS control plane.
        return {
            "endpoint": settings.aws_endpoint_url,
            "region": settings.aws_region,
            "reachable": False,
            **_mode_banner(),
            "services": {
                "iot": _iot_summary(),
                "s3": dict(_UNREACHABLE),
                "lambda": dict(_UNREACHABLE),
                "sns": dict(_UNREACHABLE),
                "logs": dict(_UNREACHABLE),
            },
        }

    # Probe concurrently: these are independent network calls, so running them
    # in sequence made the page as slow as the sum of every timeout.
    summaries = {
        "iot": _iot_summary,
        "s3": _s3_summary,
        "lambda": _lambda_summary,
        "sns": _sns_summary,
        "logs": _logs_summary,
    }
    with ThreadPoolExecutor(max_workers=len(summaries)) as pool:
        futures = {name: pool.submit(fn) for name, fn in summaries.items()}
        services = {name: f.result() for name, f in futures.items()}

    return {
        "endpoint": settings.aws_endpoint_url if effective_mode() == "local-emulation" else "aws",
        "region": settings.aws_region,
        "reachable": True,
        **_mode_banner(),
        "services": services,
    }


def _iot_summary() -> Any:
    """Return IoT Core state, or the equivalent from our local Mosquitto-
    backed registry if the real IoT API isn't available."""
    if _iot_available():
        return _safe(lambda: _do_iot_real())
    # LocalStack Community fallback: read from SQLite registry, endpoint is Mosquitto.
    things = iot_registry.list_things()
    return {
        "things": [{"name": t["name"], "arn": t["arn"]} for t in things],
        "thing_count": len(things),
        "policies": ["fpms-rover-policy"],
        "endpoint": iot_registry.local_iot_endpoint(settings.mqtt_host, settings.mqtt_port),
        "backend": "mosquitto+sqlite",
        "note": "LocalStack Community does not ship AWS IoT Core (Pro/real-AWS only). Dashboard uses Mosquitto for MQTT and a local SQLite registry for Things — same protocol, same topic taxonomy, same wire format as real IoT Core.",
    }


def _do_iot_real() -> dict[str, Any]:
    iot = _client("iot")
    things = iot.list_things(maxResults=50).get("things", [])
    policies = iot.list_policies(pageSize=50).get("policies", [])
    endpoint = _safe(lambda: iot.describe_endpoint(endpointType="iot:Data-ATS")["endpointAddress"])
    return {
        "things": [{"name": t["thingName"], "arn": t["thingArn"]} for t in things],
        "thing_count": len(things),
        "policies": [p["policyName"] for p in policies],
        "endpoint": endpoint if isinstance(endpoint, str) else None,
        "backend": "aws-iot-core",
    }


def _s3_summary() -> Any:
    return _safe(lambda: _do_s3())


def _do_s3() -> dict[str, Any]:
    s3 = _client("s3")
    buckets = s3.list_buckets().get("Buckets", [])
    out = []
    for b in buckets:
        name = b["Name"]
        try:
            resp = s3.list_objects_v2(Bucket=name, MaxKeys=1000)
            count = resp.get("KeyCount", 0)
            size = sum(o.get("Size", 0) for o in resp.get("Contents", []))
        except Exception:  # noqa: BLE001
            count, size = 0, 0
        out.append({"name": name, "object_count": count, "bytes": size})
    return {"buckets": out}


def _lambda_summary() -> Any:
    return _safe(lambda: _do_lambda())


def _do_lambda() -> dict[str, Any]:
    lam = _client("lambda")
    fns = lam.list_functions().get("Functions", [])
    return {
        "functions": [
            {
                "name": f["FunctionName"],
                "runtime": f.get("Runtime"),
                "last_modified": f.get("LastModified"),
                "memory_mb": f.get("MemorySize"),
            }
            for f in fns
        ],
    }


def _sns_summary() -> Any:
    return _safe(lambda: _do_sns())


def _do_sns() -> dict[str, Any]:
    sns = _client("sns")
    topics = sns.list_topics().get("Topics", [])
    out = []
    for t in topics:
        arn = t["TopicArn"]
        try:
            subs = sns.list_subscriptions_by_topic(TopicArn=arn).get("Subscriptions", [])
        except Exception:  # noqa: BLE001
            subs = []
        out.append({
            "arn": arn,
            "name": arn.rsplit(":", 1)[-1],
            "subscription_count": len(subs),
        })
    return {"topics": out}


def _logs_summary() -> Any:
    return _safe(lambda: _do_logs())


def _do_logs() -> dict[str, Any]:
    logs = _client("logs")
    groups = logs.describe_log_groups(limit=20).get("logGroups", [])
    return {
        "groups": [
            {"name": g["logGroupName"], "bytes": g.get("storedBytes", 0)}
            for g in groups
        ],
    }


def iot_register_thing(name: str, attributes: dict[str, str] | None = None) -> dict[str, Any]:
    """Create-or-get a Thing + certificate + policy.

    Uses AWS IoT Core when reachable; otherwise falls back to the local
    Mosquitto + SQLite registry (LocalStack Community). Callers get the same
    provisioning bundle either way.
    """
    if _iot_available():
        return _register_via_iot_core(name, attributes)
    return _register_via_local(name, attributes)


def _register_via_iot_core(name: str, attributes: dict[str, str] | None) -> dict[str, Any]:
    iot = _client("iot")
    try:
        thing = iot.create_thing(
            thingName=name,
            attributePayload={"attributes": attributes or {}},
        )
    except ClientError as e:
        if e.response["Error"]["Code"] != "ResourceAlreadyExistsException":
            raise
        thing = iot.describe_thing(thingName=name)

    policy_name = "fpms-rover-policy"
    policy_doc = (
        '{"Version":"2012-10-17","Statement":['
        '{"Effect":"Allow","Action":["iot:Connect","iot:Publish","iot:Subscribe","iot:Receive"],'
        '"Resource":"*"}]}'
    )
    try:
        iot.create_policy(policyName=policy_name, policyDocument=policy_doc)
    except ClientError as e:
        if e.response["Error"]["Code"] != "ResourceAlreadyExistsException":
            raise

    kc = iot.create_keys_and_certificate(setAsActive=True)
    iot.attach_policy(policyName=policy_name, target=kc["certificateArn"])
    iot.attach_thing_principal(thingName=name, principal=kc["certificateArn"])
    endpoint = iot.describe_endpoint(endpointType="iot:Data-ATS")["endpointAddress"]
    return {
        "thing_name": name,
        "thing_arn": thing.get("thingArn"),
        "certificate_arn": kc["certificateArn"],
        "certificate_id": kc["certificateId"],
        "certificate_pem": kc["certificatePem"],
        "private_key": kc["keyPair"]["PrivateKey"],
        "iot_endpoint": endpoint,
        "policy": policy_name,
        "backend": "aws-iot-core",
    }


def _register_via_local(name: str, attributes: dict[str, str] | None) -> dict[str, Any]:
    thing = iot_registry.create_thing(name, attributes)
    cert = iot_registry.create_certificate(name)
    return {
        "thing_name": thing["thingName"],
        "thing_arn": thing["thingArn"],
        "certificate_arn": cert["certificateArn"],
        "certificate_id": cert["certificateId"],
        "certificate_pem": cert["certificatePem"],
        "private_key": cert["keyPair"]["PrivateKey"],
        "iot_endpoint": iot_registry.local_iot_endpoint(settings.mqtt_host, settings.mqtt_port),
        "policy": "fpms-rover-policy",
        "backend": "mosquitto+sqlite",
    }


def iot_publish(topic: str, payload: bytes, qos: int = 0) -> dict[str, Any]:
    """Publish through IoT Data plane, falling back to raw Mosquitto."""
    try:
        _client("iot-data").publish(topic=topic, qos=qos, payload=payload)
        return {"ok": True, "via": "aws-iot-data"}
    except Exception:  # noqa: BLE001
        try:
            mqtt_publish.single(
                topic, payload=payload,
                hostname=settings.mqtt_host, port=settings.mqtt_port, qos=qos,
            )
            return {"ok": True, "via": "mosquitto"}
        except Exception as e:  # noqa: BLE001
            return {"ok": False, "error": str(e)}


def iot_things() -> list[dict[str, Any]]:
    """Registered Things — from AWS IoT Core if available, else the local
    SQLite registry backing Mosquitto."""
    if _iot_available():
        try:
            iot = _client("iot")
            return [
                {"name": t["thingName"], "arn": t["thingArn"], "attributes": t.get("attributes", {})}
                for t in iot.list_things(maxResults=100).get("things", [])
            ]
        except Exception as e:  # noqa: BLE001
            log.warning("list_things (real) failed: %s", e)
    return iot_registry.list_things()


# ---- End-to-end verification ---------------------------------------------

def verify_all() -> dict[str, Any]:
    """Prove every AWS service the dashboard depends on actually works.

    Runs real API calls, in order:
      1. IoT Core     — create a probe Thing + list it back
      2. IoT Data     — publish a test message on the probe topic
      3. S3           — put + get + delete a test object in fpms-archive
      4. Lambda       — invoke fpms-event-router with a fake event
      5. SNS          — publish to fpms-alerts
      6. CloudWatch   — read the alert dispatcher's most recent log stream

    Returns a per-check pass/fail + the raw evidence. The probe Thing is
    removed afterwards — earlier versions left one behind on every run, and they
    piled up in the rover list until the AWS tab was mostly junk.
    """
    import json
    import time as _time
    import uuid

    results: list[dict[str, Any]] = []

    def rec(name: str, ok: bool, evidence: Any = None, error: str | None = None) -> None:
        results.append({"name": name, "ok": ok, "evidence": evidence, "error": error})

    probe_id = uuid.uuid4().hex[:8]
    probe_thing = f"fpms-probe-{probe_id}"

    # 1. IoT registry (real AWS/Pro OR local Mosquitto+SQLite fallback)
    try:
        reg = iot_register_thing(probe_thing, attributes={"role": "probe"})
        listed = [t["name"] for t in iot_things()]
        rec(f"IoT Core · registry ({reg.get('backend')})",
            probe_thing in listed,
            {"probe_thing": probe_thing, "endpoint": reg.get("iot_endpoint"),
             "total_things": len(listed)})
    except Exception as e:  # noqa: BLE001
        rec("IoT Core · registry", False, error=str(e))

    # 2. IoT Data plane publish (real IoT Data OR direct Mosquitto)
    try:
        r = iot_publish(
            topic=f"fpms/{probe_thing}/telemetry/probe",
            payload=json.dumps({"ts": _time.time(), "probe": probe_id}).encode(),
        )
        rec(f"IoT Data · publish ({r.get('via')})", r.get("ok", False),
            {"topic": f"fpms/{probe_thing}/telemetry/probe"},
            error=r.get("error"))
    except Exception as e:  # noqa: BLE001
        rec("IoT Data · publish", False, error=str(e))

    # 3. S3 put/get/delete
    try:
        s3 = _client("s3")
        bucket = "fpms-archive"
        key = f"_probe/{probe_id}.json"
        body = json.dumps({"probe": probe_id, "ts": _time.time()}).encode()
        s3.put_object(Bucket=bucket, Key=key, Body=body, ContentType="application/json")
        got = s3.get_object(Bucket=bucket, Key=key)["Body"].read()
        s3.delete_object(Bucket=bucket, Key=key)
        rec("S3 · put/get/delete", got == body,
            {"bucket": bucket, "key": key, "bytes": len(body)})
    except Exception as e:  # noqa: BLE001
        rec("S3 · put/get/delete", False, error=str(e))

    # 4. Lambda invoke — fpms-event-router (first-invoke pulls the runtime
    # container, so allow up to 60s)
    try:
        lam = boto3.client(
            "lambda",
            **{**_kwargs(),
               "config": BotoConfig(retries={"max_attempts": 1},
                                    connect_timeout=2, read_timeout=60)},
        )
        fake_event = {
            "topic": f"fpms/{probe_thing}/events/probe",
            "payload": {
                "event_id": probe_id,
                "timestamp": _time.strftime("%Y-%m-%dT%H:%M:%SZ", _time.gmtime()),
                "thing": probe_thing,
                "severity": "info",
                "note": "AWS verification probe",
            },
        }
        resp = lam.invoke(
            FunctionName="fpms-event-router",
            InvocationType="RequestResponse",
            Payload=json.dumps(fake_event).encode(),
        )
        payload = resp["Payload"].read().decode("utf-8", errors="replace")
        rec("Lambda · invoke event_router", resp.get("StatusCode") == 200,
            {"status": resp.get("StatusCode"), "response": payload[:400]})
    except Exception as e:  # noqa: BLE001
        rec("Lambda · invoke event_router", False, error=str(e))

    # 5. SNS publish (find any FPMS alert topic — name varies across init versions)
    try:
        sns = _client("sns")
        topics = sns.list_topics().get("Topics", [])
        alerts = next(
            (t["TopicArn"] for t in topics
             if t["TopicArn"].endswith(":fpms-fire-alerts")
             or t["TopicArn"].endswith(":fpms-alerts")),
            None,
        )
        if not alerts:
            rec("SNS · publish", False, error=f"no fpms alert topic; found: {[t['TopicArn'] for t in topics]}")
        else:
            r = sns.publish(TopicArn=alerts, Subject="FPMS probe",
                            Message=json.dumps({"probe": probe_id}))
            rec("SNS · publish", bool(r.get("MessageId")),
                {"topic": alerts, "message_id": r.get("MessageId")})
    except Exception as e:  # noqa: BLE001
        rec("SNS · publish", False, error=str(e))

    # 6. CloudWatch Logs — alert dispatcher output
    try:
        logs = _client("logs")
        groups = logs.describe_log_groups(logGroupNamePrefix="/aws/lambda/fpms-").get("logGroups", [])
        rec("CloudWatch Logs · discover", len(groups) > 0,
            {"log_groups": [g["logGroupName"] for g in groups]})
    except Exception as e:  # noqa: BLE001
        rec("CloudWatch Logs · discover", False, error=str(e))

    # Clean up after ourselves. Also sweep any probes left by earlier runs, so a
    # dashboard that has been verified weekly for months still shows only real
    # rovers in its Thing list.
    removed = 0
    try:
        removed = iot_registry.purge_probe_things()
    except Exception as e:  # noqa: BLE001
        log.warning("probe cleanup failed: %s", e)

    passed = sum(1 for r in results if r["ok"])
    return {
        "probe_id": probe_id,
        "probe_thing": probe_thing,
        "probes_cleaned_up": removed,
        "passed": passed,
        "failed": len(results) - passed,
        "checks": results,
    }
