"""Local Thing registry backed by SQLite.

LocalStack Community doesn't ship the AWS IoT Core control plane (Pro-only),
but Mosquitto IS a real MQTT broker doing exactly what IoT Core does for the
data plane. This module fills the control-plane gap: register/list Things,
attach policies, store certs — locally, with the same JSON shape boto3 would
return, so callers can treat it as a drop-in.

The registry auto-graduates to real IoT Core when the underlying boto3 call
succeeds (real AWS or LocalStack Pro). See `aws.py` for the wiring.
"""
from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any

DB_PATH = Path.home() / ".fpms" / "iot-registry.sqlite"
DB_PATH.parent.mkdir(parents=True, exist_ok=True)

_lock = threading.Lock()


def _conn() -> sqlite3.Connection:
    c = sqlite3.connect(DB_PATH, isolation_level=None, check_same_thread=False)
    c.execute("PRAGMA journal_mode=WAL")
    c.executescript("""
        CREATE TABLE IF NOT EXISTS things (
            name TEXT PRIMARY KEY,
            arn TEXT NOT NULL,
            attributes TEXT NOT NULL,
            created_at REAL NOT NULL
        );
        CREATE TABLE IF NOT EXISTS certificates (
            cert_id TEXT PRIMARY KEY,
            thing_name TEXT NOT NULL,
            cert_pem TEXT NOT NULL,
            private_key TEXT NOT NULL,
            created_at REAL NOT NULL,
            FOREIGN KEY(thing_name) REFERENCES things(name)
        );
    """)
    return c


def _self_signed(thing_name: str) -> tuple[str, str, str]:
    """Small self-signed X.509 pair. Good enough for LAN dev; real deploy uses
    the AWS-issued certs from boto3 create_keys_and_certificate."""
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import NameOID
        from datetime import datetime, timedelta, timezone
    except ImportError:
        # Cryptography isn't a hard dep. Fall back to a marker string.
        marker = f"-----BEGIN CERTIFICATE-----\nlocal-dev-cert-{thing_name}-{uuid.uuid4().hex}\n-----END CERTIFICATE-----\n"
        return marker, marker.replace("CERTIFICATE", "PRIVATE KEY"), uuid.uuid4().hex

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, thing_name),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "FPMS Local"),
    ])
    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject).issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now).not_valid_after(now + timedelta(days=365 * 3))
        .sign(key, hashes.SHA256())
    )
    cert_pem = cert.public_bytes(serialization.Encoding.PEM).decode()
    key_pem = key.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption(),
    ).decode()
    cert_id = cert.fingerprint(hashes.SHA256()).hex()[:40]
    return cert_pem, key_pem, cert_id


def create_thing(name: str, attributes: dict[str, str] | None = None) -> dict[str, Any]:
    with _lock, _conn() as c:
        row = c.execute("SELECT arn, attributes FROM things WHERE name=?", (name,)).fetchone()
        if row:
            return {"thingName": name, "thingArn": row[0], "attributes": json.loads(row[1])}
        arn = f"arn:aws:iot:local:000000000000:thing/{name}"
        attrs = attributes or {}
        c.execute(
            "INSERT INTO things(name, arn, attributes, created_at) VALUES (?,?,?,?)",
            (name, arn, json.dumps(attrs), time.time()),
        )
        return {"thingName": name, "thingArn": arn, "attributes": attrs}


def list_things() -> list[dict[str, Any]]:
    with _lock, _conn() as c:
        rows = c.execute("SELECT name, arn, attributes FROM things ORDER BY created_at DESC").fetchall()
    return [
        {"name": r[0], "arn": r[1], "attributes": json.loads(r[2])}
        for r in rows
    ]


def delete_thing(name: str) -> bool:
    """Remove a Thing and its certificates. Returns True if one was deleted.

    Needed because the AWS verification flow creates a throwaway probe Thing on
    every run. Without cleanup those accumulate forever and clutter the real
    rover list on the AWS tab.
    """
    with _lock, _conn() as c:
        c.execute("DELETE FROM certificates WHERE thing_name=?", (name,))
        cur = c.execute("DELETE FROM things WHERE name=?", (name,))
        return cur.rowcount > 0


def purge_probe_things(prefix: str = "fpms-probe-") -> int:
    """Drop leftover verification probes. Returns how many were removed."""
    with _lock, _conn() as c:
        rows = c.execute("SELECT name FROM things WHERE name LIKE ?", (prefix + "%",)).fetchall()
        for (n,) in rows:
            c.execute("DELETE FROM certificates WHERE thing_name=?", (n,))
        c.execute("DELETE FROM things WHERE name LIKE ?", (prefix + "%",))
        return len(rows)


def create_certificate(thing_name: str) -> dict[str, Any]:
    cert_pem, key_pem, cert_id = _self_signed(thing_name)
    with _lock, _conn() as c:
        c.execute(
            "INSERT INTO certificates(cert_id, thing_name, cert_pem, private_key, created_at) "
            "VALUES (?,?,?,?,?)",
            (cert_id, thing_name, cert_pem, key_pem, time.time()),
        )
    return {
        "certificateId": cert_id,
        "certificateArn": f"arn:aws:iot:local:000000000000:cert/{cert_id}",
        "certificatePem": cert_pem,
        "keyPair": {"PrivateKey": key_pem, "PublicKey": ""},
    }


def local_iot_endpoint(mqtt_host: str, mqtt_port: int) -> str:
    return f"{mqtt_host}:{mqtt_port}"
