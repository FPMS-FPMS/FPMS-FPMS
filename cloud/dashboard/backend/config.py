"""Runtime configuration for the FPMS dashboard backend.

Everything is env-driven so the same code runs against LocalStack, a real AWS
account, or the Docker-free `local-dev` variant. Defaults match the ports the
existing docker-compose stack exposes on localhost.
"""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Settings:
    # AWS mode — "local" uses LocalStack, "cloud" uses real AWS in your account.
    # Set FPMS_AWS_MODE=cloud plus normal AWS credentials to flip.
    aws_mode: str = os.getenv("FPMS_AWS_MODE", "local").lower()

    mqtt_host: str = os.getenv("FPMS_MQTT_HOST", "localhost")
    mqtt_port: int = int(os.getenv("FPMS_MQTT_PORT", "1883"))
    mqtt_client_id: str = os.getenv("FPMS_MQTT_CLIENT_ID", "fpms-dashboard")
    # Set when the broker requires auth (allow_anonymous false). The rovers use
    # the same account, so this is also what gets baked into a Pi's config.
    mqtt_username: str = os.getenv("FPMS_MQTT_USERNAME", "")
    mqtt_password: str = os.getenv("FPMS_MQTT_PASSWORD", "")
    # When on, requests arriving through the public tunnel lose the tabs that
    # can act on this machine (terminal, SSH, LAN scan, firewall). Viewing rover
    # data still works. Defaults ON: the dashboard is internet-reachable and a
    # guessed password should not equal a shell. Set FPMS_PUBLIC_SAFE_MODE=0
    # to disable.
    public_safe_mode: bool = os.getenv("FPMS_PUBLIC_SAFE_MODE", "1") != "0"

    # Which of the two FPMS apps this process is:
    #   "edge"  — runs on the operator's own machine. Full control of the
    #             rovers: terminal, SSH, LAN scan, provisioning.
    #   "cloud" — the public streaming deployment. Data only. Machine-control
    #             routes are refused for EVERY request, not just ones that look
    #             remote, because on a custom domain there is no tunnel
    #             hostname to detect and the host header can be spoofed.
    role: str = os.getenv("FPMS_ROLE", "edge").strip().lower()

    @property
    def is_cloud(self) -> bool:
        return self.role == "cloud"
    mqtt_tls: bool = os.getenv("FPMS_MQTT_TLS", "0") == "1"
    mqtt_ca_cert: str = os.getenv("FPMS_MQTT_CA_CERT", "")
    mqtt_client_cert: str = os.getenv("FPMS_MQTT_CLIENT_CERT", "")
    mqtt_client_key: str = os.getenv("FPMS_MQTT_CLIENT_KEY", "")

    # If FPMS_AWS_MODE=cloud, endpoint_url is left empty so boto3 uses real AWS.
    aws_endpoint_url: str = os.getenv(
        "FPMS_AWS_ENDPOINT",
        "" if os.getenv("FPMS_AWS_MODE", "local").lower() == "cloud" else "http://localhost:4566",
    )
    aws_region: str = os.getenv("AWS_DEFAULT_REGION", "us-east-1")
    aws_access_key: str = os.getenv("AWS_ACCESS_KEY_ID", "test")
    aws_secret_key: str = os.getenv("AWS_SECRET_ACCESS_KEY", "test")

    archive_bucket: str = os.getenv("FPMS_ARCHIVE_BUCKET", "fpms-archive")

    # Everything the browser needs to hit us from another laptop on the LAN.
    bind_host: str = os.getenv("FPMS_BIND_HOST", "0.0.0.0")
    bind_port: int = int(os.getenv("FPMS_BIND_PORT", "8000"))


settings = Settings()
