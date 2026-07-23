# FPMS local-dev — no Docker required

Same FPMS pipeline as `cloud/localstack/`, but runs entirely as native
processes so you don't need Docker installed. Best fit for Windows laptops
where Docker Desktop can't be installed (Home edition without WSL2, no
admin rights, etc.).

## What you get

- **Mock S3 + SNS** (Python `moto`) — same S3 API and endpoints as real AWS
- **Real MQTT broker** (Mosquitto for Windows) — standards-compliant, same
  wire protocol as AWS IoT Core
- **One Python worker** (`pipeline.py`) that ties them together and runs
  the same archive-to-S3 + alert-on-fire logic the Docker Lambdas run

Topic taxonomy, event schema, and S3 key layout are identical to the
Docker path — same rover code works against either.

## Windows setup

Run everything from PowerShell.

**1. Install Python 3.11+** — https://www.python.org/downloads/ (or Microsoft
Store). During install, tick "Add python.exe to PATH".

**2. Install Mosquitto for Windows** — https://mosquitto.org/download/ →
"Windows" → run the installer. It installs a Windows service that starts on
boot. Confirm it's running:

```powershell
Get-Service mosquitto
```

**3. Install the AWS CLI (v2)** —
https://awscli.amazonaws.com/AWSCLIV2.msi (needed only for the
`check-archive.ps1` helper; `pipeline.py` doesn't need it).

**4. Install Python dependencies** in a virtual environment:

```powershell
cd cloud\local-dev
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell blocks activation with an execution-policy error, run once:
```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```
then re-open PowerShell.

## Run it

**Terminal 1** — start the pipeline (mock S3 + MQTT subscriber):

```powershell
cd cloud\local-dev
.\.venv\Scripts\Activate.ps1
python pipeline.py
```

You should see:
```
moto (mock AWS) listening on http://127.0.0.1:4566
created bucket fpms-archive
connected to mqtt://localhost:1883
subscribed to fpms/+/events/# — ctrl-c to stop
```

**Terminal 2** — publish a test event:

```powershell
cd cloud\local-dev
.\publish-fire.ps1
```

Back in Terminal 1 you'll see:
```
INFO archived s3://fpms-archive/events/thing=rover1/type=fire-detected/...
WARNING ALERT [rover1] fire-detected severity=high at lat=43.6532 lon=-79.3832
```

**Optional** — inspect the archive from a third terminal:

```powershell
cd cloud\local-dev
.\check-archive.ps1 rover1 fire-detected
```

**Try heritage documentation:**
```powershell
.\publish-heritage.ps1
```

## Shut it down

Ctrl-C in Terminal 1. Mosquitto keeps running as a Windows service — that's
fine; if you want to stop it, `Stop-Service mosquitto` in an admin
PowerShell.

## What's different from the Docker path

| Piece                 | Docker path              | local-dev path              |
|-----------------------|--------------------------|-----------------------------|
| S3 / SNS emulation    | LocalStack container     | `moto` Python library       |
| Lambda functions      | Real Lambda runtime      | Called directly by pipeline.py |
| MQTT broker           | Mosquitto in container   | Native Mosquitto service    |
| IoT rule dispatch     | `iot-rule-bridge` container | Built into pipeline.py   |

The rover-side code and MQTT topic names don't change — you can develop
against either and switch later.

## When you'd want the Docker path instead

- You want to test real Lambda cold-start behavior
- You want to run multiple worker processes
- You want the `docker compose logs` inspection UX

For everything else, local-dev is faster to start and simpler to debug.
