# FPMS MQTT topic taxonomy

Every MQTT topic FPMS uses starts with `fpms/`. Within that root, topics
follow one shape:

```
fpms/<thing>/<channel>/<subtype>
```

- **`<thing>`** — the publishing device. Currently `rover1`, `rover2`, or
  `station`. Add new things by registering them via
  `cloud/iot-core/register-thing.sh` — never invent new thing names ad hoc.
- **`<channel>`** — the intent of the message.
- **`<subtype>`** — a specific message shape within the channel.

## Channels

| Channel     | Purpose                                                  | Retention |
|-------------|----------------------------------------------------------|-----------|
| `events`    | Discrete, archivable happenings. **Written to S3.**      | forever   |
| `telemetry` | High-rate state (pose, battery). Dashboard live view only. | ephemeral |
| `commands`  | Cloud → thing. Configuration pushes, manual overrides.   | ephemeral |
| `shadow`    | Reserved for future device-shadow / desired-state sync.  | —         |

## Event subtypes (subject to S3 archive)

| Topic                                    | When it fires                                              |
|------------------------------------------|-------------------------------------------------------------|
| `fpms/rover1/events/fire-detected`       | Both cameras agree on a hotspot → severity + coordinates.   |
| `fpms/rover1/events/water-dispensed`     | Pump fired → volume, target coords, before/after thermal.   |
| `fpms/rover1/events/heritage-documented` | Marker photographed → GPS, description, photo S3 key.       |
| `fpms/rover1/events/mission-complete`    | Return-to-dock finished → duration, distance, events count. |
| `fpms/rover2/events/risk-observation`    | Vegetation/humidity sample → risk score + coordinates.      |
| `fpms/station/events/dispense-refill`    | Pump refill sequence finished → volume delivered.           |

Both rovers share the same event schema (`event.schema.json`). New event
subtypes only need a topic — the rule bridge already routes
`fpms/+/events/#` to the archiver Lambda.

## Telemetry subtypes (not archived)

| Topic                                    | Rate    |
|------------------------------------------|---------|
| `fpms/rover1/telemetry/pose`             | 5 Hz    |
| `fpms/rover1/telemetry/battery`          | 0.2 Hz  |
| `fpms/rover1/telemetry/health`           | 0.1 Hz  |
| `fpms/station/telemetry/water-level`     | 0.1 Hz  |

## What we don't publish

- **Raw video** — never leaves the rover. Only frame stills tied to events do.
- **Continuous LiDAR scans** — dashboard subscribes on the LAN instead.
- **Model outputs at inference cadence** — only cross-validated agreements.

## Design notes

- Every event payload MUST include `event_id`, `timestamp`, `thing`. See
  `cloud/lambda/event_router/event.schema.json`.
- Rovers use QoS 1 for events (at-least-once). The router handles duplicates
  by S3 key idempotency (same `event_id` overwrites same object).
- Telemetry uses QoS 0 (fire-and-forget) — losing one frame doesn't matter.
- Prefer flat topic depth (four segments). Deeper hierarchies make wildcard
  filters harder to reason about later.
