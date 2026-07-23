# FPMS S3 bucket layout

Three buckets, each with a single clear purpose. Keys use Hive-style
partitioning (`key=value/`) wherever we expect to query by that key later
(Athena, or just human `aws s3 ls` grep).

## `fpms-archive` — every event, forever

Written by `event_router` Lambda on every message to `fpms/+/events/#`.

```
fpms-archive/
  events/
    thing=rover1/
      type=fire-detected/
        year=2026/month=07/day=23/
          123045-evt-abc123.json
      type=heritage-documented/
        year=2026/month=07/day=23/
          131200-evt-def456.json
    thing=rover2/
      type=risk-observation/
        year=2026/month=07/day=23/
          140015-evt-ghi789.json
    thing=station/
      type=dispense-refill/
        year=2026/month=07/day=23/
          150500-evt-jkl012.json
```

- Filename format: `HHMMSS-<event_id>.json` (UTC).
- Same `event_id` → same key → idempotent (rover can safely retry).
- Retention: none (permanent). Add a lifecycle rule if size ever matters.

## `fpms-heritage` — versioned marker record

Photos and descriptions of every cultural heritage marker the rover
documents. **Versioning is enabled** so an accidental overwrite can be
undone. This is the record we promised communities we'd keep.

```
fpms-heritage/
  markers/
    <marker-id>/
      metadata.json         # coords, description, first-seen timestamp
      photos/
        2026-07-23T14-05Z.jpg
        2026-08-01T09-18Z.jpg
```

- Never delete objects from this bucket in scripts. Deletion is a human
  decision after community consultation.
- CORS may need to be configured for the dashboard to fetch photos —
  add when the dashboard is deployed.

## `fpms-media` — dashboard static assets

Thumbnails, generated map tiles, static HTML/JS for the public
dashboard. Fully replaceable — nothing here is a system-of-record.

```
fpms-media/
  thumbnails/
  tiles/
  static/
```

## Local vs real AWS

Everything above works identically against LocalStack and real S3 — the
difference is just the endpoint URL. Set `AWS_ENDPOINT_URL=http://localhost:4566`
for local, unset it for production.
