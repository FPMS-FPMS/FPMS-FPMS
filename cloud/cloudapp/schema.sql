-- FPMS telemetry archive.
--
-- Every reading and event is appended here, so the dashboard can show history
-- rather than only the live value the Durable Object holds.

CREATE TABLE IF NOT EXISTS readings (
  id       INTEGER PRIMARY KEY AUTOINCREMENT,
  ts       REAL    NOT NULL,          -- unix seconds, as sent by the rover
  thing    TEXT    NOT NULL,          -- rover1, rover2, ...
  subtype  TEXT    NOT NULL,          -- pose | lidar | camera | thermal | online | ...
  kind     TEXT    NOT NULL,          -- telemetry | events
  data     TEXT    NOT NULL           -- original JSON payload
);

-- The dashboard almost always asks "latest N for this rover/subtype", so index
-- for that access pattern rather than adding one index per column.
CREATE INDEX IF NOT EXISTS idx_readings_lookup ON readings (thing, subtype, ts DESC);

-- Events are queried on their own for the alert feed, and are far rarer than
-- telemetry, so they get a dedicated partial-ish index.
CREATE INDEX IF NOT EXISTS idx_readings_events ON readings (kind, ts DESC);

-- The scheduled agents (agents.js loadWindow) query "everything since <ts>"
-- across ALL things/subtypes/kinds — i.e. WHERE ts >= ? ORDER BY ts DESC.
-- Neither index above helps that query: both lead with a column (thing, kind)
-- that isn't part of this predicate, and SQLite can only use an index's prefix,
-- so it can't skip into either one on ts alone. Without an index that leads
-- with ts, this becomes a full table scan of `readings` on every cron tick —
-- LIMIT only caps rows returned, not rows the engine has to read to get there.
-- This index exists specifically to give that query a leading column it can
-- search on. It is NOT redundant with idx_readings_lookup/idx_readings_events
-- even though ts appears in both — do not remove it as a "duplicate".
CREATE INDEX IF NOT EXISTS idx_readings_ts ON readings (ts DESC);

-- Analysis reports produced by the scheduled agents.
CREATE TABLE IF NOT EXISTS reports (
  id        INTEGER PRIMARY KEY AUTOINCREMENT,
  ts        REAL    NOT NULL,
  severity  TEXT    NOT NULL,          -- ok | warning | critical
  summary   TEXT    NOT NULL,          -- one-line headline
  findings  TEXT    NOT NULL,          -- JSON array of per-agent findings
  emailed   INTEGER NOT NULL DEFAULT 0 -- 1 once an alert email went out
);

CREATE INDEX IF NOT EXISTS idx_reports_ts ON reports (ts DESC);

