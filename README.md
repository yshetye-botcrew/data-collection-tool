# dds-mcap-bag

A `rosbag`-style **record / play** pair for the in-house `message_manager`
Fast-DDS stack (the same stack the `foxglove-aggregator` uses). Capture live DDS
traffic to a single MCAP file and replay it later onto DDS — your consumers
(terrain mapping, SLAM, RViz/Foxglove) see it exactly as if it were live, with
**no changes to the consumer**.

- **`dds_recorder`** — subscribes to a configured/discovered set of DDS topics
  and writes every message to one **MCAP file** as raw **CDR** (the exact
  Fast-DDS wire payload). The bag is **lossless**: PointCloud2, images, TF,
  custom `gravion_msgs`, … all round-trip byte-for-byte.
- **`dds_player`** — reads that MCAP back and **re-publishes** every message onto
  DDS, in ascending log-time order, preserving the original inter-message
  timing.

---

## Contents

- [Why CDR (not the aggregator's JSON MCAP)](#why-cdr-not-the-aggregators-json-mcap)
- [How it works](#how-it-works)
- [Build](#build)
- [Quick start](#quick-start)
- [Record](#record)
- [Play](#play)
- [Config reference](#config-reference)
- [Tests](#tests)
- [Troubleshooting](#troubleshooting)
- [Limitations & notes](#limitations--notes)
- [Repository layout](#repository-layout)

---

## Why CDR (not the aggregator's JSON MCAP)

`message_manager`'s `MCAPLoggerTransport` (aitheon `data_logger`) writes **JSON**
MCAP for Foxglove visualization, and there is **no JSON→DDS path** in the
library. That file is great to *look at* but cannot be faithfully replayed. This
tool stores **CDR** instead, which deserializes straight back into the typed DDS
message via each type's generated `PubSubType`.

## How it works

```
record:  DDS sample ──(PubSubType::serialize, XCDR1)──▶ CDR bytes ─▶ MCAP (encoding "cdr")
play:    MCAP ─▶ CDR bytes ──(PubSubType::deserialize)──▶ DDS sample ─▶ FastDDSTransport::publish
```

Both directions are generic over the compile-time `SupportedTypes` list via
`dispatchByTypeIndex`. Each channel stores the DDS type name plus its QoS
(reliability/durability) in MCAP channel metadata, so the bag is
**self-describing** and the player republishes with matching QoS. See
`include/dds_mcap/cdr_codec.hpp`.

## Build

```bash
./build.sh                 # RelWithDebInfo, parallel; wraps the cmake calls below
# or, manually:
cmake -B build -S . -DCMAKE_PREFIX_PATH=/opt/botcrew
cmake --build build -j
```

Produces `build/dds_recorder` and `build/dds_player`.

**Prerequisites**
- The `message_manager` debian package at `/opt/botcrew` (pulls in Fast-DDS
  3.4.1 + fastcdr and exports the `message_manager::message_manager` target).
- `liblz4` and `libzstd` (MCAP chunk compression).
- A C++20 compiler, CMake ≥ 3.14.

> MCAP is vendored under `third_party/mcap` (complete 2.1.1 headers incl. the
> reader `.inl`, which the system `data_logger` copy lacks). The executables link
> with `-Wl,--allow-multiple-definition` so this complete copy wins over the
> partial symbols in `libdata_logger.a`. This is expected, not a bug.

## Quick start

```bash
# 1. record everything on domain 0 until Ctrl-C
./build/dds_recorder --all --domain 0 --output run.mcap

# 2. replay it onto domain 0
./build/dds_player run.mcap --domain 0
```

## Record

```
dds_recorder [--config FILE] [--output OUT.mcap] [--domain N]
             [--all] [--no-shm] [--shm-mb 1024]
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--config FILE` | — | JSON config (topic list, QoS, excludes — see below) |
| `--output OUT` | `recording.mcap` | output bag path (or config `output`) |
| `--domain N` | `0` | DDS domain (or config `domain_id`) |
| `--all` | off | force auto-discovery even if a topic list is present |
| `--no-shm` | off | disable the SHM transport (zero-copy topics are then skipped) |
| `--shm-mb N` | `1024` | SHM segment size in MB (or config `shm_segment_mb`) |

Stop with **Ctrl-C** (SIGINT/SIGTERM) — this finalizes the MCAP footer/summary.
**Do not `kill -9`**: that leaves a truncated, index-less bag (recoverable only
with `mcap recover`).

**Automatic discovery** (default when no topic list is given, or with `--all`):
a passive DDS discovery participant watches the domain; each remote DataWriter is
demangled from its `rt/…` wire name back to the ROS topic and subscribed
automatically. New topics are picked up live. Types not in `SupportedTypes` (and
`rq/`/`rr/`/`rs/` service endpoints) are silently skipped.

**Transport & QoS are matched automatically** — essential for sensor data:
- **SHM** — bounded zero-copy types (`PointCloud2_*MB_`, bounded images) are
  delivered over shared memory, *not* UDP. The recorder runs a second SHM
  transport for them. Disable with `--no-shm`; size it with `--shm-mb`.
- **Reliability/durability** — copied from each discovered writer. Lidar clouds
  are typically **best-effort**, and a *reliable* reader will not match a
  *best-effort* writer — without this they record **0 messages**.

**Explicit list** (subset / fixed set):

```bash
./build/dds_recorder --config config/record.example.json
# CLI overrides config: --output run.mcap  --domain 0
```

## Play

```
dds_player <bag.mcap> [--domain N] [--rate R] [--loop] [--warmup SECS]
           [--no-shm] [--shm-mb 1024] [--topics /a,/b]
```

| Flag | Default | Meaning |
|------|---------|---------|
| `<bag.mcap>` | — | bag to replay (required, first arg) |
| `--domain N` | `0` | DDS domain to publish on |
| `--rate R` | `1.0` | playback speed multiplier (`2.0` = 2×, `50` = fast) |
| `--loop` | off | repeat forever (see loop caveat in [Limitations](#limitations--notes)) |
| `--warmup SECS` | `1.0` | wait this long for subscribers before the first sample |
| `--no-shm` | off | UDP-only; zero-copy SHM topics are skipped |
| `--shm-mb N` | `1024` | SHM segment size in MB |
| `--topics /a,/b` | all | comma-separated **include-list**: replay only these topics |

The player advertises one publisher per channel (restoring each channel's
recorded QoS), then replays messages in **ascending log-time order**, sleeping to
reproduce the original inter-message timing scaled by `--rate`.

**Replaying a subset of topics** — pass a comma-separated include-list (exact
topic names, no spaces). Only those channels get a publisher; all other messages
are skipped:

```bash
./build/dds_player run.mcap --topics /ins/odometry,/ins/imu,/tf,/tf_static
```

Omitting `--topics` replays everything. (For a persistent subset you can also
pre-carve a smaller bag with `mcap filter -y '<regex>'` — useful when you want a
standalone file rather than a play-time filter.)

## Config reference

```jsonc
{
  "domain_id": 0,                 // DDS domain
  "output": "recording.mcap",     // output path
  "compression": "zstd",          // "zstd" (default) | "lz4" | "none"
  "discover": false,              // true => auto-discovery in addition to the list
  "exclude": ["/noisy/topic"],    // topics to drop (applies to discovery too)
  "topics": [                     // omit/empty => auto-discovery
    { "topic": "/lio/odometry",  "type": "nav_msgs/Odometry" },
    { "topic": "/cloud_registered", "type": "sensor_msgs/PointCloud2" },
    { "topic": "/tf_static", "type": "tf2_msgs/TFMessage",
      "qos": { "reliable": true, "transient_local": true } }
  ]
}
```

- `type` is the ROS-style schema name (`nav_msgs/Odometry`) **or** the DDS name
  (`nav_msgs::msg::dds_::Odometry_`); it must be in `SupportedTypes`.
- `qos` is optional; defaults are `reliable: true`, `transient_local: false`.
  (Auto-discovery infers QoS from the live writer, so you rarely need this.)
- See `config/record.example.json` and `config/test_record.json`.

## Tests

```bash
cmake --build build -j
./test/run_loopback_test.sh    # record on domain 88, replay on 89, assert intact + in order
./test/run_shm_test.sh         # bounded PointCloud2 over SHM + best-effort, record & replay
```

Record and playback use **different DDS domains**, so a PASS proves the data came
out of the bag, not from leftover live traffic. `run_loopback_test.sh` honors
`RECORD_MODE=discover` to exercise auto-discovery instead of the config list.

## Troubleshooting

**`Failed to create segment …` / `Could not initialize DataSharing writer pool`**
— `/dev/shm` is full of stale Fast-DDS segments (each bounded-cloud writer
reserves a large data-sharing pool; `kill -9`'d runs leak them):

```bash
df -h /dev/shm                              # ~100% full ⇒ this is it
rm -f /dev/shm/fast_datasharing_* /dev/shm/fastdds_* /dev/shm/sem.fastdds_*
```

Only do this with no DDS processes running.

**Recorded 0 messages on a topic** — almost always a QoS mismatch (reliable
reader vs best-effort writer) or a SHM topic with `--no-shm`. Prefer
auto-discovery, which matches QoS and transport automatically.

**`failed to open '…': … magic` / player reads very few messages** — the bag is
truncated (recorder was `kill -9`'d / crashed / ran out of disk before the footer
was written). Repair it with the MCAP CLI:

```bash
mcap recover broken.mcap -o fixed.mcap     # rebuilds footer + summary index
mcap doctor fixed.mcap                      # verify
```

## Limitations & notes

- **Discovery is writer-driven** — a topic is recorded only once a *publisher*
  exists on the domain. Start the recorder before/alongside the producers; it
  picks topics up live.
- **Only `SupportedTypes` are recorded.** Anything else is skipped. Extend the
  `message_manager` type list (`message_traits.hpp`) to add types.
- **Warmup** lets volatile subscribers catch the start. Transient-local writers
  (e.g. `/tf_static`) are replayed transient-local, so late joiners still get the
  latest sample.
- **SHM `keep_last` depth is 2** (`kShmDepth` in both `recorder.cpp` and
  `player.cpp`) to limit `/dev/shm` use. Under heavy load (many large clouds) the
  recorder can drop frames while writing, and the player can fall behind
  real-time. Raise `--shm-mb` for buffering headroom; lower `--rate` if playback
  lags.
- **`--loop` and timestamps** — the player replays each message with its
  *original* embedded `header.stamp`. On loop restart those stamps jump backward,
  which time-aware consumers (tf2, Foxglove) treat as stale, so a looped run can
  appear frozen at the loop boundary. For a continuous loop, time would need to
  advance monotonically across laps (not yet implemented). A single pass is
  unaffected.
- **`log_time` = recorder reception time.** If the recorder is backlogged (disk
  pressure, huge clouds), `log_time` can lag the sensor `header.stamp`. The bag
  is still correct for `header.stamp`-based fusion; only replay *pacing* is
  affected. Re-stamping `log_time = header.stamp` restores faithful pacing.

## Repository layout

```
src/recorder.cpp        dds_recorder — DDS subscribe → CDR → MCAP
src/player.cpp          dds_player   — MCAP → CDR → DDS publish
src/mcap_impl.cpp       single TU that compiles the vendored MCAP implementation
include/dds_mcap/       cdr_codec.hpp (serialize/deserialize), topic_discovery.hpp
config/                 record.example.json, test_record.json
test/                   loopback & SHM end-to-end tests (+ test_publisher/subscriber)
third_party/mcap/       vendored MCAP 2.1.1 headers
build.sh                convenience build wrapper
```
