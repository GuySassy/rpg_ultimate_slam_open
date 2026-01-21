# RT1060 Telemetry Modes and Keypoint Ablation

## Why compact raw is required
UltimateSLAM consumes per-event timestamps (`dvs_msgs::Event`) and derives rate
and window metrics from timestamp deltas. RT1060 compact raw `(x,y,pol_dt)`
reconstructs per-event timestamps and polarity, so it is the default SLAM-safe
mode. Packet-only timestamps or XY-only points are not faithful for SLAM.

## Firmware modes supported
From `third_party/RT1060_newest/py_evtelemetry.c`:
- RAW (compact): `x_u16, y_u16, pol_dt_u16`
- RAW (full): `raw_u16, unused_u16, dt_ticks_u16, pol_u16, x_u16, y_u16`
- RAW_XY_ONLY: `x_u16, y_u16`
- KP: keypoints with descriptor bytes
- KP_ID: optional keypoint `id_u16`
- ACCEL: `ax, ay, az` (i16)
- GYRO: `gx, gy, gz` (i16)
- IMU_TS: optional `imu_timestamp_us` (u32)

## Event modes
Compact raw (default):
- Per-event timestamps reconstructed by accumulating `dt`.
- Anchor selection: `--rt1060_ts_anchor=start|end` (default `end`).

Full raw (optional):
- Per-event timestamps reconstructed from 24 MHz `dt_ticks`.
- Enable/override format selection with:
  - `--rt1060_raw_format=auto|compact|full`
  - scripts accept `RT1060_RAW_FORMAT=full` to override.

RAW_XY_ONLY (test-only):
- Parsed always, but **not emitted as events** unless synth is enabled.
- Enable synth: `--rt1060_allow_xy_only_synth=true`
- Control window/polarity:
  - `--rt1060_xy_only_window_us=3000`
  - `--rt1060_xy_only_polarity=1`
- A warning is logged when synth mode is active.

## Keypoint ablation (internal vs firmware)
Firmware keypoint injection uses the ELIS cached packet hook:
- Provider decodes firmware keypoints into `elis_link::KeypointPacket`.
- Packets are cached by slice timestamp (`events.back().ts.toNSec()`).
- Frontend consumes cached packets when:
  - `--vio_use_elis_link=true`
  - `--vio_elis_use_cached_packets=true`

Strict ablation (no fallback):
- `--vio_elis_require_cached_packets=true`
- If cached packet is missing, the slice is left empty (no internal keypoints).

Notes:
- If KP_ID is not present, per-slice indices are used as track IDs and a warning
  is logged. For stable tracking, enable KP_ID in firmware or set a short TTL
  (e.g., `--vio_elis_track_ttl_frames=2`) to limit handle growth.

## Example runs (same as scripts)
Internal keypoints baseline:
```bash
third_party/UltimateSLAM/scripts/run_rt1060_internal_kp.sh
```

Firmware keypoints injection:
```bash
third_party/UltimateSLAM/scripts/run_rt1060_firmware_kp.sh
```

Strict firmware ablation (no fallback):
```bash
third_party/UltimateSLAM/scripts/run_rt1060_firmware_kp_strict.sh
```

RAW_XY_ONLY synth (test-only):
```bash
third_party/UltimateSLAM/scripts/run_rt1060_xy_only_synth.sh
```

## Parser stats and logs
The RT1060 provider logs packet rates, checksum failures, resync count, and
queue drops every `--rt1060_log_stats_interval_s` seconds (0 disables).
