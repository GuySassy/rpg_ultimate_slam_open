#!/usr/bin/env bash
set -euo pipefail

# Ground-truth baseline: internal FAST+KLT on RT1060 full-raw events.
# Env overrides: RT1060_PORT, RT1060_RAW_FORMAT, CALIB_FILE, RUN_CMD, LOG_FLAGS,
#                EXPERIMENT_FLAGS, EXTRA_FLAGS, VIO_DESCRIPTOR_USE_DUMMY.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

PORT="${RT1060_PORT:-/dev/ttyACM0}"
RAW_FORMAT="${RT1060_RAW_FORMAT:-full}"
CALIB_FILE="${CALIB_FILE:-${REPO_ROOT}/third_party/UltimateSLAM/calibration/DAVIS-example.yaml}"
RUN_CMD="${RUN_CMD:-rosrun ze_vio_ceres ze_vio_ceres_node}"
LOG_FLAGS="${LOG_FLAGS:---v=1 --alsologtostderr}"
VIO_DESCRIPTOR_USE_DUMMY="${VIO_DESCRIPTOR_USE_DUMMY:-true}"

EXPERIMENT_FLAGS="${EXPERIMENT_FLAGS:---data_use_time_interval=false \
  --data_interval_between_event_packets=5000 \
  --data_size_augmented_event_packet=20000 \
  --vio_frame_size=8000 \
  --noise_event_rate=5000 \
  --rt1060_queue_size=8 \
  --rt1060_log_stats_interval_s=1 \
  --rt1060_debug_log_path=/tmp/rt1060_debug_packets.csv \
  --rt1060_debug_log_max_lines=0 \
  --rt1060_debug_every_n_packets=1 \
  --rt1060_debug_log_flush_every=50 \
  --vio_min_tracked_features_total=50 \
  --vio_kfselect_numfts_lower_thresh=20 \
  --vio_kfselect_numfts_upper_thresh=120 \
  --imp_detector_threshold=5 \
  --imp_detector_grid_size=16 \
  --imp_detector_max_features_per_frame=600}"

EXTRA_FLAGS="${EXTRA_FLAGS:-}"

${RUN_CMD} \
  --data_source=3 \
  --rt1060_port="${PORT}" \
  --rt1060_baud=115200 \
  --rt1060_raw_format="${RAW_FORMAT}" \
  --rt1060_keypoint_source=elis_code \
  --vio_use_events=true \
  --vio_use_events_and_images=false \
  --vio_use_elis_link=false \
  --vio_descriptor_use_dummy="${VIO_DESCRIPTOR_USE_DUMMY}" \
  --num_imus=0 \
  --calib_filename="${CALIB_FILE}" \
  ${LOG_FLAGS} \
  ${EXPERIMENT_FLAGS} \
  ${EXTRA_FLAGS}
