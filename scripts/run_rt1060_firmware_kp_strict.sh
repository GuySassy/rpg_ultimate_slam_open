#!/usr/bin/env bash
set -euo pipefail

# Env overrides: RT1060_PORT, RT1060_RAW_FORMAT, CALIB_FILE, RUN_CMD, LOG_FLAGS, EXTRA_FLAGS

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

PORT="${RT1060_PORT:-/dev/ttyACM0}"
RAW_FORMAT="${RT1060_RAW_FORMAT:-compact}"
CALIB_FILE="${CALIB_FILE:-${REPO_ROOT}/third_party/UltimateSLAM/calibration/DAVIS-example.yaml}"
RUN_CMD="${RUN_CMD:-rosrun ze_vio_ceres ze_vio_ceres_node}"
LOG_FLAGS="${LOG_FLAGS:---v=1}"
EXTRA_FLAGS="${EXTRA_FLAGS:-}"

${RUN_CMD} \
  --data_source=3 \
  --rt1060_port="${PORT}" \
  --rt1060_baud=115200 \
  --rt1060_raw_format="${RAW_FORMAT}" \
  --rt1060_keypoint_source=firmware \
  --vio_use_events=true \
  --vio_use_events_and_images=false \
  --vio_use_elis_link=true \
  --vio_elis_use_cached_packets=true \
  --vio_elis_require_cached_packets=true \
  --num_imus=0 \
  --calib_filename="${CALIB_FILE}" \
  ${LOG_FLAGS} \
  ${EXTRA_FLAGS}
