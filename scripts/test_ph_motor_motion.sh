#!/usr/bin/env bash
set -euo pipefail

# PH 电机运动测试脚本（通过统一接口脚本调用）
# 用法：
#   bash scripts/test_ph_motor_motion.sh car_a 0x601 2.0 2.0
#   参数: profile motor_id speed duration_sec

PROFILE="${1:-car_a}"
MOTOR_ID="${2:-0x601}"
SPEED="${3:-2.0}"
DURATION="${4:-2.0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACE="${SCRIPT_DIR}/ph_motor_interface.py"

if [[ ! -f "${IFACE}" ]]; then
  echo "[TEST-PH] 接口脚本不存在: ${IFACE}" >&2
  exit 2
fi

echo "[TEST-PH] profile=${PROFILE} motor=${MOTOR_ID} speed=${SPEED} duration=${DURATION}s"

echo "[TEST-PH] 1) ENABLE"
python3 "${IFACE}" --profile "${PROFILE}" --action enable --motor-id "${MOTOR_ID}"

echo "[TEST-PH] 2) MODE=VELOCITY"
python3 "${IFACE}" --profile "${PROFILE}" --action mode --motor-id "${MOTOR_ID}" --value 1

echo "[TEST-PH] 3) 正转"
python3 "${IFACE}" --profile "${PROFILE}" --action velocity --motor-id "${MOTOR_ID}" --value "${SPEED}"
sleep "${DURATION}"

echo "[TEST-PH] 4) 反转"
NEG_SPEED=$(python3 - <<PY
v = float("${SPEED}")
print(-v)
PY
)
python3 "${IFACE}" --profile "${PROFILE}" --action velocity --motor-id "${MOTOR_ID}" --value "${NEG_SPEED}"
sleep "${DURATION}"

echo "[TEST-PH] 5) STOP"
python3 "${IFACE}" --profile "${PROFILE}" --action stop --motor-id "${MOTOR_ID}"

echo "[TEST-PH] 6) DISABLE"
python3 "${IFACE}" --profile "${PROFILE}" --action disable --motor-id "${MOTOR_ID}"

echo "[TEST-PH] 完成"
