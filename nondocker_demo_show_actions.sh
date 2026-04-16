#!/usr/bin/env bash

set -euo pipefail

# Demo common arm actions sequentially using the standalone Dynamixel test script.
# After each action, return to the appropriate Home pose.
#
# Usage examples:
#  
#   /home/rrl/FloSystemV2_GameDemo/demo_show_actions.sh   --ports "$FLO_MOTORS_DEVICE"   --side right   --repeat 2   --profile-accel 800 --profile-vel 400
#   --p-gain 80 --d-gain 24

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/flo_humanoid/scripts/test/arm_poses_test.py"
eval "$(python3 "${SCRIPT_DIR}/utility/device_paths.py" --format shell)"

# Defaults
PORTS=""
SIDE="left"           # left | right | dual
REPEAT=2
EEPROM_WAIT_MS=50
PROFILE_ACCEL=1000
PROFILE_VEL=400
P_GAIN=81
I_GAIN=""
D_GAIN=18
NO_WAIT=0
FINAL_TORQUE_OFF=0

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ports)
      PORTS=${2-}; shift 2 ;;
    --side)
      SIDE=${2-}; shift 2 ;;
    --repeat)
      REPEAT=${2-}; shift 2 ;;
    --eeprom-wait-ms)
      EEPROM_WAIT_MS=${2-}; shift 2 ;;
    --profile-accel)
      PROFILE_ACCEL=${2-}; shift 2 ;;
    --profile-vel)
      PROFILE_VEL=${2-}; shift 2 ;;
    --p-gain)
      P_GAIN=${2-}; shift 2 ;;
    --i-gain)
      I_GAIN=${2-}; shift 2 ;;
    --d-gain)
      D_GAIN=${2-}; shift 2 ;;
    --no-wait)
      NO_WAIT=1; shift ;;
    --final-torque-off)
      FINAL_TORQUE_OFF=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--ports \"\$FLO_MOTORS_DEVICE\"] --side <left|right|dual> [--repeat N] [--profile-accel A] [--profile-vel V] [--p-gain P] [--i-gain I] [--d-gain D] [--no-wait] [--final-torque-off]";
      exit 0 ;;
    *)
      echo "[ERR] Unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "${PORTS}" ]]; then
  PORTS="${FLO_MOTORS_DEVICE}"
fi

if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "[ERR] Test script not found: ${PY_SCRIPT}" >&2
  exit 2
fi

# Build ports array to pass as multiple args
read -r -a PORT_ARR <<< "${PORTS}"

# Determine home pose name based on side
case "${SIDE}" in
  right) HOME_POSE="Rhome" ;;
  left)  HOME_POSE="Lhome" ;;
  dual)  HOME_POSE="D_home" ;;
  *) echo "[ERR] --side must be left|right|dual" >&2; exit 2 ;;
esac

# Common gain/trajectory args (conditionally appended if provided)
COMMON_ARGS=("--eeprom-wait-ms" "${EEPROM_WAIT_MS}" "--profile-accel" "${PROFILE_ACCEL}" "--profile-vel" "${PROFILE_VEL}")
if [[ -n "${P_GAIN}" ]]; then COMMON_ARGS+=("--p-gain" "${P_GAIN}"); fi
if [[ -n "${I_GAIN}" ]]; then COMMON_ARGS+=("--i-gain" "${I_GAIN}"); fi
if [[ -n "${D_GAIN}" ]]; then COMMON_ARGS+=("--d-gain" "${D_GAIN}"); fi

# Action aliases supported by the test script
ACTIONS=("wave" "raise" "reach_side" "swing_lateral")

echo "[INFO] Ports: ${PORTS}"
echo "[INFO] Side: ${SIDE}  | Home: ${HOME_POSE}  | Repeat per action: ${REPEAT}"
echo "[INFO] Profile: accel=${PROFILE_ACCEL}, vel=${PROFILE_VEL}; Gains: P=${P_GAIN:-NA} I=${I_GAIN:-NA} D=${D_GAIN:-NA}"

# Initial home with initialization
echo "\n[STEP] Going to Home with --init"
python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --pose "${HOME_POSE}" --init "${COMMON_ARGS[@]}"

# Loop through actions
for act in "${ACTIONS[@]}"; do
  if [[ "${SIDE}" == "dual" ]]; then
    echo "\n[STEP] Dual Action: ${act} (repeat=${REPEAT}) — LEFT arm"
    python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --arm left  --pose "${act}" --repeat "${REPEAT}" "${COMMON_ARGS[@]}"
    echo "[STEP] Dual Action: ${act} — RIGHT arm"
    python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --arm right --pose "${act}" --repeat "${REPEAT}" "${COMMON_ARGS[@]}"
    echo "[STEP] Returning to Dual Home"
    python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --pose "D_home" "${COMMON_ARGS[@]}"
  else
    echo "\n[STEP] Action: ${act} (repeat=${REPEAT})"
    python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --arm "${SIDE}" --pose "${act}" --repeat "${REPEAT}" "${COMMON_ARGS[@]}"
    echo "[STEP] Returning to Home"
    python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --pose "${HOME_POSE}" "${COMMON_ARGS[@]}"
  fi

  if [[ "${NO_WAIT}" -eq 0 ]]; then
    read -r -p "[PAUSE] Press Enter to continue to next action..." _
  fi
done

if [[ "${FINAL_TORQUE_OFF}" -eq 1 ]]; then
  echo "\n[STEP] Final Home with torque off"
  python3 "${PY_SCRIPT}" --ports "${PORT_ARR[@]}" --pose "${HOME_POSE}" "${COMMON_ARGS[@]}" --final-torque-off
fi

echo "\n[DONE] Demo completed."
