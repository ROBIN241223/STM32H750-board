#!/usr/bin/env bash
set -eu

PROJECT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
MODEL_PATH="${PROJECT_DIR}/models"
WORLD_PATH="${PROJECT_DIR}/worlds/test.world.sdf"

if [ -n "${GZ_SIM_RESOURCE_PATH:-}" ]; then
  export GZ_SIM_RESOURCE_PATH="${MODEL_PATH}:${GZ_SIM_RESOURCE_PATH}"
else
  export GZ_SIM_RESOURCE_PATH="${MODEL_PATH}"
fi

# Custom system plugins (e.g. IMU gyro publisher) built from ./imu_plugin.
if [ -n "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" ]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_DIR}/imu_plugin:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
else
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_DIR}/imu_plugin"
fi

# Keep local server and GUI discovery on the loopback interface.
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="${GZ_PARTITION:-stm32_h750_sim}"

# `gz sim -s` is headless and the CLI states it "overrides -g, if it is also
# present", so passing both never opens a window. Only ask for the headless
# server when the caller did not request a GUI; otherwise the server and the
# GUI client run in one process (verified: gz-sim-gui-client renders).
GUI=0
for arg in "$@"; do
  case "${arg}" in
    -g|--gui) GUI=1 ;;
  esac
done

if [ "${GUI}" -eq 1 ]; then
  exec gz sim --force-version 10 -r "${WORLD_PATH}"
fi

exec gz sim --force-version 10 -s -r "${WORLD_PATH}" "$@"
