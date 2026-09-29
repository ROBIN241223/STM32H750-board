#!/usr/bin/env bash
set -eu

PROJECT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
MODEL_PATH="${PROJECT_DIR}/models"
WORLD_PATH="${PROJECT_DIR}/worlds/test_paused.world.sdf"
GZ_SIM_RESOURCE_PATH="${MODEL_PATH}"
GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_DIR}/imu_plugin"
export GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="${GZ_PARTITION:-stm32_h750_sim}"

exec gz sim --force-version 10 -s "${WORLD_PATH}" "$@"