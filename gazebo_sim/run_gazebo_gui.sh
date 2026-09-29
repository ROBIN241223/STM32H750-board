#!/usr/bin/env bash
# Chạy Gazebo server + GUI.
#
# Không dùng `-s` ở đây: `gz sim -s` là headless và CLI ghi rõ nó "overrides -g,
# if it is also present", nên `-s -g` chỉ chạy server, cửa sổ GUI không bao giờ
# mở. Bỏ `-s` thì server và GUI client chạy chung một process.
#
# Không cần ép Mesa/llvmpipe: sau khi reboot, module NVIDIA nạp đúng 580.178.04
# khớp userspace nên EGL/GLX hoạt động bình thường. (Trước đó module cũ
# 580.173.02 còn lại từ lúc boot làm Ogre crash — xem PROJECT_LOG.md.)
set -eu

PROJECT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
export DISPLAY="${DISPLAY:-:1}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
# Cùng partition với run_gazebo.sh, để controller/acceptance và `gz topic`
# nhìn thấy server khi sim chạy kèm GUI.
export GZ_PARTITION="${GZ_PARTITION:-stm32_h750_sim}"

# Kiểm tra driver trước khi mở, để không phải đoán vì sao viewport đen.
if command -v nvidia-smi >/dev/null 2>&1 && [ -r /proc/driver/nvidia/version ]; then
  # /proc/driver/nvidia.version = "NVRM version: NVIDIA UNIX x86_64 Kernel Module  580.178.04  Tue Jul ..."
  loaded="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+' /proc/driver/nvidia/version | head -1)"
  ondisk="$(modinfo -F version nvidia 2>/dev/null || echo "$loaded")"
  if [ "$loaded" != "$ondisk" ]; then
    echo "WARNING: module dang load $loaded, tren dia la $ondisk." >&2
    echo "         NVML/EGL se hong -> GUI co the crash o Ogre. Reboot la fix." >&2
  else
    echo "NVIDIA module $loaded khop voi userspace."
  fi
fi

exec bash "${PROJECT_DIR}/run_gazebo.sh" -g
