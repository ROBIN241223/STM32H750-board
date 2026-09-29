#!/usr/bin/env bash
# Locate a ROS 2 distro, source it, then build the stm32_bridge package with colcon.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${script_dir}/_ws"

ros_setup=""
for dist_dir in /opt/ros/*/; do
    [ -f "${dist_dir}setup.bash" ] || continue
    ros_setup="${dist_dir}setup.bash"
done

if [ -z "${ros_setup}" ]; then
    echo "ERROR: no ROS 2 distribution found under /opt/ros" >&2
    exit 1
fi

echo ">>> Sourcing ${ros_setup}"
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

echo ">>> Preparing workspace ${ws_dir}"
mkdir -p "${ws_dir}/src"
ln -sfn "${script_dir}" "${ws_dir}/src/stm32_bridge"

echo ">>> Building stm32_bridge"
cd "${ws_dir}"
colcon build --packages-select stm32_bridge --symlink-install "$@"

echo ""
echo ">>> Done. Source the overlay:  source ${ws_dir}/install/setup.bash"