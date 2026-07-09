#!/bin/bash
# Setup script for STM32 ROS2 Bridge on Raspberry Pi 5
# Run this on your Raspberry Pi 5

echo "=== STM32 ROS2 Bridge Setup ==="
echo ""

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Install ROS2 Humble first:"
    echo "  sudo apt install ros-humble-desktop"
    exit 1
fi
echo "ROS2 found: $(ros2 --version 2>&1)"

# Install pyserial
echo ""
echo "Installing pyserial..."
pip3 install pyserial

# Setup workspace
echo ""
echo "Setting up workspace..."
WORKSPACE=~/stm32_ws
mkdir -p $WORKSPACE/src
cp -r "$(dirname "$0")" $WORKSPACE/src/stm32_bridge

# Build
echo ""
echo "Building..."
source /opt/ros/humble/setup.bash
cd $WORKSPACE
colcon build --packages-select stm32_bridge
echo ""
echo "Source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Run the bridge:"
echo "  ros2 launch stm32_bridge bridge.launch.py serial_port:=/dev/ttyUSB0"
echo ""
echo "=== Setup Complete ==="
