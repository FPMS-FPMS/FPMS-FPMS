#!/bin/bash
# micro_ros_platformio runs `. $HOME/.platformio/penv/bin/activate` and then
# invokes colcon inside it. A `pip install --user platformio` never creates that
# venv, so the micro-ROS library build dies before it starts. Same failure shape
# as the ESP-IDF catkin_pkg blocker already documented in firmware/README.md:
# the ROS build tooling has to live in the venv the build actually activates,
# not in system or user site-packages.
set -x
PW="$1"
echo "$PW" | sudo -S apt-get install -y python3-venv 2>&1 | tail -3

rm -rf "$HOME/.platformio/penv"
python3 -m venv "$HOME/.platformio/penv" || exit 1
"$HOME/.platformio/penv/bin/python" -m pip install -U pip 2>&1 | tail -2
"$HOME/.platformio/penv/bin/python" -m pip install -U \
    platformio catkin_pkg lark lark-parser "empy==3.3.4" \
    colcon-common-extensions pyyaml 2>&1 | tail -4

"$HOME/.platformio/penv/bin/pio" --version
echo "=== SETUP2 DONE ==="
