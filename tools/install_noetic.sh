#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-host}"
if [[ "$ROLE" != "host" && "$ROLE" != "robot" && "$ROLE" != "auxiliary" ]]; then
  echo "Usage: $0 host|robot|auxiliary" >&2
  exit 2
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
  echo "ROS Noetic for this paper is supported here only on Ubuntu 20.04 (Focal)." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release ca-certificates

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | \
  sudo tee /etc/apt/sources.list.d/ros1-latest.list >/dev/null

sudo apt-get update
COMMON_PACKAGES=(
  python3-catkin-tools
  python3-cryptography
  python3-paho-mqtt
  python3-pip
  python3-rosdep
  python3-yaml
  ros-noetic-navigation
  ros-noetic-turtlebot3-msgs
)

case "$ROLE" in
  host)
    sudo apt-get install -y \
      ros-noetic-desktop-full \
      ros-noetic-turtlebot3 \
      ros-noetic-turtlebot3-simulations \
      ros-noetic-slam-gmapping \
      mosquitto mosquitto-clients openssl sqlite3 \
      "${COMMON_PACKAGES[@]}"
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
    sudo npm install --global --unsafe-perm node-red@3.1.15
    ;;
  robot)
    sudo apt-get install -y \
      ros-noetic-ros-base \
      ros-noetic-turtlebot3-bringup \
      ros-noetic-hls-lfcd-lds-driver \
      "${COMMON_PACKAGES[@]}"
    ;;
  auxiliary)
    sudo apt-get install -y \
      ros-noetic-ros-base \
      python3-rpi.gpio i2c-tools \
      "${COMMON_PACKAGES[@]}"
    python3 -m pip install --user adafruit-blinka adafruit-circuitpython-ina219
    for device_group in gpio i2c; do
      if getent group "$device_group" >/dev/null; then
        sudo usermod -aG "$device_group" "$USER"
      fi
    done
    ;;
esac

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "Installed ROS Noetic dependencies for role: $ROLE"
echo "Log out and back in after the auxiliary install so gpio/i2c groups take effect."
