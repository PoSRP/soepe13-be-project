#!/usr/bin/bash

set -e

apt-get update -q
apt-get install -qy locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
  /usr/share/keyrings/ros-archive-keyring.gpg &> /dev/null
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  tee /etc/apt/sources.list.d/ros2.list &> /dev/null

apt-get update -q
apt-get install -qy ros-humble-desktop
apt-get install -qy python3-colcon-ros python3-colcon-bash python3-colcon-zsh \
  python3-rosdep libpcap-dev python3-pyqtgraph python3-colcon-package-selection

exit 0
