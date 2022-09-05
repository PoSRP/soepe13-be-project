#!/usr/bin/bash

set -e

case $SHELL in 
  */bash)
    SHELL_TYPE=bash
    RCFILE=/home/$USER/.bashrc;;
  */zsh)
    SHELL_TYPE=zsh
    RCFILE=/home/$USER/.zshrc;;
  *)
    SHELL_TYPE=sh;;
esac

if [[ ! -f /opt/ros/humble/setup.$SHELL_TYPE ]]; then
  printf "NO ROS2 HUMBLE SETUP FILE FOUND: /opt/ros/humble/setup.$SHELL_TYPE \n"
  exit 1
fi

if [[ ! -f $RCFILE ]]; then
  printf "NO RC FILE FOUND: $RCFILE \n"
  exit 2
fi

source $RCFILE
source /opt/ros/humble/setup.$SHELL_TYPE

git clone https://github.com/PoSRP/ethercat_grant.git
cd ethercat_grant
git checkout --track origin/devel

mkdir build
cd build

sudo ../scripts/preinst
cmake ..
sudo make install
sudo ../scripts/postinst configure

exit 0
