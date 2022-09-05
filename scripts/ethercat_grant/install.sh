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

destination_dir="/home/$USER/ethercat_grant"
mkdir $destination_dir

git clone -b devel git@github.com:PoSRP/ethercat_grant.git $destination_dir
cd $destination_dir

mkdir build
cd build

sudo ../scripts/preinst
cmake ..
sudo make install
sudo ../scripts/postinst configure

exit 0
