#!/usr/bin/bash

set -e

launch_path=$(pwd)
script_path=$(dirname $0)
script_path=$(cd $script_path && pwd)
lib_path=$script_path/../../lib
shell_type=bash
rcfile=/home/$USER/.bashrc

if [ -z $script_path ]; then printf "\nPATH ERROR: $script_path\n\n" && exit 1; fi
if [[ ! -f /opt/ros/humble/setup.$shell_type ]]; then printf "\nNO ROS2 HUMBLE SETUP FILE FOUND: /opt/ros/humble/setup.$SHELL_TYPE\n\n" && exit 1; fi
if [[ ! -f $rcfile ]]; then printf "\nNO RC FILE FOUND: $rcfile\n\n" && exit 2; fi

source $rcfile
source /opt/ros/humble/setup.$shell_type

sudo apt-get install -qy libpcap-dev libcap-dev

source $rcfile
source /opt/ros/humble/setup.$shell_type

if [ ! -d $lib_path ]; then mkdir $lib_path; fi
cd $lib_path
git clone https://github.com/PoSRP/ethercat_grant.git
cd ethercat_grant
git checkout --track origin/devel

mkdir build
cd build
sudo ../scripts/preinst
cmake ..
sudo make install
sudo ../scripts/postinst configure

cd $launch_path
exit 0
