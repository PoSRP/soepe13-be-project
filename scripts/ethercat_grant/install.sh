#!/usr/bin/bash

set -e

launch_path=$(pwd)
script_path=$(dirname $0)
script_path=$(cd $script_path && pwd)
lib_path=$script_path/../../lib
rcfile=/home/$USER/.bashrc
shell_type=bash

if [ -z $script_path ]; then printf "\nPATH ERROR: $script_path\n\n" && exit 1; fi
if [[ ! -f /opt/ros/humble/setup.$shell_type ]]; then printf "\nNO ROS2 HUMBLE SETUP FILE FOUND: /opt/ros/humble/setup.$shell_type\n\n" && exit 1; fi
if [[ ! -f $rcfile ]]; then printf "\nNO RC FILE FOUND: $rcfile\n\n" && exit 2; fi

sudo apt-get install -qy libpcap-dev libcap-dev

source $rcfile
source /opt/ros/humble/setup.$shell_type

cd $lib_path/ethercat_grant

current_branch=$(git rev-parse --abbrev-ref HEAD)
if [[ $current_branch == "devel" ]]; then git pull; else git checkout -b devel origin/devel; fi

rm -rf build
mkdir build
cd build

sudo ../scripts/preinst
cmake ..
sudo make install
sudo ../scripts/postinst configure

cd $launch_path
exit 0
