#!/usr/bin/bash

set -e

launch_path=$(pwd)
script_path=$(dirname $0)
script_path=$(cd $script_path && pwd)
roslib_path=$script_path/../../ros2/lib
shell_type=bash
rcfile=/home/$USER/.bashrc

if [ -z $script_path ]; then printf "\nPATH ERROR: $script_path\n\n" && exit 1; fi
if [[ ! -f /opt/ros/humble/setup.$shell_type ]]; then printf "\nNO ROS2 HUMBLE SETUP FILE FOUND: /opt/ros/humble/setup.$SHELL_TYPE\n\n" && exit 1; fi
if [[ ! -f $rcfile ]]; then printf "\nNO RC FILE FOUND: $rcfile\n\n" && exit 2; fi

source $rcfile
source /opt/ros/humble/setup.$shell_type

if [ ! -d $roslib_path ]; then mkdir $roslib_path; fi
cd $roslib_path
git clone https://github.com/PoSRP/soem.git
cd soem
git checkout --track origin/devel

cd $launch_path
exit 0
