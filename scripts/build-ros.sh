#!/usr/bin/bash

set -e

MY_PATH=$(dirname $0)
MY_PATH=$(cd $MY_PATH && pwd)
if [ -z $MY_PATH ] ; then
  printf "\nPath error: ${MY_PATH}\n"
  exit 1
fi

cd $MY_PATH/../ros2

source /opt/ros/humble/setup.bash
rosdep install -iyr --from-path src --rosdistro humble 
if [[ -f $PWD/install/setup.bash ]]; then
  source $PWD/install/setup.bash
fi

colcon build --symlink-install --packages-select ecat_interfaces
colcon build --symlink-install --cmake-args ' -DSECURITY=ON' ' -DBUILD_TESTING=ON' ' -DCMAKE_BUILD_TYPE=Debug'

if [[ ! ($1 == "--no-test" || $1 == '-n') ]]; then
  colcon test
  colcon test-result --all --verbose
fi

cd - > /dev/null
