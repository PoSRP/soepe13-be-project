#!/usr/bin/bash

MY_PATH=$(dirname $0)
MY_PATH=$(cd $MY_PATH && pwd)
if [ -z $MY_PATH ] ; then
  exit 1
fi

cd $MY_PATH/../ros2
source $PWD/install/setup.bash

colcon build --symlink-install --cmake-args -DSECURITY=ON -DBUILD_TESTING=ON
colcon test
colcon test-result --all --verbose

cd - > /dev/null