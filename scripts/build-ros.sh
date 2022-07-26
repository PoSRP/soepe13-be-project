#!/usr/bin/bash

set -e

if [[ $1 == "-h" || $1 == "--help" ]]; then
  printf "$0 <arg>\n"
  printf "  --no-test | -n    - Disable running tests\n"
  printf "  --help    | -h    - Show this message\n"
  exit
fi

my_path=$(dirname $0)
my_path=$(cd $my_path && pwd)
if [ -z $my_path ] ; then
  printf "\nPath error: ${my_path}\n"
  exit 1
fi

cd $my_path/../ros2
source /opt/ros/humble/setup.bash
rosdep install -iyr --from-path src --rosdistro humble 
cmake_args="-DSECURITY=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug"
colcon build --symlink-install --packages-select ecat_interfaces --cmake-args ${cmake_args}
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args ${cmake_args}

if [[ ! ($1 == "--no-test" || $1 == '-n') ]]; then
  colcon test
  colcon test-result --all --verbose
fi

exit 0
