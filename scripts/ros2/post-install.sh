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

rosdep init
rosdep update

printf "
# User added - ROS2
export LANG=en_US.UTF-8
export ROS_DOMAIN_ID=0
export ROSCONSOLE_FORMAT='\${logger}: \${message}'
export ROS_SECURITY_KEYSTORE=
export ROS_SECURITY_ENABLE=
export ROS_SECURITY_STRATEGY=
" >> $RCFILE

exit 0
