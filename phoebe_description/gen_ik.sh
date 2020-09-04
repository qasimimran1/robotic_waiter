#!/bin/bash

# Generate the analytic IK solution

# 2 12 6

if [ $# -lt 3 ]; then
  echo "Usage: $0 baselink eelink freeindex [info]" # 2 10 6
  exit
fi

rosrun collada_urdf urdf_to_collada phoebe.urdf phoebe.dae
rosrun moveit_ikfast round_collada_numbers.py phoebe.dae phoebe.rounded.dae 5
if [ $# -gt 3 ]; then
  openrave-robot.py phoebe.rounded.dae --info links
  exit
fi
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=phoebe.rounded.dae --iktype=transform6d --baselink=$1 --eelink=$2 --freeindex=$3 --savefile=ik.cpp
