#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source catkin_server/devel/setup.bash
rosrun active_sensing_continuous peg_hole_2d_sim catkin_server/src/active_sensing/active_sensing_yml/peg_hole_2d/peg_hole_2d.yml outputfile.txt
BASEDIR=$(dirname "$0")
echo "$BASEDIR"
