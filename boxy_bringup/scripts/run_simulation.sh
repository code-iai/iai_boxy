#!/bin/bash

pushd robot_roundup/robots
sh ./make_urdf.sh
popd

roslaunch ./robot_roundup/launch/display.launch
