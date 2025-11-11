#!/bin/bash

source /environment.sh

dt-launchfile-init
roslaunch dynamic_obstacle race_demo.launch
dt-launchfile-join

