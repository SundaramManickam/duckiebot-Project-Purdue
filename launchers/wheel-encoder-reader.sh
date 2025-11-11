#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun dynamic_obstacle wheel_encoder_reader_node.py
dt-launchfile-join