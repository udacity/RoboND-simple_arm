#! /bin/bash
# This script safely launches ros nodes with buffer time to allow arm_mover service to start up in time
x-terminal-emulator -e roslaunch simple_arm robot_spawn.launch &
sleep 5 &&
x-terminal-emulator -e ./look_away