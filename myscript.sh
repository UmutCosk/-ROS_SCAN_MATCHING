#!/bin/bash

# source /opt/ros/kinetic/setup.bash
# source ~/catkin_ws/devel/setup.bash

#ADMA
sudo ip link set can0 up type can bitrate 500000
gnome-terminal -e "roslaunch ros_can_gps cangps.launch"

#Bridge
sleep 1
gnome-terminal -e "roslaunch Model_Development_bridge Model_Development_bridge_start_all.launch"

#Laserscanner
sleep 1
gnome-terminal -e "roslaunch ibeo_lux ibeo_lux_transform.launch"

#Localizer
sleep 1
gnome-terminal -e "roslaunch scan_matching localizer.launch"



