#!/bin/bash

sudo pkill -f wifi_node
sudo  pkill -f roscore

roscore >/dev/null &2>&1 &
sleep 5
sudo airmon-ng start wlan0 >/dev/null &2>&1 &
sudo su -c ../../wifi_node/./launch_script.sh >/dev/null &2>&1 &
sudo sh -c "echo 1 >/proc/sys/net/ipv4/ip_forward"
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"

roslaunch recurrent_connectivity husarion1.launch robot_id:=1 relay_robots:=4,5,7,8 robot_type:=1
