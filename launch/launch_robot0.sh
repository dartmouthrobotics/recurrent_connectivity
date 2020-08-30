#!/bin/bash
sudo sh -c "echo 1 >/proc/sys/net/ipv4/ip_forward"
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"

roslaunch gvgexploration husarion.launch robot_id:=0 relay_robots:=4,5,7,8 robot_type:=1
