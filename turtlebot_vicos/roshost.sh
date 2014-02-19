#!/bin/bash
ROS_MASTER=/tmp/roshost.$USER
export ROS_IP=`ifconfig | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}' | head -1` 
if [ -z "$1" ]; then
        if [ -f $ROS_MASTER ]; then 
        export ROS_MASTER_URI=`cat $ROS_MASTER` 
        else 
        export ROS_MASTER_URI="http://${ROS_IP}:11311" 
        fi
else
export ROS_MASTER_URI="http://${1}:11311"
fi
echo -n "$ROS_MASTER_URI" > "$ROS_MASTER"
echo
echo "ROS_IP is set to " $ROS_IP
echo "ROS_MASTER_URI is set to" $ROS_MASTER_URI
echo
