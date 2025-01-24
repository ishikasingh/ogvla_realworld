source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=172.16.0.1
export ROS_HOSTNAME=localhost
export ROS_TCP_PORT_MIN=40000
export ROS_TCP_PORT_MAX=40100


cd /home/abrar/hsc/Allegro-Hand-Controller-DIME
source devel/setup.bash
roslaunch allegro_hand allegro_hand.launch