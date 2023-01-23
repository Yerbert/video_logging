#!/bin/bash

camera="/camera/color/image_raw/compressed/processed"
point_cloud="/velodyne_points/processed"
info_logs="/infologs"
transform="/tf/processed"

# name=Scene_$(date +'%k-%M-%S_%d-%m-%Y').bag
name=SceneRecording

rosbag record -O $name $camera $point_cloud $info_logs $transform __name:=record_node

echo -e "\nRECORDING STOPPED"

rosnode kill /record_node

rosbag info $name