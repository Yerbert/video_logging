#!/bin/bash

camera="camera/color/image_raw/compressed/processed"
point_cloud="velodyne_points/processed"
info_logs="infologs"
transform="tf"

name=$(pwd)/$(date +'%k-%M-%S_%d-%m-%Y').bag

rosbag record -O $name $camera $point_cloud $info_logs $transform

echo -e "\nRECORDING STOPPED"

rosbag info $name