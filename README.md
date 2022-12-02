# video_logging

#Docker
From a terminal running in the video_logging folder, to open the docker container run:
./docker/run.sh

#To run camera node
roslaunch realsense2_camera rs_camera.launch

#To run tcp connector (to communicate with unity)
roslaunch ros_tcp_endpoint endpoint.launch

#To run video_logging (bagging of files and replay when error detected)
rosrun video_logging video_logging.py

#If you dont have a roslaunch file running and want to run a python file, make sure to start a roscore or launch a file
roscore
