# "video_logging"

## Unity

Download Unity 2020.3.26f1.

For AR, must build to UWP, and deploy the solution to the HoloLens using Visual Studio.

For Tablet, build and run to Android.

## ROS

Clone repo to ~/catkin_ws/src/ and run `catkin_make`

## To run tcp connector
`roslaunch ros_tcp_endpoint endpoint.launch`

## To run message processor
`rosrun video_logging process_messages`

## To run main experimenter script
`rosrun video_logging AR_Diagnostics_Launch.py`