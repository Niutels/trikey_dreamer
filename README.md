### This package was tested on ROS Indigo Ubuntu 14.04 LTS. Gazebo 7. 

###Example method to launch the controller for Trikey is via:
$roslaunch trikey_dreamer trikey_gazebo.launch 

### In order to use the gazebo launch file, you have to add gazebo_world/models to gazebo's model path environment variable.

#### Add this to your ~/.bashrc file:
export GAZEBO_MODEL_PATH=~/catkin_ws/src/trikey_dreamer/gazebo_world/models:$GAZEBO_MODEL_PATH

