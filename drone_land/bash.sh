source /home/ljq/Documents/drone_land/devel/setup.bash
export GAZEBO_MODEL_PATH=:/home/ljq/Documents/some/src/simulation/models:~/gazebo_models
source /home/ljq/Documents/Firmware/Tools/setup_gazebo.bash /home/ljq/Documents/Firmware /home/ljq/Documents/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ljq/Documents/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ljq/Documents/Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ljq/Documents/some/src

