source /opt/ros/humble/setup.bash
source /root/ros_workspace/install/setup.bash
ros2 run rviz2 rviz2 &
ros2 launch rosbot_gazebo simulation.launch.py world:=/ros2_ws/src/husarion_gz_worlds/worlds/empty_with_plugins.sdf