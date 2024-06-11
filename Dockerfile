FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get update
RUN apt-get install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress
RUN apt-get install -y python3-pip ros-dev-tools stm32flash


WORKDIR /root/ros_workspace/src
COPY . /root/ros_workspace/src/


RUN source /opt/ros/humble/setup.bash
WORKDIR /root/ros_workspace
RUN vcs import src < src/rosbot/rosbot_hardware.repos
RUN vcs import src < src/rosbot/rosbot_simulation.repos


RUN export HUSARION_ROS_BUILD=simulation

# Build only diff_drive_controller and imu_sensor_broadcaster from ros2_controllers
RUN cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers



# RUN rosdep init
RUN rosdep update --rosdistro humble
RUN rosdep install -i --from-path src --rosdistro humble -y
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


# RUN source /opt/ros/noetic/setup.bash && /opt/ros/noetic/bin/catkin_init_workspace
# WORKDIR /root/ros_workspace
# RUN source /opt/ros/noetic/setup.bash && catkin_make

# RUN apt-get update
# RUN source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y
# RUN source /opt/ros/noetic/setup.bash && catkin_make
# RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
# RUN echo 'source /root/ros_workspace/devel/setup.bash' >> ~/.bashrc
# RUN apt-get install -y pip git
# RUN /usr/bin/pip install ultralytics
# RUN /usr/bin/pip install ipython numpy==1.20.3
# RUN /usr/bin/pip install git+https://github.com/eric-wieser/ros_numpy.git
# RUN /usr/bin/pip install open3d==0.13
# WORKDIR /workspace