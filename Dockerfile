FROM osrf/ros:noetic-desktop-full
WORKDIR /root/ros_workspace/src
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && /opt/ros/noetic/bin/catkin_init_workspace
WORKDIR /root/ros_workspace
RUN source /opt/ros/noetic/setup.bash && catkin_make
COPY src/ /root/ros_workspace/src
RUN apt-get update
RUN source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && catkin_make
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
RUN echo 'source /root/ros_workspace/devel/setup.bash' >> ~/.bashrc
RUN apt-get install -y pip git
RUN /usr/bin/pip install ultralytics
RUN /usr/bin/pip install ipython numpy==1.20.3
RUN /usr/bin/pip install git+https://github.com/eric-wieser/ros_numpy.git
WORKDIR /workspace