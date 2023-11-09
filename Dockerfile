FROM turlucode/ros-noetic:cpu
# Install apt packages
RUN apt-get update && apt-get install -y git vim wget tmux terminator && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y   ros-noetic-joy ros-noetic-teleop-twist-joy   ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc   ros-noetic-rgbd-launch ros-noetic-rosserial-arduino   ros-noetic-rosserial-python ros-noetic-rosserial-client   ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server   ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro   ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz   ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers   ros-noetic-dynamixel-sdk   ros-noetic-turtlebot3-msgs   ros-noetic-turtlebot3 &&   apt-get clean && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN mkdir -p /root/catkin_ws/src
RUN cd /root/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.sh; catkin_make"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN cd /root/catkin_ws/src && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN cd /root/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

######
# chsh
RUN perl -p -i.bak -e 's%https?://(?!security)[^ \t]+%http://jp.archive.ubuntu.com/ubuntu/%g' /etc/apt/sources.list
ENV TZ=Asia/Tokyo
RUN chsh -s /usr/bin/bash root
