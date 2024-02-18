#!/bin/bash
TARGET=entrypoint4.sh
cat <<EOF >assets/${TARGET}
#! /bin/bash


\$(which sshd)

#sudo -u ubuntu "bash \$@"
sudo -u ubuntu "terminator"
#bash -c "\$@"

EOF
chmod +x assets/${TARGET}

funcBuild2() {

    cat <<EOF >Dockerfile2
FROM myros2

# setup
RUN mkdir -p /run/sshd

# ROOTにパスワードをセット
RUN echo 'root:root00' |chpasswd

# ユーザを作成
RUN useradd -m ubuntu
RUN echo 'ubuntu:ubuntu' |chpasswd
RUN echo "ubuntu    ALL=(ALL)       ALL" >> /etc/sudoers


## Install ROS packages
RUN apt-get update && apt-get install -y \
    openssh-server && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


# entrypoint
COPY data/.tmux.conf /root/
COPY assets/${TARGET} /
ENTRYPOINT ["/${TARGET}"]
EOF

    #########################################
    OPT=--no-cache
    OPT=
    docker build $OPT -t myros2:custom -f Dockerfile2 .
}

#funcBuild2
#########################################

cat <<EOF >Dockerfile3
FROM myros2:custom

## Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-dev-tools && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


RUN chsh -s $(which bash) ubuntu
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc

### key
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#
#
#
#RUN rosdep init && rosdep update 
#
#
#
#



RUN apt-get update && \
apt-get install -y curl gnupg2 && \
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN apt-get update && apt-get install -y \
 python3-colcon-common-extensions python3-rosdep python3-argcomplete  \
 ros-humble-cartographer \
 ros-humble-cartographer-ros \
 ros-humble-gazebo-* \
 ros-humble-navigation2 \
 ros-humble-nav2-bringup \
 ros-humble-dynamixel-sdk \
 ros-humble-turtlebot3-msgs \
 ros-humble-turtlebot3 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* 


## sample build
RUN mkdir -p /root/ws/src
RUN cd /root/ws/src && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN cd /root/ws && /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"



# Terminator Config
RUN mkdir -p /home/ubuntu/.config/terminator/
COPY assets/terminator_config /home/ubuntu/.config/terminator/config 
COPY assets/terminator_background.png /root/.config/terminator/background.png

RUN echo "source /usr/share/gazebo/setup.bash" >> /home/ubuntu/.bashrc



RUN apt-get -y update && apt-get -y install python3-pip
RUN pip3 install setuptools==58.2.0

## entry
#ENV TURTLEBOT3_MODEL=waffle_pi
COPY data/.tmux.conf /home/ubuntu
COPY assets/${TARGET} /

RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/ubuntu/.bashrc
RUN echo "source /usr/share/gazebo/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/${TARGET}"]

EOF

docker build $OPT -t myros2:custom2 -f Dockerfile3 .
