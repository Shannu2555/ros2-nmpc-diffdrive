FROM osrf/ros:humble-desktop-full
RUN rm -f /etc/apt/sources.list.d/ros2*.list && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.osuosl.org/pub/ros/packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-ros-gz xvfb ffmpeg python3-pip x11-utils && rm -rf /var/lib/apt/lists/*
RUN pip3 install casadi pytest matplotlib
