FROM dustynv/ros:noetic-pytorch-l4t-r35.2.1

# ssh
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts 

# deps
RUN apt update
RUN apt install -qy python3-rosdep libgflags-dev python3-catkin-tools python3-pip tmux git vim python3-vcstool libeigen3-dev
RUN apt install -qy ros-noetic-eigen-conversions ros-noetic-tf ros-noetic-tf-conversions

# zed
RUN apt install -qy zstd ros-noetic-tf2-geometry-msgs ros-noetic-diagnostic-updater
RUN wget https://download.stereolabs.com/zedsdk/4.2/l4t35.2/jetsons && chmod +x jetsons && ./jetsons -- silent

# workspace
RUN mkdir -p /root/catkin_ws/src
RUN . /opt/ros/noetic/setup.sh && mkdir -p /root/catkin_ws/src && cd /root/catkin_ws && catkin init

# repos
RUN --mount=type=ssh cd /root/catkin_ws/src && git clone git@github.com:ongdexter/neurofly_interface.git
RUN --mount=type=ssh cd /root/catkin_ws/src && git clone git@github.com:ongdexter/kr_mav_control.git
RUN --mount=type=ssh cd /root/catkin_ws/src && git clone git@github.com:ongdexter/motion_capture_system.git
RUN --mount=type=ssh cd /root/catkin_ws/src && git clone --recursive https://github.com/KumarRobotics/zed-ros-wrapper.git

RUN cd /root/catkin_ws && . /opt/ros/noetic/setup.sh && catkin build

