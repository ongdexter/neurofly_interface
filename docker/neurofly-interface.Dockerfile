FROM stereolabs/zed:5.0-tools-devel-l4t-r36.4

ARG DEBIAN_FRONTEND=noninteractive

RUN apt -yq update && apt -yq upgrade

# ssh
RUN apt -yqq install ssh
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# ros
RUN apt install -yq locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt install -yq software-properties-common
RUN add-apt-repository universe

ARG ROS_APT_SOURCE_VERSION=1.1.0
ARG UBUNTU_CODENAME=jammy

RUN apt update && apt install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt -yq update
RUN apt install -yq ros-humble-desktop

RUN apt install -qy python3-rosdep \
    python3-colcon-common-extensions \
    libgflags-dev \
    python3-pip \
    tmux \
    git \
    vim \
    python3-vcstool \
    libeigen3-dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    ros-dev-tools \
    ros-humble-ament-* \
    vim

# workspace
WORKDIR /root/ros2_ws
RUN rosdep init
RUN rosdep update
RUN mkdir -p /root/ros2_ws/src

# repos
WORKDIR /root/ros2_ws/src
RUN --mount=type=ssh git clone https://github.com/stereolabs/zed-ros2-wrapper.git
RUN --mount=type=ssh git clone -b ros2-humble https://github.com/KumarRobotics/motion_capture_system.git
# RUN --mount=type=ssh git clone https://github.com/ongdexter/kr_mav_control.git

# ros deps and build
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# RUN rosdep install --from-paths src --ignore-src -r -y
# colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
# RUN --mount=type=ssh git clone git@github.com:ongdexter/neurofly_interface.git

# RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

# install dependencies and build
# WORKDIR /root/ros2_ws
# RUN . /opt/ros/humble/setup.sh && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup environment (optional)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc
