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

# upgrade opencv
RUN apt remove libopencv-dev -y && apt purge -y libopencv* && apt remove -y opencv-licenses
RUN apt install -y libopencv-dev

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

# dv-ros2 deps
RUN add-apt-repository ppa:inivation-ppa/inivation
RUN apt-get update -y
RUN apt-get install -qy cmake boost-inivation libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev libssl-dev
# # remove opencv for dv-ros2 deps
# RUN apt remove -y libopencv-dev
# RUN add-apt-repository ppa:ubuntu-toolchain-r/test
# RUN apt-get -yq update
# RUN apt-get install --only-upgrade libstdc++6 -qy
# switch gcc and g++ to 13
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
RUN apt-get update -y
RUN apt install -y gcc-13 g++-13
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 60 --slave /usr/bin/g++ g++ /usr/bin/g++-13
RUN update-alternatives --config gcc
# build dv-processing 1.7.9 from source
WORKDIR /root/
RUN --mount=type=ssh git clone -b 1.7.9 https://gitlab.com/inivation/dv/dv-processing.git
WORKDIR /root/dv-processing
RUN mkdir build
WORKDIR /root/dv-processing/build
RUN CC=gcc-13 CXX=g++-13 cmake -DCMAKE_INSTALL_PREFIX=/usr ..
RUN make -j8
RUN make install

RUN rosdep init
RUN rosdep update

# autonomy workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN --mount=type=ssh git clone https://github.com/stereolabs/zed-ros2-wrapper.git
RUN --mount=type=ssh git clone -b ros2-humble https://github.com/KumarRobotics/motion_capture_system.git
RUN --mount=type=ssh git clone https://github.com/berndpfrommer/rosbag2_composable_recorder.git
RUN --mount=type=ssh git clone https://github.com/ongdexter/kr_mav_control.git
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select kr_mav_msgs kr_tracker_msgs kr_mav_controllers kr_trackers_manager kr_trackers kr_mav_manager \
        rqt_mav_manager mocap_base mocap_vicon \
        kr_betaflight_interface rosbag2_composable_recorder
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to zed_wrapper

# dvs workspace
RUN mkdir -p /root/dvs_ws/src
WORKDIR /root/dvs_ws/src
RUN --mount=type=ssh git clone https://github.com/ongdexter/dv-ros2.git
WORKDIR /root/dvs_ws
RUN . /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to dv_ros2_unified

# setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /root/dvs_ws/install/local_setup.bash" >> ~/.bashrc
