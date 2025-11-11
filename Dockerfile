FROM ubuntu:22.04

LABEL maintainer="Nidhi Raj <nidhirajr@gmail.com>"
LABEL description="ROS2 Humble + PX4 v1.15 + Gazebo Sim (Garden) + Micro XRCE-DDS + PX4-ROS2 Bridge (Ubuntu 22.04)"

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=0 \
    PX4_HOME=/app/PX4-Autopilot \
    PX4_HOME_LAT=47.397742 \
    PX4_HOME_LON=8.545594 \
    PX4_HOME_ALT=488

# Base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales curl wget git sudo lsb-release gnupg2 \
    build-essential cmake ninja-build software-properties-common \
    python3 python3-pip python3-dev python3-setuptools python3-wheel \
    nano vim gdb tmux openjdk-11-jdk ruby-full \
    && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble installation
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
    && rosdep init && rosdep update \
    && echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# PX4 build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    zip qtcreator genromfs exiftool \
    python3-jinja2 python3-empy python3-toml python3-numpy python3-yaml \
    python3-pygments libeigen3-dev libxml2-utils clang clang-format clang-tidy lcov \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Gazebo Garden + ROS-GZ bridge
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg | tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-garden ros-humble-ros-gz \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python libraries
RUN pip install --no-cache-dir \
    pyserial empy toml numpy pandas jinja2 kconfiglib pyulog pyros-genmsg \
    pyquaternion packaging pyproj

# PX4-Autopilot
WORKDIR /app
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.15.0
WORKDIR /app/PX4-Autopilot
RUN bash Tools/setup/ubuntu.sh --no-nuttx --no-simtools && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && make px4_sitl_default"

# Micro XRCE-DDS Agent
WORKDIR /app
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig

# PX4 ROS 2 bridge (px4_msgs + px4_ros_com)
WORKDIR /app
RUN mkdir -p ros2_ws/src && cd ros2_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git -b release/1.15 && \
    git clone https://github.com/PX4/px4_ros_com.git -b release/1.15
WORKDIR /app/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install" && \
    echo "source /app/ros2_ws/install/setup.bash" >> /root/.bashrc

# Tools & custom scripts
RUN gem install tmuxinator
COPY scripts /app/scripts

# Add GPU-related groups for compatibility with host devices
RUN groupadd -r render || true && groupadd -r video || true

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

