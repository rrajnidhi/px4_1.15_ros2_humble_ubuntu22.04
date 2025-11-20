FROM osrf/ros:humble-desktop-full

LABEL maintainer="Nidhi Raj <nidhirajr@gmail.com>"
LABEL description="ROS2 Humble + PX4 v1.15 + Gazebo Garden + Micro XRCE-DDS + PX4-ROS2 Bridge "

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DOMAIN_ID=3

# Base dependencies (non-ROS)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        tzdata locales lsb-release gnupg2 software-properties-common \
        curl wget git sudo nano vim tmux gdb openjdk-11-jdk ruby-full tree \
        build-essential cmake ninja-build clang lldb python3-dev python3-pip python3-venv \
        python3-setuptools python3-wheel libgtest-dev libeigen3-dev libyaml-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-tools \
        zip qtcreator genromfs exiftool python3-jinja2 python3-empy python3-toml python3-numpy python3-yaml \
        python3-pygments libxml2-utils clang-format clang-tidy lcov \
        libopencv-dev libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev libopencv-videoio-dev \
        libopencv-imgcodecs-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-video-dev \
        libopencv-objdetect-dev \
        ros-humble-rqt ros-humble-rqt-common-plugins ros-humble-rqt-image-view \
    && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Gazebo Garden + ROS-GZ bridge
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        | tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-garden \
        ros-humble-ros-gz \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python libraries
RUN pip3 install --no-cache-dir \
        pyserial empy toml numpy pandas jinja2 kconfiglib pyulog pyros-genmsg \
        pyquaternion packaging pyproj ultralytics opencv-python pygame aioconsole mavsdk

# Solve numpy/scipy compatibility issues
RUN pip3 uninstall -y numpy

# PX4-Autopilot v1.15.2
WORKDIR /app
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.15.2
WORKDIR /app/PX4-Autopilot
RUN bash Tools/setup/ubuntu.sh --no-nuttx && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && make px4_sitl_default"

# Micro XRCE-DDS Agent
WORKDIR /app
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig

# PX4 ROS 2 bridge workspace
WORKDIR /app
RUN mkdir -p ros2_ws/src && cd ros2_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git -b release/1.15 && \
    git clone https://github.com/PX4/px4_ros_com.git -b release/1.15

WORKDIR /app/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Tools
RUN gem install tmuxinator

# Copy launch files, models, worlds, and project files
COPY launch_files/single_drone_sitl.sh /app/launch_files/single_drone_sitl.sh
COPY launch_files/camera_detection.sh /app/launch_files/camera_detection.sh
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY /resources/simulation/models/. /root/.gz/models/
COPY /resources/simulation/models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY /resources/simulation/worlds/default_docker.sdf /app/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
RUN echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc

# GPU-related groups for host device compatibility
RUN groupadd -r render || true && groupadd -r video || true

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

