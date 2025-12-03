FROM osrf/ros:humble-desktop-full

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes
ENV ROS_DISTRO=humble
ENV WORKSPACE=/ros2_ws

# Update package list and install apt-utils first to suppress debconf warnings
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    apt-utils \
    debconf-utils && \
    rm -rf /var/lib/apt/lists/*

# Configure debconf to use noninteractive frontend
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install ROS2 and system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-sensor-msgs-py \
    python3-numpy \
    git \
    wget \
    curl \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Create workspace
WORKDIR $WORKSPACE

# Copy the ROS2 workspace src directory
COPY src/ src/

# Install rosdep dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select pointcloud_downsampling"

# Source the workspace in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# Reset environment variables
ENV DEBIAN_FRONTEND=
ENV DEBCONF_NOWARNINGS=

# Set the default command
CMD ["/bin/bash"]