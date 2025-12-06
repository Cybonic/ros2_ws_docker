# Base image with Ubuntu 22.04
FROM ubuntu:22.04

# Build arguments for user creation
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

# Set environment variables to avoid interactive prompts and warnings
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS=yes
ENV WORKSPACE=/ros_ws
ENV ROS_DISTRO=humble
ENV PIP_ROOT_USER_ACTION=ignore
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

# Build mode argument: development or production
ARG BUILD_MODE=development
ENV BUILD_MODE="${BUILD_MODE}"
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    software-properties-common \
    nano rsync net-tools iputils-ping

# Mode-specific installations
RUN if [ "${BUILD_MODE}" = "production" ]; then \
      echo "Installing production packages"; \
    else \
      echo "Installing dev tools including git, vim"; \
      apt-get install -y git vim htop; \
    fi

# Add the ROS 2 GPG key and source list (modern way)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-desktop

# Source ROS 2 environment on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Install colcon and other ROS tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
 && pip3 install -U argcomplete

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y \
    libspnav-dev \
    libbluetooth-dev \
    libcwiid-dev \
    ros-humble-geographic-msgs \
    ros-humble-test-msgs \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-realsense2-camera-msgs \
    ros-humble-sick-safetyscanners2-interfaces \
    ros-humble-sick-safetyscanners-base \
    ros-humble-tf2-sensor-msgs \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    libpcl-dev \
    libceres-dev \
    libopencv-dev \
    ros-humble-cv-bridge \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


    # Create user with same UID/GID as host user (handle existing group)
RUN if ! getent group $USER_GID > /dev/null 2>&1; then \
      groupadd --gid $USER_GID $USERNAME; \
    fi && \
    if ! getent passwd $USER_UID > /dev/null 2>&1; then \
      useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
    fi && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Create workspace and set permissions
WORKDIR $WORKSPACE
RUN mkdir -p $WORKSPACE && \
    chown -R $USER_UID:$USER_GID $WORKSPACE

# Switch to the created user
USER $USER_UID:$USER_GID


# COPY instructions that only apply to production builds
# These will be skipped in development mode due to build context
COPY ./src /ros_ws/src

# Source the workspace in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "if [ -f $WORKSPACE/install/setup.bash ]; then source $WORKSPACE/install/setup.bash; fi" >> ~/.bashrc

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


# Default command
CMD ["/bin/bash"]
