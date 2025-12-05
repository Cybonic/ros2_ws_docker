#!/bin/bash

# Script to build the Docker image and run a container

# --- Configuration ---
IMAGE_NAME="pharmarobot"
IMAGE_TAG="dev"
CONTAINER_NAME="pharma"

# get Input Arguments

# 1st argument: action to perform (build, run, or both)
# 2nd argument: "localhost" or "jackal" to set ROS_MASTER_URI
# Usage: ./build_and_run_docker.sh [action] [localhost|jackal]

# --- Set ROS_MASTER_URI and ROS_IP based on the second argument ---
# This section allows you to choose between running the container with ROS_MASTER_URI set to localhost
# or to the Jackal's IP address, depending on your setup.
# The second argument should be either "localhost" or "jackal".
# If you want to run the container on the Jackal robot, you can set the second argument to "jackal".





#ROS_MASTER_URI="http://169.254.119.61:11311" 
#ROS_IP="169.254.165.124"



ws_dir="ros_ws" # Directory inside the container where the host workspace will be mounted
# This is the directory where your catkin workspace will be mounted inside the container.

# Action to perform: "build", "run", or "both" (default)
ACTION="${1:-both}"


# Host directory to map to /workspace in the container.
# This script assumes it's located in the same directory as your Dockerfile.
HOST_WORKSPACE_DIR_RAW="$(pwd)/src" # Using "." means the directory where the script is run.
                           # "$(pwd)" would also work but "." is often simpler here.

# Resolve to the canonical, absolute path, following all symlinks
HOST_WORKSPACE_DIR=$(readlink -f "${HOST_WORKSPACE_DIR_RAW}")
if [ ! -d "${HOST_WORKSPACE_DIR}" ]; then
    echo "Error: Resolved HOST_WORKSPACE_DIR '${HOST_WORKSPACE_DIR}' is not a directory."
    echo "       (Original path was: '${HOST_WORKSPACE_DIR_RAW}', resolved to '${HOST_WORKSPACE_DIR}')"
    exit 1
fi





# Dockerfile location (relative to this script's execution path)
DOCKERFILE_PATH="Dockerfile"

# Build context (directory containing files for the build, including Dockerfile)
BUILD_CONTEXT="."

# JupyterLab port on the host
#JUPYTER_HOST_PORT="8888"

# --- Build Docker Image ---
if [ "$ACTION" = "build" ] || [ "$ACTION" = "both" ]; then
    DO_BUILD=false
    if [ "$ACTION" = "build" ]; then
        echo "Action is 'build'. Preparing to build image '${IMAGE_NAME}:${IMAGE_TAG}'."
        DO_BUILD=true
    # For 'both' mode (default), check if image exists
    elif ! docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" >/dev/null 2>&1; then
        echo "Image '${IMAGE_NAME}:${IMAGE_TAG}' not found. Preparing to build for default 'both' mode."
        DO_BUILD=true
    else
        echo "Image '${IMAGE_NAME}:${IMAGE_TAG}' already exists. Skipping build for default 'both' mode."
    fi

    if [ "$DO_BUILD" = true ]; then
        echo "Building Docker image '${IMAGE_NAME}:${IMAGE_TAG}'..."

        # echo "pruning unused Docker images..." # Original script had this echo
        #docker prune -f # Original script had this commented
        echo "Dockerfile: ${DOCKERFILE_PATH}"
        echo "Build Context: ${BUILD_CONTEXT}"

        docker build --no-cache -t \
        "${IMAGE_NAME}:${IMAGE_TAG}" \
        -f "${DOCKERFILE_PATH}" \
        "${BUILD_CONTEXT}"


        
        echo "Docker build command executed.\n"
        if [ $? -ne 0 ]; then
            echo "Docker build failed. Exiting."
            exit 1
        fi
        echo "*** Docker image built successfully: ${IMAGE_NAME}:${IMAGE_TAG}\n\n"
    fi
fi

# --- Prune unused Docker images (optional, uncomment to enable) ---
# echo "Pruning unused Docker images..."
# docker image prune -f

# --- Run Docker Container ---
if [ "$ACTION" = "run" ] || [ "$ACTION" = "both" ]; then
    # Ensure image exists before attempting to run
    if ! docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" >/dev/null 2>&1; then
        echo "Error: Docker image '${IMAGE_NAME}:${IMAGE_TAG}' does not exist."
        if [ "$ACTION" = "run" ]; then
            echo "Please build it first (e.g., use './$(basename "$0") build' or './$(basename "$0")')."
        else # ACTION is "both", implies build failed or was unexpectedly skipped
            echo "Build may have failed or was not triggered. Cannot run."
        fi
        exit 1
    fi

    echo "Attempting to stop and remove existing container named '${CONTAINER_NAME}' (if any)..."
    docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true

    echo "Running Docker container '${CONTAINER_NAME}' from image '${IMAGE_NAME}:${IMAGE_TAG}'..."
    echo "  Host workspace mounted to /workspace: ${HOST_WORKSPACE_DIR}"
    #echo "  JupyterLab will be accessible on host at: http://localhost:${JUPYTER_HOST_PORT} (or http://<your-docker-host-ip>:${JUPYTER_HOST_PORT})"
    echo "  GPU access will be enabled."


    xhost +local:root
    # Default value
    HOST_ROSBAGS_DIR="datasets"

    # Parse arguments for --rosbag_path
    while [[ $# -gt 0 ]]; do
        case $1 in
            --rosbag_path)
                HOST_ROSBAGS_DIR="$2"
                shift 2
                echo "Host ROS bags directory: ${HOST_ROSBAGS_DIR} (will be mounted to /rosbags in the container)"
                ;;
                
            *)
                echo "  Unrecognized argument: $1"
                shift
                ;;
        esac
    done

    echo "*** 1) DONE parsing arguments."

    # Resolve the full path of the host ROS bags directory
    FULL_HOST_ROSBAGS_DIR=$(readlink -f "${HOST_ROSBAGS_DIR}") # Directory for storing ROS bags on the host
    if [ ! -d "${FULL_HOST_ROSBAGS_DIR}" ]; then
        echo "Error: Resolved HOST_ROSBAGS_DIR '${FULL_HOST_ROSBAGS_DIR}' is not a directory."
        exit 1
        # Directory for storing ROS bags on the host
    fi

    echo "*** 2) Resolved HOST_ROSBAGS_DIR: ${FULL_HOST_ROSBAGS_DIR}"
    # -it: Interactive TTY
    # --rm: Automatically remove the container when it exits
    # --gpus all: Make all GPUs available (ensure your Docker and NVIDIA drivers support this)
    # -v: Mount host directory to container directory
    # -p: Map host port to container port
    # --name: Assign a name to the container
    # The last line is the command to run inside the container

    # Allow X11 forwarding for GUI applications (if needed)
    # Replace this line:
    # xhost + #local:root

    
    # On the remote machine
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth

    # Create .Xauthority file if it doesn't exist
    touch ~/.Xauthority

    # Create .docker.xauth file
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    docker run --rm -it --tty \
        --privileged \
        --network=host \
        --memory=4g \
        --cpus=2.0 \
        --device=/dev/ttyUSB0 \
        --device=/dev/input/js0 \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --name "${CONTAINER_NAME}" \
        -v "${HOST_WORKSPACE_DIR}:/${ws_dir}" \
        -v "${FULL_HOST_ROSBAGS_DIR}:/${ws_dir}/rosbag2" \
        --env ROS_HOSTNAME=localhost \
        --env ROS_MASTER_URI=http://localhost:11311 \
        "${IMAGE_NAME}:${IMAGE_TAG}" \
        bash #-c "source /opt/ros/humble/setup.bash"
                 
    
    echo "Container '${CONTAINER_NAME}' has exited."
fi

# Handle invalid action
if [ "$ACTION" != "build" ] && [ "$ACTION" != "run" ] && [ "$ACTION" != "both" ]; then
    echo "Invalid action: '${ACTION}'. Please use 'build', 'run', or leave blank (for 'both')."
    exit 1
fi