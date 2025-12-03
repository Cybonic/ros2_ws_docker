#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Change to project directory
cd "$PROJECT_DIR"

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -d, --dev               Run ROS2 development container"
    echo "  -r, --rviz              Run RViz container"
    echo "  -p, --production        Run production pointcloud downsampler"
    echo "  -b, --build             Build Docker images before running"
    echo ""
    echo "Production options (only with -p):"
    echo "  -i, --input TOPIC       Input topic (default: /velodyne_points)"
    echo "  -o, --output TOPIC      Output topic (default: /downsampled_points)"
    echo "  -m, --method METHOD     Downsampling method: voxel, uniform, random (default: voxel)"
    echo "  -s, --voxel-size SIZE   Voxel size for voxel downsampling (default: 0.1)"
    echo ""
    echo "Examples:"
    echo "  $0 --dev                # Run development container"
    echo "  $0 --rviz               # Run RViz"
    echo "  $0 --production         # Run production downsampler"
    echo "  $0 --build --dev        # Build and run development container"
    echo ""
    echo "In development container, you can run any ROS2 commands:"
    echo "  ros2 launch lidar_simulator lidar_simulator.launch.py"
    echo "  ros2 run lidar_simulator lidar_simulator"
    echo "  ros2 topic list"
    echo "  rviz2"
}

# Default values
DEV_MODE=false
RVIZ=false
PRODUCTION=false
BUILD=false
INPUT_TOPIC="/velodyne_points"
OUTPUT_TOPIC="/downsampled_points"
METHOD="voxel"
VOXEL_SIZE="0.1"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -d|--dev)
            DEV_MODE=true
            shift
            ;;
        -r|--rviz)
            RVIZ=true
            shift
            ;;
        -p|--production)
            PRODUCTION=true
            shift
            ;;
        -b|--build)
            BUILD=true
            shift
            ;;
        -i|--input)
            INPUT_TOPIC="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_TOPIC="$2"
            shift 2
            ;;
        -m|--method)
            METHOD="$2"
            shift 2
            ;;
        -s|--voxel-size)
            VOXEL_SIZE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option $1"
            show_help
            exit 1
            ;;
    esac
done

# Default to dev mode if no mode specified
if [ "$DEV_MODE" = false ] && [ "$RVIZ" = false ] && [ "$PRODUCTION" = false ]; then
    DEV_MODE=true
fi

# Build if requested
if [ "$BUILD" = true ]; then
    echo "Building Docker images..."
    ./scripts/build.sh
fi

# Allow X11 forwarding
xhost +local:docker > /dev/null 2>&1

# Export environment variables for docker-compose
export INPUT_TOPIC
export OUTPUT_TOPIC
export METHOD
export VOXEL_SIZE

# Export user information for docker-compose
export USER=$(whoami)
export UID=$(id -u)
export GID=$(id -g)

echo "Building/running with user: $USER ($UID:$GID)"

# Suppress pkg_resources deprecation warning
export PYTHONWARNINGS="ignore::DeprecationWarning"

# Use docker compose (newer version) if available, fallback to docker-compose
if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    DOCKER_COMPOSE_CMD="docker compose"
else
    DOCKER_COMPOSE_CMD="docker-compose"
fi

echo "Using: $DOCKER_COMPOSE_CMD"

# Change to docker directory for docker-compose
cd docker

if [ "$DEV_MODE" = true ]; then
    echo "Starting ROS2 development container..."
    echo "Volume mounted: $PROJECT_DIR/src -> /ros2_ws/src"
    echo ""
    echo "Inside the container, you can run:"
    echo "  ros2 launch lidar_simulator lidar_simulator.launch.py"
    echo "  ros2 run lidar_simulator lidar_simulator"
    echo "  rviz2"
    echo ""
    $DOCKER_COMPOSE_CMD run --rm ros2-dev
elif [ "$RVIZ" = true ]; then
    echo "Starting RViz container..."
    $DOCKER_COMPOSE_CMD run --rm rviz
elif [ "$PRODUCTION" = true ]; then
    echo "Starting production pointcloud downsampler..."
    echo "Input topic: $INPUT_TOPIC"
    echo "Output topic: $OUTPUT_TOPIC"
    echo "Method: $METHOD"
    echo "Voxel size: $VOXEL_SIZE"
    $DOCKER_COMPOSE_CMD up pointcloud-downsampler
fi