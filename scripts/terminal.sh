#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_colored() {
    echo -e "${2}${1}${NC}"
}

show_help() {
    echo "Usage: $0 [SERVICE] [OPTIONS]"
    echo ""
    echo "Open a new terminal in a running docker-compose service"
    echo ""
    echo "Services:"
    echo "  ros2-dev              ROS2 development container (docker-ros2-dev)"
    echo "  rviz                  RViz container (ros2_rviz)"
    echo "  pointcloud-downsampler Production container"
    echo ""
    echo "Options:"
    echo "  -h, --help            Show this help message"
    echo "  -l, --list            List running services"
    echo "  -r, --root            Connect as root user"
    echo "  -w, --workdir PATH    Set working directory (default: /ros2_ws)"
    echo "  -c, --container       Use direct container names instead of services"
    echo ""
    echo "Examples:"
    echo "  $0                    # Connect to ros2-dev service"
    echo "  $0 rviz               # Connect to rviz service"
    echo "  $0 --list             # Show running services"
    echo "  $0 --root             # Connect as root to ros2-dev"
    echo "  $0 --container        # Use container names directly"
    echo ""
}

list_services() {
    cd "$PROJECT_DIR/docker"
    
    print_colored "=== Docker Compose Services Status ===" "$BLUE"
    echo ""
    
    # Export environment variables needed by docker-compose
    export USER=$(whoami)
    export UID=$(id -u)
    export GID=$(id -g)
    export DISPLAY=${DISPLAY:-":0"}
    export PYTHONWARNINGS="ignore::DeprecationWarning"
    
    # Use docker compose or docker-compose
    if command -v docker &> /dev/null && docker compose version &> /dev/null; then
        DOCKER_COMPOSE_CMD="docker compose"
    else
        DOCKER_COMPOSE_CMD="docker-compose"
    fi
    
    # Show service status
    $DOCKER_COMPOSE_CMD ps
    
    echo ""
    print_colored "=== Running Containers ===" "$BLUE"
    docker ps --filter "name=ros2" --filter "name=pointcloud" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
    
    echo ""
    print_colored "Available services:" "$YELLOW"
    echo "  ros2-dev              Development environment (container: docker-ros2-dev)"
    echo "  rviz                  Visualization (container: ros2_rviz)"
    echo "  pointcloud-downsampler Production service"
}

connect_to_service() {
    local service=${1:-"ros2-dev"}
    local workdir=${2:-"/ros2_ws"}
    local as_root=${3:-false}
    local use_container=${4:-false}
    
    cd "$PROJECT_DIR/docker"
    
    # Export environment variables needed by docker-compose
    export USER=$(whoami)
    export UID=$(id -u)
    export GID=$(id -g)
    export DISPLAY=${DISPLAY:-":0"}
    export PYTHONWARNINGS="ignore::DeprecationWarning"
    
    # Use docker compose or docker-compose
    if command -v docker &> /dev/null && docker compose version &> /dev/null; then
        DOCKER_COMPOSE_CMD="docker compose"
    else
        DOCKER_COMPOSE_CMD="docker-compose"
    fi
    
    if [ "$use_container" = true ]; then
        # Use direct container names
        local container_name
        case $service in
            "ros2-dev"|"dev"|"development")
                container_name="docker-ros2-dev"
                ;;
            "rviz"|"viz")
                container_name="ros2_rviz"
                ;;
            "pointcloud-downsampler"|"prod"|"production")
                container_name="pointcloud_downsampler"
                ;;
            *)
                container_name="$service"
                ;;
        esac
        
        # Check if container is running
        if ! docker ps --format "{{.Names}}" | grep -q "^${container_name}$"; then
            print_colored "❌ Container '$container_name' is not running" "$RED"
            echo ""
            print_colored "Available containers:" "$YELLOW"
            docker ps --filter "name=ros2" --filter "name=pointcloud" --format "{{.Names}}"
            return 1
        fi
        
        print_colored "🚀 Opening terminal in container: $container_name" "$GREEN"
        
        # Prepare user options
        local exec_options=""
        if [ "$as_root" = true ]; then
            exec_options="--user root"
            print_colored "👑 Connecting as root user" "$YELLOW"
        fi
        
        # Connect to the container directly
        docker exec -it $exec_options --workdir="$workdir" "$container_name" /bin/bash
        
    else
        # Use docker-compose service names
        
        # Check if service is running
        if ! $DOCKER_COMPOSE_CMD ps | grep -q "$service.*Up"; then
            print_colored "❌ Service '$service' is not running" "$RED"
            echo ""
            print_colored "Start it first with:" "$YELLOW"
            case $service in
                "ros2-dev")
                    echo "  ./scripts/run.sh --dev"
                    ;;
                "rviz")
                    echo "  ./scripts/run.sh --rviz"
                    ;;
                "pointcloud-downsampler")
                    echo "  ./scripts/run.sh --production"
                    ;;
            esac
            echo ""
            print_colored "Or try using container mode:" "$YELLOW"
            echo "  $0 --container $service"
            return 1
        fi
        
        print_colored "🚀 Opening terminal in service: $service" "$GREEN"
        
        # Prepare user and working directory options
        local exec_options=""
        if [ "$as_root" = true ]; then
            exec_options="--user root"
            print_colored "👑 Connecting as root user" "$YELLOW"
        fi
        
        # Connect to the service
        $DOCKER_COMPOSE_CMD exec $exec_options --workdir="$workdir" "$service" /bin/bash
    fi
}

# Default values
SERVICE="ros2-dev"
WORKDIR="/ros2_ws"
AS_ROOT=false
LIST_SERVICES=false
USE_CONTAINER=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -l|--list)
            LIST_SERVICES=true
            shift
            ;;
        -r|--root)
            AS_ROOT=true
            shift
            ;;
        -c|--container)
            USE_CONTAINER=true
            shift
            ;;
        -w|--workdir)
            WORKDIR="$2"
            shift 2
            ;;
        ros2-dev|dev|development)
            SERVICE="ros2-dev"
            shift
            ;;
        rviz|viz)
            SERVICE="rviz"
            shift
            ;;
        -*)
            echo "Unknown option $1"
            show_help
            exit 1
            ;;
        *)
            SERVICE="$1"
            shift
            ;;
    esac
done

# Handle list services
if [ "$LIST_SERVICES" = true ]; then
    list_services
    exit 0
fi

# Connect to the service
connect_to_service "$SERVICE" "$WORKDIR" "$AS_ROOT" "$USE_CONTAINER"