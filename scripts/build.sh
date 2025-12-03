#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Change to project directory
cd "$PROJECT_DIR"

# Build Docker images
echo "Building Docker images for ros2..."

echo "Building productiloon image..."
docker build -t ros2:latest -f docker/Dockerfile .

echo "Building development image..."
docker build -t ros2:dev -f docker/Dockerfile.dev .

echo "Build complete!"
echo ""
echo "Usage:"
echo "  Production: ./scripts/run.sh"
echo "  Development: ./scripts/run.sh --dev"
echo "  With RViz: ./scripts/run.sh --rviz"
echo "  Help: ./scripts/run.sh --help"