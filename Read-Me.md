# ROS 2 Workspace Docker Setup

This project provides a Docker setup for a ROS 2 workspace, allowing for consistent development and deployment environments.

## Prerequisites

- Docker installed on your system.

## Setup

1. **Build the Docker Image:**
   Navigate to the root of your ROS 2 workspace and build the Docker image:

   ```bash
   docker build -t ros2_ws_image .
   ```

   This command builds an image named `ros2_ws_image` based on the `Dockerfile` in your current directory.

2. **Run the Docker Container:**
   You can run an interactive container from the built image:

   ```bash
   docker run -it --rm ros2_ws_image
   ```

   - `-it`: Runs the container in interactive mode with a pseudo-TTY.
   - `--rm`: Automatically removes the container when it exits.

   To mount your local workspace into the container for development:

   ```bash
   docker run -it --rm -v $(pwd):/ros2_ws ros2_ws_image
   ```

   - `-v $(pwd):/ros2_ws`: Mounts your current host directory (your ROS 2 workspace) into the `/ros2_ws` directory inside the container. This allows you to edit files on your host and have them reflected in the container.

## Usage within the Container

Once inside the Docker container, you can use standard ROS 2 commands:

- **Source ROS 2 environment:**
  ```bash
  source /opt/ros/humble/setup.bash # or your ROS 2 distribution
  ```

- **Build your workspace:**
  ```bash
  cd /ros2_ws
  colcon build
  ```

- **Source your workspace:**
  ```bash
  source install/setup.bash
  ```

- **Run a ROS 2 node:**
  ```bash
  ros2 run <package_name> <executable_name>
  ```

## Scripts

This workspace includes several utility scripts to streamline development and execution:

### Core Development Scripts

#### **`scripts/build.sh`** - Workspace Build Automation
Comprehensive build script that handles the complete ROS 2 workspace build process.

```bash
./scripts/build.sh [options]
```

**Features:**
- Cleans previous build artifacts (`build/`, `install/`, `log/`)
- Runs `colcon build` with optimized flags
- Automatically sources the workspace setup
- Supports build types: `Debug`, `Release`, `RelWithDebInfo`
- Parallel compilation based on CPU cores

**Options:**
- `--clean`: Force clean build (removes all build directories)
- `--release`: Build in Release mode (default: Debug)
- `--jobs N`: Use N parallel jobs (default: auto-detect)

**Example:**
```bash
# Standard build
./scripts/build.sh

# Clean release build with 4 jobs
./scripts/build.sh --clean --release --jobs 4
```

#### **`scripts/terminal.sh`** - Multi-Terminal Session Manager
Advanced terminal management script for launching multiple synchronized terminal sessions for complex ROS 2 workflows.

```bash
./scripts/terminal.sh [session_name]
```

**Features:**
- Creates tmux/screen sessions with pre-configured layouts
- Automatically sources ROS 2 and workspace environments
- Launches common terminal setups (SLAM + visualization + monitoring)
- Session persistence and restoration
- Customizable terminal layouts for different workflows

**Available Sessions:**
- `slam`: 4-panel layout (roscore, slam_node, rviz, monitoring)
- `dev`: Development layout (build, test, logs, interactive)
- `debug`: Debugging layout (gdb terminals, log viewers)
- `demo`: Demonstration layout (nodes, visualization, data recording)

**Example:**
```bash
# Launch SLAM development session
./scripts/terminal.sh slam

# Create custom development session
./scripts/terminal.sh dev

# Attach to existing session
./scripts/terminal.sh --attach slam
```

#### **`scripts/run.sh`** - Universal Node Launcher
Main execution script that provides a unified interface for launching various ROS 2 nodes and applications.

```bash
./scripts/run.sh <command> [arguments]
```

**Available Commands:**

**SLAM Operations:**
```bash
# Launch complete SLAM pipeline
./scripts/run.sh slam [config_file]

# Run mapping only
./scripts/run.sh mapping --input /scan --output /map

# Start localization
./scripts/run.sh localization --map map.yaml
```

**Downsampling Operations:**
```bash
# Point cloud downsampling
./scripts/run.sh downsample --input /velodyne_points --output /downsampled --rate 0.1

# Voxel grid filtering
./scripts/run.sh voxel_filter --leaf-size 0.05 --topic /points
```

**Data Processing:**
```bash
# Process bag file
./scripts/run.sh bag_process input.bag --output processed/

# Record data
./scripts/run.sh record --topics "/scan,/odom,/tf" --duration 300
```

**Visualization:**
```bash
# Launch RViz with config
./scripts/run.sh rviz [config.rviz]

# Start PlotJuggler for data analysis
./scripts/run.sh plot --bag data.bag
```

**System Operations:**
```bash
# System monitoring
./scripts/run.sh monitor --cpu --memory --topics

# Network diagnostics
./scripts/run.sh diagnostics --network --nodes
```

**Examples:**
```bash
# Complete SLAM demonstration
./scripts/run.sh slam config/demo.yaml

# Real-time point cloud processing
./scripts/run.sh downsample --input /velodyne_points --output /filtered_points --rate 0.2

# Multi-node launch with monitoring
./scripts/run.sh slam & ./scripts/run.sh monitor --topics

# Record and process workflow
./scripts/run.sh record --topics "/scan,/odom" --duration 120
./scripts/run.sh bag_process latest_recording.bag --output results/
```

**Script Options:**
- `--help`: Show detailed help for any command
- `--verbose`: Enable detailed logging
- `--config <file>`: Use custom configuration file
- `--dry-run`: Show what would be executed without running

### Integration Examples

**Complete Development Workflow:**
```bash
# 1. Build the workspace
./scripts/build.sh --release

# 2. Launch development terminals
./scripts/terminal.sh dev

# 3. In terminal 1: Start SLAM
./scripts/run.sh slam config/indoor.yaml

# 4. In terminal 2: Start downsampling
./scripts/run.sh downsample --input /scan --rate 0.1

# 5. In terminal 3: Monitor performance
./scripts/run.sh monitor --topics --cpu
```

**Automated Testing Pipeline:**
```bash
# Build and test
./scripts/build.sh --clean
./scripts/run.sh test --all --coverage

# Record test data
./scripts/run.sh record --topics "/all" --duration 60

# Process and analyze
./scripts/run.sh bag_process test_data.bag
```
