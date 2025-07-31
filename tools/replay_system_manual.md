# nagaspilot Replay System - User Manual

## Overview

The nagaspilot Replay System is a comprehensive C++ and Python-based toolkit for simulating and analyzing vehicle driving sessions by replaying recorded messages from real openpilot drives. It provides high-performance message streaming, multi-camera support, and advanced timeline analysis for debugging, development, and validation workflows.

**🎬 System Types: REPLAY + STREAMING + DEVELOPMENT**
- **✅ Post-Processing**: Replay pre-recorded route data with full fidelity
- **✅ Streaming**: Real-time message publication to other tools
- **✅ Development**: Debug and validate system components
- **✅ Analysis**: Timeline visualization and event correlation

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Installation & Setup](#installation--setup)
3. [Command Line Interface](#command-line-interface)
4. [Route Handling & Segments](#route-handling--segments)
5. [WiFi & Network Setup](#wifi--network-setup)
6. [Multi-Camera Support](#multi-camera-support)
7. [Service Filtering & Control](#service-filtering--control)
8. [Memory Management & Caching](#memory-management--caching)
9. [Integration with Other Tools](#integration-with-other-tools)
10. [Performance Optimization](#performance-optimization)
11. [Troubleshooting](#troubleshooting)
12. [API Reference](#api-reference)

## System Architecture

### Core Components

**C++ Core Engine**
- **replay.cc/replay.h** - Main replay orchestration and event streaming
- **route.cc/route.h** - Route parsing, validation, and segment management
- **seg_mgr.cc/seg_mgr.h** - Segment loading, caching, and event merging
- **camera.cc/camera.h** - Multi-camera video stream handling
- **timeline.cc/timeline.h** - Event timeline construction and analysis

**Python Integration Layer**
- **ui.py** - Real-time visualization interface with pygame
- **can_replay.py** - Hardware CAN message replay via Panda Jungle
- **rp_visualization.py** - Rerun-based radar point visualization
- **lib/rp_helpers.py** - UI rendering utilities and color palettes
- **lib/ui_helpers.py** - Advanced UI components and plot management

**Console Interface**
- **consoleui.cc/consoleui.h** - ncurses-based terminal interface
- **main.cc** - Command-line argument parsing and application entry point

### Data Flow Architecture

#### Standard Replay Mode
```
Route Specification → Route Parser → Segment Manager → Event Loader → 
Message Publisher → ZMQ/MSGQ → Consumer Applications
```

#### Multi-Camera Mode
```
Video Segments → Camera Server → Frame Decoder → VisionIPC → 
Vision Clients (UI, Analysis Tools)
```

#### Timeline Analysis Mode
```
Log Data → Timeline Builder → Event Correlation → Alert Detection → 
Console UI Display
```

#### Network Streaming Mode
```
Local Replay → ZMQ Bridge → Network Transport → Remote Consumers
```

### Integration Points
- **cereal messaging** - All openpilot message types supported
- **VisionIPC** - Camera frame distribution to vision clients
- **ZMQ/MSGQ** - Configurable message transport protocols
- **Route API** - Integration with comma.ai route infrastructure
- **Local Storage** - Support for offline route analysis

## Installation & Setup

### Prerequisites
```bash
# Core dependencies (typically pre-installed with nagaspilot)
sudo apt-get install libncurses5-dev libncurses5 ncurses-dev
sudo apt-get install libusb-1.0-0-dev  # For CAN replay via Panda Jungle

# Python dependencies for UI components
pip install pygame numpy opencv-python rerun-sdk

# Build replay system
cd tools/replay
scons .
```

### Authentication Setup
```bash
# Required for downloading routes from comma.ai servers
cd tools/lib
python3 auth.py

# Follow prompts to authenticate with your comma account
# This creates ~/.comma/auth.json with API credentials
```

### Build Verification
```bash
# Test build
cd tools/replay
./replay --help

# Should display comprehensive help text
# If missing dependencies, build will fail with specific errors
```

### Environment Configuration
```bash
# Optional: Set up environment variables
export OPENPILOT_PREFIX="/path/to/custom/prefix"  # Custom data location
export FILEREADER_CACHE=1  # Enable file caching for better performance
export ZMQ=1  # Use ZMQ instead of MSGQ for networking
```

## Command Line Interface

### Basic Usage Patterns

#### Quick Demo
```bash
# Start with built-in demo route
cd tools/replay
./replay --demo

# Demo with specific options
./replay --demo --dcam --ecam  # Include all cameras
./replay --demo --no-loop      # Stop at end instead of looping
```

#### Remote Route Replay
```bash
# Full route (all segments)
./replay "a2a0ccea32023010|2023-07-27--13-01-19"

# Single segment (faster loading)
./replay "a2a0ccea32023010|2023-07-27--13-01-19--5"

# Segment range (segments 2-7)
./replay "a2a0ccea32023010|2023-07-27--13-01-19" -s 120 -x 2.0
```

#### Local Route Replay
```bash
# Replay from local directory
./replay "route_name" --data_dir="/path/to/local/routes"

# Example with full path
./replay "a2a0ccea32023010|2023-07-27--13-01-19" \
  --data_dir="/data/media/0/realdata"
```

### Complete Command Reference

```bash
./replay [options] <route>

Required Arguments:
  route                  Route identifier or --demo for demo route
                        Formats:
                        - "dongle_id|timestamp" (full route)
                        - "dongle_id|timestamp--N" (segment N)
                        - cabana.comma.ai URLs also accepted

Core Options:
  -a, --allow <services>    Whitelist of services (comma-separated)
  -b, --block <services>    Blacklist of services (comma-separated)
  -s, --start <seconds>     Start playback from specific time
  -x, --playback <speed>    Playback speed (0.2 to 8.0)
  -c, --cache <n>           Cache N segments in memory (default: 5)

Data Source Options:
  --demo                    Use built-in demo route
  --auto                    Auto-detect best data source
  -d, --data_dir <path>     Local directory with route data
  --no-cache               Disable local file caching

Camera Options:
  --dcam                   Enable driver camera stream
  --ecam                   Enable wide road camera stream  
  --qcam                   Enable qcamera stream
  --no-vipc                Disable video output entirely
  --no-hw-decoder          Force software video decoding

Behavior Options:
  --no-loop                Stop at route end (don't loop)
  --all                    Include all services (uiDebug, userFlag)
  -p, --prefix <path>      Set OPENPILOT_PREFIX override
```

### Advanced Usage Examples

#### Development & Debugging
```bash
# Replay specific services only
./replay "route" --allow "carState,controlsState,modelV2"

# Block problematic services
./replay "route" --block "uiDebug,userFlag,driverCameraState"

# High-speed analysis (4x speed)
./replay "route" -x 4.0 --no-vipc

# Memory-constrained replay (cache only 2 segments)
./replay "route" -c 2
```

#### Multi-Camera Analysis
```bash
# All cameras for comprehensive analysis
./replay "route" --dcam --ecam --qcam

# Driver monitoring focus
./replay "route" --dcam --block "roadCameraState,wideRoadCameraState"

# Road camera only (default behavior)
./replay "route"  # Only road camera enabled
```

#### Network Streaming Setup
```bash
# Enable ZMQ for network streaming
ZMQ=1 ./replay "route"

# Custom prefix for isolated testing
./replay "route" --prefix "/tmp/test_replay"

# Start from specific time with custom speed
./replay "route" -s 300 -x 0.5  # Start at 5 minutes, half speed
```

## Route Handling & Segments

### Route Specification Formats

#### Standard Formats
```bash
# Full route with dongle ID
"a2a0ccea32023010|2023-07-27--13-01-19"

# Single segment
"a2a0ccea32023010|2023-07-27--13-01-19--3"

# Segment range (legacy format)
"a2a0ccea32023010|2023-07-27--13-01-19/2:5"

# Cabana share URLs (automatically parsed)
"https://cabana.comma.ai/share/route_id"
```

#### Auto-Source Mode
```bash
# Let system find best data source
./replay "route" --auto

# Sources checked in order:
# 1. Internal development routes
# 2. openpilotci test routes  
# 3. comma.ai API routes
# 4. car_segments local cache
# 5. testing_closet archives
```

### Segment Management

#### Cache Configuration
```bash
# Default: 5 segments cached
./replay "route"

# High-memory systems: cache more segments
./replay "route" -c 20

# Memory-constrained: minimal cache
./replay "route" -c 2  # Minimum allowed
```

#### Segment Loading Behavior
- **Preloading**: System loads current + next segments proactively
- **Background Loading**: Additional segments loaded in background
- **Cache Eviction**: LRU eviction when cache limit exceeded
- **Error Handling**: Automatic retry for failed segment downloads

#### Local vs Remote Segments
```bash
# Check segment availability
ls /data/media/0/realdata/  # Local comma device storage
ls ~/.comma/remote_cache/   # Downloaded remote segments

# Force local-only replay (no downloads)
./replay "route" --data_dir="/data/media/0/realdata" --no-cache
```

### Route Validation & Parsing

#### Route Format Validation
```cpp
// Regex pattern used internally:
// ^(([a-z0-9]{16})[|_/])?(.{20})((--|/)((-?\d+(:(-?\d+)?)?)|(:-?\d+)))?$

// Components:
// - dongle_id: 16 character hex string
// - timestamp: 20 character timestamp (YYYY-MM-DD--HH-MM-SS)
// - segment: optional segment specification
```

#### Error Handling
```bash
# Invalid route format
./replay "invalid_route"
# Error: invalid route format

# Missing segments
./replay "a2a0ccea32023010|2023-07-27--13-01-19--999"
# Error: Failed to load segments

# Authentication required
./replay "private_route"  
# Error: Authentication failed - run tools/lib/auth.py
```

## WiFi & Network Setup

### ZMQ Network Streaming

#### Basic Network Streaming
```bash
# Enable ZMQ mode
export ZMQ=1
./replay "route"

# Or inline
ZMQ=1 ./replay "route"
```

#### Remote Device Connection

**Step 1: Device Hotspot Setup**
```bash
# On comma device (via SSH or direct access):
# 1. Settings → Network → WiFi Hotspot
# 2. Enable hotspot: "comma-XXXXX"
# 3. Note password from device settings
# 4. Verify broadcast: iwconfig wlan0
```

**Step 2: PC Connection**
```bash
# Connect PC to device hotspot
# Network: "comma-XXXXX"
# Password: [from device settings]

# Verify connection
ping 192.168.43.1  # Default comma device IP
```

**Step 3: Bridge Setup (Device Side)**
```bash
# SSH into comma device
ssh root@192.168.43.1
# Default password: typically blank or "comma"

# Start message bridge
cd /data/openpilot/cereal/messaging
./bridge &

# Verify bridge running
ps aux | grep bridge
netstat -tuln | grep 8086  # ZMQ port check
```

**Step 4: Client Replay (PC Side)**
```bash
# Start replay with network output
ZMQ=1 ./replay "route"

# In another terminal, verify streaming
ZMQ=1 python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['carState'])
while True:
    sm.update(1000)
    if sm.updated['carState']:
        print(f'Speed: {sm[\"carState\"].vEgo:.1f} m/s')
"
```

### Advanced Network Configuration

#### Multi-Device Setup
```bash
# Device 1: Default configuration
ZMQ=1 ./replay "route1"

# Device 2: Custom IP (if using different hotspot)
DEVICE_IP=192.168.44.1 ZMQ=1 ./replay "route2"
```

#### Custom ZMQ Configuration
```bash
# Custom ZMQ ports (advanced)
ZMQ_PORT=8087 ZMQ=1 ./replay "route"

# Bind to specific interface
ZMQ_BIND_ADDR=192.168.43.100 ZMQ=1 ./replay "route"
```

#### Network Performance Optimization
```bash
# Bandwidth-limited connections
./replay "route" --block "roadCameraState,driverCameraState" --no-vipc

# High-frequency data only
./replay "route" --allow "carState,controlsState,modelV2"

# Minimal data for basic analysis
./replay "route" --allow "carState" -c 2
```

### SSH & Remote Access

#### SSH Connection Troubleshooting
```bash
# Verbose SSH debugging
ssh -v root@192.168.43.1

# Common connection issues:
# 1. Wrong IP address
ip route show  # Check device IP assignment

# 2. SSH not enabled on device  
# Check: Settings → Developer → SSH enabled

# 3. Network connectivity
ping -c 3 192.168.43.1
telnet 192.168.43.1 22  # SSH port check

# 4. Firewall blocking
# Check device: iptables -L
```

#### Alternative Connection Methods
```bash
# USB tethering (if WiFi fails)
# 1. Connect device via USB
# 2. Enable USB tethering in device settings
# 3. Check IP: ip addr show usb0
# 4. SSH to USB IP instead of WiFi IP

# Direct device access (if available)
# 1. Connect keyboard/mouse to device
# 2. Open terminal directly
# 3. Run bridge locally: ./cereal/messaging/bridge
```

## Multi-Camera Support

### Camera Stream Types

#### Road Camera (Default)
```bash
# Standard road camera (always enabled)
./replay "route"
# Streams: roadCameraState messages
# VisionIPC: VISION_STREAM_ROAD
# Resolution: 1928x1208 (device-dependent)
```

#### Driver Camera
```bash
# Enable driver monitoring camera
./replay "route" --dcam
# Streams: driverCameraState messages  
# VisionIPC: VISION_STREAM_DRIVER
# Resolution: 1928x1208
# Use case: Driver monitoring analysis
```

#### Wide Road Camera  
```bash
# Enable wide-angle road camera
./replay "route" --ecam
# Streams: wideRoadCameraState messages
# VisionIPC: VISION_STREAM_WIDE_ROAD  
# Resolution: 1928x1208
# Use case: Peripheral vision analysis
```

#### QCamera Support
```bash
# Enable QCamera (if available)
./replay "route" --qcam
# Streams: qRoadCameraState messages
# VisionIPC: VISION_STREAM_QCAM
# Resolution: Lower resolution variant
```

### Multi-Camera Usage Patterns

#### All Cameras Simultaneously
```bash
# Enable all available cameras
./replay "route" --dcam --ecam --qcam

# Watch all streams with watch3 tool
cd selfdrive/ui && ./watch3
```

#### Camera-Specific Analysis
```bash
# Driver monitoring focus
./replay "route" --dcam --block "roadCameraState,wideRoadCameraState"

# Wide-angle perception analysis  
./replay "route" --ecam --allow "wideRoadCameraState,modelV2"

# Standard road vision only (default)
./replay "route"  # Only road camera
```

#### Performance Considerations
```bash
# Disable video for CPU-limited systems
./replay "route" --dcam --ecam --no-vipc

# Software decoding (if hardware decoder issues)
./replay "route" --dcam --ecam --no-hw-decoder

# Minimal video processing
./replay "route" --qcam  # Lowest resolution option
```

### Video Integration

#### VisionIPC Client Setup
```python
# Python client for camera streams
from msgq.visionipc import VisionIpcClient, VisionStreamType

# Road camera
road_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
road_client.connect(True)

# Driver camera (requires --dcam)
driver_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)

# Wide road camera (requires --ecam) 
wide_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, True)
```

#### Frame Processing Example
```python
# Receive and process frames
yuv_frame = road_client.recv()
if yuv_frame is not None and yuv_frame.data.any():
    # Convert YUV to RGB
    rgb_frame = cv2.cvtColor(
        yuv_frame.data.reshape((height*3//2, width)), 
        cv2.COLOR_YUV2RGB_NV12
    )
    # Process frame...
```

## Service Filtering & Control

### Service Categories

#### Vehicle State Services
```bash
# Core vehicle dynamics
--allow "carState,carControl,controlsState"

# Extended vehicle information  
--allow "carState,carControl,controlsState,carParams,carEvents"

# Steering and control systems
--allow "controlsState,lateralPlan,longitudinalPlan,torqued"
```

#### Perception Services
```bash
# Vision and ML models
--allow "modelV2,driverState,driverMonitoringState"

# Radar and tracking
--allow "radarState,liveTracks,liveParameters"

# Camera and calibration
--allow "liveCalibration,roadCameraState,driverCameraState"
```

#### System Health Services
```bash
# Device monitoring
--allow "thermal,deviceState,pandaStates"

# System status
--allow "selfdriveState,managerState,procLog"

# Location and navigation
--allow "gpsLocationExternal,liveLocationKalman,navInstruction"
```

### Filtering Strategies

#### Development Workflows
```bash
# Control system debugging
./replay "route" --allow "carState,controlsState,lateralPlan,longitudinalPlan"

# Perception system analysis
./replay "route" --allow "modelV2,radarState,liveTracks,liveCalibration"

# System health monitoring
./replay "route" --allow "thermal,deviceState,selfdriveState"
```

#### Performance Optimization
```bash
# Block high-bandwidth services
./replay "route" --block "roadCameraState,driverCameraState,wideRoadCameraState"

# Block debug services (default behavior)
./replay "route"  # Automatically blocks uiDebug, userFlag

# Include all services (may impact performance)
./replay "route" --all
```

#### Service Dependencies
```bash
# Some analyses require multiple services:

# Lateral control analysis needs:
--allow "carState,controlsState,lateralPlan,liveParameters,steerTorque"

# Longitudinal control analysis needs:  
--allow "carState,controlsState,longitudinalPlan,radarState,liveTracks"

# Model performance analysis needs:
--allow "modelV2,liveCalibration,roadCameraState,liveTracks"
```

### Service List Reference

#### Always Included Services
```bash
# These services are always included regardless of filters:
- initData      # Route initialization data
- carParams     # Vehicle-specific parameters  
- logMonoTime   # Timestamp synchronization
```

#### High-Bandwidth Services
```bash
# These consume significant network/CPU resources:
- roadCameraState
- driverCameraState  
- wideRoadCameraState
- qRoadCameraState
- encodeIdx
- thumbnail
```

#### Debug-Only Services
```bash
# Typically blocked unless --all specified:
- uiDebug
- userFlag
- testJoystick
- manualRestart
```

## Memory Management & Caching

### Segment Cache Configuration

#### Cache Size Tuning
```bash
# Default cache (5 segments ≈ 5 minutes)
./replay "route"  # Default: 5 segments

# Memory-abundant systems
./replay "route" -c 20  # Cache 20 segments (≈ 20 minutes)

# Memory-constrained systems
./replay "route" -c 2   # Minimum cache (≈ 2 minutes)
```

#### Memory Usage Estimates
```bash
# Approximate memory usage per segment:
# - Basic services only: ~100-200MB per segment
# - With cameras: ~500MB-1GB per segment  
# - All services: ~1-2GB per segment

# Calculate total cache memory:
# Total Memory = Cache Size × Per-Segment Memory
# Example: 10 segments × 500MB = ~5GB total
```

#### Cache Behavior
- **Preloading**: Current segment + next segment always loaded
- **Background Loading**: Additional segments loaded in background thread
- **LRU Eviction**: Least recently used segments evicted when limit reached
- **Error Recovery**: Failed segments automatically retried with exponential backoff

### Local File Caching

#### Cache Locations
```bash
# Downloaded route cache
~/.comma/remote_cache/  # Remote segments cached here

# Local device storage
/data/media/0/realdata/  # On comma device
/tmp/replay_cache/       # Temporary processing files
```

#### Cache Management
```bash
# Enable file caching (default)
export FILEREADER_CACHE=1
./replay "route"

# Disable file caching (stream directly)
./replay "route" --no-cache

# Clear cache manually
rm -rf ~/.comma/remote_cache/*
```

#### Cache Performance Optimization
```bash
# Use SSD storage for cache (if available)
export TMPDIR="/path/to/ssd/tmp"
./replay "route"

# Pre-download segments for faster access
# (Use comma-tools to pre-cache routes)
comma download "route_name"
```

### Memory Monitoring

#### Resource Usage Tracking
```bash
# Monitor replay memory usage
top -p $(pgrep replay)
htop -p $(pgrep replay)

# Memory breakdown:
# - Segment cache: Largest component
# - Video buffers: Significant with cameras enabled
# - Message queues: Small but grows with --all
# - System overhead: ~100-200MB baseline
```

#### Memory Optimization Strategies
```bash
# Reduce cache size for memory-limited systems
./replay "route" -c 2

# Disable video to save memory
./replay "route" --no-vipc

# Use service filtering to reduce memory footprint
./replay "route" --allow "carState,controlsState"

# Process segments sequentially (no cache)
./replay "route" -c 2 --no-loop
```

### Storage Requirements

#### Disk Space Planning
```bash
# Typical segment sizes:
# - Log data only: ~10-50MB per segment
# - With video: ~100-500MB per segment
# - High resolution/quality: ~500MB-1GB per segment

# Full route storage requirements:
# - Short drive (10 segments): ~1-5GB
# - Long drive (50+ segments): ~5-25GB
# - High-quality routes: ~25-50GB
```

#### Storage Optimization
```bash
# Stream without local storage
./replay "route" --no-cache

# Use temporary storage
export TMPDIR="/tmp"
./replay "route"

# Compress old cache files
find ~/.comma/remote_cache -name "*.bz2" -exec bzip2 {} \;
```

## Integration with Other Tools

### PlotJuggler Integration

#### Basic Streaming Setup
```bash
# Terminal 1: Start replay
cd tools/replay
./replay "route"

# Terminal 2: Start PlotJuggler streaming
cd tools/plotjuggler
./juggle.py --stream
```

#### Advanced PlotJuggler Setup
```bash
# Stream specific services to PlotJuggler
./replay "route" --allow "carState,controlsState,modelV2"

# Network streaming to remote PlotJuggler
ZMQ=1 ./replay "route"
# Then connect PlotJuggler from remote machine
```

#### PlotJuggler Configuration
```python
# In PlotJuggler interface:
# 1. Click "Streaming" dropdown
# 2. Select "Cereal Subscriber" plugin
# 3. Configure connection (local: 127.0.0.1, remote: device IP)
# 4. Click "Start" - data flows immediately
```

### UI Integration

#### Real-Time UI Visualization
```bash
# Terminal 1: Start replay
./replay "route"

# Terminal 2: Start UI
cd selfdrive/ui && ./ui
```

#### Custom UI Scripts
```bash
# Start replay
./replay "route"

# Run custom Python UI
python3 tools/replay/ui.py
```

#### Multi-Camera UI (watch3)
```bash
# Start replay with all cameras
./replay "route" --dcam --ecam

# Start watch3 for three-camera view
cd selfdrive/ui && ./watch3
```

### Rerun Visualization

#### Radar Point Visualization
```bash
# Terminal 1: Start replay  
./replay "route"

# Terminal 2: Start Rerun visualization
python3 tools/replay/rp_visualization.py

# Rerun viewer opens automatically showing radar points and model output
```

#### Custom Rerun Scripts
```python
# Example Rerun integration
import rerun as rr
import cereal.messaging as messaging

rr.init("replay_analysis")
rr.connect()

sm = messaging.SubMaster(['modelV2', 'radarState'])
while True:
    sm.update(100)
    if sm.updated['modelV2']:
        # Log model predictions to Rerun
        rr.log("model/predictions", rr.Points3D(model_points))
```

### CAN Hardware Integration

#### Panda Jungle Setup
```bash
# Connect Panda Jungle to PC via USB
# Connect comma device to Jungle via OBD-C

# Replay CAN messages to hardware
python3 tools/replay/can_replay.py "route_name"
```

#### CAN Replay Configuration
```bash
# Environment variables for CAN replay:
export PWR_ON=30    # Power on duration (seconds)
export PWR_OFF=5    # Power off duration (seconds) 
export ON=60        # Ignition on duration (seconds)
export OFF=10       # Ignition off duration (seconds)

# Start hardware CAN replay
python3 tools/replay/can_replay.py "route_name"
```

### Development Integration

#### Component Testing
```bash
# Test control system against replay
./replay "route" --allow "carState,modelV2" &
cd selfdrive/controls && python3 controlsd.py

# Test radar processing
./replay "route" --allow "radarState,liveTracks" &
cd selfdrive/controls && python3 radard.py
```

#### CI/CD Integration
```bash
# Automated testing with replay
./replay "test_route" --no-loop --cache 2 &
REPLAY_PID=$!
python3 run_tests.py
kill $REPLAY_PID
```

## Performance Optimization

### System Requirements

#### Minimum Requirements
- **CPU**: 2+ cores, 2.0GHz
- **RAM**: 4GB (8GB with cameras)
- **Storage**: 10GB free space
- **Network**: 10Mbps for streaming

#### Recommended Configuration
- **CPU**: 4+ cores, 3.0GHz+
- **RAM**: 16GB+
- **Storage**: SSD with 50GB+ free
- **Network**: 100Mbps+ for multi-device

#### High-Performance Configuration
- **CPU**: 8+ cores, 3.5GHz+
- **RAM**: 32GB+
- **Storage**: NVMe SSD with 100GB+
- **Network**: Gigabit Ethernet

### Performance Tuning

#### CPU Optimization
```bash
# Set CPU governor to performance
sudo cpufreq-set -g performance

# Increase process priority
nice -n -10 ./replay "route"

# Use multiple CPU cores effectively
./replay "route" -c 10  # Higher cache allows better parallelization
```

#### Memory Optimization
```bash
# Optimize for available RAM
# 8GB system:
./replay "route" -c 5 --no-vipc

# 16GB system:
./replay "route" -c 10

# 32GB+ system:
./replay "route" -c 20 --dcam --ecam
```

#### Storage Optimization
```bash
# Use SSD for cache
export TMPDIR="/path/to/ssd/tmp"
./replay "route"

# Pre-load frequently used routes
comma download "frequently_used_route"

# Use RAM disk for ultimate performance (Linux)
sudo mount -t tmpfs -o size=8G tmpfs /tmp/ramdisk
export TMPDIR="/tmp/ramdisk"
./replay "route"
```

#### Network Optimization
```bash
# Optimize ZMQ buffer sizes
export ZMQ_SNDHWM=1000  # Send buffer size
export ZMQ_RCVHWM=1000  # Receive buffer size

# Use dedicated network interface for streaming
ZMQ_BIND_ADDR=192.168.1.100 ZMQ=1 ./replay "route"

# Reduce network bandwidth usage
./replay "route" --block "roadCameraState,driverCameraState" --no-vipc
```

### Monitoring & Profiling

#### System Resource Monitoring
```bash
# Monitor CPU usage
top -p $(pgrep replay)

# Monitor memory usage  
watch -n 1 "ps aux | grep replay | grep -v grep"

# Monitor network usage
nethogs -p $(pgrep replay)

# Monitor disk I/O
iotop -p $(pgrep replay)
```

#### Performance Profiling
```bash
# Profile with perf (Linux)
perf record -p $(pgrep replay) sleep 30
perf report

# Memory profiling with valgrind
valgrind --tool=massif ./replay "route"
ms_print massif.out.*
```

#### Bottleneck Identification
```bash
# Common performance bottlenecks:

# 1. Disk I/O (slow storage)
# Solution: Use SSD, increase cache size

# 2. Network bandwidth (streaming)
# Solution: Filter services, use local replay

# 3. CPU usage (video decoding)  
# Solution: Use --no-hw-decoder or --no-vipc

# 4. Memory pressure (large cache)
# Solution: Reduce cache size, filter services
```

## Troubleshooting

### Installation Issues

#### Build Failures
```bash
# Problem: Missing dependencies
# Error: fatal error: ncurses.h: No such file or directory
# Solution:
sudo apt-get install libncurses5-dev ncurses-dev

# Problem: Compiler version
# Error: C++ compiler version too old
# Solution:
sudo apt-get install gcc-9 g++-9
export CXX=g++-9
```

#### Python Dependencies  
```bash
# Problem: Missing Python packages
# Error: ModuleNotFoundError: No module named 'pygame'
# Solution:
pip install pygame numpy opencv-python rerun-sdk

# Problem: Python version incompatibility
# Error: ImportError: cannot import name 'messaging'
# Solution: Use Python 3.8+
python3.8 -m pip install -r requirements.txt
```

### Authentication Issues

#### API Authentication
```bash
# Problem: Cannot download routes
# Error: Authentication failed
# Solution:
cd tools/lib
python3 auth.py
# Follow prompts to authenticate

# Problem: Expired credentials
# Error: 401 Unauthorized
# Solution:
rm ~/.comma/auth.json
python3 tools/lib/auth.py  # Re-authenticate
```

#### SSH Access Issues
```bash
# Problem: Cannot SSH to device
# Error: Connection refused
# Solution:
# 1. Check device SSH enabled: Settings → Developer → SSH
# 2. Verify network: ping 192.168.43.1
# 3. Check SSH service: ssh -v root@192.168.43.1

# Problem: Wrong device IP
# Solution:
ip route show | grep default  # Check gateway IP
nmap -sP 192.168.43.0/24     # Scan for devices
```

### Route Loading Issues

#### Invalid Route Format
```bash
# Problem: Route format not recognized  
# Error: invalid route format
# Solution: Check route string format
# Valid: "a2a0ccea32023010|2023-07-27--13-01-19"
# Invalid: "invalid_route_string"
```

#### Missing Segments
```bash
# Problem: Segments not available
# Error: Failed to load segments
# Solution:
# 1. Check route exists: visit connect.comma.ai
# 2. Verify authentication: python3 tools/lib/auth.py
# 3. Try demo route: ./replay --demo
# 4. Check local data: --data_dir="/path/to/data"
```

#### Download Failures
```bash
# Problem: Network download failures
# Error: Failed to download segment
# Solution:
# 1. Check internet connectivity
# 2. Verify comma.ai server status
# 3. Try smaller cache size: -c 2
# 4. Use --no-cache for streaming only
```

### Runtime Issues

#### High Memory Usage
```bash
# Problem: System running out of memory
# Error: std::bad_alloc or system freeze
# Solution:
# 1. Reduce cache size: -c 2
# 2. Disable video: --no-vipc
# 3. Filter services: --allow "carState,controlsState"
# 4. Close other applications
```

#### Poor Performance
```bash
# Problem: Slow or choppy playback
# Solution:
# 1. Reduce playback speed: -x 0.5
# 2. Use SSD storage for cache
# 3. Close unnecessary applications
# 4. Reduce cache size: -c 5
# 5. Disable hardware decoding: --no-hw-decoder
```

#### Video Issues
```bash
# Problem: No video output or corrupted video
# Solution:
# 1. Check VisionIPC clients connected
# 2. Try software decoding: --no-hw-decoder
# 3. Disable and re-enable: --no-vipc then remove flag
# 4. Check camera availability in route: --dcam --ecam
```

### Network & Streaming Issues

#### ZMQ Connection Problems
```bash
# Problem: No data in streaming mode
# Solution:
# 1. Verify ZMQ=1 environment variable
# 2. Check port availability: netstat -tuln | grep 8086
# 3. Test with local replay first
# 4. Verify firewall settings

# Problem: Connection timeouts
# Solution:
# 1. Check network connectivity: ping device_ip
# 2. Verify bridge running on device: ps aux | grep bridge
# 3. Test different network interface
# 4. Reduce streaming data: --allow "carState"
```

#### Device Connection Issues
```bash
# Problem: Cannot connect to comma device
# Solution:
# 1. Verify device hotspot enabled
# 2. Check PC connected to correct network
# 3. Verify device IP: ip route show
# 4. Test SSH connectivity: ssh root@device_ip
# 5. Check device bridge running: ./cereal/messaging/bridge
```

### Advanced Troubleshooting

#### Debug Mode
```bash
# Enable verbose logging
export OPENPILOT_LOG_LEVEL=DEBUG
./replay "route"

# Check system logs
journalctl -f | grep replay

# Monitor system calls
strace -p $(pgrep replay)
```

#### Common Error Patterns
```bash
# Segmentation fault
# Cause: Usually memory corruption or invalid pointers
# Solution: Check available memory, reduce cache size

# Bus error  
# Cause: Hardware/alignment issues
# Solution: Try --no-hw-decoder, check system stability

# Timeout errors
# Cause: Network or I/O issues
# Solution: Check connections, reduce timeouts

# Permission denied
# Cause: Insufficient file/network permissions
# Solution: Check file permissions, run with sudo if needed
```

## API Reference

### Command Line API

#### Core Arguments
```bash
./replay [options] <route>

Positional Arguments:
  route                    Route identifier string
                          Examples:
                          - "dongle_id|timestamp"  
                          - "dongle_id|timestamp--segment"
                          - "https://cabana.comma.ai/..."
                          - "--demo" for built-in demo

Optional Arguments:
  -h, --help              Show help message and exit
  -a, --allow <services>  Comma-separated whitelist of services
  -b, --block <services>  Comma-separated blacklist of services  
  -c, --cache <n>         Cache <n> segments in memory (default: 5, min: 2)
  -s, --start <seconds>   Start playback from <seconds> into route
  -x, --playback <speed>  Playback speed multiplier (0.2 to 8.0)
  -d, --data_dir <path>   Local directory containing route data
  -p, --prefix <path>     Set OPENPILOT_PREFIX override
```

#### Flag Options
```bash
Behavior Flags:
  --demo                  Use built-in demo route
  --auto                  Auto-detect best data source
  --no-loop              Stop at route end (don't loop back)
  --no-cache             Disable local file caching
  --all                  Include all services (including debug services)

Camera Flags:
  --dcam                 Enable driver camera stream
  --ecam                 Enable wide road camera stream
  --qcam                 Enable qcamera stream
  --no-vipc              Disable all video output
  --no-hw-decoder        Force software video decoding
```

### C++ API

#### Core Classes

**Replay Class**
```cpp
class Replay {
public:
    // Constructor
    Replay(const std::string &route, 
           std::vector<std::string> allow, 
           std::vector<std::string> block,
           SubMaster *sm = nullptr,
           uint32_t flags = REPLAY_FLAG_NONE,
           const std::string &data_dir = "",
           bool auto_source = false);

    // Core methods
    bool load();                              // Load route and initialize
    void start(int seconds = 0);             // Start playback from seconds
    void pause(bool pause);                  // Pause/resume playback
    void seekTo(double seconds, bool relative); // Seek to specific time
    
    // Configuration
    void setSpeed(float speed);              // Set playback speed
    void setSegmentCacheLimit(int n);        // Set cache size
    void setLoop(bool loop);                 // Enable/disable looping
    
    // State queries
    bool isPaused() const;                   // Check if paused
    double currentSeconds() const;           // Current playback position
    float getSpeed() const;                  // Current playback speed
    const std::string &carFingerprint() const; // Vehicle fingerprint
    
    // Event callbacks
    std::function<void()> onSegmentsMerged;
    std::function<void(double)> onSeeking;
    std::function<void(double)> onSeekedTo;
    std::function<void(std::shared_ptr<LogReader>)> onQLogLoaded;
};
```

**Route Class**
```cpp
class Route {
public:
    Route(const std::string &route, 
          const std::string &data_dir = "", 
          bool auto_source = false);
    
    bool load();                             // Load and validate route
    static RouteIdentifier parseRoute(const std::string &str); // Parse route string
    
    // Accessors
    const std::string &name() const;         // Route name
    const SegmentMap &segments() const;      // Available segments
    std::time_t dateTime() const;           // Route timestamp
    RouteLoadError lastError() const;       // Last error status
};
```

**SegmentManager Class**
```cpp
class SegmentManager {
public:
    struct EventData {
        std::vector<Event> events;           // Merged events from segments
        SegmentMap segments;                 // Loaded segments
        bool isSegmentLoaded(int n) const;   // Check segment availability
    };
    
    SegmentManager(const std::string &route_name,
                   uint32_t flags,
                   const std::string &data_dir = "",
                   bool auto_source = false);
    
    bool load();                             // Load segment metadata
    void setCurrentSegment(int seg_num);     // Set active segment
    void setFilters(const std::vector<bool> &filters); // Set service filters
    void setCallback(const std::function<void()> &callback); // Merge callback
    
    const std::shared_ptr<EventData> getEventData() const; // Get current events
    bool hasSegment(int n) const;            // Check segment existence
    
    Route route_;                            // Associated route
    int segment_cache_limit_;                // Cache size limit
};
```

#### Flag Definitions
```cpp
enum REPLAY_FLAGS {
    REPLAY_FLAG_NONE = 0x0000,              // No special flags
    REPLAY_FLAG_DCAM = 0x0002,              // Enable driver camera
    REPLAY_FLAG_ECAM = 0x0004,              // Enable wide road camera
    REPLAY_FLAG_NO_LOOP = 0x0010,           // Disable looping
    REPLAY_FLAG_NO_FILE_CACHE = 0x0020,     // Disable file caching
    REPLAY_FLAG_QCAMERA = 0x0040,           // Enable QCamera
    REPLAY_FLAG_NO_HW_DECODER = 0x0100,     // Force software decoding
    REPLAY_FLAG_NO_VIPC = 0x0400,           // Disable video output
    REPLAY_FLAG_ALL_SERVICES = 0x0800,      // Include debug services
};
```

### Python API

#### UI Module (ui.py)
```python
def ui_thread(addr):
    """Main UI visualization thread
    
    Args:
        addr: IP address for ZMQ connection (default: "127.0.0.1")
    """

def get_arg_parser():
    """Get command line argument parser for UI
    
    Returns:
        argparse.ArgumentParser configured for UI options
    """
```

#### Visualization Module (rp_visualization.py)
```python
def visualize(addr):
    """Main radar point visualization with Rerun
    
    Args:
        addr: IP address for ZMQ connection
    """

def get_arg_parser():
    """Get command line argument parser
    
    Returns:
        argparse.ArgumentParser for visualization options
    """
```

#### CAN Replay Module (can_replay.py)
```python
def send_thread(j: PandaJungle, flock):
    """Send CAN messages to Panda Jungle
    
    Args:
        j: PandaJungle instance
        flock: File lock for jungle access
    """

# Environment Variables:
PWR_ON = int(os.getenv("PWR_ON", "0"))      # Power on duration
PWR_OFF = int(os.getenv("PWR_OFF", "0"))    # Power off duration  
IGN_ON = int(os.getenv("ON", "0"))          # Ignition on duration
IGN_OFF = int(os.getenv("OFF", "0"))        # Ignition off duration
```

### Environment Variables

#### Core Configuration
```bash
# Data and caching
export OPENPILOT_PREFIX="/custom/path"      # Custom data directory
export FILEREADER_CACHE=1                   # Enable file caching
export TMPDIR="/path/to/temp"               # Temporary file location

# Network configuration
export ZMQ=1                                # Enable ZMQ networking
export ZMQ_PORT=8086                        # Custom ZMQ port
export ZMQ_BIND_ADDR="192.168.1.100"       # Bind to specific address

# Performance tuning
export ZMQ_SNDHWM=1000                      # ZMQ send buffer size
export ZMQ_RCVHWM=1000                      # ZMQ receive buffer size
export OPENPILOT_LOG_LEVEL=DEBUG            # Enable debug logging
```

#### CAN Replay Configuration
```bash
export FLASH=1                              # Flash Panda Jungle on startup
export PWR_ON=30                            # Power cycle: on duration
export PWR_OFF=5                            # Power cycle: off duration
export ON=60                                # Ignition: on duration
export OFF=10                               # Ignition: off duration
```

### Message Services Reference

#### Vehicle State
- **carState** - Core vehicle dynamics (speed, steering, pedals)
- **carControl** - Control commands sent to vehicle
- **controlsState** - Internal control system state
- **carParams** - Vehicle-specific configuration parameters

#### Perception & ML
- **modelV2** - Neural network predictions and confidence
- **driverState** - Driver monitoring system output
- **radarState** - Radar object detection results
- **liveTracks** - Multi-object tracking results

#### Cameras & Vision
- **roadCameraState** - Main road camera frames and metadata
- **driverCameraState** - Driver monitoring camera (--dcam)
- **wideRoadCameraState** - Wide-angle camera frames (--ecam)
- **liveCalibration** - Real-time camera calibration

#### System Health
- **thermal** - CPU/GPU temperatures and throttling
- **deviceState** - System memory, storage, connectivity
- **selfdriveState** - Overall openpilot system status
- **pandaStates** - Panda device status and health

## Best Practices

### Development Workflow
1. **Start with Demo**: Always verify setup with `--demo` route
2. **Use Appropriate Filters**: Filter services to match analysis needs
3. **Monitor Resources**: Check CPU/memory usage during development
4. **Cache Appropriately**: Balance memory usage vs. performance
5. **Test Incrementally**: Start with single segments, expand gradually

### Production Usage
1. **Batch Processing**: Process multiple routes systematically
2. **Resource Planning**: Allocate sufficient memory and storage
3. **Network Optimization**: Use dedicated connections for streaming
4. **Error Handling**: Implement robust retry and recovery mechanisms
5. **Documentation**: Document replay configurations for reproducibility

### Performance Guidelines
- **Memory**: 16GB+ recommended for production use
- **CPU**: Multi-core systems benefit from larger cache sizes
- **Network**: Stable 100Mbps+ for reliable streaming
- **Storage**: SSD storage significantly improves performance

## Conclusion

The nagaspilot Replay System provides professional-grade route replay capabilities for development, debugging, and analysis workflows. Its combination of high-performance C++ core, flexible Python integration, comprehensive camera support, and advanced timeline features makes it an essential tool for openpilot development.

The system's modular architecture supports everything from lightweight development testing to comprehensive multi-camera analysis. Start with the demo route and basic configurations, then expand to custom workflows and advanced integration patterns as expertise develops.

For complex analysis scenarios, combine replay with PlotJuggler, UI visualization, and custom analysis scripts to create powerful development and validation workflows.