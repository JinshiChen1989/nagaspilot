# PlotJuggler Analysis System - User Manual

## Overview

PlotJuggler is a professional time-series data analysis and visualization tool integrated with nagaspilot for advanced route analysis and real-time debugging. It provides powerful plotting capabilities, streaming support, and pre-configured layouts for vehicle tuning and performance analysis.

**📊 System Types: REPLAY + STREAMING + REAL-TIME**
- **✅ Runtime**: Can connect to live vehicles for real-time analysis
- **✅ Streaming**: Real-time data visualization from running systems
- **✅ Post-Processing**: Analyze pre-recorded route data
- **✅ Offline Analysis**: Process cached route segments

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Installation & Setup](#installation--setup)
3. [Basic Usage](#basic-usage)
4. [Streaming Mode](#streaming-mode)
5. [Layout System](#layout-system)
6. [Data Analysis](#data-analysis)
7. [CAN Integration](#can-integration)
8. [Performance Optimization](#performance-optimization)
9. [Troubleshooting](#troubleshooting)
10. [API Reference](#api-reference)

## System Architecture

### Core Components
- **juggle.py** - Main orchestration and launcher script
- **PlotJuggler Binary** - Native Qt-based visualization application
- **Comma Plugins** - Custom nagaspilot data parsing plugins
- **Layout System** - Pre-configured dashboard templates
- **DBC Integration** - CAN database automatic loading

### Data Flow Options

#### Replay Mode
```
Route Data → LogReader → Migration → PlotJuggler → Interactive Analysis
```

#### Streaming Mode
```
Live System → ZMQ/MSGQ → cereal bridge → PlotJuggler → Real-time Plots
```

#### Hybrid Mode
```
Device → WiFi → PC → PlotJuggler Streaming + Route Archive
```

### Integration Points
- **cereal Services** - All message types supported
- **DBC Files** - Automatic CAN database loading
- **Route System** - Direct comma connect integration
- **Layout Templates** - Pre-configured analysis dashboards

## Installation & Setup

### Automatic Installation
```bash
# Navigate to plotjuggler directory
cd tools/plotjuggler

# Install or update PlotJuggler + plugins
./juggle.py --install

# This will:
# 1. Download platform-specific binary (Linux/macOS)
# 2. Install comma-specific plugins
# 3. Set up environment variables
# 4. Configure DBC integration
```

### Manual Installation Check
```bash
# Verify installation
ls bin/plotjuggler

# Check version (minimum required: 3.5.2)
bin/plotjuggler -v

# Test with demo
./juggle.py --demo
```

### Supported Platforms
- **Linux x86_64** - Primary development platform
- **Linux aarch64** - ARM64 systems  
- **macOS x86_64** - Intel Macs
- **macOS ARM64** - Apple Silicon Macs

### Authentication (for remote routes)
```bash
# Authenticate with comma account
cd tools/lib
python3 auth.py
```

## Basic Usage

### Quick Start - Demo Route
```bash
cd tools/plotjuggler

# Launch with demo route
./juggle.py --demo

# Demo with tuning layout
./juggle.py --demo --layout layouts/tuning.xml

# This will:
# 1. Download demo route data
# 2. Process all services except CAN
# 3. Auto-detect vehicle DBC
# 4. Open PlotJuggler with data loaded
```

### Analyze Your Own Route
```bash
# Basic route analysis
./juggle.py "your_route_id|timestamp"

# Specific segment
./juggle.py "your_route_id|timestamp/5"

# Segment range  
./juggle.py "your_route_id|timestamp/2:5"

# With specific layout
./juggle.py --layout layouts/longitudinal.xml "route_name"
```

### Route Specification Formats
```bash
# Full route (all segments)
./juggle.py "a2a0ccea32023010|2023-07-27--13-01-19"

# Single segment (fast loading)
./juggle.py "a2a0ccea32023010|2023-07-27--13-01-19/3"

# Segment range (segments 1-4)
./juggle.py "a2a0ccea32023010|2023-07-27--13-01-19/1:4"

# Cabana share URLs also accepted
./juggle.py "https://cabana.comma.ai/share/..."
```

## Streaming Mode

### Real-Time Data Analysis

#### From Replay System
```bash
# Terminal 1: Start replay
cd tools/replay
./replay "route_name"

# Terminal 2: Start streaming PlotJuggler  
cd tools/plotjuggler
./juggle.py --stream

# In PlotJuggler:
# 1. Click "Streaming" dropdown
# 2. Select "Cereal Subscriber"
# 3. Click "Start" - data flows immediately
```

#### From Live Vehicle (WiFi Streaming)

**Step 1: Device WiFi Hotspot Setup**
```bash
# On comma device (via SSH or direct access):
# 1. Enable WiFi hotspot in comma device settings
# 2. Note the hotspot name and password (usually "comma-XXXXX")
# 3. Verify hotspot is broadcasting
```

**Step 2: PC WiFi Connection**
```bash
# On your PC:
# 1. Connect to comma device WiFi hotspot
#    Network: "comma-XXXXX" 
#    Password: [from device settings]
# 2. Verify connection:
ping 192.168.43.1  # Default device IP
```

**Step 3: Device Bridge Setup (SSH)**
```bash
# SSH into comma device
ssh root@192.168.43.1
# Default password is typically blank or "comma"

# Navigate and start bridge
cd /data/openpilot/cereal/messaging
./bridge &

# Verify bridge is running:
ps aux | grep bridge
# Should show: ./bridge running in background

# Check which services are being bridged:
netstat -tuln | grep 8086  # ZMQ port
```

**Step 4: PC PlotJuggler Streaming**
```bash
# On PC (connected to device hotspot)
cd tools/plotjuggler
ZMQ=1 ./juggle.py --stream

# In PlotJuggler interface:
# 1. Click "Streaming" dropdown
# 2. Select "Cereal Subscriber" plugin
# 3. Configure connection:
#    - Address: 192.168.43.1 (device IP)
#    - Port: 8086 (default ZMQ port)
# 4. Click "Start" - live data flows
```

**WiFi Troubleshooting:**
```bash
# Device hotspot not visible:
# - Check device settings → Network → Hotspot
# - Restart device networking
# - Try 2.4GHz vs 5GHz bands

# Cannot SSH to device:
ssh -v root@192.168.43.1  # Verbose debugging
# Common issues:
# - Wrong IP address (check: ip route show)
# - SSH not enabled (check device settings)
# - Firewall blocking (check iptables)

# Bridge not starting:
cd /data/openpilot/cereal/messaging
./bridge  # Run in foreground to see errors
# Check for:
# - Port conflicts: netstat -tuln | grep 8086
# - Permission issues: ls -la bridge
# - Missing dependencies: ldd bridge

# No data in PlotJuggler:
# - Verify bridge running: ps aux | grep bridge
# - Check network connectivity: ping 192.168.43.1
# - Test ZMQ connection: telnet 192.168.43.1 8086
# - Verify services available: ./bridge --list-services
```

**Advanced WiFi Configuration:**
```bash
# Custom IP configuration (if needed):
# On device:
ip addr show wlan0  # Check current IP

# On PC (if DHCP fails):
sudo ip addr add 192.168.43.100/24 dev wlan0
sudo ip route add default via 192.168.43.1

# Multiple device connections:
# Device 1: 192.168.43.1
# Device 2: Connect to different hotspot/IP
ZMQ=1 DEVICE_IP=192.168.44.1 ./juggle.py --stream

# Bandwidth optimization:
# Limit services for better performance:
cd /data/openpilot/cereal/messaging
./bridge --services carState,modelV2,controlsState
```

### Network Configuration
```bash
# Enable ZMQ networking
export ZMQ=1

# Or inline
ZMQ=1 ./juggle.py --stream

# Device bridge setup (on comma device)
cd /data/openpilot/cereal/messaging
./bridge &  # Runs in background
```

### Streaming Performance
- **Buffer Size**: 1000 samples (configurable)
- **Update Rate**: Real-time (20Hz typical)
- **Network Protocol**: ZMQ for low latency
- **Memory Usage**: ~100MB for streaming buffer

## Layout System

### Pre-configured Layouts

#### Tuning Layout (`layouts/tuning.xml`)
**Purpose**: Vehicle parameter optimization and tuning analysis
```bash
./juggle.py --layout layouts/tuning.xml "route_name"
```
**Key Plots**:
- Lateral control performance
- Longitudinal control accuracy
- Speed tracking vs cruise setpoint
- Steering angle response
- Control system status

#### Longitudinal Layout (`layouts/longitudinal.xml`)
**Purpose**: Speed and acceleration control analysis
```bash
./juggle.py --layout layouts/longitudinal.xml "route_name"
```
**Key Plots**:
- Speed error vs target
- Acceleration commands vs actual
- Lead vehicle tracking
- Control state transitions
- Braking/acceleration events

#### Torque Controller Layout (`layouts/torque-controller.xml`)
**Purpose**: Steering system analysis
```bash
./juggle.py --layout layouts/torque-controller.xml "route_name"
```
**Key Plots**:
- Steering torque commands
- Motor response characteristics  
- Angle error vs target
- Control saturation events
- Steering system health

#### Debugging Layouts
```bash
# CAN bus debugging
./juggle.py --layout layouts/CAN-bus-debug.xml "route_name"

# System timing analysis
./juggle.py --layout layouts/system_lag_debug.xml "route_name"

# GPS/localization debugging
./juggle.py --layout layouts/locationd_debug.xml "route_name"

# Camera timing analysis
./juggle.py --layout layouts/camera-timings.xml "route_name"
```

### Custom Layouts
1. **Create Custom Layout**:
   - Load route data in PlotJuggler
   - Arrange desired plots/widgets
   - Save as XML: File → Save Layout

2. **Use Custom Layout**:
```bash
./juggle.py --layout /path/to/custom.xml "route_name"
```

3. **Share Layouts**:
   - Save to `layouts/` directory
   - Contribute via PR for community use

## Data Analysis

### Available Data Streams

#### Vehicle Dynamics
- **carState**: Speed, acceleration, steering angle, gear
- **carControl**: Control commands to vehicle systems
- **controlsState**: Control system internal states
- **liveParameters**: Adaptive parameter updates

#### Perception Systems  
- **modelV2**: Neural network predictions and confidence
- **radarState**: Object detection and tracking
- **liveCalibration**: Camera calibration parameters
- **driverState**: Driver monitoring system data

#### System Health
- **thermal**: CPU/GPU temperatures, throttling
- **deviceState**: Memory, storage, connectivity
- **selfdriveState**: Overall system status and alerts

### Data Processing Pipeline
```python
# Automatic data processing chain:
1. LogReader(route) → Raw cereal messages
2. Migration system → Updates old log formats  
3. Service filtering → Removes unwanted streams
4. DBC inference → Auto-detects CAN database
5. PlotJuggler → Interactive visualization
```

### Analysis Workflows

#### Performance Analysis
1. **Load route** with tuning layout
2. **Identify issues** via visual inspection
3. **Correlate signals** using synchronized timeline
4. **Export data** for further analysis
5. **Generate reports** with key findings

#### Real-Time Debugging  
1. **Connect streaming** to live system
2. **Monitor key signals** during test drives
3. **Identify anomalies** as they occur
4. **Adjust parameters** and observe effects
5. **Log sessions** for later detailed analysis

## CAN Integration

### Automatic DBC Loading
```python
# System automatically:
1. Reads carParams from route
2. Determines vehicle platform 
3. Selects appropriate DBC file
4. Loads CAN signal definitions
5. Enables CAN data plotting
```

### Manual DBC Specification
```bash
# Force specific DBC file
./juggle.py --dbc honda_civic_touring_2016_can_generated "route_name"

# List available DBC files
ls opendbc/opendbc/dbc/*.dbc
```

### CAN Data Analysis
```bash
# Include CAN data in analysis
./juggle.py --can "route_name"

# CAN-specific debugging layout
./juggle.py --can --layout layouts/CAN-bus-debug.xml "route_name"
```

### CAN Signal Processing
- **Automatic Parsing**: DBC definitions applied automatically
- **Signal Names**: Human-readable signal names from DBC
- **Unit Conversion**: Automatic scaling and offset application
- **Multi-Bus Support**: Separate analysis of different CAN buses

## Performance Optimization

### Memory Management
```python
# Configuration in juggle.py:
MAX_STREAMING_BUFFER_SIZE = 1000  # Samples
```

### Processing Optimization

#### For Large Routes
```bash
# Process specific segments only
./juggle.py "route_name/0:2"  # First 3 segments

# Skip CAN data for faster processing
./juggle.py "route_name"  # CAN excluded by default

# Use efficient layouts
./juggle.py --layout layouts/tuning.xml "route_name"
```

#### For Real-Time Streaming
```bash
# Optimize buffer size for latency vs stability
# Edit juggle.py: MAX_STREAMING_BUFFER_SIZE = 500  # Lower latency
# Edit juggle.py: MAX_STREAMING_BUFFER_SIZE = 2000 # More stable
```

### System Resource Usage
- **CPU**: Moderate (Qt-based UI)
- **Memory**: ~200MB + route data size
- **Network**: ~1MB/s for streaming
- **Storage**: Temporary files in plotjuggler directory

## Troubleshooting

### Installation Issues

#### Missing PlotJuggler Binary
```
Error: PlotJuggler is missing

Solution:
./juggle.py --install
# Auto-downloads and installs platform-specific binary
```

#### Version Compatibility
```
Error: PlotJuggler is out of date

Solution:
./juggle.py --install
# Updates to minimum required version 3.5.2+
```

#### Platform Support
```
Error: Unsupported platform

Solution:
# Check supported platforms:
# - Linux x86_64, Linux aarch64  
# - macOS x86_64, macOS ARM64
# Windows requires WSL2 with Linux setup
```

### Streaming Issues

#### No Data in Streaming Mode
```
Problem: PlotJuggler opens but no data appears

Solution:
1. Verify replay/bridge is running
2. Check ZMQ=1 environment variable
3. Select "Cereal Subscriber" plugin
4. Click "Start" in streaming dialog
5. Check firewall/network settings
```

#### Connection Timeouts
```
Problem: Cannot connect to live device

Solution:
1. Verify device bridge: ./cereal/messaging/bridge
2. Check WiFi connection to device hotspot
3. Test with local replay first
4. Verify device IP address
```

### Data Processing Issues

#### Route Download Failures
```
Problem: Cannot download route data

Solution:
1. Check authentication: tools/lib/auth.py
2. Verify internet connection
3. Try demo route first: --demo
4. Check route name format
```

#### DBC Loading Errors
```
Problem: CAN data not parsing correctly

Solution:
1. Verify DBC auto-detection in logs
2. Manually specify: --dbc filename
3. Check vehicle compatibility
4. Use --can flag for CAN inclusion
```

### Performance Issues

#### Slow Route Loading
```
Problem: Long loading times

Solution:
1. Use single segments: "route/0"
2. Skip CAN data: default behavior
3. Use SSD storage for temp files
4. Close other applications
```

#### UI Responsiveness
```
Problem: PlotJuggler UI sluggish

Solution:
1. Reduce plot complexity
2. Limit visible time range
3. Use decimation for large datasets
4. Increase system RAM if possible
```

## API Reference

### Command Line Interface
```bash
juggle.py [-h] [--demo] [--can] [--stream] [--no-migration] 
          [--layout [LAYOUT]] [--install] [--dbc DBC] 
          [route_or_segment_name]

Arguments:
  route_or_segment_name   Route to analyze (cabana URLs accepted)

Options:
  --demo                  Use demo route
  --can                   Include CAN data parsing
  --stream               Start in streaming mode  
  --no-migration         Skip log format migration
  --layout [LAYOUT]      Load pre-defined layout
  --install              Install/update PlotJuggler
  --dbc DBC             Specify DBC file for CAN parsing
```

### Core Functions

#### Installation Management
```python
def install()              # Download and install PlotJuggler
def get_plotjuggler_version()  # Check current version
```

#### Route Processing
```python
def juggle_route(route_or_segment_name, can, layout, dbc, should_migrate)
def process(can, lr)       # Filter data streams
```

#### PlotJuggler Integration
```python
def start_juggler(fn=None, dbc=None, layout=None, route_or_segment_name=None, platform=None)
```

### Configuration Constants
```python
DEMO_ROUTE = "a2a0ccea32023010|2023-07-27--13-01-19"
RELEASES_URL = "https://github.com/commaai/PlotJuggler/releases/download/latest"
MINIMUM_PLOTJUGGLER_VERSION = (3, 5, 2)
MAX_STREAMING_BUFFER_SIZE = 1000
```

### Environment Variables
```bash
# Required for proper operation
export BASEDIR="/path/to/nagaspilot"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:tools/plotjuggler/bin"
export ZMQ=1  # For network streaming
export DBC_NAME="vehicle_specific.dbc"  # For manual DBC override
```

## Best Practices

### Development Workflow
1. **Start with Demo**: Verify installation with `--demo`
2. **Use Appropriate Layouts**: Select layout matching analysis goal
3. **Process Incrementally**: Start with single segments
4. **Stream for Real-Time**: Use streaming for live debugging
5. **Save Custom Layouts**: Create reusable analysis templates

### Production Analysis
1. **Batch Processing**: Process multiple routes systematically
2. **Layout Standardization**: Use consistent layouts for comparable analysis
3. **Data Export**: Export key findings for reports
4. **Version Control**: Track PlotJuggler version for reproducibility

### Performance Guidelines
- **Memory**: 500MB+ recommended for large routes
- **CPU**: Multi-core benefits parallel processing
- **Network**: Stable connection for streaming mode
- **Storage**: SSD recommended for temporary files

## Conclusion

PlotJuggler provides professional-grade time-series analysis capabilities for nagaspilot development. Its combination of replay analysis, real-time streaming, and pre-configured layouts makes it an essential tool for vehicle tuning, debugging, and performance optimization.

The streaming capability enables real-time debugging during test drives, while the replay functionality supports detailed post-analysis of recorded data. Start with demo routes and pre-configured layouts, then expand to custom analysis workflows as expertise develops.