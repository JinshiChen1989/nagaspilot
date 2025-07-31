# Helper Features & Debugging Tools Report

## Executive Summary

The nagaspilot codebase contains an extensive ecosystem of debugging, visualization, simulation, and helper tools that significantly enhance development, testing, and analysis capabilities. These tools provide comprehensive coverage from low-level CAN analysis to high-level simulation and real-time debugging.

## 1. Visualization & Analysis Tools

### 1.1 BEV Plotting System *(Detailed in bev_plotting_report.md)*
- **Real-time BEV visualization** with ego trajectory, lane lines, and lead vehicles
- **Multi-framework support:** Rerun, Pygame, PyRay, Matplotlib
- **Network streaming** capabilities for remote debugging

### 1.2 Rerun Integration (`tools/rerun/`)
**Purpose:** Modern 3D visualization and data logging framework

**🖥️ GUI Location: PC BROWSER (not device SSH)**

**Key Features:**
- **Timeline Visualization:** Interactive timeline scrubbing with multi-stream synchronization
- **3D Camera Views:** Support for qcam, fcam, ecam, dcam with NV12 format
- **Web Interface:** Browser-accessible visualization with remote access
- **Multi-processing:** Parallel processing across segments for performance

**Usage Examples:**
```bash
# Run on PC (NOT via SSH to device)
# Browser opens on YOUR PC screen
tools/rerun/run.py --qcam --fcam --ecam <route-name>

# Demo mode with low-res camera  
tools/rerun/run.py --qcam --demo
# ↳ Rerun web interface opens in YOUR browser
```

**Technical Implementation:**
- **Camera Reader:** Automatic ffmpeg/ffprobe integration for video processing
- **Parallel Processing:** Uses multiprocessing for segment analysis
- **Blueprint System:** Dynamic UI layout generation based on available data streams

### 1.3 PlotJuggler Integration (`tools/plotjuggler/`)
**Purpose:** Professional time-series data analysis and visualization

**🖥️ GUI Location: PC APPLICATION (device SSH for setup only)**

**Key Features:**
- **Streaming Mode:** Real-time data visualization from live devices
- **Pre-defined Layouts:** Tuning, debugging, and analysis layouts
- **CAN Data Support:** Automatic DBC file integration
- **Route Analysis:** Support for route segments and cabana share URLs

**WiFi Streaming Architecture:**
```bash
# Device SSH (setup only):
ssh root@192.168.43.1 && ./bridge & && exit

# PC GUI (where you see the interface):
./juggle.py --stream  # PlotJuggler window opens on YOUR PC
```

**Layout Types:**
- `layouts/tuning.xml` - Vehicle tuning and parameter analysis
- `layouts/longitudinal.xml` - Longitudinal control debugging  
- `layouts/torque-controller.xml` - Steering torque analysis
- `layouts/thermal_debug.xml` - Thermal performance monitoring

**Network Streaming:**
```bash
# Stream from comma device
ZMQ=1 ./juggle.py --stream

# Live streaming from device hotspot
cd /data/openpilot && ./cereal/messaging/bridge
```

### 1.4 Cabana - CAN Analysis Tool (`tools/cabana/`)
**Purpose:** Professional CAN data analysis and DBC file editing

**🖥️ GUI Location: PC APPLICATION (device SSH for WiFi streaming setup)**

**Key Features:**
- **Real-time CAN Analysis:** Live data from panda, socketcan, or ZMQ streams
- **DBC Integration:** Direct integration with commaai/opendbc repository
- **Multi-source Support:** Routes, live streams, local data
- **Signal Analysis:** Advanced signal detection and bit pattern analysis

**Live WiFi CAN Analysis:**
```bash
# Device SSH (setup only):
ssh root@192.168.43.1 && ./bridge & && exit

# PC GUI (CAN analysis interface):
./cabana --zmq 192.168.43.1  # Cabana GUI opens on YOUR PC
```

**Data Sources:**
- **Live Streaming:** `--zmq <ip>`, `--panda`, `--socketcan`
- **Route Replay:** Support for comma connect routes
- **Local Data:** `--data_dir` for offline analysis
- **Multi-camera:** `--qcam`, `--ecam`, `--dcam` integration

## 2. Replay & Simulation Systems

### 2.1 Advanced Replay System (`tools/replay/`)
**Purpose:** High-fidelity driving session replay and analysis

**Core Features:**
- **Multi-format Support:** ZMQ, MSGQ, local/remote routes
- **Service Filtering:** Whitelist/blacklist specific services
- **Speed Control:** Playback speed 0.2x - 3x
- **Memory Management:** Configurable segment caching (default 5 segments)
- **Video Integration:** Hardware-accelerated video decoding

**Network Capabilities:**
```bash
# Remote replay with ZMQ
ZMQ=1 tools/replay/replay <route-name>

# Local route replay
tools/replay/replay <route-name> --data_dir="/path/to/routes"

# Service filtering
tools/replay/replay --allow="carState,modelV2" <route-name>
```

**Integration Points:**
- **UI Visualization:** Works with `selfdrive/ui/ui`
- **PlotJuggler:** Real-time streaming to analysis tools
- **CAN Replay:** Physical CAN bus replay via panda jungle

### 2.2 Simulation Framework (`tools/sim/`)
**Purpose:** MetaDrive simulator integration for safe testing

**Key Features:**
- **MetaDrive Bridge:** Real-time connection to MetaDrive simulator
- **Control Integration:** Full openpilot control stack in simulation
- **Keyboard/Joystick:** Manual override capabilities
- **Dual Camera:** Support for stereo camera simulation

**Control Interface:**
```
Key Mappings:
1 - Cruise Resume/Accel    |  r - Reset Simulation
2 - Cruise Set/Decel       |  i - Toggle Ignition  
3 - Cruise Cancel          |  q - Exit All
wasd - Manual Control      |  S - User Brake (disengage)
```

**Launch Process:**
```bash
# Start openpilot stack
./tools/sim/launch_openpilot.sh

# Start MetaDrive bridge
./tools/sim/run_bridge.py --joystick --high_quality
```

### 2.3 Webcam Integration (`tools/webcam/`)
**Purpose:** Run openpilot on PC with standard webcams

**Hardware Requirements:**
- Ubuntu 24.04 or macOS (WSL2 not supported)
- GPU recommended
- USB webcam ≥720p, 78° FOV (Logitech C920/C615)
- Car harness + panda for CAN integration

**Multi-camera Support:**
```bash
# Specify camera assignments
USE_WEBCAM=1 ROAD_CAM=1 DRIVER_CAM=2 system/manager/manager.py

# Basic setup
USE_WEBCAM=1 system/manager/manager.py
```

## 3. Debugging & Analysis Tools

### 3.1 Debug Script Collection (`selfdrive/debug/`)
**Purpose:** Comprehensive debugging utilities for system analysis

**Key Tools:**

#### CAN Analysis:
- **`can_printer.py`** - Real-time CAN message viewer with filtering
- **`can_table.py`** - CAN message frequency and statistics analysis  
- **`can_print_changes.py`** - CAN message change detection

#### Performance Analysis:
- **`check_freq.py`** - Service frequency monitoring
- **`check_lag.py`** - System latency analysis
- **`check_timings.py`** - Process timing validation
- **`cpu_usage_stat.py`** - CPU utilization tracking
- **`live_cpu_and_temp.py`** - Real-time system monitoring

#### Vehicle Integration:
- **`get_fingerprint.py`** - Vehicle fingerprint extraction
- **`fw_versions.py`** - ECU firmware version detection
- **`clear_dtc.py`** - Diagnostic trouble code clearing
- **`read_dtc_status.py`** - DTC status monitoring

#### Data Analysis:
- **`dump.py`** - Universal cereal message dumping tool
- **`qlog_size.py`** - Log size analysis and optimization
- **`filter_log_message.py`** - Message filtering and extraction

### 3.2 Universal Dump Tool (`selfdrive/debug/dump.py`)
**Purpose:** Real-time message monitoring and analysis

**Key Features:**
- **Multi-format Output:** Raw, JSON, hex dump formats
- **Service Filtering:** Monitor specific sockets/services
- **Value Extraction:** Monitor specific message fields
- **Network Support:** Remote monitoring via IP address

**Usage Examples:**
```bash
# Dump all services in JSON format
./dump.py --json

# Monitor specific services
./dump.py carState modelV2 --values="vEgo,steeringAngleDeg"

# Remote monitoring
./dump.py --addr=192.168.1.100 carState
```

### 3.3 Touch & UI Debugging (`selfdrive/debug/uiview.py`, `touch_replay.py`)
**Purpose:** UI interaction analysis and touch event debugging

**Features:**
- **Touch Event Replay:** Reproduce user interactions
- **UI State Analysis:** Monitor UI component states
- **Visual Debugging:** Screen capture and interaction logging

## 4. Remote Control & Teleoperation

### 4.1 Joystick Control (`tools/joystick/`)
**Purpose:** Manual vehicle control for testing and debugging

**Control Modes:**
- **Local Joystick:** Direct USB connection to comma device
- **Network Control:** Joystick via laptop with ZMQ streaming
- **Keyboard Mode:** WASD controls for basic testing

**Network Setup:**
```bash
# On comma device - enable joystick mode
echo -n "1" > /data/params/d/JoystickDebugMode

# Bridge joystick data from laptop
cereal/messaging/bridge {LAPTOP_IP} testJoystick

# On laptop - start joystick control
export ZMQ=1
tools/joystick/joystick_control.py
```

**Safety Features:**
- Requires cruise control engagement
- Real-time status display on UI
- Immediate manual override capability

### 4.2 Body Teleoperation (`tools/bodyteleop/`)
**Purpose:** Remote vehicle operation via web interface

**Features:**
- **Web-based Control:** Browser interface for remote operation
- **WebRTC Integration:** Low-latency video streaming
- **Real-time Control:** Direct vehicle control via web interface
- **Safety Monitoring:** Continuous connection and safety checks

### 4.3 Camera Streaming (`tools/camerastream/`)
**Purpose:** Remote camera access and monitoring

**Features:**
- **Multi-camera Support:** Road, driver, wide cameras
- **Hardware Decoding:** NVIDIA GPU acceleration support
- **VisionIPC Integration:** Standard openpilot camera interface
- **Network Streaming:** Real-time camera streaming over network

**Complete WiFi Setup Process:**
```bash
# Step 1: Device WiFi Hotspot Setup
# On comma device: Settings → Network → WiFi Hotspot → Enable
# Network: "comma-XXXXX", Password: [secure password]

# Step 2: PC WiFi Connection
sudo nmcli dev wifi connect "comma-XXXXX" password "your_password"
ping 192.168.43.1  # Verify connection

# Step 3: SSH into Device (Control Only - NO GUI)
ssh root@192.168.43.1

# Step 4: Start Background Services on Device
cd /data/openpilot/cereal/messaging && ./bridge &
cd /data/openpilot/system/camerad && ./camerad &
cd /data/openpilot/system/loggerd && ./encoderd &
exit  # Exit SSH, services keep running

# Step 5: GUI Applications Run on PC (NOT device)
# On PC - decode and display (GUI opens on YOUR screen)
./compressed_vipc.py <device_ip> --cams 0
./selfdrive/ui/watch3  # GUI displays on PC
```

**WiFi Architecture Explanation:**
```
Comma Device (SSH Terminal)        PC (GUI Applications)
┌─────────────────────┐           ┌─────────────────────────┐
│ SSH: Control only   │           │                         │
│ ./bridge &          │ ◄────────► │  [Camera View Window]   │
│ ./camerad &         │    WiFi    │  [Watch3 GUI]          │
│ ./encoderd &        │            │  [Analysis Tools]      │
│                     │            │                         │
│ (services run in    │            │  (GUI on YOUR screen)  │
│  background)        │            │                         │
└─────────────────────┘            └─────────────────────────┘
```

**Why SSH + PC GUI Architecture:**
- **Device**: Limited CPU/RAM, small screen - used for data collection
- **PC**: Powerful hardware, large display - used for GUI and analysis
- **Network Efficiency**: Only data transmitted, not GUI rendering
- **Better Performance**: GUI applications run locally on your PC

## 5. Advanced Features

### 5.1 Longitudinal Maneuvers (`tools/longitudinal_maneuvers/`)
**Purpose:** Automated testing of longitudinal control performance

**Key Features:**
- **Automated Test Scenarios:** Predefined driving maneuvers
- **Performance Reporting:** Quantitative analysis of control performance
- **Route Analysis:** Batch processing of driving routes
- **Visualization:** Integrated plotting and analysis tools

### 5.2 Car Porting Tools (`tools/car_porting/`)
**Purpose:** Vehicle integration and compatibility testing

**Tools:**
- **Auto Fingerprinting:** Automatic vehicle detection and configuration
- **Test Car Model:** Vehicle-specific testing framework
- **Example Notebooks:** Interactive analysis examples for specific vehicles

**Example Categories:**
- Ford VIN fingerprinting
- Subaru longitudinal acceleration analysis  
- HKG CANFD gear message analysis
- Steering temperature fault analysis

### 5.3 Profiling & Performance (`tools/profiling/`)
**Purpose:** System performance analysis and optimization

**Tools:**
- **CLPeak:** OpenCL performance benchmarking
- **Perfetto:** System-wide performance tracing
- **py-spy:** Python performance profiling
- **Snapdragon Tools:** Platform-specific optimization tools

## 6. Integration & Connectivity

### 6.1 SSH + GUI Architecture Understanding
**Purpose:** Clarify the distributed architecture used throughout nagaspilot tools

**Common Misconception:**
Many users expect to run GUI applications directly on the comma device via SSH. However, nagaspilot uses a **distributed architecture** where:

**Architecture Pattern:**
```
Comma Device (Data Source)         Network (WiFi)         PC (Analysis & GUI)
│                                 │                       │
├── SSH Terminal Access           │                       ├── GUI Applications
│   ├── Control commands only     │                       │   ├── PlotJuggler
│   ├── Start/stop services       │                       │   ├── Cabana  
│   └── No GUI applications       │                       │   ├── Rerun browser
│                                 │                       │   └── Watch3
├── Background Services           │                       │
│   ├── ./bridge (data forward)   │ ◄────────────────────► ├── Data Reception
│   ├── ./camerad (camera data)   │        ZMQ/TCP        │   └── Live analysis
│   └── ./encoderd (video encode) │                       │
│                                 │                       └── Local file access
└── Data collection only          │                       └── Export capabilities
```

**Why This Architecture:**

1. **Performance**: Comma device has limited CPU/GPU for heavy GUI applications
2. **Display**: Small device screen vs. large PC monitors for detailed analysis  
3. **Processing Power**: PC hardware better suited for complex visualizations
4. **Development Tools**: Advanced analysis tools built for PC environments
5. **Multitasking**: Run multiple analysis tools simultaneously on PC
6. **Network Efficiency**: Only data transmitted, not GUI rendering

**Typical Workflow:**
```bash
# Step 1: SSH for Control (NO GUI)
ssh root@192.168.43.1
./bridge &  # Start background service
exit        # Close SSH, service keeps running

# Step 2: GUI on PC (NOT via SSH)
./juggle.py --stream      # PlotJuggler GUI opens on PC
./cabana --zmq device_ip  # Cabana GUI opens on PC  
./run.py --demo          # Rerun opens browser on PC
```

**Alternative: Direct Device GUI (Not Recommended)**
```bash
# X11 Forwarding (slow, not suitable for real-time)
ssh -X root@192.168.43.1
./cabana  # GUI appears on PC but runs on device (laggy)

# VNC Server (complex setup, limited use cases)
# On device: vncserver :1
# On PC: vncviewer device_ip:5901
```

### 6.2 Network Bridge System
**Purpose:** Multi-device communication and data sharing

**Components:**
- **cereal/messaging/bridge** - Core message bridging service
- **ZMQ Integration** - Cross-platform message streaming
- **Service Discovery** - Automatic service detection and routing

### 6.2 Authentication & API (`tools/lib/auth.py`)
**Purpose:** Secure access to comma cloud services

**Features:**
- **OAuth Integration:** Secure authentication with comma services
- **Route Access:** Download and access personal driving data
- **API Abstraction:** Simplified interface to comma cloud services

## 7. Usage Patterns & Workflows

### 7.1 Development Workflow
```bash
# 1. Replay driving session
tools/replay/replay <route-name>

# 2. Analyze with PlotJuggler
tools/plotjuggler/juggle.py --stream --layout layouts/tuning.xml

# 3. Debug specific issues
selfdrive/debug/dump.py carState modelV2 --json

# 4. Test changes in simulation
tools/sim/launch_openpilot.sh
tools/sim/run_bridge.py
```

### 7.2 Remote Debugging Workflow
```bash
# 1. Setup device streaming
ssh comma-device
cd /data/openpilot && ./cereal/messaging/bridge

# 2. Remote visualization
tools/replay/rp_visualization.py <device_ip>

# 3. CAN analysis
tools/cabana/cabana --zmq <device_ip>

# 4. Remote control testing  
tools/joystick/joystick_control.py --keyboard
```

### 7.3 Performance Analysis Workflow
```bash
# 1. Collect performance data
selfdrive/debug/cpu_usage_stat.py > performance.log

# 2. Analyze system timing
selfdrive/debug/check_timings.py

# 3. Profile specific processes
tools/profiling/py-spy/profile.sh controlsd

# 4. Generate comprehensive report
tools/longitudinal_maneuvers/generate_report.py <route>
```

## 8. Technical Architecture

### 8.1 Message System Integration
- **cereal Protocol:** Universal message format across all tools
- **Service Discovery:** Automatic detection of available services
- **Multi-transport:** MSGQ, ZMQ, direct socket support

### 8.2 Video Processing Pipeline
- **Multi-format Support:** HEVC, NV12, raw formats
- **Hardware Acceleration:** GPU decoding where available  
- **VisionIPC:** Standard interface for camera data
- **Network Streaming:** Efficient compression and transmission

### 8.3 Cross-platform Compatibility
- **Ubuntu 24.04:** Primary development platform
- **macOS Support:** Native compatibility for most tools
- **WSL Integration:** Windows support via WSL2
- **ARM64 Support:** Native support for embedded devices

## 9. Future Enhancement Opportunities

1. **WebGL Visualization:** Browser-based 3D visualization
2. **AI-Assisted Debugging:** Machine learning for anomaly detection
3. **Cloud Integration:** Distributed analysis and processing
4. **Mobile Apps:** Smartphone-based monitoring and control
5. **Advanced Simulation:** More sophisticated simulation scenarios

## Conclusion

The nagaspilot ecosystem provides a comprehensive and sophisticated toolkit for autonomous vehicle development, debugging, and analysis. The multi-layered approach - from low-level CAN analysis to high-level simulation - creates a complete development environment that supports both rapid prototyping and production-quality testing.

Key strengths include:
- **Comprehensive Coverage:** Tools for every aspect of development
- **Network Integration:** Seamless remote debugging and monitoring
- **Multi-platform Support:** Flexibility across development environments  
- **Professional Quality:** Production-ready tools with robust error handling
- **Extensibility:** Modular design supporting custom extensions

This toolkit significantly reduces development time and increases reliability through its integrated approach to debugging, visualization, and testing.