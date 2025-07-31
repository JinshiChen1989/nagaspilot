# Cabana CAN Analysis Tool - User Manual

## Overview

Cabana is a professional CAN (Controller Area Network) data analysis and DBC file editing tool integrated with nagaspilot. It provides real-time CAN message analysis, signal decoding, and DBC file management with direct integration to comma's opendbc database and multiple data sources.

**📡 System Types: LIVE + STREAMING + REPLAY + ANALYSIS**
- **✅ Runtime**: Live CAN analysis from panda/devices during driving
- **✅ Streaming**: Real-time CAN data from ZMQ/network sources  
- **✅ Replay**: Post-analysis of recorded route CAN data
- **✅ Hardware**: Direct panda/SocketCAN interface
- **✅ Development**: DBC file creation and editing

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Installation & Setup](#installation--setup)
3. [Data Source Configuration](#data-source-configuration)
4. [Basic Usage](#basic-usage)
5. [Live CAN Analysis](#live-can-analysis)
6. [DBC Management](#dbc-management)
7. [Signal Analysis](#signal-analysis)
8. [Advanced Features](#advanced-features)
9. [Export & Integration](#export--integration)
10. [Troubleshooting](#troubleshooting)
11. [API Reference](#api-reference)

## System Architecture

### Core Components
- **MainWindow** (`mainwin.cc`) - Primary Qt-based interface
- **Stream System** - Multiple data source handlers
- **DBC Manager** (`dbc/dbcmanager.cc`) - Database file management  
- **Signal Analysis** (`signalview.cc`) - Real-time signal decoding
- **Video Integration** (`videowidget.cc`) - Synchronized video playback

### Stream Types
```cpp
// Available data sources:
- ReplayStream    // Route replay from comma connect
- LiveStream      // ZMQ network streaming  
- PandaStream     // Direct panda hardware
- SocketCanStream // Linux SocketCAN interface
- DeviceStream    // Remote device connection
```

### Data Flow Architecture
```
CAN Source → Stream Handler → DBC Parser → Signal Decoder → UI Visualization
     ↓              ↓             ↓             ↓              ↓
   Hardware    Protocol      Database     Value Extract    Analysis
   Network     Handling      Loading      & Convert       Interface  
   Replay                                                            
```

### Integration Points
- **opendbc Repository** - Automatic DBC file management
- **comma connect** - Direct route access and sharing
- **panda Hardware** - Native CAN interface support
- **Video Synchronization** - Matched timeline with camera data

## Installation & Setup

### Prerequisites
```bash
# Ensure nagaspilot environment is set up
cd /path/to/nagaspilot
source .venv/bin/activate

# Build cabana (included in main build)
scons tools/cabana/

# Or build specifically
cd tools/cabana
scons .
```

### Platform Requirements
- **Linux**: Full support with SocketCAN
- **macOS**: Supported (no SocketCAN)
- **panda Hardware**: USB connectivity required
- **Qt Framework**: Built-in with scons build

### Authentication Setup
```bash
# For comma connect routes
cd tools/lib
python3 auth.py
```

## Data Source Configuration

### 1. Route Replay (Post-Analysis)
```bash
# Load specific route
./cabana "a2a0ccea32023010|2023-07-27--13-01-19"

# Load with specific cameras
./cabana --qcam --ecam "route_name"

# Load segment range  
./cabana "route_name/2:5"

# Auto-download from best source
./cabana --auto "route_name"
```

### 2. Live Device Streaming (Real-Time WiFi)

**Complete WiFi Setup Process:**

**Step 1: Comma Device WiFi Hotspot Configuration**
```bash
# On comma device (physical access or SSH):
# 1. Access device settings:
#    Settings → Network → WiFi Hotspot
# 2. Enable hotspot:
#    - Name: "comma-XXXXX" (device serial)
#    - Password: Set secure password
#    - Band: 2.4GHz (better range) or 5GHz (faster)
# 3. Verify hotspot status:
#    Should show "Hotspot Active" with client count
```

**Step 2: PC WiFi Connection Setup**
```bash
# Connect PC to comma device hotspot:
# Method 1: GUI
# - Open WiFi settings
# - Connect to "comma-XXXXX" 
# - Enter password from device

# Method 2: Command line (Linux)
sudo nmcli dev wifi connect "comma-XXXXX" password "your_password"

# Method 3: Manual configuration (if needed)
sudo wpa_supplicant -B -i wlan0 -c <(wpa_passphrase "comma-XXXXX" "your_password")
sudo dhclient wlan0

# Verify connection:
ping 192.168.43.1  # Default comma device IP
ip route show  # Should show route via 192.168.43.1
```

**Step 3: SSH Access Verification**
```bash
# Test SSH connection:
ssh root@192.168.43.1
# Common login methods:
# - Password: blank (just press Enter)
# - Password: "comma"
# - Key-based (if configured)

# If SSH fails, enable it on device:
# Settings → Network → SSH → Enable

# Alternative SSH methods:
ssh -o StrictHostKeyChecking=no root@192.168.43.1
ssh -o UserKnownHostsFile=/dev/null root@192.168.43.1

# For persistent connection:
ssh -o ServerAliveInterval=30 root@192.168.43.1
```

**Step 4: Device Bridge Setup (Critical Step)**
```bash
# SSH into device:
ssh root@192.168.43.1

# Navigate to messaging directory:
cd /data/openpilot/cereal/messaging

# Check if bridge exists and is executable:
ls -la bridge
# Should show: -rwxr-xr-x ... bridge

# Start bridge in background:
./bridge &

# Verify bridge is running:
ps aux | grep bridge
# Output should show: root ... ./bridge

# Check bridge network status:
netstat -tuln | grep 8086
# Should show: tcp 0.0.0.0:8086 LISTEN (ZMQ port)

# Verify bridge functionality:
./bridge --help  # Show available options
./bridge --list-services  # Show available data streams
```

**Step 5: PC Cabana Connection**
```bash
# On PC (must be connected to comma device hotspot):
cd tools/cabana

# Connect via ZMQ:
./cabana --zmq 192.168.43.1

# If connection successful, Cabana shows:
# - "Connected to 192.168.43.1" in status bar
# - Live CAN messages appearing in real-time
# - Message frequency counters updating

# Alternative connection syntax:
./cabana --zmq comma-device  # If hostname resolution works
./cabana --zmq 192.168.43.1:8086  # Explicit port specification
```

**Advanced WiFi Troubleshooting:**

**Connection Issues:**
```bash
# Device hotspot not visible:
# On device, restart WiFi:
ifconfig wlan0 down && ifconfig wlan0 up
# Or restart networking entirely:
systemctl restart NetworkManager

# Check hostapd (hotspot daemon):
systemctl status hostapd
journalctl -u hostapd -f  # Follow logs

# PC connection issues:
# Clear WiFi cache:
sudo nmcli con delete "comma-XXXXX"
sudo systemctl restart NetworkManager

# Manual IP assignment (if DHCP fails):
sudo ip addr add 192.168.43.100/24 dev wlan0
sudo ip route add default via 192.168.43.1 dev wlan0
```

**SSH Connection Issues:**
```bash
# SSH permission denied:
# Check SSH service on device:
systemctl status ssh
systemctl start ssh  # If not running

# Check SSH configuration:
cat /etc/ssh/sshd_config | grep PermitRootLogin
# Should be: PermitRootLogin yes

# Reset SSH keys (if corrupted):
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.43.1

# SSH timeout issues:
ssh -o ConnectTimeout=30 -o ServerAliveInterval=60 root@192.168.43.1
```

**Bridge Connection Issues:**
```bash
# Bridge not starting:
cd /data/openpilot/cereal/messaging

# Check bridge dependencies:
ldd bridge  # Should show all libraries found

# Run bridge in foreground (debug mode):
./bridge  # Shows error messages directly

# Common bridge errors:
# - Port 8086 in use: killall bridge; ./bridge
# - Permission denied: chmod +x bridge
# - Library missing: Update device or reinstall

# Check firewall (if bridge runs but no connection):
iptables -L  # Check for blocking rules
# Disable firewall temporarily: iptables -F
```

**Data Flow Issues:**
```bash
# Bridge running but no data:
# Check available services:
./bridge --list-services

# Monitor bridge activity:
strace -p $(pgrep bridge) 2>&1 | grep -E '(recv|send)'

# Test ZMQ connection directly:
# Install zeromq tools: apt install zeromq-utils
zmq_sub tcp://192.168.43.1:8086 ""  # Subscribe to all messages

# Check CAN data availability:
candump can0  # If direct CAN access available
```

**Performance Optimization:**
```bash
# Reduce WiFi latency:
# On device, adjust WiFi power management:
iwconfig wlan0 power off

# Optimize bridge for specific services:
./bridge --services carState,modelV2,controlsState,radarState

# Monitor bandwidth usage:
# On device:
iftop -i wlan0  # Real-time bandwidth monitor
# On PC:
nethogs  # Per-process network usage

# WiFi channel optimization:
# Check for interference:
iwlist wlan0 scan | grep -E "(ESSID|Channel|Quality)"
# Change to less crowded channel in device settings
```

**Multiple Device Management:**
```bash
# Connect to multiple comma devices:
# Each device needs unique IP range:
# Device 1: 192.168.43.1 (default)
# Device 2: Configure to 192.168.44.1
# Device 3: Configure to 192.168.45.1

# Launch multiple Cabana instances:
./cabana --zmq 192.168.43.1 &  # Device 1
./cabana --zmq 192.168.44.1 &  # Device 2

# Network namespace isolation (advanced):
sudo ip netns add comma1
sudo ip netns add comma2
# Run each connection in separate namespace
```

### 3. Panda Hardware (Direct CAN)
```bash
# Connect to any available panda
./cabana --panda

# Connect to specific panda by serial
./cabana --panda-serial <serial_number>

# Hardware requirements:
# - panda connected via USB
# - Vehicle CAN access (OBD-II or direct)
# - Appropriate CAN speeds configured
```

### 4. SocketCAN Interface (Linux)
```bash
# Connect to SocketCAN device
./cabana --socketcan can0

# Prerequisites:
sudo modprobe can
sudo modprobe vcan  
sudo ip link add dev can0 type can
sudo ip link set can0 up type can bitrate 500000
```

### 5. Demo Mode
```bash
# Quick start with demo data
./cabana --demo
```

## Basic Usage

### Launch Modes

#### Stream Selection Dialog
```bash
# Launch without arguments for GUI selector
./cabana

# Dialog provides options for:
# - Route selection with search
# - Live device configuration  
# - Hardware source selection
# - Local file browsing
```

#### Direct Launch
```bash
# Route analysis
./cabana "route_name"

# Live streaming
./cabana --zmq <ip_address>

# Hardware connection
./cabana --panda
```

### Interface Overview

#### Main Window Layout
- **Left Panel**: Message list with frequency/status
- **Center**: Detailed message view and signal analysis  
- **Right Panel**: Signal plotting and statistics
- **Bottom**: Video player (when available)
- **Top Menu**: File operations, DBC management, tools

#### Key Features
- **Real-time Updates**: Live message frequency counters
- **Signal Decoding**: Automatic value conversion via DBC
- **Multi-bus Support**: Separate analysis for different CAN buses
- **Time Navigation**: Scrub through historical data
- **Export Tools**: CSV, signal data, DBC files

## Live CAN Analysis

### Real-Time Monitoring

#### Panda Connection
```bash
# Start live monitoring
./cabana --panda

# Interface shows:
# - Live message frequencies (Hz)
# - Signal value updates in real-time  
# - Bus utilization statistics
# - Error frame detection
```

#### Network Streaming
```bash
# Connect to device
ssh comma-device
cd /data/openpilot/cereal/messaging && ./bridge

# Start streaming analysis
./cabana --zmq <device_ip>

# Features:
# - Remote CAN monitoring
# - No physical panda required
# - Network latency indicators
# - Automatic reconnection
```

### CAN Bus Configuration

#### Speed Settings (Panda)
```cpp
// Default CAN speeds (configurable via interface):
Bus 0: 500 kbps  // Standard automotive CAN
Bus 1: 250 kbps  // Lower speed networks
Bus 2: 125 kbps  // Diagnostic networks

// CAN-FD Support (Red Panda):
Data Rate: Up to 5 Mbps
Fallback: Automatic to standard CAN
```

#### Safety Model
```cpp
// Panda safety configuration:
panda->set_safety_model(cereal::CarParams::SafetyModel::SILENT);
// Enables full CAN monitoring without vehicle control restrictions
```

### Live Data Features

#### Message Monitoring
- **Frequency Display**: Real-time Hz counters per message
- **Delta Highlighting**: Changes highlighted in real-time
- **Bus Load Analysis**: Utilization percentage per bus
- **Error Detection**: CAN error frames and statistics

#### Signal Tracking
- **Live Values**: Decoded signal values update continuously
- **Range Monitoring**: Min/max tracking over time
- **Trend Analysis**: Visual signal plotting
- **Threshold Alerts**: Configurable value warnings

## DBC Management

### Automatic DBC Loading

#### Vehicle Detection
```cpp
// Automatic process:
1. Read carParams from route/stream
2. Identify vehicle fingerprint
3. Match to opendbc database
4. Load appropriate DBC file
5. Enable signal decoding
```

#### Supported Vehicles
- **Toyota**: Prius, RAV4, Corolla, Camry, etc.
- **Honda**: Civic, Accord, Pilot, CR-V, etc.  
- **Hyundai/Genesis**: Sonata, Elantra, Genesis G90, etc.
- **Ford**: Focus, Escape, F-150, etc.
- **200+ Vehicle Models** in opendbc database

### Manual DBC Operations

#### Load DBC File
```bash
# Load specific DBC
./cabana --dbc toyota_prius_2017.dbc "route_name"

# Available DBCs:
ls opendbc_repo/opendbc/dbc/*.dbc
```

#### DBC File Editing
1. **Open DBC**: File → Open DBC File
2. **Edit Signals**: Double-click signals to modify
3. **Add Messages**: Right-click message list
4. **Signal Properties**: Name, offset, scale, units
5. **Save Changes**: File → Save DBC File

#### opendbc Integration
- **Direct GitHub Access**: Load DBC files from source
- **Fork Management**: Save to personal opendbc fork
- **Version Control**: Track DBC changes and updates
- **Contribution**: Submit improvements back to community

### DBC Validation
- **Signal Range Checking**: Verify values within expected bounds
- **Message Frequency**: Ensure expected transmission rates
- **Dependency Analysis**: Check signal relationships
- **Format Compliance**: DBC syntax validation

## Signal Analysis

### Real-Time Signal Decoding

#### Automatic Processing
```cpp
// Signal decoding pipeline:
Raw CAN Data → DBC Lookup → Scale/Offset → Units → Display
   (hex)         (signal)      (math)     (name)    (value)
```

#### Signal Properties
- **Name**: Human-readable signal identifier
- **Offset/Scale**: Mathematical conversion factors
- **Units**: Engineering units (mph, °C, %, etc.)
- **Range**: Valid value bounds
- **Precision**: Decimal places for display

### Analysis Tools

#### Signal Plotting
- **Time Series**: Historical signal values over time
- **Correlation**: Compare multiple signals simultaneously  
- **Statistics**: Min, max, average, standard deviation
- **Export**: CSV data for external analysis

#### Pattern Detection
- **Change Detection**: Identify when signals change
- **Bit Pattern Analysis**: Find similar bit patterns across messages
- **Signal Search**: Locate signals by name or characteristics
- **Dependency Mapping**: Understand signal relationships

### Advanced Analysis

#### Reverse Engineering
1. **Find Similar Bits**: Tools → Find Similar Bits
2. **Signal Discovery**: Tools → Find Signal
3. **Pattern Matching**: Analyze unknown message structures
4. **Bit Manipulation**: Manual bit-level analysis

#### Statistical Analysis
- **Message Frequency**: Transmission rate analysis
- **Signal Correlation**: Cross-signal relationship analysis  
- **Data Quality**: Missing message detection
- **Performance Metrics**: Bus utilization and efficiency

## Advanced Features

### Video Synchronization

#### Multi-Camera Support
```bash
# Load route with video
./cabana --qcam --dcam "route_name"

# Features:
# - Synchronized timeline between CAN and video
# - Visual correlation of events
# - Frame-accurate navigation
# - Multi-camera switching
```

#### Timeline Navigation
- **Scrub Control**: Drag timeline to any point
- **Frame Stepping**: Arrow keys for precise navigation
- **Event Bookmarking**: Mark interesting events
- **Zoom Controls**: Focus on specific time ranges

### Export & Integration

#### Data Export
```cpp
// Available export formats:
1. CSV (Time, Address, Bus, Data)
2. Signal CSV (Time, Signal Name, Value)  
3. DBC File Export
4. Message Logs
5. Statistical Reports
```

#### CSV Export Options
```bash
# Export raw CAN data
File → Export to CSV → Raw Data

# Export decoded signals  
File → Export to CSV → Signal Data

# Custom time ranges supported
# Configurable precision and formatting
```

### Development Tools

#### DBC Creation Workflow
1. **New DBC**: File → New DBC File
2. **Add Messages**: Define CAN message IDs
3. **Define Signals**: Add signals with properties
4. **Test & Validate**: Verify against live data
5. **Share**: Export or commit to opendbc

#### Signal Development
- **Live Testing**: Test DBC changes against live data
- **Iteration**: Rapid prototyping of signal definitions
- **Validation**: Automatic range and sanity checking
- **Documentation**: Built-in signal documentation

## Export & Integration

### Comma Connect Integration

#### Route Sharing
- **URL Support**: Direct cabana.comma.ai URLs accepted
- **Share Generation**: Create shareable analysis links
- **Collaborative Analysis**: Team review and discussion
- **Version Control**: Track analysis iterations

#### Cloud Storage
- **Automatic Sync**: Route data cached locally
- **Offline Mode**: Work with downloaded data
- **Selective Download**: Choose specific segments
- **Storage Management**: Automatic cleanup options

### External Tool Integration

#### PlotJuggler Connection
```bash
# Export to PlotJuggler format
File → Export for PlotJuggler

# Direct streaming to PlotJuggler
# 1. Start cabana with live data
# 2. Start PlotJuggler streaming mode
# 3. Select Cereal subscriber
```

#### Data Pipeline
```python
# Python integration example:
import pandas as pd
can_data = pd.read_csv('cabana_export.csv')
# Further analysis in Python/MATLAB/R
```

## Troubleshooting

### Connection Issues

#### Panda Not Detected
```
Error: Failed to connect to panda

Solutions:
1. Check USB connection
2. Verify panda LED status (should be solid)
3. Check permissions: sudo usermod -a -G dialout $USER
4. Restart panda: unplug/replug USB
5. Try different USB port/cable
```

#### ZMQ Streaming Problems
```
Error: Cannot connect to remote device

Solutions:
1. Verify device bridge: ssh device && ./cereal/messaging/bridge
2. Check network connectivity: ping <device_ip>
3. Confirm firewall settings
4. Try local replay first: ./cabana --demo
5. Verify ZMQ port accessibility
```

#### SocketCAN Issues
```
Error: SocketCAN plugin not available

Solutions (Linux only):
1. Install Qt CAN support: sudo apt install qtcanbus5-dev
2. Load kernel modules: sudo modprobe can && sudo modprobe vcan
3. Configure interface: sudo ip link set can0 up type can bitrate 500000
4. Check permissions: sudo usermod -a -G can $USER
```

### Data Analysis Issues

#### Missing Signal Decoding
```
Problem: CAN messages show but no signal names

Solutions:
1. Verify DBC file loaded: Check status bar
2. Manual DBC load: File → Open DBC File
3. Check vehicle compatibility: Verify fingerprint match
4. Update opendbc: git pull in opendbc_repo/
5. Custom DBC creation: File → New DBC File
```

#### Performance Issues
```
Problem: UI freezing or slow response

Solutions:
1. Reduce message filtering: Focus on specific addresses
2. Limit time range: Use smaller segments
3. Close unused panels: Hide video if not needed
4. Restart application: Fresh memory allocation
5. Check system resources: Monitor RAM/CPU usage
```

### DBC Management Issues

#### File Format Errors
```
Error: Invalid DBC file format

Solutions:  
1. Verify DBC syntax: Use DBC validation tools
2. Check character encoding: Ensure UTF-8
3. Manual editing: Fix syntax errors
4. Use templates: Start from working DBC
5. opendbc reference: Check similar vehicle DBCs
```

## API Reference

### Command Line Interface
```bash
cabana [options] [route]

Arguments:
  route                 Route name, segment, or cabana URL

Data Source Options:
  --demo               Use demo route
  --auto               Auto-select best source  
  --zmq <ip>           Connect via ZMQ to IP address
  --panda              Connect to any available panda
  --panda-serial <s>   Connect to panda with specific serial
  --socketcan <dev>    Connect to SocketCAN device
  --msgq               Read from local message queue

Camera Options:  
  --qcam               Load low-res road camera
  --ecam               Load wide-angle camera
  --dcam               Load driver camera
  --no-vipc            Disable video processing

DBC Options:
  --dbc <file>         Load specific DBC file

Data Options:
  --data_dir <path>    Local route directory
```

### Core Classes

#### Stream Management
```cpp
class AbstractStream {
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool liveStreaming() const = 0;
    virtual void seekTo(double ts) = 0;
};

// Concrete implementations:
class PandaStream : public LiveStream
class SocketCanStream : public LiveStream  
class ReplayStream : public AbstractStream
class DeviceStream : public LiveStream
```

#### DBC Management
```cpp
class DBCManager {
    void open(const QString &dbc_file_name);
    void save(const QString &filename = "");
    void generateDBC();
    const DBCFile *file() const { return dbc_file.get(); }
};
```

#### Signal Processing
```cpp
class Signal {
    QString name;
    double offset, scale;
    QString unit;
    int precision;
    double getValue(const uint8_t *data) const;
};
```

### Configuration Structures
```cpp
struct PandaStreamConfig {
    QString serial;
    std::vector<BusConfig> bus_config;
    
    struct BusConfig {
        int can_speed_kbps = 500;
        bool can_fd = false;
        int data_speed_kbps = 2000;
    };
};

struct SocketCanStreamConfig {
    QString device = "can0";
};
```

### Integration APIs

#### Export Functions
```cpp
// CSV export
void exportToCSV(const QString &filename, 
                 double start_time, double end_time,
                 const SourceSet &sources);

// Signal export  
void exportSignalsToCSV(const QString &filename,
                        const std::vector<Signal*> &signals,
                        double start_time, double end_time);
```

#### DBC Operations
```cpp
// DBC file generation
void generateDBCFromFingerprint(const QString &fingerprint);

// Signal manipulation
void addSignal(Message *msg, const Signal &signal);
void removeSignal(Message *msg, Signal *signal);
void editSignal(Signal *signal, const Signal &new_signal);
```

## Best Practices

### Development Workflow
1. **Start with Demo**: Verify installation with demo data
2. **Use Appropriate Source**: Match data source to analysis goal
3. **DBC Validation**: Always verify signal decoding accuracy
4. **Incremental Analysis**: Focus on specific messages/signals
5. **Documentation**: Record findings and DBC changes

### Production Analysis
1. **Live Monitoring**: Use panda/streaming for real-time analysis
2. **Systematic Approach**: Analyze all relevant signals
3. **Cross-Validation**: Verify findings across multiple routes
4. **Integration**: Export results for further analysis
5. **Collaboration**: Share findings via comma connect

### Performance Guidelines
- **Hardware**: USB 3.0 for panda connections
- **Network**: Stable connection for streaming mode  
- **Memory**: 2GB+ recommended for large routes
- **CPU**: Multi-core benefits for real-time processing

## Conclusion

Cabana provides comprehensive CAN analysis capabilities essential for vehicle integration and debugging. Its multi-source support (panda, SocketCAN, network streaming, route replay) combined with professional DBC management makes it the primary tool for CAN bus analysis in the nagaspilot ecosystem.

The real-time capabilities enable live vehicle debugging, while the replay functionality supports detailed post-analysis. The integrated DBC editor and opendbc connection provide complete workflow support from reverse engineering to production deployment.