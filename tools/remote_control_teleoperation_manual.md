# Remote Control & Teleoperation System - User Manual

## Overview

The nagaspilot Remote Control & Teleoperation system provides comprehensive capabilities for remote vehicle operation, manual control testing, and development workflows. It includes joystick control, web-based teleoperation, and PC webcam integration for safe testing and development.

**🎮 System Types: RUNTIME + CONTROL + TELEOPERATION + DEVELOPMENT**
- **✅ Runtime**: Live vehicle control during operation
- **✅ Remote Control**: Network-based vehicle operation via WiFi
- **✅ Teleoperation**: Web interface for remote operation
- **✅ Development**: PC-based testing with webcams
- **✅ Safety Systems**: Manual override and emergency controls

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Installation & Setup](#installation--setup)
3. [Joystick Control System](#joystick-control-system)
4. [Body Teleoperation](#body-teleoperation)
5. [Webcam Integration](#webcam-integration)
6. [WiFi & Network Setup](#wifi--network-setup)
7. [Safety Systems](#safety-systems)
8. [Control Interfaces](#control-interfaces)
9. [Performance Optimization](#performance-optimization)
10. [Troubleshooting](#troubleshooting)
11. [API Reference](#api-reference)

## System Architecture

### Core Components
- **Joystick Control** (`tools/joystick/`) - Manual control via gamepad/keyboard
- **Body Teleoperation** (`tools/bodyteleop/`) - Web-based remote operation
- **Webcam System** (`tools/webcam/`) - PC camera integration
- **Safety Override** - Emergency manual control systems
- **Network Bridge** - Remote control message routing

### Data Flow Architecture
```
Control Input → Processing → Network → Vehicle → Safety Check → Actuation
     ↓             ↓           ↓         ↓           ↓            ↓
  Joystick      Command     WiFi/ZMQ   Comma      Override    Steering
  Keyboard      Validation  Bridge     Device     System      Acceleration
  Web UI        Rate Limit  Message    Safety     Monitor     Braking
                Safety      Queue      Check                  
```

### Integration Points
- **Vehicle Safety Systems** - Cruise control, manual override
- **Network Infrastructure** - WiFi, ZMQ messaging
- **Hardware Interfaces** - USB joysticks, webcams, panda
- **Safety Monitoring** - Continuous connection validation

## Installation & Setup

### Prerequisites
```bash
# Ensure nagaspilot environment
cd /path/to/nagaspilot
source .venv/bin/activate

# Install joystick dependencies
pip install inputs pygame

# Install web interface dependencies
pip install flask websockets webrtc

# System hardware support
sudo apt install joystick jstest-gtk  # Linux joystick tools
```

### Hardware Requirements

#### Joystick Control
- **Supported Controllers**: Xbox, PlayStation, Logitech gamepads
- **USB Connection**: USB 2.0+ for wired controllers
- **Wireless**: Bluetooth 4.0+ for wireless controllers
- **Compatibility**: Windows, Linux, macOS support

#### Body Teleoperation
- **Network**: Stable WiFi connection (5GHz recommended)
- **Browser**: Chrome/Firefox with WebRTC support
- **Bandwidth**: 2+ Mbps for video streaming
- **Latency**: <100ms for responsive control

#### Webcam Integration
- **Camera**: USB webcam, 720p+, 78° FOV minimum
- **Recommended**: Logitech C920/C615, NexiGo N60
- **GPU**: NVIDIA GPU recommended for processing
- **Platform**: Ubuntu 24.04 or macOS (WSL2 not supported)

## Joystick Control System

### Local Joystick Control

#### Setup on Comma Device
```bash
# SSH into comma device
ssh root@192.168.43.1

# Ensure vehicle is OFF and openpilot is OFFROAD
# Connect joystick to comma device aux USB-C port

# Start joystick control (local mode)
cd /data/openpilot/tools/joystick
./joystick_control.py

# Or keyboard mode (no joystick required)
./joystick_control.py --keyboard
```

#### Control Mappings (Default)
```bash
# Keyboard Controls (WASD):
W - Accelerate (5% increments)
S - Brake (5% increments)
A - Steer left (5% increments)
D - Steer right (5% increments)
SPACE - Emergency brake
ESC - Exit control mode

# Joystick Controls (Standard gamepad):
Left Stick Y-axis - Acceleration/braking
Left Stick X-axis - Steering input
Right Trigger - Acceleration
Left Trigger - Braking
Start Button - Engage/disengage
Select Button - Emergency stop
```

### Network Joystick Control (WiFi)

#### Complete WiFi Setup Process

**Step 1: Comma Device Configuration**
```bash
# Enable joystick debug mode on device
ssh root@192.168.43.1
echo -n "1" > /data/params/d/JoystickDebugMode

# Start message bridge for network control
cd /data/openpilot/cereal/messaging
./bridge {LAPTOP_IP} testJoystick

# Example with actual IP:
./bridge 192.168.43.100 testJoystick
```

**Step 2: PC Network Setup**
```bash
# Connect PC to comma device hotspot
sudo nmcli dev wifi connect "comma-XXXXX" password "your_password"

# Verify connection and get PC IP
ip addr show | grep 192.168.43
# Note your PC IP (e.g., 192.168.43.100)

# Set ZMQ environment for network mode
export ZMQ=1
```

**Step 3: PC Joystick Control**
```bash
# Connect joystick to PC
# Verify joystick detection:
ls /dev/input/js*  # Linux
# Or use System Preferences on macOS

# Start network joystick control
cd tools/joystick
ZMQ=1 ./joystick_control.py

# Keyboard mode over network
ZMQ=1 ./joystick_control.py --keyboard
```

**Step 4: Vehicle Operation**
```bash
# 1. Start car engine
# 2. Engage cruise control (required for joystick mode)
# 3. Joystick control becomes active
# 4. Monitor status on device screen
# 5. Use manual override for immediate control
```

## Body Teleoperation

### Web-Based Remote Control

#### Architecture Overview
```
Browser (PC) → WebRTC → WiFi → Comma Device → Vehicle Control
     ↓           ↓        ↓        ↓             ↓
   Web UI    Video     Network   Safety      Steering
   Controls  Stream   Bridge    Monitor     Acceleration
   Gamepad   Audio    ZMQ       Override    Braking
```

#### Setup Process

**Step 1: Device Web Server**
```bash
# SSH into comma device
ssh root@192.168.43.1

# Start teleoperation server
cd /data/openpilot/tools/bodyteleop
python3 web.py

# Server starts on port 8080
# Access via: http://192.168.43.1:8080
```

**Step 2: PC Browser Access**
```bash
# Connect PC to comma device WiFi
# Open browser and navigate to:
http://192.168.43.1:8080

# Or use device hostname if configured:
http://comma-device.local:8080
```

**Step 3: Web Interface Features**
- **Video Stream**: Real-time camera feed from device
- **Control Interface**: Virtual joystick for steering/acceleration
- **Status Display**: Connection status, vehicle state
- **Emergency Controls**: Stop button, manual override
- **Gamepad Support**: Physical controller via browser

### WebRTC Configuration

#### Video Streaming Setup
```javascript
// WebRTC configuration (automatic)
{
  iceServers: [
    { urls: 'stun:stun.l.google.com:19302' },
    { urls: 'turn:relay.example.com', username: 'user', password: 'pass' }
  ],
  video: {
    width: 1280,
    height: 720,
    framerate: 30
  },
  audio: false  // Disabled for bandwidth
}
```

#### Network Optimization
```bash
# On comma device - optimize for streaming
echo 'net.core.rmem_max = 26214400' >> /etc/sysctl.conf
echo 'net.core.rmem_default = 26214400' >> /etc/sysctl.conf
sysctl -p

# Adjust video quality based on connection
# High quality: 1280x720@30fps
# Medium quality: 854x480@30fps  
# Low quality: 640x360@15fps
```

## Webcam Integration

### PC-Based Operation

#### Hardware Setup
```bash
# Install OpenCL drivers (Ubuntu)
sudo apt install pocl-opencl-icd

# Connect hardware:
# 1. USB webcam to PC
# 2. Panda to PC via USB-A cable
# 3. Car harness connected to panda
```

#### Camera Configuration
```bash
# Specify camera devices
USE_WEBCAM=1 system/manager/manager.py

# Multi-camera setup
USE_WEBCAM=1 ROAD_CAM=0 DRIVER_CAM=1 WIDE_CAM=2 system/manager/manager.py

# Check available cameras
ls /dev/video*  # Linux
system_profiler SPCameraDataType  # macOS
```

#### Operation Procedure
```bash
# 1. Start PC webcam system
USE_WEBCAM=1 system/manager/manager.py

# 2. Start vehicle (engine running)
# 3. Webcam system calibrates automatically
# 4. Engage openpilot when calibration complete
# 5. Manual override available via steering wheel
```

## WiFi & Network Setup

### Complete Network Configuration

#### Device Hotspot Optimization
```bash
# SSH into comma device
ssh root@192.168.43.1

# Optimize WiFi for low latency
iwconfig wlan0 power off  # Disable power saving
iwconfig wlan0 rate 54M   # Set fixed rate

# Configure hostapd for performance
cat > /etc/hostapd/hostapd.conf << EOF
interface=wlan0
driver=nl80211
ssid=comma-XXXXX
hw_mode=g
channel=6
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=your_password
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF
```

#### PC Network Optimization
```bash
# Optimize network interface
sudo ethtool -G wlan0 rx 4096 tx 4096  # Increase buffers
sudo iwconfig wlan0 power off          # Disable power management

# Set QoS for real-time traffic
sudo tc qdisc add dev wlan0 root handle 1: htb default 30
sudo tc class add dev wlan0 parent 1: classid 1:1 htb rate 54mbit
sudo tc class add dev wlan0 parent 1:1 classid 1:10 htb rate 20mbit ceil 54mbit prio 1
sudo tc filter add dev wlan0 protocol ip parent 1:0 prio 1 u32 match ip dport 8086 0xffff flowid 1:10
```

#### Latency Optimization
```bash
# Monitor network latency
ping -i 0.1 192.168.43.1  # 100ms interval
mtr 192.168.43.1          # Continuous monitoring

# Optimize kernel network stack
echo 'net.ipv4.tcp_congestion_control=bbr' >> /etc/sysctl.conf
echo 'net.core.default_qdisc=fq' >> /etc/sysctl.conf
sysctl -p
```

## Safety Systems

### Emergency Controls

#### Manual Override
```bash
# Always available overrides:
# 1. Steering wheel input - immediate steering override
# 2. Brake pedal - immediate deceleration
# 3. Accelerator pedal - immediate acceleration override
# 4. Emergency stop button - cuts all automated control
```

#### Connection Monitoring
```python
# Automatic safety checks:
class SafetyMonitor:
    def __init__(self):
        self.last_heartbeat = time.time()
        self.connection_timeout = 2.0  # seconds
        
    def check_connection(self):
        if time.time() - self.last_heartbeat > self.connection_timeout:
            self.emergency_stop()
            
    def emergency_stop(self):
        # 1. Disable automated control
        # 2. Apply gradual braking
        # 3. Alert driver
        # 4. Log incident
```

#### Operational Limits
```bash
# Built-in safety limits:
Max Speed: 25 mph in joystick mode
Max Acceleration: 2.0 m/s²
Max Deceleration: -3.0 m/s²
Max Steering Rate: 3.0 degrees/second
Connection Timeout: 2.0 seconds
```

## Control Interfaces

### Command Structure

#### Joystick Messages
```python
# testJoystick message structure
{
  'axes': [0.0, 0.0, 0.0, 0.0],      # [steering, accel, brake, unused]
  'buttons': [False, False, ...],     # Button states
  'timestamp': 1234567890            # Message timestamp
}
```

#### Web Control API
```javascript
// Control command structure
{
  "steering": 0.0,      // -1.0 to 1.0
  "acceleration": 0.0,  // -1.0 to 1.0 (negative = brake)
  "timestamp": Date.now(),
  "sequence": 12345
}
```

### Input Validation
```python
# All control inputs validated:
def validate_control(steering, acceleration):
    steering = np.clip(steering, -1.0, 1.0)
    acceleration = np.clip(acceleration, -1.0, 1.0)
    
    # Rate limiting
    if abs(steering - last_steering) > MAX_STEERING_RATE:
        steering = last_steering + np.sign(steering - last_steering) * MAX_STEERING_RATE
    
    return steering, acceleration
```

## Performance Optimization

### Network Performance
```bash
# Monitor bandwidth usage
iftop -i wlan0         # Real-time bandwidth
nethogs               # Per-process usage
tc -s qdisc show dev wlan0  # QoS statistics
```

### Control Latency
```bash
# Measure control loop latency
echo 'Control input → Processing → Network → Vehicle → Response'
# Target: <100ms total latency
# Network: <50ms
# Processing: <30ms  
# Vehicle response: <20ms
```

### System Resources
```bash
# Monitor system performance
htop                  # CPU usage
iotop                 # I/O usage
nethogs               # Network usage per process
```

## Troubleshooting

### Connection Issues

#### Joystick Not Detected
```bash
# Linux joystick troubleshooting
ls /dev/input/js*     # Check device files
jstest /dev/input/js0 # Test joystick input
sudo usermod -a -G input $USER  # Add user to input group

# Check joystick library
python3 -c "import inputs; print(inputs.get_gamepad())"
```

#### Network Control Fails
```bash
# Check ZMQ connectivity
telnet 192.168.43.1 8086  # Test ZMQ port
nc -zv 192.168.43.1 8086  # Port connectivity

# Check bridge process
ssh root@192.168.43.1
ps aux | grep bridge
netstat -tuln | grep 8086
```

#### WebRTC Streaming Issues
```bash
# Browser console errors
# Check for WebRTC support:
navigator.mediaDevices.getUserMedia({video: true})

# Network connectivity
ping 192.168.43.1
traceroute 192.168.43.1

# Firewall issues
sudo ufw allow 8080/tcp
iptables -L | grep 8080
```

### Vehicle Integration Issues

#### Cruise Control Required
```bash
# Joystick mode requires cruise engagement
# 1. Start engine
# 2. Reach minimum speed (usually 20+ mph)
# 3. Press cruise control SET/RESUME
# 4. Joystick mode activates
```

#### Safety System Conflicts
```bash
# Common conflicts:
# - Lane departure warning active
# - Collision avoidance system engaged
# - Electronic stability control intervention
# - Manual transmission (not supported)
```

### Performance Issues

#### High Latency
```bash
# Network latency troubleshooting
ping -c 100 192.168.43.1 | tail -1  # Average latency
mtr --report 192.168.43.1           # Network path analysis

# Optimize network stack
echo 'net.ipv4.tcp_low_latency=1' >> /etc/sysctl.conf
```

#### Video Streaming Lag
```bash
# Reduce video quality
# In web interface: Settings → Video Quality → Low
# Or modify WebRTC constraints:
video: { width: 640, height: 360, framerate: 15 }
```

## API Reference

### Command Line Interfaces

#### Joystick Control
```bash
joystick_control.py [-h] [--keyboard]

Options:
  --keyboard    Use keyboard instead of joystick input
  
Environment Variables:
  ZMQ=1        Enable network mode (required for remote control)
  
Examples:
  ./joystick_control.py                    # Local joystick mode
  ./joystick_control.py --keyboard         # Local keyboard mode
  ZMQ=1 ./joystick_control.py             # Network joystick mode
```

#### Body Teleoperation
```bash
web.py [-h] [--port PORT] [--host HOST]

Options:
  --port PORT   Web server port (default: 8080)
  --host HOST   Bind address (default: 0.0.0.0)
  
Examples:
  python3 web.py                           # Default configuration
  python3 web.py --port 8888              # Custom port
```

#### Webcam System
```bash
USE_WEBCAM=1 [ROAD_CAM=0] [DRIVER_CAM=1] [WIDE_CAM=2] system/manager/manager.py

Environment Variables:
  USE_WEBCAM=1    Enable webcam mode
  ROAD_CAM=N      Road camera device number
  DRIVER_CAM=N    Driver camera device number  
  WIDE_CAM=N      Wide camera device number
```

### Core Classes

#### Joystick Control
```python
class JoystickController:
    def __init__(self, use_keyboard=False):
        self.use_keyboard = use_keyboard
        self.gamepad = None
        
    def get_input(self):
        """Get current control input"""
        return {
            'steering': float,     # -1.0 to 1.0
            'acceleration': float, # -1.0 to 1.0  
            'buttons': dict       # Button states
        }
```

#### Web Teleoperation
```python
class WebTeleopServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        
    def start_server(self):
        """Start web server and WebRTC stream"""
        
    def handle_control(self, data):
        """Process control commands from web interface"""
```

### Safety Functions
```python
def validate_control_input(steering, acceleration):
    """Validate and limit control inputs"""
    
def check_safety_conditions():
    """Verify vehicle safety conditions"""
    
def emergency_stop():
    """Execute emergency stop procedure"""
```

## Best Practices

### Development Workflow
1. **Start with Simulation**: Test control logic in simulation first
2. **Local Testing**: Use local joystick mode before network mode
3. **Safety First**: Always have manual override ready
4. **Incremental Testing**: Test each component individually
5. **Monitor Performance**: Watch latency and connection quality

### Production Deployment
1. **Network Optimization**: Configure QoS and optimize WiFi
2. **Safety Validation**: Test all emergency procedures
3. **Performance Monitoring**: Continuous latency monitoring
4. **Backup Systems**: Redundant control paths
5. **Documentation**: Document all safety procedures

### Performance Guidelines
- **Network Latency**: <50ms for responsive control
- **Control Loop**: 20Hz minimum update rate
- **Video Stream**: 15-30fps depending on bandwidth
- **Connection Timeout**: 2 seconds maximum

## Conclusion

The nagaspilot Remote Control & Teleoperation system provides comprehensive capabilities for safe remote vehicle operation and development testing. The multi-modal approach (joystick, web interface, webcam integration) supports various use cases from development to field testing.

Key strengths include robust safety systems, flexible control interfaces, and comprehensive WiFi networking support. Start with local testing and simulation, then gradually progress to network-based remote control as expertise develops.