# Camera Streaming System - WiFi Setup Manual

## Overview

The nagaspilot camera streaming system enables real-time video transmission from comma devices to remote development machines over WiFi networks. This comprehensive system supports multiple camera streams, hardware acceleration, and robust network configuration for debugging, development, and monitoring purposes.

**📹 System Types: RUNTIME + STREAMING + MULTI-CAMERA + DEVELOPMENT**
- **✅ Runtime**: Live camera streams from operational devices
- **✅ Streaming**: Real-time H.265/HEVC encoded video transmission
- **✅ Multi-Camera**: Support for qcam, fcam, ecam, dcam simultaneously
- **✅ Development**: Remote debugging and analysis capabilities
- **✅ Hardware Acceleration**: NVIDIA and software decoding options

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Requirements](#hardware-requirements)
3. [WiFi Network Configuration](#wifi-network-configuration)
4. [SSH Setup & Security](#ssh-setup--security)
5. [Device-Side Configuration](#device-side-configuration)
6. [PC-Side Configuration](#pc-side-configuration)
7. [Multiple Camera Support](#multiple-camera-support)
8. [Network Bridge Configuration](#network-bridge-configuration)
9. [Performance Optimization](#performance-optimization)
10. [Network Troubleshooting](#network-troubleshooting)
11. [Advanced Configuration](#advanced-configuration)
12. [Security Best Practices](#security-best-practices)

## System Architecture

### Core Components
- **camerad** - Hardware camera interface and frame capture
- **encoderd** - H.265/HEVC video encoding and compression
- **bridge** - ZMQ network messaging bridge for data transmission
- **compressed_vipc.py** - Video decoder and VisionIPC republisher
- **VisionIPC Server** - Local video buffer management

### Camera Stream Types
```cpp
// Available camera streams:
VISION_STREAM_ROAD      = 0,  // Main forward camera (qcam)
VISION_STREAM_DRIVER    = 1,  // Driver monitoring camera (dcam) 
VISION_STREAM_WIDE_ROAD = 2,  // Wide-angle road camera (fcam/ecam)
```

### Data Flow Architecture
```
Device Side:              Network:                PC Side:
-----------               --------               --------
camerad → encoderd → bridge → WiFi/Ethernet → compressed_vipc.py → watch3
   ↓         ↓         ↓                           ↓                  ↓
Hardware   H.265     ZMQ                       Decoder           Display
Capture   Encoding  Bridge                    VisionIPC         Interface
```

### Message Flow
```
Camera Hardware → FrameData → H.265 Stream → ZMQ Messages → Network → Decoder → Local VisionIPC
```

## Hardware Requirements

### Device Side (Comma Device)
- **CPU**: Qualcomm Snapdragon 845+ or compatible
- **RAM**: Minimum 4GB for stable streaming
- **Storage**: 32GB+ for temporary buffers
- **Network**: WiFi 802.11ac or Gigabit Ethernet
- **Cameras**: 
  - Road camera (AR0231/OX03C10/OS04C10 sensors)
  - Driver camera (optional)
  - Wide road camera (optional)

### PC Side (Development Machine)
- **CPU**: Intel i5 8th gen+ or AMD Ryzen 5 3600+
- **RAM**: 8GB+ (16GB recommended for multi-camera)
- **GPU**: NVIDIA GTX 1060+ (for hardware acceleration)
- **Network**: WiFi 802.11ac or Gigabit Ethernet
- **Storage**: SSD recommended for buffer management

### Network Infrastructure
- **Router**: WiFi 6 (802.11ax) or WiFi 5 (802.11ac) with QoS support
- **Bandwidth**: Minimum 50Mbps for single camera, 150Mbps+ for multi-camera
- **Latency**: <10ms for real-time applications
- **Packet Loss**: <0.1% for stable streaming

## WiFi Network Configuration

### 1. Router Setup for Optimal Streaming

#### QoS Configuration
```bash
# Configure router QoS (varies by manufacturer)
# Priority settings (highest to lowest):
1. Video streaming traffic (ports 8001-8010)
2. SSH traffic (port 22)
3. Other device traffic
```

#### WiFi Channel Optimization
```bash
# Use WiFi analyzer tools to find optimal channel
# Recommended channels:
- 2.4GHz: 1, 6, 11 (avoid overlap)
- 5GHz: 36, 40, 44, 48, 149, 153, 157, 161

# Set channel width:
- 2.4GHz: 20MHz (avoids interference)
- 5GHz: 80MHz (maximum throughput)
```

#### Network Isolation Settings
```bash
# Disable client isolation on router
Client Isolation: OFF
AP Isolation: OFF
Inter-VLAN Routing: ON (if using VLANs)
```

### 2. Device WiFi Configuration

#### Connect Device to Network
```bash
# SSH into comma device
ssh root@<device-ip>

# Configure WiFi (if not already connected)
wpa_passphrase "NetworkName" "password" >> /etc/wpa_supplicant/wpa_supplicant.conf
systemctl restart wpa_supplicant

# Verify connection
iwconfig wlan0
ip addr show wlan0
```

#### WiFi Power Management
```bash
# Disable WiFi power saving for stable streaming
iwconfig wlan0 power off

# Make persistent across reboots
echo 'iwconfig wlan0 power off' >> /etc/rc.local
```

#### Network Interface Optimization
```bash
# Set WiFi interface for high-performance streaming
ethtool -s wlan0 speed 1000 duplex full
echo 'net.core.rmem_max = 268435456' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 268435456' >> /etc/sysctl.conf
sysctl -p
```

## SSH Setup & Security

### 1. SSH Key-Based Authentication

#### Generate SSH Keys on PC
```bash
# Generate new SSH key pair
ssh-keygen -t ed25519 -C "camera-streaming@$(hostname)"

# Copy to known location
cp ~/.ssh/id_ed25519.pub /tmp/comma_ssh_key.pub
```

#### Deploy Keys to Device
```bash
# Method 1: Using setup script
cd tools/scripts
./setup_ssh_keys.py <github-username>

# Method 2: Manual deployment
ssh-copy-id root@<device-ip>

# Method 3: Direct copy
cat ~/.ssh/id_ed25519.pub | ssh root@<device-ip> 'mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys'
```

#### Verify SSH Access
```bash
# Test passwordless SSH
ssh root@<device-ip> 'echo "SSH working"'

# Test with specific key
ssh -i ~/.ssh/id_ed25519 root@<device-ip> 'uptime'
```

### 2. SSH Configuration Optimization

#### Client-Side SSH Config
```bash
# Add to ~/.ssh/config
Host comma-dev
    HostName <device-ip>
    User root
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 60
    ServerAliveCountMax 3
    Compression yes
    TCPKeepAlive yes
    ControlMaster auto
    ControlPath ~/.ssh/master-%r@%h:%p
    ControlPersist 10m
```

#### Device-Side SSH Hardening
```bash
# Edit /etc/ssh/sshd_config on device
PermitRootLogin yes
PasswordAuthentication no
PubkeyAuthentication yes
ClientAliveInterval 60
ClientAliveCountMax 3
MaxStartups 10:30:100
```

### 3. Firewall Configuration

#### Device-Side Firewall
```bash
# Allow SSH and streaming ports
iptables -A INPUT -p tcp --dport 22 -j ACCEPT
iptables -A INPUT -p tcp --dport 8001:8010 -j ACCEPT
iptables -A INPUT -p udp --dport 8001:8010 -j ACCEPT

# Save rules
iptables-save > /etc/iptables/rules.v4
```

#### PC-Side Firewall
```bash
# Ubuntu/Debian
sudo ufw allow from <device-subnet>/24 to any port 8001:8010
sudo ufw allow ssh
sudo ufw enable

# CentOS/RHEL
firewall-cmd --permanent --add-port=8001-8010/tcp
firewall-cmd --permanent --add-port=8001-8010/udp
firewall-cmd --reload
```

## Device-Side Configuration

### 1. Camera System Startup

#### Automatic Service Method (Recommended)
```bash
# SSH into device
ssh root@<device-ip>

# Create combined service script
cat << 'EOF' > /data/openpilot/start_streaming.sh
#!/bin/bash
export STREAMING_MODE=1

# Start bridge for network communication
cd /data/openpilot/cereal/messaging/
./bridge &
BRIDGE_PID=$!

# Start camerad for camera capture
cd /data/openpilot/system/camerad/
./camerad &
CAMERAD_PID=$!

# Start encoderd for video encoding
cd /data/openpilot/system/loggerd/
./encoderd &
ENCODERD_PID=$!

# Wait for interrupt signal
trap 'kill $BRIDGE_PID $CAMERAD_PID $ENCODERD_PID' SIGINT SIGTERM
wait
EOF

chmod +x /data/openpilot/start_streaming.sh
```

#### Manual Process Management
```bash
# Terminal 1: Start messaging bridge
cd /data/openpilot/cereal/messaging/
./bridge

# Terminal 2: Start camera daemon  
cd /data/openpilot/system/camerad/
./camerad

# Terminal 3: Start encoder daemon
cd /data/openpilot/system/loggerd/
./encoderd
```

#### Single Command Method (For Testing)
```bash
# All processes in background with signal handling
(
  cd /data/openpilot/cereal/messaging/
  ./bridge &

  cd /data/openpilot/system/camerad/
  ./camerad &

  cd /data/openpilot/system/loggerd/
  ./encoderd &

  wait
) ; trap 'kill $(jobs -p)' SIGINT
```

### 2. Process Monitoring and Management

#### Check Process Status
```bash
# Verify all processes are running
ps aux | grep -E "(bridge|camerad|encoderd)" | grep -v grep

# Check network connections
netstat -an | grep :8001

# Monitor system resources
top -p $(pgrep -d, -f "bridge|camerad|encoderd")
```

#### Process Health Monitoring
```bash
# Create monitoring script
cat << 'EOF' > /data/openpilot/monitor_streaming.sh
#!/bin/bash
while true; do
    if ! pgrep bridge > /dev/null; then
        echo "Bridge process died, restarting..."
        cd /data/openpilot/cereal/messaging/ && ./bridge &
    fi
    
    if ! pgrep camerad > /dev/null; then
        echo "Camerad process died, restarting..."
        cd /data/openpilot/system/camerad/ && ./camerad &
    fi
    
    if ! pgrep encoderd > /dev/null; then
        echo "Encoderd process died, restarting..."
        cd /data/openpilot/system/loggerd/ && ./encoderd &
    fi
    
    sleep 10
done
EOF

chmod +x /data/openpilot/monitor_streaming.sh
```

### 3. System Resource Optimization

#### CPU Affinity Settings
```bash
# Set CPU affinity for streaming processes
echo 'taskset -cp 4-7 $(pgrep camerad)' >> /data/openpilot/start_streaming.sh
echo 'taskset -cp 0-3 $(pgrep encoderd)' >> /data/openpilot/start_streaming.sh
echo 'taskset -cp 0-1 $(pgrep bridge)' >> /data/openpilot/start_streaming.sh
```

#### Memory Management
```bash
# Increase network buffer sizes
echo 'net.core.netdev_max_backlog = 5000' >> /etc/sysctl.conf
echo 'net.core.netdev_budget = 600' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_congestion_control = bbr' >> /etc/sysctl.conf
sysctl -p
```

## PC-Side Configuration

### 1. Environment Setup

#### Install Dependencies
```bash
# Navigate to openpilot directory
cd /path/to/nagaspilot

# Ensure environment is activated
source .venv/bin/activate

# Install camera streaming dependencies
pip install av msgpack pyzmq

# For NVIDIA hardware acceleration (optional)
pip install pynvcodec
```

#### Verify Installation
```bash
# Test basic imports
python3 -c "import av, msgpack, zmq; print('Dependencies OK')"

# Check for NVIDIA codec support
python3 -c "import PyNvCodec; print('NVIDIA codec available')" 2>/dev/null || echo "Software decoding only"
```

### 2. Basic Camera Streaming

#### Single Camera Stream
```bash
# Navigate to camera streaming directory
cd tools/camerastream

# Start single camera stream (road camera only)
./compressed_vipc.py <device-ip> --cams 0

# Start with debug output
./compressed_vipc.py <device-ip> --cams 0 --silent=false
```

#### Display Stream
```bash
# In separate terminal, start video display
cd selfdrive/ui
./watch3

# Alternative: Use UI for full interface
cd selfdrive/ui
./ui
```

### 3. Advanced Streaming Options

#### Hardware Acceleration
```bash
# Use NVIDIA hardware decoding (if available)
./compressed_vipc.py <device-ip> --cams 0,1,2 --nvidia

# Check GPU utilization during streaming
nvidia-smi -l 1
```

#### Multiple Camera Configuration
```bash
# Stream all cameras
./compressed_vipc.py <device-ip> --cams 0,1,2

# Stream specific combinations:
# Road + Driver cameras
./compressed_vipc.py <device-ip> --cams 0,1

# Road + Wide Road cameras  
./compressed_vipc.py <device-ip> --cams 0,2

# Driver camera only
./compressed_vipc.py <device-ip> --cams 1
```

## Multiple Camera Support

### 1. Camera Stream Mapping

#### Camera Type Reference
```python
# Camera stream type definitions
VISION_STREAM_ROAD      = 0  # qcam - Main forward camera (1928x1208)
VISION_STREAM_DRIVER    = 1  # dcam - Driver monitoring (1928x1208)  
VISION_STREAM_WIDE_ROAD = 2  # fcam/ecam - Wide angle road (1928x1208)
```

#### Service Name Mapping
```python
ENCODE_SOCKETS = {
    0: "roadEncodeData",      # Main road camera stream
    1: "driverEncodeData",    # Driver monitoring stream
    2: "wideRoadEncodeData",  # Wide road camera stream
}
```

### 2. Multi-Camera Network Requirements

#### Bandwidth Calculations
```bash
# Per-camera bandwidth (approximate):
# - Single camera (1928x1208 @ 20fps): ~15-25 Mbps
# - All three cameras: ~45-75 Mbps
# - Add 20% overhead for network protocols

# Network capacity check:
iperf3 -c <device-ip> -t 30 -P 3
```

#### Buffer Management
```python
# VisionIPC buffer configuration
CAMERA_BUFFERS = {
    "road": 4,      # 4 frame circular buffer
    "driver": 4,    # 4 frame circular buffer  
    "wide_road": 4, # 4 frame circular buffer
}

# Total memory usage: ~150MB per camera stream
```

### 3. Synchronized Multi-Camera Streaming

#### Timestamp Synchronization
```bash
# Verify timestamp alignment across cameras
./compressed_vipc.py <device-ip> --cams 0,1,2 --debug | grep "timestamp"

# Check frame alignment (should be within ~5ms)
```

#### Camera Calibration Verification
```bash
# Check camera calibration data
cd common/transformations
python3 -c "
import camera
print('Road camera:', camera.ROAD_CAM)
print('Driver camera:', camera.DRIVER_CAM)
print('Wide road camera:', camera.WIDE_ROAD_CAM)
"
```

## Network Bridge Configuration

### 1. ZMQ Bridge Setup

#### Understanding the Bridge
```cpp
// Bridge functionality (from bridge.cc):
// - msgq_to_zmq: Internal message queue to ZMQ network
// - zmq_to_msgq: ZMQ network to internal message queue
// - Bidirectional message forwarding
// - Service filtering and whitelisting
```

#### Bridge Configuration
```bash
# Start bridge with specific service filtering
cd /data/openpilot/cereal/messaging

# Bridge all camera services
./bridge

# Check bridge status
netstat -an | grep 8001
lsof -i :8001
```

#### Custom Bridge Configuration
```bash
# Advanced bridge setup with service filtering
export BRIDGE_SERVICES="roadEncodeData,driverEncodeData,wideRoadEncodeData"
./bridge --services=$BRIDGE_SERVICES
```

### 2. Network Port Management

#### Default Port Assignments
```bash
# Standard ZMQ ports used by bridge:
8001  # Primary camera data port
8002  # Secondary services
8003  # Control messages
8004-8010  # Additional services as needed
```

#### Port Conflict Resolution
```bash
# Check for port conflicts
sudo netstat -tulpn | grep -E ":(8001|8002|8003|8004|8005)"

# Kill conflicting processes
sudo lsof -ti:8001 | xargs kill -9
```

#### Firewall Port Configuration
```bash
# Open required ports on device
iptables -A INPUT -p tcp --dport 8001:8010 -s <pc-subnet>/24 -j ACCEPT
iptables -A INPUT -p udp --dport 8001:8010 -s <pc-subnet>/24 -j ACCEPT

# Save configuration
iptables-save > /etc/iptables.rules
```

### 3. Network Protocol Optimization

#### TCP vs UDP Configuration
```bash
# Camera streaming uses TCP by default for reliability
# Configure TCP optimization for video streaming

# On device:
echo 'net.ipv4.tcp_congestion_control = bbr' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_window_scaling = 1' >> /etc/sysctl.conf
echo 'net.core.rmem_max = 67108864' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 67108864' >> /etc/sysctl.conf
sysctl -p
```

#### Quality of Service (QoS)
```bash
# Set packet priority for camera streams
tc qdisc add dev wlan0 root handle 1: htb default 30
tc class add dev wlan0 parent 1: classid 1:1 htb rate 100mbit
tc class add dev wlan0 parent 1:1 classid 1:10 htb rate 80mbit ceil 100mbit prio 1
tc filter add dev wlan0 protocol ip parent 1:0 prio 1 u32 match ip dport 8001 0xffff flowid 1:10
```

## Performance Optimization

### 1. Network Performance Tuning

#### Router Optimization
```bash
# Router settings for optimal streaming:
- Channel Width: 80MHz (5GHz) or 40MHz (2.4GHz)
- QoS: Enable with video priority
- Band Steering: Prefer 5GHz for high bandwidth
- Buffer Bloat: Enable Smart Queue Management (SQM)
- MIMO: Enable spatial streams
```

#### Network Interface Tuning
```bash
# Device-side network optimization
ethtool -K wlan0 gso off gro off tso off
echo 'net.core.netdev_max_backlog = 5000' >> /etc/sysctl.conf
echo 'net.core.netdev_budget = 600' >> /etc/sysctl.conf

# PC-side network optimization  
sudo ethtool -K $(ip route | grep default | head -1 | grep -oP 'dev \K\w+') gso off gro off
```

### 2. System Resource Optimization

#### CPU Optimization
```bash
# Device-side CPU governor
echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo performance > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor

# Set CPU affinity for optimal performance
taskset -cp 4-7 $(pgrep camerad)    # High-performance cores for camera
taskset -cp 0-3 $(pgrep encoderd)   # Efficiency cores for encoding
taskset -cp 0-1 $(pgrep bridge)     # Minimal cores for bridge
```

#### Memory Optimization
```bash
# Increase network buffer allocations
echo 'net.core.rmem_default = 262144' >> /etc/sysctl.conf
echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.core.wmem_default = 262144' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' >> /etc/sysctl.conf

# TCP buffer optimization
echo 'net.ipv4.tcp_rmem = 4096 65536 16777216' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_wmem = 4096 65536 16777216' >> /etc/sysctl.conf
sysctl -p
```

### 3. Encoding Optimization

#### H.265 Encoder Settings
```bash
# Encoder optimization parameters (in encoderd source):
# - Profile: Main10 (high efficiency)
# - Level: 5.1 (4K support)
# - Bitrate: Adaptive based on content
# - Keyframe interval: 1 second (20 frames)
# - B-frames: 0 (low latency)
```

#### Quality vs Latency Balance
```python
# Encoding parameters balance:
QUALITY_SETTINGS = {
    "low_latency": {
        "bitrate": "15M",      # Lower bitrate
        "preset": "ultrafast", # Faster encoding
        "tune": "zerolatency"  # Minimize latency
    },
    "high_quality": {
        "bitrate": "25M",      # Higher bitrate
        "preset": "medium",    # Balanced encoding
        "tune": "film"         # Better quality
    }
}
```

### 4. PC-Side Optimization

#### Hardware Decoding
```bash
# NVIDIA GPU optimization
export CUDA_VISIBLE_DEVICES=0
export NV_LOW_LATENCY=3

# Verify GPU utilization
nvidia-smi dmon -s pucvmet -d 1
```

#### Software Decoding Optimization
```bash
# CPU-based decoding optimization
export OMP_NUM_THREADS=4
export FFMPEG_THREADS=4

# Monitor CPU usage
htop -u $USER
```

## Network Troubleshooting

### 1. Connectivity Diagnostics

#### Basic Network Tests
```bash
# Test basic connectivity
ping <device-ip>

# Test bandwidth
iperf3 -c <device-ip> -t 30

# Test latency under load
ping <device-ip> -i 0.1 -c 100

# Check packet loss
mtr <device-ip> --report-cycles 100
```

#### Port Connectivity Tests
```bash
# Test specific streaming ports
nc -zv <device-ip> 8001
telnet <device-ip> 8001

# Check if bridge is listening
ssh root@<device-ip> "netstat -an | grep :8001"
```

#### Network Route Analysis
```bash
# Check routing path
traceroute <device-ip>

# Check network interface statistics
ip -s link show

# Monitor real-time network statistics
watch -n 1 'cat /proc/net/dev'
```

### 2. Streaming-Specific Issues

#### Camera Stream Debug
```bash
# Enable detailed debug output
./compressed_vipc.py <device-ip> --cams 0 --silent=false 2>&1 | tee stream_debug.log

# Look for common issues in log:
grep -E "(DROP|ERROR|timeout|connection)" stream_debug.log
```

#### Frame Rate Analysis
```bash
# Monitor frame rates and latency
./compressed_vipc.py <device-ip> --cams 0 | grep -E "latency|fps"

# Expected output:
# - Frame latency: <50ms total
# - Network latency: <20ms
# - Process latency: <10ms
# - PC latency: <20ms
```

#### Buffer Overflow Detection
```bash
# Check for buffer overflow indicators
dmesg | grep -i "buffer"
journalctl | grep -i "overflow\|buffer"

# Monitor network buffer usage
cat /proc/net/sockstat
```

### 3. Performance Troubleshooting

#### Bandwidth Bottlenecks
```bash
# Continuous bandwidth monitoring
iftop -i wlan0 -P

# Check for bandwidth limiting
tc -s qdisc show dev wlan0

# Network saturation test
iperf3 -c <device-ip> -P 4 -t 60
```

#### CPU/GPU Bottlenecks
```bash
# Monitor device CPU usage
ssh root@<device-ip> "top -bn1 | head -20"

# Monitor PC GPU usage (NVIDIA)
nvidia-smi dmon -s pucvmet -d 1

# Monitor PC CPU usage
htop -u $USER
```

#### Memory Issues
```bash
# Check device memory usage
ssh root@<device-ip> "free -h && cat /proc/meminfo | grep Available"

# Check PC memory usage
free -h
cat /proc/meminfo | grep Available

# Monitor memory allocation
watch -n 1 'ps aux --sort=-%mem | head'
```

### 4. Common Error Resolutions

#### "Connection Refused" Errors
```bash
# Check if bridge is running
ssh root@<device-ip> "pgrep bridge || echo 'Bridge not running'"

# Check firewall
ssh root@<device-ip> "iptables -L INPUT | grep 8001"

# Restart bridge service
ssh root@<device-ip> "cd /data/openpilot/cereal/messaging && ./bridge &"
```

#### "Frame Drop" Issues
```bash
# Reduce camera count temporarily
./compressed_vipc.py <device-ip> --cams 0  # Single camera only

# Check network quality
ping <device-ip> -c 100 | grep "packet loss"

# Verify encoding is not overloaded
ssh root@<device-ip> "top -p \$(pgrep encoderd)"
```

#### "Decoder Errors" Issues
```bash
# Try software decoding
./compressed_vipc.py <device-ip> --cams 0  # Without --nvidia flag

# Check codec compatibility
python3 -c "import av; print(av.codec.Codec('hevc', 'r').name)"

# Verify frame format
./compressed_vipc.py <device-ip> --cams 0 --debug | grep "resolution\|format"
```

## Advanced Configuration

### 1. Custom Streaming Parameters

#### Encoder Configuration
```bash
# Create custom encoder configuration
cat << 'EOF' > /data/openpilot/custom_encoder.conf
bitrate=20M
keyint=20
preset=fast
profile=main
level=4.0
EOF

# Apply custom configuration (requires encoder modification)
```

#### Network Buffer Tuning
```bash
# Advanced network buffer configuration
# For high-bandwidth, low-latency streaming:

# Increase socket buffer sizes
echo 'net.core.rmem_max = 134217728' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' >> /etc/sysctl.conf

# TCP-specific optimizations
echo 'net.ipv4.tcp_congestion_control = bbr' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_slow_start_after_idle = 0' >> /etc/sysctl.conf

sysctl -p
```

### 2. Multi-Device Streaming

#### Load Balancing Setup
```bash
# Stream different cameras from multiple devices
# Device 1: Road camera only
./compressed_vipc.py <device1-ip> --cams 0 &

# Device 2: Driver and wide cameras
./compressed_vipc.py <device2-ip> --cams 1,2 &

wait
```

#### Network Aggregation
```bash
# Bond multiple network interfaces for increased bandwidth
ip link add bond0 type bond mode 802.3ad
ip link set wlan0 master bond0
ip link set eth0 master bond0
ip link set bond0 up
```

### 3. Custom Development Integration

#### API Integration
```python
# Custom streaming client example
import cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient

class CustomStreamClient:
    def __init__(self, addr):
        self.addr = addr
        self.vipc_client = VisionIpcClient("camerad", ["road"], True)
    
    def get_frame(self):
        buf = self.vipc_client.recv("road")
        if buf is not None:
            return buf.dat.reshape((buf.height, buf.width, 3))
        return None

# Usage
client = CustomStreamClient("<device-ip>")
frame = client.get_frame()
```

#### Custom Encoding Pipeline
```bash
# Implement custom encoding with different parameters
# This requires modification of encoderd source code
# Location: system/loggerd/encoderd.cc

# Key parameters to modify:
# - ENCODE_BITRATE: Target bitrate
# - KEYFRAME_INTERVAL: I-frame frequency  
# - ENCODE_PRESET: Encoding speed/quality trade-off
```

## Security Best Practices

### 1. Network Security

#### VPN Configuration
```bash
# Set up VPN for secure streaming over public networks
# Install WireGuard on both device and PC

# Device configuration (/etc/wireguard/wg0.conf):
[Interface]
PrivateKey = <device-private-key>
Address = 10.8.0.2/24

[Peer]
PublicKey = <pc-public-key>
Endpoint = <pc-public-ip>:51820
AllowedIPs = 10.8.0.1/32

# Start VPN
wg-quick up wg0
```

#### SSH Hardening
```bash
# Additional SSH security measures
# Add to /etc/ssh/sshd_config:
Protocol 2
MaxAuthTries 3
LoginGraceTime 30
StrictModes yes
PubkeyAuthentication yes
AuthorizedKeysFile .ssh/authorized_keys
IgnoreRhosts yes
HostbasedAuthentication no
PermitEmptyPasswords no
X11Forwarding no
AllowUsers root
```

### 2. Data Protection

#### Stream Encryption
```python
# Implement stream encryption (requires custom modification)
import cryptography
from cryptography.fernet import Fernet

class EncryptedStream:
    def __init__(self, key):
        self.cipher = Fernet(key)
    
    def encrypt_frame(self, frame_data):
        return self.cipher.encrypt(frame_data)
    
    def decrypt_frame(self, encrypted_data):
        return self.cipher.decrypt(encrypted_data)
```

#### Access Control
```bash
# Implement IP-based access control
# Device firewall rules:
iptables -A INPUT -s <authorized-pc-ip>/32 -p tcp --dport 8001 -j ACCEPT
iptables -A INPUT -p tcp --dport 8001 -j REJECT

# Time-based access control
iptables -A INPUT -s <authorized-pc-ip>/32 -m time --timestart 08:00 --timestop 18:00 -j ACCEPT
```

### 3. Monitoring and Auditing

#### Connection Logging
```bash
# Log all streaming connections
cat << 'EOF' > /data/openpilot/log_connections.sh
#!/bin/bash
while true; do
    netstat -an | grep :8001 | grep ESTABLISHED | while read line; do
        echo "$(date): $line" >> /var/log/camera_streaming.log
    done
    sleep 60
done
EOF

chmod +x /data/openpilot/log_connections.sh
./log_connections.sh &
```

#### Security Alerts
```bash
# Monitor for suspicious activity
cat << 'EOF' > /data/openpilot/security_monitor.sh
#!/bin/bash
LOG_FILE="/var/log/camera_streaming.log"

# Monitor for unusual connection patterns
tail -f $LOG_FILE | while read line; do
    if echo "$line" | grep -E "(multiple.*connections|rapid.*disconnect)"; then
        echo "SECURITY ALERT: Unusual activity detected: $line"
        # Send alert (email, webhook, etc.)
    fi
done
EOF

chmod +x /data/openpilot/security_monitor.sh
```

---

## Quick Reference Commands

### Device Setup
```bash
# Complete device setup in one command
ssh root@<device-ip> "
cd /data/openpilot/cereal/messaging && ./bridge &
cd /data/openpilot/system/camerad && ./camerad &
cd /data/openpilot/system/loggerd && ./encoderd &
wait
"
```

### PC Setup
```bash
# Complete PC setup
cd tools/camerastream
./compressed_vipc.py <device-ip> --cams 0,1,2
```

### Troubleshooting
```bash
# Quick diagnostic commands
ping <device-ip>                                    # Basic connectivity
ssh root@<device-ip> "pgrep bridge"                # Check bridge
iperf3 -c <device-ip>                              # Bandwidth test
./compressed_vipc.py <device-ip> --cams 0 --debug  # Stream debug
```

---

This manual provides comprehensive coverage of the nagaspilot camera streaming system with detailed WiFi setup instructions, multiple camera support, network optimization, and troubleshooting procedures. The system supports real-time streaming of H.265-encoded video from comma devices to development machines over WiFi networks, enabling remote debugging and monitoring capabilities.