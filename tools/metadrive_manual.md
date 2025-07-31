# MetaDrive Simulation Manual for Nagaspilot
*A Complete Novice Guide to Using MetaDrive with Nagaspilot*

## 📊 Overview & System Architecture

### What is MetaDrive?
MetaDrive is a sophisticated physics-based driving simulator that provides realistic vehicle dynamics, camera simulation, and traffic scenarios. It serves as the core simulation engine that allows you to test nagaspilot's autonomous driving capabilities in a safe, virtual environment without needing a real car.

### 🌍 Big Picture: How Everything Works Together

```
┌─────────────────────────────────────────────────────────────────────┐
│                          METADRIVE ECOSYSTEM                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │
│  │   REAL WORLD    │    │   METADRIVE     │    │   NAGASPILOT    │  │
│  │                 │    │   SIMULATOR     │    │      STACK      │  │
│  │ • Physical Car  │────│ • Virtual Car   │────│ • Planning      │  │
│  │ • Real Cameras  │    │ • RGB Cameras   │    │ • Control       │  │
│  │ • GPS/IMU       │    │ • Physics Sim   │    │ • Vision        │  │
│  │ • CAN Messages  │    │ • CAN Emulation │    │ • Navigation    │  │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 🔄 Data Flow Diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│                            SIMULATION LOOP                             │
│                                                                        │
│  ┌──────────────┐  Camera   ┌──────────────┐  Vision   ┌─────────────┐ │
│  │   MetaDrive  │  Images   │  nagaspilot  │  Data     │   Control   │ │
│  │   Physics    │──────────▶│    Vision    │──────────▶│  Planning   │ │
│  │   Engine     │           │   Pipeline   │           │   System    │ │
│  └──────────────┘           └──────────────┘           └─────────────┘ │
│         ▲                                                      │       │
│         │                                                      ▼       │
│  ┌──────────────┐           ┌──────────────┐           ┌─────────────┐ │
│  │   Vehicle    │           │     CAN      │           │  Steering   │ │
│  │   Movement   │◀──────────│   Messages   │◀──────────│    Brake    │ │
│  │   Physics    │  Control  │  Simulation  │  Commands │  Throttle   │ │
│  └──────────────┘  Update   └──────────────┘           └─────────────┘ │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 🎯 Key Components

| Component | Purpose | What It Does |
|-----------|---------|--------------|
| **MetaDrive Engine** | Core simulator | Creates virtual world with physics, roads, and vehicle |
| **Bridge System** | Communication | Connects MetaDrive to nagaspilot processes |
| **Camera Simulation** | Vision input | Provides RGB camera feeds that mimic real C3 cameras |
| **CAN Simulation** | Vehicle data | Generates Honda Civic 2022 CAN messages |
| **Control Interface** | User input | Keyboard/joystick controls for testing |

---

## 🚀 Getting Started: Complete Setup Guide

### Step 1: Prerequisites Check

Before starting, verify your system meets the requirements:

**Hardware Requirements:**
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: Minimum 8GB (16GB recommended)
- **GPU**: NVIDIA GPU preferred for acceleration (optional but recommended)
- **Storage**: 2GB free space
- **USB Port**: For joystick/wheel controller (optional)

**Check Your System:**
```bash
# Check CPU cores
nproc

# Check memory
free -h

# Check GPU (NVIDIA)
nvidia-smi 2>/dev/null || echo "No NVIDIA GPU found (will use CPU)"

# Check disk space
df -h
```

### Step 2: Install MetaDrive Dependencies

**Install System Dependencies:**
```bash
# Update package list
sudo apt-get update

# Install OpenGL libraries for rendering
sudo apt-get install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev

# Install OpenCL for GPU acceleration (optional but recommended)
sudo apt-get install ocl-icd-libopencl1 opencl-headers clinfo

# Install Python development headers
sudo apt-get install python3-dev

# For joystick support (optional)
sudo apt-get install joystick jstest-gtk
```

**Install Python Dependencies:**
```bash
# Install MetaDrive simulator
pip install metadrive-simulator

# Install OpenCL Python bindings
pip install pyopencl

# Install controller support (optional)
pip install evdev
```

**Verify Installation:**
```bash
# Test MetaDrive installation
python3 -c "import metadrive; print('✅ MetaDrive installed successfully')"

# Test OpenCL (optional)
python3 -c "import pyopencl as cl; print('✅ OpenCL available')" 2>/dev/null || echo "ℹ️  OpenCL not available (will use CPU)"

# Test CUDA (optional)
python3 -c "import cupy; print('✅ CUDA available')" 2>/dev/null || echo "ℹ️  CUDA not available (will use CPU)"
```

### Step 3: Setup Nagaspilot for Simulation

**Build Nagaspilot with Simulation Support:**
```bash
# Navigate to nagaspilot directory
cd nagaspilot

# Run simulation setup script
./tools/sim/setup.sh

# This script will:
# - Install required Python packages
# - Build simulation-specific components
# - Configure environment variables
```

### Step 4: Test Your Installation

**Quick Hardware Test (Optional - Joystick):**
```bash
# List available joystick devices
ls /dev/input/js* 2>/dev/null || echo "No joystick devices found"

# Test joystick (if available)
jstest /dev/input/js0  # Press Ctrl+C to exit
```

**Quick OpenCL Test (Optional):**
```bash
# List OpenCL devices
clinfo 2>/dev/null | grep "Device Name" || echo "No OpenCL devices found"
```

---

## 🎮 Basic Usage: Your First Simulation

### Quick Start Method (Easiest)

**Terminal 1 - Start Nagaspilot:**
```bash
cd nagaspilot
./tools/sim/launch_openpilot.sh
```
*This terminal will show nagaspilot processes starting up*

**Terminal 2 - Start MetaDrive Bridge:**
```bash
cd nagaspilot
./tools/sim/run_bridge.py
```
*This terminal will show the simulation starting*

**🎉 You should now see:**
- Terminal 1: Various nagaspilot processes launching
- Terminal 2: MetaDrive initialization messages
- The simulation is ready when you see "Simulation started" messages

### Step-by-Step First Experience

**1. Understanding What Happens:**
When you launch the simulation, several processes start:
- **nagaspilot processes**: Vision, planning, control systems
- **MetaDrive engine**: Physics simulation and 3D world
- **Bridge process**: Communication between nagaspilot and MetaDrive
- **Camera simulation**: Provides vision feed to nagaspilot

**2. Basic Controls:**
```
┌─────────────────────────────────────────────────────────────┐
│                    KEYBOARD CONTROLS                        │
├─────────────────────────────────────────────────────────────┤
│  Key  │  Function              │  What It Does              │
├───────┼────────────────────────┼────────────────────────────┤
│   1   │ Cruise Resume/Accel    │ Increases cruise speed     │
│   2   │ Cruise Set/Decel       │ Decreases cruise speed     │
│   3   │ Cruise Cancel          │ Disables cruise control    │
│   r   │ Reset Simulation       │ Resets vehicle position    │
│   i   │ Toggle Ignition        │ Turns car on/off           │
│   q   │ Quit                   │ Exits simulation           │
│   w   │ Manual Throttle        │ Accelerate manually        │
│   s   │ Manual Brake           │ Brake manually             │
│   a   │ Steer Left             │ Turn left manually         │
│   d   │ Steer Right            │ Turn right manually        │
│   z   │ Left Blinker           │ Activate left turn signal  │
│   x   │ Right Blinker          │ Activate right turn signal │
└───────┴────────────────────────┴────────────────────────────┘
```

**3. Your First Test Drive:**

**Step 3a: Start the Car**
- Press `i` to turn on ignition (if not already on)
- You should see "Ignition: True" in the output

**Step 3b: Enable Cruise Control**
- Press `2` to set cruise control
- The car should start moving slowly
- Press `1` to increase speed
- Press `2` again to decrease speed

**Step 3c: Manual Override**
- Press `s` to brake (this will disable cruise control)
- Use `w` to manually accelerate
- Use `a` and `d` to steer manually

**Step 3d: Reset if Needed**
- Press `r` to reset the simulation if the car gets stuck
- Press `q` to quit when you're done

---

## 🔧 Advanced Configuration

### Camera Configuration Options

**Single Camera Mode (Default):**
```bash
./tools/sim/run_bridge.py
```
- Uses only the road-facing camera (40° field of view)
- Matches the primary camera on comma three device
- Best performance, adequate for most testing

**Dual Camera Mode:**
```bash
./tools/sim/run_bridge.py --dual_camera
```
- Adds wide-angle camera (120° field of view)
- Provides peripheral vision for better lane detection
- Higher computational load but more realistic

**High Quality Mode:**
```bash
./tools/sim/run_bridge.py --high_quality
```
- Enhanced rendering quality
- Better graphics and visual effects
- Significantly higher computational requirements

**Combined Modes:**
```bash
./tools/sim/run_bridge.py --dual_camera --high_quality
```

### Joystick/Wheel Control Setup

**Enable Joystick Mode:**
```bash
./tools/sim/run_bridge.py --joystick
```

**Joystick Configuration:**
```bash
# Find your joystick device
ls /dev/input/js*

# Test joystick functionality
jstest /dev/input/js0

# Check joystick permissions
ls -l /dev/input/js0
# Should show read permissions for your user
```

**Common Joystick Mappings:**
- **Steering Wheel**: Left stick X-axis or wheel axis
- **Gas Pedal**: Right trigger or designated pedal axis
- **Brake Pedal**: Left trigger or designated brake axis
- **Buttons**: Various cruise control and function buttons

### GPU Acceleration Setup

**NVIDIA CUDA Configuration:**
```bash
# Check NVIDIA driver
nvidia-smi

# Verify CUDA installation
nvcc --version

# Test CUDA with Python
python3 -c "
import cupy
print('CUDA devices:', cupy.cuda.runtime.getDeviceCount())
"
```

**OpenCL Configuration:**
```bash
# Check OpenCL platforms
clinfo

# Verify OpenCL with Python
python3 -c "
import pyopencl as cl
print('OpenCL platforms:')
for platform in cl.get_platforms():
    print(f'  - {platform.name}')
"
```

---

## 🏗️ Understanding the Architecture

### Process Architecture Deep Dive

**Process Hierarchy:**
```
Main Terminal (launch_openpilot.sh)
├── controlsd (steering/throttle control)
├── plannerd (path planning)
├── radard (obstacle detection)
├── visiond (camera processing)
├── locationd (GPS/positioning)
└── ... (other nagaspilot processes)

Bridge Terminal (run_bridge.py)
├── Bridge Process (main coordinator)
├── MetaDrive Process (physics simulation)
├── Camera Thread (image processing)
└── Control Thread (input handling)
```

**Communication Flow:**

1. **MetaDrive → Bridge:**
   - Vehicle position and velocity
   - Camera images (RGB)
   - Physics state updates

2. **Bridge → nagaspilot:**
   - Camera images (converted to YUV420)
   - CAN messages (speed, steering, etc.)
   - Sensor data (IMU, GPS)

3. **nagaspilot → Bridge:**
   - Steering commands
   - Throttle/brake commands
   - System status

4. **Bridge → MetaDrive:**
   - Control inputs
   - Reset commands
   - Simulation parameters

### Camera System Details

**Road Camera (Primary):**
- **Resolution**: 1928 x 1208 pixels
- **Field of View**: 40 degrees
- **Purpose**: Main vision input for lane detection and object recognition
- **Position**: Mounted at comma three height (1.22m) on vehicle

**Wide Camera (Optional):**
- **Resolution**: 1928 x 1208 pixels  
- **Field of View**: 120 degrees
- **Purpose**: Peripheral vision for enhanced lane monitoring
- **Usage**: Activated with `--dual_camera` flag

**Image Processing Pipeline:**
```
MetaDrive RGB → OpenCL Conversion → YUV420 → VisionIPC → nagaspilot
  (1928x1208x3)                    (1928x1208)              Vision Pipeline
```

### Vehicle Simulation Details

**Simulated Vehicle**: Honda Civic 2022
- **Physics Model**: Realistic tire model, suspension, and dynamics
- **CAN Messages**: Complete Honda Civic CAN protocol simulation
- **Sensors**: IMU, GPS, wheel speed sensors, steering angle sensor
- **Safety Systems**: Brake override, ignition control, emergency stops

**Track Layout:**
```
┌─────────────────────────────────────────────────────────┐
│                    SIMULATION TRACK                     │
│                                                         │
│    60m straight    ┌─────────────────┐    60m straight  │
│  ┌─────────────────┤                 ├─────────────────┐ │
│  │                 │     Curves:     │                 │ │
│  │                 │   90° turns     │                 │ │
│  │                 │   120m length   │                 │ │
│  │                 │                 │                 │ │
│  └─────────────────┤                 ├─────────────────┘ │
│    60m straight    └─────────────────┘    60m straight  │
│                                                         │
│  Total track perimeter: ~480m                           │
│  Lane width: 4.5m (2 lanes)                            │
└─────────────────────────────────────────────────────────┘
```

---

## 🔍 Detailed Operation Guide

### Understanding System Status Messages

**Normal Startup Sequence:**
1. **nagaspilot Launch Messages:**
   ```
   starting manager
   starting controlsd
   starting plannerd
   ...
   ``` 

2. **Bridge Startup Messages:**
   ```
   Simulation bridge starting...
   MetaDrive initializing...
   Camera system ready...
   Bridge connected to nagaspilot
   ```

3. **Ready State:**
   ```
   Simulation started
   Controls: 2 to engage, 1/2 to adjust speed, 's' to brake, 'q' to quit
   ```

### Control Modes Explained

**1. Manual Control Mode (Default State):**
- nagaspilot is **not engaged**
- You control the car directly with WASD keys
- No autonomous behavior
- Good for: Initial testing, manual driving, getting familiar with controls

**2. Cruise Control Mode (Autonomous):**
- Press `2` to **engage** nagaspilot
- Car will maintain speed and follow lanes automatically
- Press `1` to increase cruise speed
- Press `2` to decrease cruise speed
- Good for: Testing autonomous driving capabilities

**3. Manual Override Mode:**
- Any manual input (especially braking with `s`) **disengages** nagaspilot
- Returns to manual control immediately
- Safety feature that mimics real-world behavior
- Good for: Emergency stops, testing override behavior

### Monitoring System Health

**Performance Indicators:**
- **Frame Rate**: Should maintain ~20 FPS for smooth operation
- **CPU Usage**: Monitor with `htop` in another terminal
- **Memory Usage**: Should remain stable (not continuously increasing)
- **GPU Usage**: Check with `nvidia-smi` if using NVIDIA GPU

**Health Check Commands:**
```bash
# Monitor CPU and memory usage
htop

# Check GPU usage (NVIDIA only)
watch -n 1 nvidia-smi

# Monitor nagaspilot processes
ps aux | grep -E "(controlsd|plannerd|visiond)"

# Check message flow
cd nagaspilot
python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['carState'])
sm.update(1000)
print(f'Car speed: {sm['carState'].vEgo:.1f} m/s')
"
```

### Understanding Error Messages

**Common Issues and Solutions:**

**"MetaDrive failed to initialize":**
- **Cause**: Missing dependencies or GPU issues
- **Solution**: 
  ```bash
  pip install --upgrade metadrive-simulator
  # Try without GPU acceleration:
  export METADRIVE_GPU=0
  ./tools/sim/run_bridge.py
  ```

**"OpenCL error" or "PyOpenCL not found":**
- **Cause**: OpenCL not properly installed
- **Solution**:
  ```bash
  sudo apt-get install ocl-icd-libopencl1 opencl-headers
  pip install --upgrade pyopencl
  ```

**"Bridge connection failed":**
- **Cause**: nagaspilot not started or communication issue
- **Solution**: 
  1. Ensure nagaspilot is running first
  2. Wait 10-15 seconds after nagaspilot startup
  3. Then start the bridge

**"No joystick found":**
- **Cause**: Joystick not connected or no permissions
- **Solution**:
  ```bash
  # Check device exists
  ls /dev/input/js*
  # Add user to input group
  sudo usermod -a -G input $USER
  # Log out and log back in
  ```

---

## 🧪 Testing and Validation

### Basic Functionality Tests

**Test 1: Engine Start/Stop**
```bash
# Start simulation
./tools/sim/run_bridge.py

# In simulation:
i  # Toggle ignition off
i  # Toggle ignition on
q  # Quit

# Expected: Clean startup and shutdown
```

**Test 2: Manual Control**
```bash
# Start simulation
./tools/sim/run_bridge.py

# Test sequence:
w  # Accelerate forward
s  # Brake to stop
a  # Steer left
d  # Steer right
r  # Reset position
q  # Quit

# Expected: Responsive manual control
```

**Test 3: Autonomous Engagement**
```bash
# Start simulation
./tools/sim/run_bridge.py

# Test sequence:
2  # Engage cruise control
1  # Increase speed (repeat 3-4 times)
s  # Emergency brake (should disengage)
q  # Quit

# Expected: Car drives autonomously, stops on brake
```

### Advanced Testing Scenarios

**Scenario 1: Lane Following Test**
1. Start simulation
2. Engage cruise control (`2`)
3. Set moderate speed (`1` x3)
4. Observe: Car should follow lane markers
5. Duration: Let run for 2-3 minutes
6. Reset if needed (`r`)

**Scenario 2: Speed Control Test**
1. Engage cruise control (`2`)
2. Gradually increase speed (`1` x10)
3. Gradually decrease speed (`2` x10)
4. Observe: Smooth acceleration/deceleration
5. Test emergency brake (`s`)

**Scenario 3: Corner Handling Test**
1. Engage cruise control with moderate speed
2. Let car navigate through all four corners
3. Observe: Smooth steering through curves
4. Check: No lane departures or erratic behavior

**Scenario 4: Reset Recovery Test**
1. Drive car off track intentionally (manual mode)
2. Press `r` to reset
3. Engage autonomous mode
4. Verify: Normal operation after reset

### Performance Benchmarking

**Frame Rate Test:**
```bash
# Add FPS monitoring to bridge output
# Look for messages like:
# "Camera FPS: 19.8"
# "Physics FPS: 20.1" 

# Target performance:
# - Camera: 15-20 FPS
# - Physics: 18-20 FPS
# - Overall: Smooth visual movement
```

**Memory Usage Test:**
```bash
# Before starting simulation:
free -h

# After running for 10 minutes:
free -h

# Expected: Memory usage should be stable, not continuously growing
```

**CPU Usage Test:**
```bash
# Monitor during normal operation:
htop

# Expected CPU usage:
# - Bridge process: 10-30%
# - MetaDrive process: 20-50%
# - nagaspilot processes: 30-60% combined
# - Total system: <80% on multi-core systems
```

---

## 🛠️ Troubleshooting Guide

### Installation Issues

**Problem**: `pip install metadrive-simulator` fails
```bash
# Solution 1: Update pip and try again
pip install --upgrade pip
pip install metadrive-simulator

# Solution 2: Use conda if available
conda install -c conda-forge metadrive-simulator

# Solution 3: Install from source
git clone https://github.com/metadriverse/metadrive.git
cd metadrive
pip install -e .
```

**Problem**: OpenGL errors on headless systems
```bash
# Solution: Install virtual display
sudo apt-get install xvfb
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &
./tools/sim/run_bridge.py
```

**Problem**: Permission denied for joystick
```bash
# Solution: Add user to input group
sudo usermod -a -G input $USER
# Log out and log back in, or:
newgrp input
```

### Runtime Issues

**Problem**: Black screen or no camera feed
```bash
# Check 1: Verify MetaDrive camera initialization
python3 -c "
from metadrive import MetaDriveEnv
env = MetaDriveEnv()
print('MetaDrive cameras OK')
env.close()
"

# Check 2: Test OpenCL
python3 -c "
import pyopencl as cl
print('OpenCL contexts:', len(cl.get_platforms()))
"

# Solution: Restart with CPU-only mode
export METADRIVE_GPU=0
./tools/sim/run_bridge.py
```

**Problem**: Car doesn't move when engaged
```bash
# Debug checklist:
# 1. Check ignition status (press 'i' to toggle)
# 2. Verify engagement (press '2' to engage)
# 3. Check speed setting (press '1' to increase)
# 4. Monitor control messages

# Debug command:
cd nagaspilot
python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['carControl', 'selfdriveState'])
while True:
    sm.update(1000)
    print(f'Engaged: {sm[\"selfdriveState\"].active}')
    print(f'Controls: Accel={sm[\"carControl\"].actuators.accel:.2f}')
    break
"
```

**Problem**: Simulation crashes or freezes
```bash
# Solution 1: Clean restart
pkill -f metadrive
pkill -f bridge
# Wait 5 seconds
./tools/sim/launch_openpilot.sh
# In new terminal:
./tools/sim/run_bridge.py

# Solution 2: Reduce computational load
./tools/sim/run_bridge.py  # (no high_quality flag)

# Solution 3: Check system resources
free -h
df -h
```

### Performance Issues

**Problem**: Low frame rate or stuttering
```bash
# Solution 1: Close unnecessary applications
# Solution 2: Lower quality settings
./tools/sim/run_bridge.py  # Default quality

# Solution 3: CPU governor optimization
sudo cpufreq-set -g performance

# Solution 4: Memory optimization
sudo sysctl vm.swappiness=10
```

**Problem**: High CPU usage
```bash
# Check specific processes:
top -p $(pgrep -f "metadrive|bridge|controlsd")

# Solutions:
# 1. Reduce physics frequency (requires code modification)
# 2. Use single camera mode (default)
# 3. Disable unused nagaspilot processes in launch script
```

### Debugging Tools

**Enable Verbose Logging:**
```bash
# Set environment variables before running:
export PYTHONUNBUFFERED=1
export METADRIVE_DEBUG=1
./tools/sim/run_bridge.py
```

**Message Flow Debugging:**
```bash
# Monitor specific message types:
cd nagaspilot
python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState'])
while True:
    sm.update(100)
    print(f'Speed: {sm[\"carState\"].vEgo:.1f} m/s, ' +
          f'Engaged: {sm[\"selfdriveState\"].active}, ' +
          f'Steer: {sm[\"carControl\"].actuators.steeringAngleDeg:.1f}°')
"
```

**Camera Feed Debugging:**
```bash
# Check camera data flow:
python3 -c "
import numpy as np
from openpilot.tools.sim.lib.camerad import W, H
print(f'Expected camera size: {W}x{H}')
"
```

---

## 📚 Advanced Features and Customization

### Custom Track Creation

You can modify the track layout by editing the `create_map()` function in `/tools/sim/bridge/metadrive/metadrive_bridge.py`:

```python
def create_map(track_size=60):
    # Create custom track with different segments
    return dict(
        type=MapGenerateMethod.PG_MAP_FILE,
        lane_num=2,        # Number of lanes
        lane_width=4.5,    # Width in meters
        config=[
            None,
            straight_block(100),     # Longer straight section
            curve_block(60, 45),     # Gentler 45° turn
            straight_block(50),      # Shorter straight
            curve_block(80, 90, 1),  # 90° turn, opposite direction
            # Add more segments as needed
        ]
    )
```

### Camera Position Customization

Adjust camera positioning in the MetaDrive world files:

```python
# Camera height and position (in meters)
C3_POSITION = Vec3(0.0, 0, 1.22)  # x, y, z coordinates
C3_HPR = Vec3(0, 0, 0)            # heading, pitch, roll
```

### Physics Parameter Tuning

Modify physics settings in the bridge configuration:

```python
config = dict(
    physics_world_step_size=0.05,  # 50ms steps (20Hz physics)
    decision_repeat=1,             # Control update frequency
    # Vehicle dynamics
    vehicle_config=dict(
        max_speed_km_h=120,        # Top speed limit
        wheel_friction=0.8,        # Tire grip (0.0-1.0)
    )
)
```

### Environment Conditions

**Add Traffic (Experimental):**
```python
config = dict(
    traffic_density=0.1,  # Light traffic (0.0-1.0)
    random_traffic=True,
)
```

**Weather Effects:**
```python
config = dict(
    weather="cloudy",      # Options: sunny, cloudy, rainy
    time_of_day="day",     # Options: day, night, dawn, dusk
)
```

### Automated Testing

**Create Test Scripts:**
```python
# test_autonomous_driving.py
import time
from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

def run_automated_test():
    # 30-second autonomous driving test
    bridge = MetaDriveBridge(
        dual_camera=False,
        high_quality=False,
        test_duration=30,
        test_run=True
    )
    
    bridge.run()
    # Test automatically engages and measures performance

if __name__ == "__main__":
    run_automated_test()
```

**Batch Testing:**
```bash
# Run multiple test scenarios
for scenario in lane_follow speed_control cornering; do
    echo "Testing $scenario..."
    python3 test_${scenario}.py
done
```

---

## 🤝 Integration with Development Workflow

### Using Simulation for Development

**1. Feature Testing:**
- Test new nagaspilot features safely before real-world deployment
- Validate parameter changes in controlled environment
- Debug vision pipeline issues with consistent camera feeds

**2. Performance Optimization:**
- Profile CPU/GPU usage under different conditions
- Optimize control algorithms with repeatable scenarios
- Measure system latency and responsiveness

**3. Regression Testing:**
- Automated tests for continuous integration
- Verify that code changes don't break existing functionality
- Performance benchmarking across code versions

### Data Collection and Analysis

**Recording Simulation Data:**
```bash
# Enable logging during simulation
export LOGDIR="/tmp/simulation_logs"
mkdir -p $LOGDIR
./tools/sim/launch_openpilot.sh
```

**Analyzing Performance:**
```bash
# View recorded data
cd nagaspilot
python3 tools/plotjuggler/juggle.py /tmp/simulation_logs
```

### Custom Scenario Development

**Create Specific Test Cases:**
1. **Highway Scenario**: Long straight sections, high speed
2. **City Driving**: Frequent turns, lower speeds  
3. **Parking Lot**: Tight maneuvering, low-speed precision
4. **Weather Testing**: Reduced visibility conditions

---

## 📈 Performance Optimization Guide

### System-Level Optimizations

**CPU Optimization:**
```bash
# Set CPU governor to performance
sudo cpufreq-set -g performance

# Pin processes to specific CPU cores (if needed)
taskset -c 0,1 ./tools/sim/launch_openpilot.sh &
taskset -c 2,3 ./tools/sim/run_bridge.py &
```

**Memory Optimization:**
```bash
# Increase swap space if needed
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Optimize memory management
sudo sysctl vm.swappiness=10
sudo sysctl vm.dirty_ratio=15
```

**GPU Optimization (NVIDIA):**
```bash
# Set maximum performance mode
sudo nvidia-smi -pm 1
sudo nvidia-smi -ac 4004,1911  # Adjust values for your GPU

# Verify settings
nvidia-smi -q -d CLOCK
```

### Application-Level Optimizations

**Reduce Simulation Load:**
```python
# In bridge configuration, reduce quality:
config = dict(
    preload_models=False,        # Faster startup
    anisotropic_filtering=False, # Lower GPU load  
    show_logo=False,            # Skip splash screens
    physics_world_step_size=0.1, # Lower physics frequency
)
```

**Camera Optimization:**
```bash
# Use single camera mode (default)
./tools/sim/run_bridge.py

# Avoid high quality mode for better performance
# (Don't use --high_quality flag)
```

---

## 🔍 Frequently Asked Questions (FAQ)

**Q: Why is MetaDrive so slow on my system?**
A: MetaDrive is computationally intensive. Try:
- Ensure no unnecessary applications are running
- Use single camera mode (default)
- Avoid `--high_quality` flag
- Check if GPU acceleration is working
- Consider upgrading hardware (more RAM, better GPU)

**Q: Can I use MetaDrive without a GPU?**
A: Yes, MetaDrive works on CPU-only systems, but performance will be reduced. The simulation automatically detects and uses available hardware.

**Q: How do I add more traffic to make it realistic?**
A: Increase `traffic_density` in the configuration, but be aware this significantly impacts performance. Start with low values like 0.1.

**Q: Why doesn't the car move when I engage cruise control?**
A: Check these common issues:
- Ignition is on (press `i`)
- Cruise is actually engaged (press `2`)
- Speed is set appropriately (press `1` multiple times)
- No brake input is being sent

**Q: Can I record and replay simulation sessions?**
A: Yes, nagaspilot's standard logging works in simulation. Use the `LOGDIR` environment variable to specify where to save logs.

**Q: How accurate is the simulation compared to real driving?**
A: MetaDrive provides realistic physics and sensor simulation, but it's still a simplified model. Use it for development and testing, but always validate on real hardware.

**Q: Can I connect a real steering wheel?**
A: Yes, use the `--joystick` flag and connect a compatible USB steering wheel or gamepad. Test with `jstest` first.

**Q: Why do I get OpenCL errors?**
A: OpenCL is optional but recommended for performance. If you get errors:
- Install OpenCL drivers for your GPU
- Install `pyopencl` package
- If issues persist, the simulation will fall back to CPU mode

**Q: How do I run simulation in headless mode (no display)?**
A: Use `xvfb` for virtual display:
```bash
sudo apt-get install xvfb
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &
./tools/sim/run_bridge.py
```

**Q: Can I modify the simulated vehicle model?**
A: The current implementation simulates a Honda Civic 2022. Modifying this requires changes to CAN message generation in the bridge code.

---

## 📞 Getting Help and Support

### Community Resources

**GitHub Issues:**
- Report bugs: [nagaspilot issues](https://github.com/commaai/openpilot/issues)
- Feature requests and discussions

**Discord/Forums:**
- Join community discussions
- Ask questions and share experiences
- Get help from other users

### Troubleshooting Checklist

Before asking for help, please try:

1. **✅ System Requirements**: Verify your system meets minimum requirements
2. **✅ Clean Installation**: Try reinstalling MetaDrive and dependencies  
3. **✅ Process Check**: Ensure all necessary processes are running
4. **✅ Log Review**: Check both terminal outputs for error messages
5. **✅ Resource Check**: Monitor CPU, memory, and disk usage
6. **✅ Minimal Test**: Try running without optional features first

### Reporting Issues

**Include This Information:**
- Operating system and version
- Hardware specifications (CPU, RAM, GPU)
- Python version and virtual environment details
- Complete error messages and stack traces
- Steps to reproduce the problem
- Output of diagnostic commands

**Diagnostic Commands:**
```bash
# System info
uname -a
python3 --version
pip list | grep -E "(metadrive|pyopencl|cupy)"

# Hardware info  
lscpu | head -20
free -h
lspci | grep -i vga

# Test basic functionality
python3 -c "import metadrive; print('MetaDrive OK')"
python3 -c "import pyopencl; print('PyOpenCL OK')"
```

---

## 🎯 Conclusion

This manual provides comprehensive guidance for using MetaDrive with nagaspilot. The simulation environment offers:

- **Safe Testing**: No risk to physical vehicles or people
- **Repeatable Scenarios**: Consistent conditions for development
- **Rapid Iteration**: Quick testing of changes and features
- **Performance Monitoring**: Detailed metrics and debugging tools

**Next Steps:**
1. Complete the installation and basic testing
2. Experiment with different control modes
3. Try advanced features like dual cameras
4. Develop custom test scenarios for your use case
5. Integrate simulation into your development workflow

**Remember**: Simulation is a tool for development and testing, but real-world validation is always required before deployment.

For the most up-to-date information, refer to the [MetaDrive documentation](https://metadrive-simulator.readthedocs.io/) and the [nagaspilot codebase](https://github.com/commaai/openpilot).

---

*This manual covers nagaspilot simulation framework integration with MetaDrive. For questions or improvements, please contribute to the project or reach out to the community.*