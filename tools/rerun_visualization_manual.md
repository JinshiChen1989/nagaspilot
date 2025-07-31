# Rerun Visualization System - User Manual

## Overview

Rerun is a powerful 3D visualization and data logging framework integrated into nagaspilot for advanced route analysis and debugging. It provides timeline-based visualization with multi-camera support, allowing developers to replay and analyze driving data in an interactive 3D environment.

**🔄 System Type: REPLAY/POST-ANALYSIS ONLY**
- **NOT Runtime**: Does not run during live driving
- **NOT Streaming**: Not for real-time data visualization  
- **POST-PROCESSING**: Analyzes pre-recorded route data
- **OFFLINE ANALYSIS**: Requires downloaded/cached route segments

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Installation & Setup](#installation--setup)
3. [Basic Usage](#basic-usage)
4. [Advanced Features](#advanced-features)
5. [Camera Configuration](#camera-configuration)
6. [Timeline Controls](#timeline-controls)
7. [Data Analysis](#data-analysis)
8. [Performance Optimization](#performance-optimization)
9. [Troubleshooting](#troubleshooting)
10. [API Reference](#api-reference)

## System Architecture

### Core Components
- **Rerunner Class** (`tools/rerun/run.py`) - Main orchestration system
- **CameraReader** (`tools/rerun/camera_reader.py`) - Video stream processing
- **Blueprint System** - Dynamic UI layout generation
- **Message Parser** - Cereal message extraction and formatting

### Data Flow
```
Route Data → LogReader → Message Parser → Rerun Viewer
     ↓
Camera Files → CameraReader → Video Processor → 3D Display
```

### Integration Points
- **cereal Services** - All nagaspilot message types supported
- **Route System** - Direct integration with comma connect routes
- **Multi-processing** - Parallel segment processing for performance

## Installation & Setup

### Prerequisites
```bash
# Ensure rerun is installed in your environment
pip install rerun-sdk

# Verify nagaspilot environment
source .venv/bin/activate
cd /path/to/nagaspilot
```

### Authentication (for remote routes)
```bash
# Authenticate with comma account
python3 tools/lib/auth.py
```

### System Requirements
- **Memory:** 4GB+ RAM (16GB+ recommended for multi-camera)
- **Storage:** SSD recommended for video processing
- **Network:** Stable connection for remote route access
- **Browser:** Modern browser for web interface

## Basic Usage

### Quick Start - Demo Route
```bash
# Launch demo with low-res camera
cd tools/rerun
./run.py --demo --qcam

# This will:
# 1. Download demo route: "a2a0ccea32023010|2023-07-27--13-01-19"
# 2. Process qcamera (low-resolution) video
# 3. Open browser with interactive viewer
```

### Analyze Your Own Route
```bash
# Basic route visualization (data only, no video)
./run.py "your_route_id|timestamp"

# With low-resolution camera
./run.py --qcam "your_route_id|timestamp"

# Multiple cameras (high memory usage)
./run.py --qcam --fcam --ecam "your_route_id|timestamp"
```

### Route Specification Formats
```bash
# Full route
./run.py "a2a0ccea32023010|2023-07-27--13-01-19"

# Specific segments (segments 2-4)
./run.py "a2a0ccea32023010|2023-07-27--13-01-19/2:4"

# Single segment  
./run.py "a2a0ccea32023010|2023-07-27--13-01-19/3"
```

## Advanced Features

### Multi-Camera Analysis
```bash
# All cameras (WARNING: High memory usage)
./run.py --qcam --fcam --ecam --dcam "route_name"

# Recommended: Low-res + one high-res camera
./run.py --qcam --fcam "route_name"
```

**Memory Requirements by Configuration:**
- `--qcam` only: ~2GB RAM
- `--qcam --fcam`: ~8GB RAM  
- All cameras: ~20GB+ RAM

### Segment Range Processing
```bash
# Process segments 0-2 (first 3 segments)
./run.py --qcam "route_name/0:2"

# Process single segment (fast analysis)
./run.py --qcam "route_name/5"

# Last 3 segments
./run.py --qcam "route_name/-3:"
```

### Performance Optimization
```bash
# Limit to specific segment for faster processing
./run.py --qcam "route_name/0"  # Process only first segment

# Use only essential data (no high-res video)
./run.py "route_name"  # Data visualization only
```

## Camera Configuration

### Camera Types
```python
# From camera_reader.py
class CameraType(StrEnum):
    qcam = "qcamera"    # Low-res road camera (480p)
    fcam = "fcamera"    # High-res road camera (1080p+) 
    ecam = "ecamera"    # Wide-angle camera
    dcam = "dcamera"    # Driver monitoring camera
```

### Camera Selection Strategy
```bash
# Development/Debugging (fast, low memory)
./run.py --qcam "route_name"

# Model Analysis (good quality, moderate memory)
./run.py --qcam --fcam "route_name"

# Complete Analysis (all perspectives, high memory)
./run.py --qcam --fcam --ecam --dcam "route_name"
```

### Video Processing Details
- **Format:** NV12 (native openpilot format)
- **Processing:** ffmpeg/ffprobe for stream analysis
- **Timestamping:** Automatic sync with log messages
- **Compression:** Real-time decompression for display

## Timeline Controls

### Rerun Viewer Interface
Once launched, the Rerun viewer provides:

#### Timeline Navigation
- **Scrub Bar** - Click/drag to navigate through time
- **Play/Pause** - Space bar or play button
- **Speed Control** - Adjust playback speed
- **Frame Stepping** - Arrow keys for precise navigation

#### View Controls
- **3D View** - Mouse drag to rotate camera view
- **Pan/Zoom** - Mouse wheel and right-click drag
- **Reset View** - Double-click to reset camera position

#### Data Panels
- **Selection Panel** - Choose visible data streams
- **Time Panel** - Precise timestamp control
- **Entity Tree** - Hierarchical data organization

### Synchronized Playback
```python
# All data streams are synchronized via:
RR_TIMELINE_NAME = "Timeline"

# Video frames synced with:
rr.set_time_nanos(RR_TIMELINE_NAME, int(ts * 1e9))
```

## Data Analysis

### Available Data Streams
The system automatically processes all cereal services:

#### Core Vehicle Data
- **carState** - Speed, steering angle, gear, etc.
- **carControl** - Control commands to vehicle
- **controlsState** - Control system status

#### Perception Data
- **modelV2** - Neural network predictions
- **radarState** - Radar object detections  
- **liveCalibration** - Camera calibration data

#### System Data
- **selfdriveState** - Overall system status
- **thermal** - Temperature and performance data
- **deviceState** - Hardware status

### Message Parsing System
```python
# Automatic extraction of numeric values
def _parse_msg(msg, parent_key=''):
    # Recursively extracts all numeric values
    # Creates hierarchical entity paths like:
    # carState/vEgo, carState/steeringAngleDeg, etc.
```

### Custom Analysis Workflows
```bash
# Focus on specific time ranges
# 1. Load route in Rerun
# 2. Navigate to interesting events
# 3. Use timeline scrubbing for detailed analysis
# 4. Compare multiple data streams simultaneously
```

## Performance Optimization

### Memory Management
```python
# The system uses multiprocessing for performance:
NUM_CPUS = multiprocessing.cpu_count()

# Processing is parallelized across segments:
pool.imap_unordered(partial(process_func), segments)
```

### Optimization Strategies

#### For Limited RAM:
```bash
# Use only qcamera
./run.py --qcam "route_name"

# Process single segments
./run.py --qcam "route_name/0"
```

#### For Fast Analysis:
```bash
# Skip video entirely (fastest)
./run.py "route_name"

# Use demo route (pre-cached)
./run.py --demo --qcam
```

#### For Complete Analysis:
```bash
# Ensure sufficient RAM (16GB+)
./run.py --qcam --fcam --ecam "route_name"

# Process shorter segments
./run.py --qcam --fcam "route_name/0:2"
```

### System Resource Monitoring
```bash
# Monitor memory usage during processing
htop

# Check disk space for video processing
df -h /tmp
```

## Troubleshooting

### Common Issues

#### Memory Errors
```
Error: Out of memory during video processing

Solution:
1. Use --qcam only (lowest memory)
2. Process single segments: "route/0"
3. Increase system RAM or swap
4. Close other applications
```

#### Network Issues
```
Error: Cannot download route data

Solution:
1. Check authentication: tools/lib/auth.py
2. Verify network connection
3. Try demo route first: --demo
4. Use cached routes if available
```

#### Video Processing Errors
```
Error: ffmpeg/ffprobe not found

Solution:
1. Install ffmpeg: sudo apt install ffmpeg
2. Verify PATH includes ffmpeg
3. Check video file accessibility
```

#### Browser Display Issues
```
Error: Rerun viewer not opening

Solution:  
1. Check browser security settings
2. Try different browser (Chrome recommended)
3. Check firewall/proxy settings
4. Use localhost address explicitly
```

### Performance Issues
```bash
# Slow processing
- Use fewer cameras: --qcam only
- Process smaller segments: "route/0:1" 
- Check SSD vs HDD storage
- Monitor CPU usage

# High memory usage
- Close other applications
- Use single segment processing
- Avoid multiple high-res cameras
- Monitor with: watch -n 1 free -h
```

### Debug Mode
```bash
# Enable verbose output for debugging
python3 -v run.py --demo --qcam

# Check video stream info
ffprobe -v quiet -show_streams video_file.hevc
```

## API Reference

### Command Line Interface
```bash
./run.py [-h] [--demo] [--qcam] [--fcam] [--ecam] [--dcam] [route_or_segment_name]

Arguments:
  route_or_segment_name   Route identifier (e.g., "route_id|timestamp")

Options:
  --demo                  Use demo route (a2a0ccea32023010|2023-07-27--13-01-19)
  --qcam                  Include low-res road camera
  --fcam                  Include high-res road camera  
  --ecam                  Include wide-angle camera
  --dcam                  Include driver monitoring camera
```

### Core Classes

#### Rerunner Class
```python
class Rerunner:
    def __init__(self, route, segment_range, camera_config)
    def load_data()  # Main processing function
    def _create_blueprint()  # UI layout generation
    def _parse_msg(msg, parent_key)  # Message extraction
```

#### CameraReader Class  
```python
class CameraReader:
    def __init__(self, camera_paths, start_time, seg_idxs)
    def run_across_segments(num_processes, func, desc)
```

#### Configuration Objects
```python
CameraConfig = namedtuple("CameraConfig", ["qcam", "fcam", "ecam", "dcam"])

class CameraType(StrEnum):
    qcam = "qcamera"
    fcam = "fcamera"  
    ecam = "ecamera"
    dcam = "dcamera"
```

### Integration Functions
```python
# Route and segment handling
from openpilot.tools.lib.route import Route, SegmentRange

# Video processing
from openpilot.tools.lib.framereader import ffprobe
def probe_packet_info(camera_path)

# Message processing  
from cereal.services import SERVICE_LIST
from openpilot.tools.lib.logreader import LogReader
```

### Rerun-Specific Functions
```python
import rerun as rr
import rerun.blueprint as rrb

# Core functions used:
rr.init(app_name, spawn=True)
rr.connect()
rr.send_blueprint(blueprint)
rr.set_time_nanos(timeline_name, timestamp)
rr.log(entity_path, data)
rr.send_columns(entity_path, times, components)
```

## Best Practices

### Development Workflow
1. **Start Simple**: Use demo route with --qcam
2. **Incremental Analysis**: Add cameras as needed
3. **Segment Focus**: Process specific segments for detailed analysis
4. **Timeline Navigation**: Use scrubbing for event correlation
5. **Multi-stream Comparison**: Leverage synchronized timeline

### Production Usage
1. **Resource Planning**: Allocate sufficient RAM for camera requirements
2. **Network Optimization**: Use local routes when possible
3. **Batch Processing**: Process multiple segments systematically
4. **Documentation**: Save interesting findings with timestamps

### Performance Guidelines
- **Memory**: 4GB per high-res camera stream
- **CPU**: Multi-core systems benefit from parallel processing
- **Storage**: SSD recommended for video processing
- **Network**: Stable connection required for remote routes

## Conclusion

The Rerun visualization system provides powerful capabilities for analyzing nagaspilot driving data. By combining timeline-based playback, multi-camera views, and comprehensive data stream access, it enables deep insights into vehicle behavior and system performance.

Start with simple configurations and gradually add complexity as needed. The browser-based interface makes it accessible for both local and remote analysis scenarios.