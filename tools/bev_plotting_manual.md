# Bird's Eye View (BEV) Plotting Report

## Executive Summary

The nagaspilot codebase contains comprehensive BEV (Bird's Eye View) plotting functionality that visualizes ego vehicle trajectory, lead vehicle tracking, and lane lines in a top-down perspective. The implementation uses multiple visualization frameworks and provides both real-time and replay capabilities.

**📍 System Types: RUNTIME + REPLAY + STREAMING**
- **✅ Runtime**: Real-time BEV during live driving (production UI)
- **✅ Replay**: Post-analysis of recorded route data  
- **✅ Streaming**: Remote BEV via network (debugging)
- **✅ Development**: Multiple debug visualization frameworks

## Key BEV Visualization Components

### 1. Core BEV Rendering System (`selfdrive/ui/onroad/model_renderer.py`)

**Location:** `selfdrive/ui/onroad/model_renderer.py:1-440`

**Purpose:** Main real-time BEV visualization renderer for the openpilot UI

**Key Features:**
- **3D to 2D Projection:** Transforms 3D model data to 2D screen coordinates using camera space transformations
- **Dynamic Path Visualization:** Renders ego vehicle trajectory with color-coded acceleration feedback
- **Lane Line Detection:** Visualizes detected lane lines with confidence-based transparency
- **Lead Vehicle Tracking:** Shows lead vehicles with distance and velocity-based indicators
- **Experimental Mode:** Acceleration-based path coloring (green for speed up, red for slow down)

**Visualization Elements:**
- **Path Rendering:** Uses gradient coloring based on throttle state and experimental mode
- **Lane Lines:** White lines with alpha based on detection confidence (0-255)
- **Road Edges:** Red lines with alpha based on detection standard deviation
- **Lead Vehicles:** Triangle indicators with size and opacity based on distance and relative velocity

### 2. Replay Visualization Tools

#### A. Pygame-based UI (`tools/replay/ui.py`)

**Location:** `tools/replay/ui.py:1-238`

**Purpose:** Interactive replay visualization with real-time plotting

**Key BEV Features:**
- **Top-down Surface:** 384x960 pixel BEV display area
- **Real-time Updates:** Processes modelV2, radarState, and liveTracks messages
- **Multi-camera Integration:** Combines front camera with BEV overlay
- **Interactive Controls:** Pygame-based UI with system status information

**BEV Parameters:**
```python
lidar_x, lidar_y, lidar_zoom = 384, 960, 6
lidar_car_x, lidar_car_y = lidar_x / 2., lidar_y / 1.1
car_hwidth = 1.7272 / 2 * lidar_zoom  # Car width visualization
car_front = 2.6924 * lidar_zoom       # Car front distance
car_back = 1.8796 * lidar_zoom        # Car back distance
```

#### B. Rerun-based Visualization (`tools/replay/rp_visualization.py`)

**Location:** `tools/replay/rp_visualization.py:1-61`

**Purpose:** Modern 3D visualization using Rerun framework

**Features:**
- **Real-time 3D Rendering:** Uses Rerun's SegmentationImage for BEV display
- **Color-coded Elements:** Standardized color palette for different object types
- **Timeline Integration:** Synchronized playback with log timestamps
- **Multi-object Tracking:** Visualizes radar points, model predictions, and lead vehicles

### 3. BEV Helper Functions

#### A. Rerun Helpers (`tools/replay/lib/rp_helpers.py`)

**Location:** `tools/replay/lib/rp_helpers.py:1-110`

**Key Functions:**
- `to_topdown_pt(y, x)`: Converts world coordinates to BEV pixel coordinates
- `plot_model()`: Renders model predictions (lane lines, road edges, lead vehicles)
- `plot_lead()`: Visualizes lead vehicle positions
- `update_radar_points()`: Updates radar track visualization
- `get_blank_lid_overlay()`: Creates base BEV canvas with ego vehicle outline

**Coordinate Transformation:**
```python
def to_topdown_pt(y, x):
    px = x * UP.lidar_zoom + UP.lidar_car_x
    py = -y * UP.lidar_zoom + UP.lidar_car_y
    return int(px), int(py)
```

#### B. UI Helpers (`tools/replay/lib/ui_helpers.py`)

**Location:** `tools/replay/lib/ui_helpers.py:1-227`

**Advanced Features:**
- **3D Calibration Integration:** Uses camera intrinsics for accurate projection
- **Matplotlib Integration:** Real-time plotting capabilities
- **Color Management:** Efficient palette-based color system
- **Path Rendering:** Sophisticated trajectory visualization with z-offset support

### 4. Map Rendering System (`porting/mapd/selfdrive/navd/map_renderer.py`)

**Location:** `porting/mapd/selfdrive/navd/map_renderer.py:1-101`

**Purpose:** Map-based BEV visualization for navigation

**Features:**
- **256x256 Pixel Resolution:** 2 meters per pixel scale
- **Real-time Position Updates:** GPS-based position tracking
- **Route Visualization:** Polyline-based route rendering
- **C++ Backend Integration:** Uses libmaprender for performance

## Visualization Frameworks Used

### 1. **Rerun** 
- **Purpose:** Modern 3D visualization and data logging
- **BEV Usage:** Real-time radar point and model visualization
- **Strengths:** Timeline integration, 3D capabilities, web-based viewer

### 2. **Pygame**
- **Purpose:** Real-time interactive visualization
- **BEV Usage:** Debug UI with top-down view overlay
- **Strengths:** Low-latency rendering, interactive controls

### 3. **PyRay (Raylib)**
- **Purpose:** High-performance 2D/3D graphics
- **BEV Usage:** Production UI rendering with hardware acceleration
- **Strengths:** GPU acceleration, polygon rendering, gradient support

### 4. **Matplotlib**
- **Purpose:** Scientific plotting and data visualization
- **BEV Usage:** Detailed analysis and debugging plots
- **Strengths:** High-quality output, extensive customization

### 5. **OpenCV**
- **Purpose:** Computer vision and image processing
- **BEV Usage:** Image transformations and camera calibration
- **Strengths:** Performance, camera geometry handling

## BEV Data Sources

### 1. **Model Outputs (`modelV2`)**
- **Lane Lines:** 4 lines with confidence probabilities
- **Road Edges:** 2 edges with standard deviation measures
- **Ego Path:** Vehicle trajectory prediction with acceleration data
- **Lead Vehicles:** Detection with position and uncertainty

### 2. **Radar Data (`radarState`, `liveTracks`)**
- **Lead Vehicle Tracking:** Distance, relative velocity, acceleration
- **Multi-object Tracking:** Track IDs, stationary/moving classification
- **Radar Points:** Raw detection points with metadata

### 3. **Camera Calibration (`liveCalibration`)**
- **Extrinsic Parameters:** Roll, pitch, yaw calibration
- **Intrinsic Parameters:** Camera focal length and center point
- **Height Estimation:** Ground plane calibration

## Color Coding Scheme

### Standard Color Palette:
- **Red (255, 0, 0):** Road edges, ego path, lead vehicles
- **Yellow (255, 255, 0):** Lane lines, model predictions
- **Green (0, 255, 0):** Lane lines (confidence-based)
- **White (255, 255, 255):** High-confidence detections
- **Pink/Magenta:** Stationary objects
- **Orange:** Moving objects

### Experimental Mode Colors:
- **Acceleration-based:** HSL color space with hue 60-120°
- **Saturation:** Based on acceleration magnitude (0-1.5x)
- **Lightness:** 0.62-0.95 based on saturation
- **Alpha:** Distance-based transparency

## Performance Characteristics

### BEV Rendering Parameters:
- **Resolution:** 384x960 pixels (typical top-down view)
- **Scale:** ~6 pixels per meter (lidar_zoom = 6)
- **Update Rate:** 20Hz (synchronized with model outputs)
- **Processing:** Real-time coordinate transformation and projection

### Memory Usage:
- **BEV Overlay:** ~370KB per frame (384×960×uint8)
- **Model Points:** Variable (typically 50-200 points per element)
- **Radar Tracks:** ~20-50 tracks maximum

## Runtime vs Replay Modes

### Runtime (Live Operation)
**Purpose:** Real-time BEV visualization during active driving

**Primary System:**
- **`selfdrive/ui/onroad/model_renderer.py`** - Production BEV renderer
- Integrated into main openpilot UI (`selfdrive/ui/ui.py`)
- Uses live sensor data (modelV2, radarState, liveCalibration)
- PyRay/Raylib rendering for hardware acceleration
- Updates at 20Hz with minimal latency

**Runtime Usage:**
```bash
# Live BEV visualization during driving
cd selfdrive/ui && ./ui
```

### Replay Mode (Post-drive Analysis)
**Purpose:** Debug and analyze recorded driving data

**Replay Tools:**
- **`tools/replay/ui.py`** - Pygame-based interactive replay
- **`tools/replay/rp_visualization.py`** - Rerun-based timeline visualization
- **`tools/rerun/run.py`** - Advanced 3D visualization with web interface

**Replay Usage:**
```bash
# Start replay of recorded route
tools/replay/replay <route-name>

# View BEV with Rerun (modern 3D visualization)
python3 tools/replay/rp_visualization.py

# View BEV with Pygame (interactive debugging)
python3 tools/replay/ui.py

# Advanced Rerun with multiple cameras
tools/rerun/run.py --qcam --demo
```

## Display Methods

### 1. On-Device UI Display (Primary)
- **Direct screen rendering** on the openpilot device
- Integrated with front camera view
- Real-time hardware-accelerated graphics
- Production-ready user interface

### 2. Network Streaming (Debug/Remote)
**ZMQ Ethernet Streaming:**
```bash
# Send BEV data via Ethernet/ZMQ
ZMQ=1 tools/replay/replay <route-name>

# Receive on remote machine
python3 tools/replay/rp_visualization.py <ip_address>
```

**Network Parameters:**
- Default: localhost (127.0.0.1)
- Configurable IP address for remote viewing
- ZMQ messaging protocol for low-latency streaming

### 3. Web Interface (Browser-based)
**Rerun Web Viewer:**
- Browser-accessible BEV visualization
- Timeline scrubbing and 3D view controls  
- Multi-camera synchronized playback
- Remote access capabilities

**Web Usage:**
```bash
# Launch web-based visualization
tools/rerun/run.py --qcam --fcam <route-name>
# Opens browser with interactive 3D viewer
```

## Usage Examples

### 1. Production BEV (Runtime):
```bash
# Live BEV during driving (on-device display)
cd selfdrive/ui && ./ui
```

### 2. Debug BEV (Replay):
```bash
# Replay with network streaming
ZMQ=1 tools/replay/replay <route-name>
python3 tools/replay/rp_visualization.py

# Interactive pygame debugging
tools/replay/replay <route-name>
python3 tools/replay/ui.py
```

### 3. Advanced Analysis (Web):
```bash
# Multi-camera web visualization
tools/rerun/run.py --qcam --fcam --ecam <route-name>
```

### 4. Map-based BEV:
```python
# Navigation BEV with map overlay
from porting.mapd.selfdrive.navd.map_renderer import get_ffi, wait_ready
```

## Implementation Insights

### 1. **Coordinate System:**
- **X-axis:** Forward direction (positive ahead of vehicle)
- **Y-axis:** Lateral direction (positive to the left)
- **Z-axis:** Vertical direction (positive upward)
- **Origin:** Rear axle center of ego vehicle

### 2. **Projection Pipeline:**
1. **3D Model Points** → Camera space transformation
2. **Camera Space** → Screen space projection (x/z, y/z)
3. **Screen Space** → BEV pixel coordinates
4. **Clipping** → Remove points outside view bounds

### 3. **Performance Optimizations:**
- **Pre-allocated arrays:** Reduces memory allocation overhead
- **Batch transformations:** Single matrix multiplication for multiple points
- **Clipping optimization:** Early rejection of out-of-bounds points
- **Color caching:** Efficient palette-based rendering

## Future Enhancement Opportunities

1. **WebGL Integration:** Browser-based BEV visualization
2. **Multi-scale Rendering:** Zoom levels for different use cases
3. **Historical Trajectory:** Long-term path visualization
4. **Semantic Segmentation:** Object-aware BEV rendering
5. **AR Integration:** Augmented reality overlay capabilities

## Conclusion

The nagaspilot codebase provides a sophisticated and comprehensive BEV plotting system that effectively visualizes ego vehicle trajectory, lead vehicle tracking, and lane line detection. The multi-framework approach (Rerun, Pygame, PyRay, Matplotlib) provides flexibility for different use cases from real-time debugging to production UI. The implementation demonstrates strong engineering practices with performance optimizations, modular design, and extensive configurability.

The BEV system successfully achieves the goal of providing intuitive top-down visualization of autonomous driving perception and planning data, making it an essential tool for development, debugging, and validation of autonomous vehicle systems.