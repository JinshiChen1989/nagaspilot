# Comprehensive Comparison: stock openpilot vs dragonpilot

## Executive Summary

This report provides a detailed comparison between stock openpilot and its sophisticated fork:
- **stock openpilot** (v0.9.9) - Located at `/home/vcar/Winsurf/openpilot` 
- **dragonpilot** (v0.9.9 r1) - Located at `/home/vcar/Winsurf/dragonpilot`

Stock openpilot is comma.ai's official open-source driver assistance system, while dragonpilot is a community-driven fork with advanced longitudinal control algorithms, vehicle-specific enhancements, and experimental features.

## Repository Overview

### stock openpilot
- **Primary Focus**: Core autonomous driving capabilities with broad vehicle compatibility
- **Latest Version**: 0.9.9 (official comma.ai release)
- **Based on**: Original comma.ai development
- **License**: MIT License
- **Maintainer**: comma.ai team

### dragonpilot  
- **Primary Focus**: Advanced longitudinal control algorithms and vehicle-specific optimizations
- **Latest Version**: 0.9.9 r1 (2025-06-20)
- **Based on**: openpilot 0.9.9 (2025-06-05)
- **License**: MIT Non-Commercial License with custom restrictions
- **Maintainer**: dragonpilot community (ricklan@gmail.com)

## Core Feature Comparison

### Longitudinal Control Systems

#### stock openpilot Features:
- **Standard Adaptive Cruise Control (ACC)**: Maintains set speed and following distance
- **Lead Vehicle Following**: Basic radar-based lead car detection and tracking
- **Gas Pedal Gating**: Standard accelerator control for safety compliance
- **Experimental Longitudinal**: Basic experimental mode toggle

#### dragonpilot Unique Features:

**1. Adaptive Experimental Mode (AEM)**
- **Location**: `/dragonpilot/selfdrive/controls/lib/aem.py`
- **Function**: Dynamically switches between ACC and Blended mode based on driving context
- **Key Features**:
  - EMA (Exponential Moving Average) filtering for smooth transitions
  - Multi-scenario detection (emergency, cut-ins, urban driving, model predictions)
  - Hysteresis mechanism to prevent mode oscillation
  - Speed thresholds: Highway (22.23 m/s), City (15.27 m/s), Low (5.56 m/s)
  - Lead car analysis with TTC (Time To Collision) calculations
  - Personality-aware adjustments for aggressive highway driving

**2. Adaptive Coasting Mode (ACM)**
- **Location**: `/dragonpilot/selfdrive/controls/lib/acm.py`
- **Function**: Allows vehicle coasting on downhill slopes without unnecessary braking
- **Key Features**:
  - Slope detection using IMU orientationNED data
  - Configurable downhill-only or general coasting modes
  - Lead vehicle TTC consideration for safety
  - Brake suppression when conditions are optimal
  - Integration with longitudinal planner trajectory modification

**3. No Gas Gating Mode (NoGG) Toggle**
- **Parameter**: `dp_no_gas_gating`
- **Function**: Removes gas pedal gating for smoother operation
- **Benefit**: Eliminates accelerator input restrictions for enhanced responsiveness

### Lane Control Systems

#### stock openpilot Features:
- **Lane Keeping Assist (LKA)**: Standard lane centering with vision-based detection
- **Lane Departure Warning**: Alerts when vehicle drifts from lane
- **Auto Lane Change**: Basic blinker-activated lane changes (experimental)

#### dragonpilot Features:
**1. Auto Lane Change (ALC) Enhancements**
- **Location**: Various files in selfdrive/controls
- **Features**:
  - Enhanced auto lane change functionality with improved logic
  - LCA Speed Changer for customizable lane change speeds
  - Road Edge Detection (RED) for improved lane boundary detection
  - More aggressive and confident lane change execution

**2. ALKA (Auto Lane Keep Assist)**
- **Parameter**: `dp_alka`
- **Function**: Advanced lane keeping assistance with enhanced responsiveness
- **Features**: More aggressive lane centering and departure prevention

**3. Road Edge Detection (RED)**
- **Location**: `/dragonpilot/selfdrive/controls/lib/road_edge_detector.py`
- **Function**: Enhanced lane boundary detection using advanced algorithms
- **Benefit**: Better performance on roads with poor lane markings

### Driver Monitoring Systems

#### stock openpilot Features:
- **Driver Monitoring Camera**: Vision-based face and eye tracking
- **Attention Alerts**: Graduated warning system for driver distraction
- **Touch Detection**: Steering wheel touch sensing for engagement

#### dragonpilot Features:
**1. DPMonitoringD (Simplified Monitoring)**
- **Location**: `/selfdrive/monitoring/dpmonitoringd.py`
- **Function**: Simplified driver monitoring with configurable thresholds
- **Key Features**:
  - Right-hand drive (RHD) support (`dp_device_is_rhd`)
  - Option to disable monitoring entirely (`dp_device_monitoring_disabled`)
  - Multiple reset conditions (steering, gas, brake, blinkers)
  - Three-tier warning system (45s, 60s, 75s)
  - Simplified alert logic without face detection complexity

### UI and Visual Enhancements

#### stock openpilot UI Features:
- **Standard HUD**: Basic speed, lead car, and lane line display
- **Settings Panel**: Core openpilot configuration options
- **Alert System**: Standard warning and error messages

#### dragonpilot UI Features:
**1. DragonPilot Settings Panel**
- **Location**: `/selfdrive/ui/qt/offroad/dp_panel.cc`
- **Features**:
  - Comprehensive settings interface with dragon emoji branding 🐲
  - Vehicle-specific toggle sections (Toyota, VAG, etc.)
  - Parameter locking mechanism for safety
  - Dynamic UI elements based on vehicle compatibility

**2. Model Selector**
- **Location**: `/selfdrive/ui/qt/offroad/model_selector.h`
- **Features**:
  - GUI for switching between different driving models
  - JSON-based model configuration system
  - Dynamic model loading capabilities
  - Integration with main settings panel

**3. Visual Enhancements**:
- **Rainbow Path**: Colorful path visualization (`dp_ui_rainbow`)
- **Radar Tracks Display**: Shows radar tracking information (`dp_ui_radar_tracks`)
- **Speed Display Options**: Hide/show speed in different units
- **Border Indicators**: Enhanced visual border effects
- **Custom Display Modes**: Multiple HUD configuration options

### Vehicle-Specific Support

#### stock openpilot Specializations:
- **Broad Vehicle Compatibility**: 250+ supported vehicles across manufacturers
- **Generic Implementations**: Features designed to work across all supported vehicles
- **Standard CAN Integration**: Basic vehicle interface implementations

#### dragonpilot Specializations:
**1. Toyota/Lexus Enhancements**:
- **Stock Longitudinal Mode**: Native Toyota ACC integration (`dp_toyota_stock_lon`)
- **Door Auto Lock/Unlock**: Automatic door control (`dp_toyota_door_auto_lock_unlock`)
- **TSS1 Stop & Go**: Enhanced stop-and-go for TSS1 vehicles (`dp_toyota_tss1_sng`)
- **Radar Filter**: Improved radar processing for Toyota vehicles
- **SDSU Support**: Enhanced low-speed control
- **ZSS Support**: Zero Speed Start functionality

**2. VAG (Volkswagen/Audi/Skoda) Enhancements**:
- **EPS Lockout Avoidance**: Prevents steering system lockouts (`dp_vag_avoid_eps_lockout`)
- **PQ Platform Steering Patch**: Enhanced steering for older VAG vehicles (`dp_vag_pq_steering_patch`)
- **MQB A0 Stop and Go**: Stop-and-go modifications for compact VAG vehicles (`dp_vag_a0_sng`)
- **Platform-Specific CAN**: Optimized CAN communication for VAG platforms

**3. HKG (Hyundai/Kia/Genesis) Enhancements**:
- **SMDPS Support**: Smart Motor Driven Power Steering integration
- **Enhanced lateral control**: Improved steering response for HKG vehicles

## Advanced Features

### Cloud Services and Connectivity

#### stock openpilot:
- **comma connect**: Basic connectivity for data uploads and remote access
- **Standard telemetry**: Vehicle and driving data collection

#### dragonpilot:
**FileServ Web Interface**
- **Location**: `/selfdrive/fileserv/fileserv.py`
- **Features**:
  - Local HTTP server on port 5000 for device file access
  - Responsive web UI optimized for mobile devices
  - HLS video player for .ts files with automatic streaming
  - File size formatting and directory navigation
  - Security path validation to prevent directory traversal attacks
  - Threaded request handling for concurrent access

### Model and AI Enhancements

#### stock openpilot:
- **Single Driving Model**: Uses comma.ai's latest supercombo model
- **Standard experimental mode**: Basic longitudinal experimental toggle

#### dragonpilot:
- **Model Selector**: Choose between different driving models with GUI interface
- **Adaptive Experimental Mode**: AI-driven automatic mode selection based on driving context
- **Enhanced Model Integration**: Support for custom and alternative driving models

### Audio and Alert Systems

#### stock openpilot:
- **Standard Alerts**: Built-in system sounds and visual alerts
- **Basic audio feedback**: Standard engagement/disengagement sounds

#### dragonpilot:
**Enhanced Audio System**
- **Beep Process**: Dedicated audio process (`beepd`) for hardware-based alerts
- **GPIO Audio Control**: Direct hardware audio control for instant feedback
- **Configurable Alert Modes**: Multiple beep patterns and volume controls
- **Custom Alert Sounds**: Support for different alert types and intensities

### Operational Enhancements

#### Common Features (present in both):
- Fast boot capabilities
- Brightness control
- Auto shutdown functionality
- SSH access

#### dragonpilot Specific:
**1. Advanced Parameter System**:
- **Extended Parameters**: 50+ dragonpilot-specific configuration options
- **Parameter Locking**: Safety locks for critical settings
- **Vehicle-Specific Defaults**: Automatic parameter optimization per vehicle type

**2. Enhanced Process Management**:
- **Additional Processes**: `beepd`, `dpmonitoringd`, `fileserv`
- **Conditional Process Control**: Dynamic process enabling based on configuration
- **Process Health Monitoring**: Enhanced monitoring of dragonpilot-specific processes

## Performance and Stability

### stock openpilot:
- **Stability**: Highly stable, extensively tested by comma.ai
- **Update Frequency**: Regular official releases following comma.ai development
- **Performance**: Optimized for broad hardware and vehicle compatibility
- **Testing**: Comprehensive QA and field testing before release

### dragonpilot:
- **Stability**: Generally stable with focus on specific vehicle makes
- **Update Frequency**: Frequent community releases with incremental improvements
- **Performance**: Optimized for supported vehicles with deep integration
- **Testing**: Community-driven testing with vehicle-specific validation

## Development Philosophy Differences

### stock openpilot:
- **Approach**: Centralized development with focus on safety and broad compatibility
- **Focus**: Robust, reliable driver assistance for maximum vehicle coverage
- **Target**: General users seeking proven, well-tested autonomous driving
- **License**: Permissive MIT license allowing commercial use
- **Development**: Professional team with structured release cycles

### dragonpilot:
- **Approach**: Community-driven development with vehicle-specific deep customizations
- **Focus**: Advanced experimental features and manufacturer-specific optimizations
- **Target**: Enthusiasts and users with supported vehicles seeking cutting-edge features
- **License**: Restrictive non-commercial license
- **Development**: Community contributors with feature-branch development model

## Recommendation Matrix

### Choose stock openpilot if you:
- ✅ Want maximum stability and reliability
- ✅ Drive any of the 250+ broadly supported vehicles
- ✅ Prefer official comma.ai support and updates
- ✅ Need commercial-use compatibility
- ✅ Want proven, extensively tested features
- ✅ Prefer simpler configuration with fewer options

### Choose dragonpilot if you:
- ✅ Drive a Toyota, VAG, or HKG vehicle
- ✅ Want advanced AI-driven longitudinal control (AEM/ACM)
- ✅ Prefer extensive customization and experimental features
- ✅ Are comfortable with non-commercial licensing restrictions
- ✅ Want vehicle-specific optimizations and enhancements
- ✅ Enjoy community-driven development and frequent updates
- ✅ Need advanced features like web-based file access

## Conclusion

Both versions offer compelling autonomous driving capabilities, but serve different user needs:

**stock openpilot** provides a robust, well-tested foundation with broad vehicle compatibility, making it ideal for users who prioritize stability, official support, and proven performance across diverse vehicle types.

**dragonpilot** builds upon stock openpilot by adding sophisticated experimental features, deep vehicle-specific integrations, and advanced AI algorithms, making it perfect for enthusiasts with supported vehicles who want cutting-edge functionality and extensive customization options.

The choice depends on your specific vehicle, risk tolerance, feature preferences, and intended use case (commercial vs. personal). Stock openpilot emphasizes broad compatibility and stability, while dragonpilot focuses on advanced features and vehicle-specific optimization.

---

*Report generated on 2025-07-11*  
*Analysis based on stock openpilot 0.9.9 and dragonpilot 0.9.9 r1*