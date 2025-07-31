# Comprehensive Comparison: dragonpilot vs sunnypilot

## Executive Summary

This report provides a detailed comparison between two major openpilot forks:
- **dragonpilot** (v0.9.9 r1) - Located at `/home/vcar/Winsurf/openpilot` 
- **sunnypilot** (v0.9.7.1) - Located at `/home/vcar/Winsurf/sunnypilot`

Both are sophisticated forks of comma.ai's openpilot with different philosophies and feature sets aimed at enhancing the autonomous driving experience.

## Repository Overview

### dragonpilot
- **Primary Focus**: Advanced longitudinal control algorithms and Toyota-specific enhancements
- **Latest Version**: 0.9.9 r1 (2025-06-20)
- **Based on**: openpilot 0.9.9 (2025-06-05)
- **License**: MIT Non-Commercial License with custom restrictions
- **Maintainer**: dragonpilot community (ricklan@gmail.com)

### sunnypilot  
- **Primary Focus**: Modified Assistive Driving Safety (MADS) and enhanced user experience
- **Latest Version**: 0.9.7.1 (2024-06-13) 
- **Based on**: openpilot master commit f8cb04e (June 10, 2024)
- **License**: MIT License (same as openpilot)
- **Maintainer**: sunnyhaibin and community

## Core Feature Comparison

### Longitudinal Control Systems

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

**2. Adaptive Coasting Mode (ACM)**
- **Location**: `/dragonpilot/selfdrive/controls/lib/acm.py`
- **Function**: Allows vehicle coasting on downhill slopes without braking
- **Key Features**:
  - Slope detection using orientationNED data
  - Configurable downhill-only mode
  - Lead vehicle TTC consideration
  - Brake suppression when conditions are met

**3. No Gas Gating Mode (NoGG) Toggle**
- Removes gas pedal gating for smoother operation

#### sunnypilot Unique Features:

**1. Modified Assistive Driving Safety (MADS)**
- **Core Philosophy**: Independent operation of Automatic Lane Centering and ACC/SCC
- **Key Features**:
  - Dedicated button controls (CRUISE MAIN, LFA, LKAS buttons)
  - Independent engagement/disengagement of lateral and longitudinal control
  - Enhanced safety compliance with comma.ai's safety rules
  - Configurable brake press behavior for lateral control

**2. Enhanced Speed Control**
- **Vision-based Turn Speed Control (V-TSC)**: Uses vision model for curve speed adjustment
- **Map-Data-based Turn Speed Control (M-TSC)**: Uses OpenStreetMap data for turns
- **Speed Limit Control (SLC)**: Automatically adapts cruise speed to road limits
- **HKG Highway Driving Assist (HDA)** integration for supported vehicles

**3. Gap Adjust Cruise (GAC)**
- Allows steering wheel button adjustment of following distance
- Three modes: Stock Gap (1.45s), Mild Gap (1.25s), Aggro Gap (1.0s)

### Lane Control Systems

#### dragonpilot Features:
**1. Auto Lane Change (ALC)**
- Enhanced auto lane change functionality
- LCA Speed Changer for customizable lane change speeds
- Road Edge Detection (RED) for improved lane boundaries

**2. ALKA (Auto Lane Keep Assist)**
- Advanced lane keeping assistance

#### sunnypilot Features:
**1. Dynamic Lane Profile (DLP)**
- **Three modes**: Auto Lane, Laneline, Laneless
- Dynamically switches between lane profiles based on confidence
- Real-time adaptation to road conditions

**2. Auto Lane Change Timer → Auto Lane Change by Blinker**
- Configurable timer delays for lane changes
- Blinker-activated lane changes without steering wheel nudge

### UI and Visual Enhancements

#### dragonpilot UI Features:
- **Border Indicators**: Visual border enhancements
- **Display Radar Tracks**: Shows radar tracking information
- **Rainbow Path**: Colorful path visualization
- **Speed Based HUD**: Adaptive heads-up display
- **Display Mode**: Configurable display options

#### sunnypilot UI Features:
- **M.A.D.S Status Icon**: Dedicated MADS engagement indicator
  - Green🟢: MADS engaged
  - White⚪: MADS suspended/disengaged
- **Lane Path Colors**: Real-time status visualization
  - Blue🔵: Laneful mode & MADS engaged
  - Green🟢: Laneless mode & MADS engaged  
  - Yellow🟡: Experimental e2e & MADS engaged
  - White⚪: MADS suspended
  - Black⚫: Manual steering override
- **Developer UI**: Real-time metrics display
- **Stand Still Timer**: Traffic light/stop time tracking
- **Braking Status**: Speed text turns red during braking

### Vehicle-Specific Support

#### dragonpilot Specializations:
- **Toyota Enhancements**:
  - Stock Longitudinal Mode
  - Door Auto Lock/Unlock
  - Radar Filter
  - TSS1 Stop and Go Modifications
  - SDSU Support
  - ZSS Support
- **VAG (Volkswagen Group)**:
  - EPS Lockout avoidance
  - PQ platform steering patches
  - MQB A0 Stop and Go modifications
- **HKG (Hyundai/Kia/Genesis)**:
  - SMDPS Support

#### sunnypilot Specializations:
- **Custom Stock Longitudinal Control** for:
  - Hyundai/Kia/Genesis (CAN and CAN-FD platforms)
  - Honda Bosch
  - Volkswagen MQB
- **Supported car count**: 250+ vs dragonpilot's focus on specific makes

## Advanced Features

### Cloud Services and Connectivity

#### sunnypilot:
**sunnylink (Alpha)**
- Remote settings backup and restore
- AES encryption with RSA private key
- GitHub account pairing via QR code
- iOS Siri Shortcuts navigation support
- Patron/Sponsor early access features

#### dragonpilot:
**FileServ**
- Local file server on port 5000
- Web-based file access

### Model and AI Enhancements

#### dragonpilot:
- **Model Selector**: Choose between different driving models
- **Adaptive Experimental Mode**: AI-driven mode selection

#### sunnypilot:
- **Driving Model Selector v4**: Multiple model options including:
  - North Dakota (NDv2)
  - WD40
  - Duck Amigo (DA)
  - Recertified Herbalist (CHLR)
- **Neural Network Lateral Control (NNLC)**
- **Torque Lateral Control Live Tune**

### Operational Enhancements

#### Common Features:
- Fast Boot capabilities
- Disable onroad uploads
- Brightness control
- Auto shutdown functionality

#### sunnypilot Specific:
- **Forced Offroad Mode**: Change settings while car is on
- **Offline OSM Maps**: Local map database for offline operation
- **Quiet Drive**: Mute notification sounds
- **Various Live Tuning**: Real-time parameter adjustment

## Performance and Stability

### dragonpilot:
- **Version**: More recent (0.9.9 r1 vs 0.9.7.1)
- **Update Frequency**: Regular releases with incremental improvements
- **Stability**: Focused on specific vehicle makes with deep integration

### sunnypilot:
- **Version**: Slightly older base but feature-rich
- **Update Frequency**: Regular updates with new feature additions
- **Stability**: Broader vehicle support with extensive testing

## Development Philosophy Differences

### dragonpilot:
- **Approach**: Deep, manufacturer-specific customizations
- **Focus**: Advanced AI algorithms for longitudinal control
- **Target**: Users seeking cutting-edge experimental features
- **License**: Restrictive non-commercial license

### sunnypilot:
- **Approach**: Enhanced user experience with safety focus
- **Focus**: Independent lateral/longitudinal control (MADS)
- **Target**: Users wanting more flexible control options
- **License**: Standard MIT license

## Recommendation Matrix

### Choose dragonpilot if you:
- ✅ Drive a Toyota, VAG, or HKG vehicle
- ✅ Want experimental AI-driven longitudinal control
- ✅ Prefer automatic system optimization
- ✅ Are comfortable with non-commercial licensing restrictions
- ✅ Want the latest openpilot base version

### Choose sunnypilot if you:
- ✅ Want independent lateral/longitudinal control (MADS)
- ✅ Need broad vehicle compatibility (250+ cars)
- ✅ Prefer extensive customization options
- ✅ Want established, well-tested features
- ✅ Need commercial-use compatibility
- ✅ Value comprehensive documentation and community support

## Conclusion

Both forks represent significant enhancements over stock openpilot, but with different design philosophies:

**dragonpilot** excels in advanced AI algorithms and deep vehicle-specific integrations, making it ideal for users who want cutting-edge experimental features and drive supported vehicles.

**sunnypilot** provides a more mature, broadly compatible platform with extensive customization options and the unique MADS system, making it suitable for users who want flexible control over their driving assistance systems.

The choice between them depends on your specific vehicle, risk tolerance, feature preferences, and intended use case (commercial vs. personal).

---

*Report generated on 2025-07-10*  
*Analysis based on dragonpilot 0.9.9 r1 and sunnypilot 0.9.7.1*