# BYD Atto3 Unmapped Signals

This document lists BYD Atto3 signals that are available in the vehicle's DBC file but cannot be mapped to BrownPanda universal interface due to architectural limitations.

## Summary
- **Total BYD Functional Signals**: 50
- **Mapped to BrownPanda**: 35 signals (100% coverage of supported functionality)  
- **BYD-Only Signals**: 15 signals (documented below as unsupported)

## BYD-Only Signals (Not Supported by BrownPanda)

### 1. Longitudinal Control Extensions (Message 814 - longCommand)
Advanced ACC control signals that exceed BrownPanda's basic longitudinal control scope:

| Signal | Type | Range | Description | Reason Not Supported |
|--------|------|-------|-------------|---------------------|
| `cmdAccOverrideStandstill` | bool | 0-1 | ACC override standstill command | BrownPanda uses simplified standstill logic |
| `cmdAccControllable` | bool | 0-1 | ACC controllable status | BrownPanda assumes always controllable when active |
| `cmdReqActiveLow` | bool | 0-1 | ACC request active low | BrownPanda uses positive logic |
| `cmdAccReqNotStandstill` | bool | 0-1 | ACC request not standstill | BrownPanda handles via speed/standstill state |
| `cmdStandstillResume` | bool | 0-1 | Standstill resume command | BrownPanda uses unified resume logic |
| `accelFactor` | uint8 | 0-15 | Acceleration factor | BrownPanda uses fixed acceleration curves |
| `decelFactor` | uint8 | 0-15 | Deceleration factor | BrownPanda uses fixed deceleration curves |

### 2. Drivetrain Control (Not Available in BrownPanda)
Direct drivetrain control signals that are vehicle-specific:

| Signal | Type | Range | Description | Reason Not Supported |
|--------|------|-------|-------------|---------------------|
| `cmdTorq` | float | -3276.8 to 3276.7 | Drivetrain torque command | BrownPanda focuses on speed/acceleration control |
| `cmdTorqMode` | uint8 | 0-15 | Drivetrain torque mode | Vehicle-specific torque management |
| `cmdDrvEn` | bool | 0-1 | Drivetrain enable command | Handled at higher system level |
| `cmdRegenEn` | bool | 0-1 | Regenerative braking enable | Vehicle-specific energy management |
| `cmdDrvMode` | uint8 | 0-15 | Drivetrain mode selection | Vehicle-specific drive modes |

### 3. Chassis Control (Not Available in BrownPanda)
Vehicle stability and chassis management signals:

| Signal | Type | Range | Description | Reason Not Supported |
|--------|------|-------|-------------|---------------------|
| `stabCtrlLvl` | float | 0-102 | Stability control level | Vehicle-specific stability systems |
| `drvMode` | uint8 | 0-15 | Drive mode selection | Vehicle-specific mode management |
| `chassCtrlAct` | bool | 0-1 | Chassis control active | Vehicle-specific chassis systems |

## Impact Analysis

### No Functional Impact
These unmapped signals represent advanced vehicle-specific controls that exceed BrownPanda's universal interface design. The core autonomous driving functionality is fully supported through the 35 mapped signals.

### BrownPanda Coverage
BrownPanda successfully provides:
- ✅ Complete steering control (angle, torque, enable/disable)
- ✅ Full longitudinal control (acceleration, braking, cruise control)
- ✅ Essential vehicle state monitoring (speed, pedals, doors, seatbelts)
- ✅ User input processing (buttons, turn signals)
- ✅ Safety systems integration (blind spot monitoring)

### BYD-Specific Features
The 15 unmapped signals represent BYD's advanced vehicle management features that are outside the scope of universal autonomous driving interfaces:
- Advanced ACC parameter tuning
- Drivetrain efficiency optimization  
- Vehicle-specific chassis dynamics
- Manufacturer-specific drive modes

## Implementation Notes
- These signals should remain in BYD's native control domain
- BrownPanda's universal interface provides sufficient abstraction for autonomous driving
- No additional mapping is recommended to maintain interface simplicity and cross-vehicle compatibility

---
*Generated from BYD Atto3 DBC analysis on 2025-07-29*