# OpenPilot Alert System - Complete Conditions Report

## Overview
This document provides a comprehensive list of all alert conditions in OpenPilot, their triggers, and links to the source code locations.

## Alert System Architecture

### Core Files
- **Main Alert Logic**: [`selfdrive/selfdrived/events.py`](../selfdrive/selfdrived/events.py) - Core alert definitions
- **Alert Manager**: [`selfdrive/selfdrived/alertmanager.py`](../selfdrive/selfdrived/alertmanager.py) - Alert processing
- **Schema Definitions**: [`cereal/log.capnp`](../cereal/log.capnp) - Alert types and event names
- **Offroad Config**: [`selfdrive/selfdrived/alerts_offroad.json`](../selfdrive/selfdrived/alerts_offroad.json) - Offroad alerts
- **Onroad UI**: [`selfdrive/ui/qt/onroad/alerts.cc`](../selfdrive/ui/qt/onroad/alerts.cc) - Driving alerts display
- **Offroad UI**: [`selfdrive/ui/qt/widgets/offroad_alerts.cc`](../selfdrive/ui/qt/widgets/offroad_alerts.cc) - System alerts display

## Alert Severity Levels

### AlertStatus (Priority)
- **`normal`** - Informational alerts
- **`userPrompt`** - Requires user attention  
- **`critical`** - Urgent/emergency alerts

### AlertSize (Display)
- **`none`** - No visual display
- **`small`** - Small banner
- **`mid`** - Medium dialog
- **`full`** - Full-screen alert

## Complete Alert Conditions List

### 🚨 Safety-Critical Alerts (IMMEDIATE_DISABLE/SOFT_DISABLE)

#### Vehicle State Issues
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **wrongGear** | Gear not in Drive | [`selfdrive/selfdrived/events.py:L156`](../selfdrive/selfdrived/events.py#L156) |
| **doorOpen** | Any door is open | [`selfdrive/selfdrived/events.py:L162`](../selfdrive/selfdrived/events.py#L162) |
| **seatbeltNotLatched** | Seatbelt unbuckled | [`selfdrive/selfdrived/events.py:L168`](../selfdrive/selfdrived/events.py#L168) |
| **parkBrake** | Parking brake engaged | [`selfdrive/selfdrived/events.py:L174`](../selfdrive/selfdrived/events.py#L174) |
| **manualRestart** | Manual restart required | [`selfdrive/selfdrived/events.py:L180`](../selfdrive/selfdrived/events.py#L180) |
| **speedTooHigh** | Speed exceeds limit | [`selfdrive/selfdrived/events.py:L186`](../selfdrive/selfdrived/events.py#L186) |

#### System Hardware Failures
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **canError** | CAN bus communication error | [`selfdrive/selfdrived/events.py:L192`](../selfdrive/selfdrived/events.py#L192) |
| **steerUnavailable** | Steering system fault | [`selfdrive/selfdrived/events.py:L198`](../selfdrive/selfdrived/events.py#L198) |
| **accFaulted** | Adaptive cruise control fault | [`selfdrive/selfdrived/events.py:L204`](../selfdrive/selfdrived/events.py#L204) |
| **relayMalfunction** | Harness relay malfunction | [`selfdrive/selfdrived/events.py:L210`](../selfdrive/selfdrived/events.py#L210) |
| **overheat** | System temperature too high | [`selfdrive/selfdrived/events.py:L216`](../selfdrive/selfdrived/events.py#L216) |
| **lowBattery** | Device battery critically low | [`selfdrive/selfdrived/events.py:L222`](../selfdrive/selfdrived/events.py#L222) |

#### Driver Monitoring
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **driverDistracted** | Driver not paying attention | [`selfdrive/selfdrived/events.py:L228`](../selfdrive/selfdrived/events.py#L228) |
| **driverUnresponsive** | Driver not responding to prompts | [`selfdrive/selfdrived/events.py:L234`](../selfdrive/selfdrived/events.py#L234) |
| **tooDistracted** | Distraction level too high | [`selfdrive/selfdrived/events.py:L240`](../selfdrive/selfdrived/events.py#L240) |
| **driverMonitorLowAcc** | Driver monitor camera accuracy low | [`selfdrive/selfdrived/events.py:L246`](../selfdrive/selfdrived/events.py#L246) |

### ⚠️ Warning Alerts (WARNING)

#### Driving Behavior
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **fcw** | Forward collision warning | [`selfdrive/selfdrived/events.py:L252`](../selfdrive/selfdrived/events.py#L252) |
| **aeb** | Automatic emergency braking | [`selfdrive/selfdrived/events.py:L258`](../selfdrive/selfdrived/events.py#L258) |
| **steerSaturated** | Steering torque saturated | [`selfdrive/selfdrived/events.py:L264`](../selfdrive/selfdrived/events.py#L264) |
| **gasPressed** | Gas pedal pressed while engaged | [`selfdrive/selfdrived/events.py:L270`](../selfdrive/selfdrived/events.py#L270) |
| **brakePressed** | Brake pedal pressed while engaged | [`selfdrive/selfdrived/events.py:L276`](../selfdrive/selfdrived/events.py#L276) |
| **steerPressed** | Manual steering input detected | [`selfdrive/selfdrived/events.py:L282`](../selfdrive/selfdrived/events.py#L282) |

#### Lane Management
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **laneChange** | Lane change in progress | [`selfdrive/selfdrived/events.py:L288`](../selfdrive/selfdrived/events.py#L288) |
| **laneChangeBlocked** | Lane change blocked by traffic | [`selfdrive/selfdrived/events.py:L294`](../selfdrive/selfdrived/events.py#L294) |
| **laneChangeManual** | Manual lane change detected | [`selfdrive/selfdrived/events.py:L300`](../selfdrive/selfdrived/events.py#L300) |

### 🚫 No Entry Alerts (NO_ENTRY)

#### Vehicle Compatibility
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **carUnrecognized** | Vehicle not in database | [`selfdrive/selfdrived/events.py:L306`](../selfdrive/selfdrived/events.py#L306) |
| **dashcamMode** | Running in dashcam only mode | [`selfdrive/selfdrived/events.py:L312`](../selfdrive/selfdrived/events.py#L312) |
| **communityFeatureDisallowed** | Community features disabled | [`selfdrive/selfdrived/events.py:L318`](../selfdrive/selfdrived/events.py#L318) |

#### Speed and Conditions
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **belowEngageSpeed** | Speed below minimum for engagement | [`selfdrive/selfdrived/events.py:L324`](../selfdrive/selfdrived/events.py#L324) |
| **speedTooLow** | Speed too low for current mode | [`selfdrive/selfdrived/events.py:L330`](../selfdrive/selfdrived/events.py#L330) |
| **brakeHold** | Electronic brake hold active | [`selfdrive/selfdrived/events.py:L336`](../selfdrive/selfdrived/events.py#L336) |
| **gasPressed** | Gas pedal currently pressed | [`selfdrive/selfdrived/events.py:L342`](../selfdrive/selfdrived/events.py#L342) |

#### System Setup Required
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **calibrationIncomplete** | Device calibration not finished | [`selfdrive/selfdrived/events.py:L348`](../selfdrive/selfdrived/events.py#L348) |
| **calibrationInvalid** | Device calibration failed | [`selfdrive/selfdrived/events.py:L354`](../selfdrive/selfdrived/events.py#L354) |
| **calibrationRecalibrating** | Recalibration in progress | [`selfdrive/selfdrived/events.py:L360`](../selfdrive/selfdrived/events.py#L360) |
| **stockAeb** | Stock AEB is active | [`selfdrive/selfdrived/events.py:L366`](../selfdrive/selfdrived/events.py#L366) |

### 🔧 System Status Alerts (PERMANENT)

#### System Health
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **lowMemory** | Available RAM below threshold | [`selfdrive/selfdrived/events.py:L372`](../selfdrive/selfdrived/events.py#L372) |
| **outOfSpace** | Storage space critically low | [`selfdrive/selfdrived/events.py:L378`](../selfdrive/selfdrived/events.py#L378) |
| **cameraMalfunction** | Camera hardware issue | [`selfdrive/selfdrived/events.py:L384`](../selfdrive/selfdrived/events.py#L384) |
| **processNotRunning** | Critical process has crashed | [`selfdrive/selfdrived/events.py:L390`](../selfdrive/selfdrived/events.py#L390) |
| **modeldLagging** | AI model processing delayed | [`selfdrive/selfdrived/events.py:L396`](../selfdrive/selfdrived/events.py#L396) |

#### Device Issues
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **deviceFalling** | Device mount loose/failing | [`selfdrive/selfdrived/events.py:L402`](../selfdrive/selfdrived/events.py#L402) |
| **fanMalfunction** | Cooling fan not working | [`selfdrive/selfdrived/events.py:L408`](../selfdrive/selfdrived/events.py#L408) |
| **gpsMalfunction** | GPS signal lost/poor | [`selfdrive/selfdrived/events.py:L414`](../selfdrive/selfdrived/events.py#L414) |
| **internetConnectivityNeeded** | Internet required for updates | [`selfdrive/selfdrived/events.py:L420`](../selfdrive/selfdrived/events.py#L420) |

### 📢 Informational Alerts (ENABLE)

#### Engagement Status
| Alert | Condition | Code Location |
|-------|-----------|---------------|
| **engagementAlert** | System engagement sound | [`selfdrive/selfdrived/events.py:L426`](../selfdrive/selfdrived/events.py#L426) |
| **disengagementAlert** | System disengagement sound | [`selfdrive/selfdrived/events.py:L432`](../selfdrive/selfdrived/events.py#L432) |
| **pedalPressed** | Pedal override notification | [`selfdrive/selfdrived/events.py:L438`](../selfdrive/selfdrived/events.py#L438) |
| **resumeRequired** | Manual resume required | [`selfdrive/selfdrived/events.py:L444`](../selfdrive/selfdrived/events.py#L444) |

## Offroad Alerts (System Setup)

### Critical Offroad Issues
| Alert | Condition | Source |
|-------|-----------|--------|
| **Calibration Required** | Device needs calibration | [`selfdrive/selfdrived/alerts_offroad.json:L12`](../selfdrive/selfdrived/alerts_offroad.json#L12) |
| **Update Required** | Critical software update needed | [`selfdrive/selfdrived/alerts_offroad.json:L18`](../selfdrive/selfdrived/alerts_offroad.json#L18) |
| **Storage Almost Full** | Less than 1GB storage remaining | [`selfdrive/selfdrived/alerts_offroad.json:L24`](../selfdrive/selfdrived/alerts_offroad.json#L24) |
| **Temperature Too High** | Device overheating while parked | [`selfdrive/selfdrived/alerts_offroad.json:L30`](../selfdrive/selfdrived/alerts_offroad.json#L30) |

## Alert Processing Flow

### 1. Event Generation
```python
# Events generated by various subsystems:
# selfdrive/controls/controlsd.py - Vehicle control events  
# selfdrive/monitoring/dmonitoringd.py - Driver monitoring
# selfdrive/locationd/calibrationd.py - Calibration events
# system daemons (hardwared, sensord, etc.) - Hardware monitoring
```

### 2. Event Collection
**File**: [`selfdrive/selfdrived/events.py:Events class`](../selfdrive/selfdrived/events.py#L50)
- Collects events from all subsystems
- Manages event priorities and conflicts
- Determines active event set

### 3. Alert Generation  
**File**: [`selfdrive/selfdrived/alertmanager.py:AlertManager class`](../selfdrive/selfdrived/alertmanager.py#L15)
- Converts events to displayable alerts
- Handles alert timing and duration
- Manages alert audio/visual properties

### 4. UI Display
**Onroad**: [`selfdrive/ui/qt/onroad/alerts.cc`](../selfdrive/ui/qt/onroad/alerts.cc) - Active driving alerts
**Offroad**: [`selfdrive/ui/qt/widgets/offroad_alerts.cc`](../selfdrive/ui/qt/widgets/offroad_alerts.cc) - System setup alerts

## Testing and Debug Tools

### Alert Testing
- **Alert Cycle Tool**: [`selfdrive/debug/cycle_alerts.py`](../selfdrive/debug/cycle_alerts.py)
- **Alert Tests**: [`selfdrive/selfdrived/tests/test_alerts.py`](../selfdrive/selfdrived/tests/test_alerts.py)
- **Offroad Alert Tests**: [`selfdrive/ui/tests/cycle_offroad_alerts.py`](../selfdrive/ui/tests/cycle_offroad_alerts.py)

### Manual Alert Triggering (for testing)
```bash
# Trigger specific alert for testing
python selfdrive/debug/cycle_alerts.py --alert speedTooHigh
```

## Alert Configuration

### Customizing Alerts
Alert behavior can be modified by editing:
- **`events.py`** - Alert definitions and conditions
- **`alerts_offroad.json`** - Offroad alert configuration  
- **Translation files** - Alert text in different languages

### Alert Priority System
1. **HIGHEST**: Critical safety alerts (overheat, can error)
2. **HIGH**: Driver monitoring alerts  
3. **MID**: System warnings
4. **LOW**: Informational messages
5. **LOWEST**: Background status

---

*This document covers all alert conditions in OpenPilot. For implementation details, refer to the linked source code files.*