# Feature Integration Plan: Advanced Controls from SunnyPilot to DragonPilot

## Executive Summary

This document outlines a comprehensive plan to integrate five advanced control features from SunnyPilot into the current DragonPilot codebase, leveraging existing architecture patterns and reusable software components.

**Target Features:**
1. MADS (Independent Lateral/Longitudinal Control)
2. Smart Offset (Truck & Touch Lane)
3. Dynamic Lane Profile (DLP)
4. Dynamic Gap Adjust Cruise by TTC
5. Dynamic Curve Speed Decrease

## Current State Analysis

### DragonPilot Architecture Strengths
- **Sophisticated existing features**: ACM (Adaptive Coasting) and AEM (Adaptive Experimental Mode) provide excellent foundation
- **Robust parameter system**: Well-established `dp_*` parameter naming and management
- **Modular control architecture**: Clean separation between lateral and longitudinal control
- **Safety-first design**: Comprehensive override and failsafe mechanisms

### SunnyPilot Reusable Components
- **Mature MADS implementation**: Independent lat/long control with proper safety interlocks
- **Vision-based curve control**: Advanced turn speed controller with map integration
- **Dynamic lane profile system**: Automatic laneful/laneless mode switching
- **Smart offset framework**: Configurable lane and path offset systems
- **Gap adjust patterns**: TTC-based following distance management

## Integration Strategy

### Phase 1: Foundation Setup (Week 1-2)

#### 1.1 Component Extraction and Analysis
**Objective**: Extract reusable components from SunnyPilot while maintaining DragonPilot conventions

**Key Actions:**
- Extract core algorithms from SunnyPilot components
- Adapt parameter naming to DragonPilot `dp_*` convention
- Analyze dependencies and integration requirements
- Create compatibility matrix for existing DragonPilot features

**Deliverables:**
- Component library in `/dragonpilot/lib/` directory
- Parameter mapping document
- Dependency analysis report

#### 1.2 Architecture Extensions
**Objective**: Extend existing DragonPilot architecture to support new features

**Key Actions:**
- Extend `DPFlags` system in `longitudinal_planner.py`
- Add new parameter definitions to `params_keys.h`
- Create feature integration framework in `controlsd.py`
- Design messaging extensions for new control states

**Implementation Details:**
```python
# Extended DPFlags system
class DPFlags:
    ACM = 1
    ACM_DOWNHILL = 2
    AEM = 2 ** 2
    NO_GAS_GATING = 2 ** 3
    MADS = 2 ** 4              # New: Independent lat/long control
    SMART_OFFSET = 2 ** 5      # New: Lane offset system
    DLP = 2 ** 6               # New: Dynamic lane profile
    GAC_TTC = 2 ** 7           # New: Gap adjust by TTC
    CURVE_SPEED = 2 ** 8       # New: Dynamic curve speed
```

### Phase 2: Core Feature Implementation (Week 3-6)

#### 2.1 MADS (Modified Assistive Driving Safety)
**Priority**: High - Foundation for independent control
**Integration Point**: `controlsd.py` - Main control loop

**Technical Approach:**
- Build upon existing ALKA (Always-on Lateral Keep Assist) system
- Create independent enable conditions for lateral and longitudinal control
- Implement brake-to-disengage-lateral functionality
- Maintain safety interlocks for both control modes

**Key Components:**
```python
# Independent control state management
class MADSController:
    def __init__(self):
        self.lat_enabled = False
        self.long_enabled = False
        self.brake_disengages_lateral = True
    
    def update_control_state(self, CC, sm):
        # Independent lat/long enable logic
        self.lat_enabled = self._check_lateral_conditions(sm)
        self.long_enabled = self._check_longitudinal_conditions(sm)
        
        CC.latActive = self.lat_enabled
        CC.longActive = self.long_enabled
```

**Parameters:**
- `dp_mads_enabled`: Master enable toggle
- `dp_mads_lat_independent`: Allow lateral-only control
- `dp_mads_long_independent`: Allow longitudinal-only control
- `dp_mads_brake_disengages_lat`: Brake disengages lateral control

#### 2.2 Smart Offset System
**Priority**: Medium - Enhances lane positioning
**Integration Point**: `controlsd.py` - Curvature calculation

**Technical Approach:**
- Implement configurable lane and path offsets
- Add touch-screen lane change functionality
- Create truck-specific offset profiles
- Use existing `modelV2.laneLines` data for positioning

**Key Components:**
```python
class SmartOffsetController:
    def __init__(self):
        self.camera_offset = 0.0
        self.path_offset = 0.0
        self.lane_offset = 0.0
        self.touch_offset = 0.0
    
    def calculate_offset_curvature(self, curvature, lane_lines):
        # Apply smart offset logic
        return modified_curvature
```

**Parameters:**
- `dp_smart_offset_enabled`: Master toggle
- `dp_camera_offset`: Camera position offset (-0.1 to 0.1m)
- `dp_path_offset`: Path following offset (-0.5 to 0.5m)
- `dp_lane_offset`: Lane positioning offset (-1.0 to 1.0m)
- `dp_touch_lane_enabled`: Touch lane change functionality

#### 2.3 Dynamic Gap Adjust Cruise by TTC
**Priority**: High - Builds on existing ACM
**Integration Point**: `longitudinal_planner.py` - Following distance

**Technical Approach:**
- Extend existing ACM TTC calculations
- Implement dynamic following distance based on TTC thresholds
- Create button-based gap adjustment interface
- Integrate with existing lead vehicle tracking

**Key Components:**
```python
class GapAdjustController:
    def __init__(self):
        self.gap_setting = 1.8  # Default following time
        self.ttc_threshold = 3.5
        self.min_gap = 1.0
        self.max_gap = 3.0
    
    def calculate_following_distance(self, lead_ttc, v_ego):
        # Dynamic gap adjustment based on TTC
        return adjusted_distance
```

**Parameters:**
- `dp_gac_enabled`: Master toggle
- `dp_gac_min_gap`: Minimum following distance (1.0-2.0s)
- `dp_gac_max_gap`: Maximum following distance (2.0-4.0s)
- `dp_gac_ttc_threshold`: TTC threshold for adjustment (2.0-5.0s)

#### 2.4 Dynamic Curve Speed Decrease
**Priority**: Medium - Safety enhancement
**Integration Point**: `longitudinal_planner.py` - Speed planning

**Technical Approach:**
- Use existing `modelV2.curvatures` data for curve detection
- Implement vision-based turn speed controller
- Create configurable speed reduction profiles
- Integrate with existing speed limit logic

**Key Components:**
```python
class CurveSpeedController:
    def __init__(self):
        self.curve_threshold = 0.001  # rad/m
        self.speed_reduction_factor = 0.8
        self.min_curve_speed = 45  # km/h
    
    def calculate_curve_speed(self, curvature, current_speed):
        # Calculate safe curve speed
        return safe_speed
```

#### 2.5 Dynamic Lane Profile (DLP)
**Priority**: Low - Advanced feature
**Integration Point**: `lateral_planner.py` - Lane planning

**Technical Approach:**
- Implement automatic laneful/laneless mode switching
- Use confidence metrics from lane detection
- Create smooth transition logic between modes
- Integrate with existing lateral MPC

### Phase 3: Integration and Safety (Week 7-8)

#### 3.1 Safety System Integration
**Critical Requirements:**
- Maintain all existing safety interlocks
- Implement proper failsafe mechanisms for independent control
- Ensure override conditions work correctly
- Add monitoring for feature conflicts

**Safety Checks:**
```python
def check_safety_conditions():
    # Ensure critical safety systems remain active
    if not driver_monitoring_ok():
        return False
    if emergency_braking_required():
        return False
    if steering_override_detected():
        return False
    return True
```

#### 3.2 Feature Interaction Management
**Conflict Resolution:**
- MADS vs ALKA: MADS supersedes ALKA when enabled
- ACM vs GAC: GAC modifies ACM behavior, not conflicts
- AEM vs Dynamic Controls: Priority system for mode switching

#### 3.3 UI Integration
**Settings Panel Integration** (`dp_panel.cc`):
- Add feature toggles with proper grouping
- Implement parameter validation and bounds checking
- Create informative descriptions and warnings
- Add experimental feature warnings where appropriate

### Phase 4: Validation and Optimization (Week 9-10)

#### 4.1 Testing Strategy
**Validation Methods:**
- Replay system testing with diverse scenarios
- Hardware-in-the-loop testing for safety systems
- Parameter boundary testing
- Feature interaction testing

**Test Scenarios:**
- Highway driving with lane changes
- Urban driving with frequent stops
- Curve handling at various speeds
- Emergency braking scenarios
- Driver override conditions

#### 4.2 Performance Optimization
**Critical Metrics:**
- Control loop timing (100Hz requirement)
- Memory usage impact
- CPU utilization
- Message latency

#### 4.3 Documentation and Training
**Documentation Requirements:**
- User-facing feature descriptions
- Safety guidelines and limitations
- Parameter tuning guides
- Troubleshooting documentation

## Technical Implementation Details

### Parameter System Extension

```python
# New parameters in params_keys.h
static const char kMadsEnabled[] = "dp_mads_enabled";
static const char kMadsLatIndependent[] = "dp_mads_lat_independent";
static const char kMadsLongIndependent[] = "dp_mads_long_independent";
static const char kSmartOffsetEnabled[] = "dp_smart_offset_enabled";
static const char kCameraOffset[] = "dp_camera_offset";
static const char kPathOffset[] = "dp_path_offset";
static const char kGacEnabled[] = "dp_gac_enabled";
static const char kGacMinGap[] = "dp_gac_min_gap";
static const char kCurveSpeedEnabled[] = "dp_curve_speed_enabled";
static const char kDlpEnabled[] = "dp_dlp_enabled";
```

### Messaging System Extensions

```python
# Extensions to controlsState message
class ControlsState:
    madsLatActive = False
    madsLongActive = False
    smartOffsetActive = False
    currentGapSetting = 1.8
    curveSpeedActive = False
    dlpMode = "auto"  # "laneful", "laneless", "auto"
```

### Control Loop Integration

```python
# Main control loop modifications in controlsd.py
def update_control_state(self, CS, sm, controlsState):
    # Existing DragonPilot features
    self.update_acm_state(sm)
    self.update_aem_state(sm)
    
    # New features
    if self.dp_flags & DPFlags.MADS:
        self.mads_controller.update(CS, sm)
    
    if self.dp_flags & DPFlags.SMART_OFFSET:
        self.smart_offset_controller.update(sm)
    
    if self.dp_flags & DPFlags.GAC_TTC:
        self.gap_adjust_controller.update(sm)
    
    # Apply control modifications
    CC = self.apply_control_modifications(CC, sm)
```

## Risk Assessment and Mitigation

### High-Risk Areas
1. **Independent Control Safety**: MADS independent lat/long control
   - **Mitigation**: Comprehensive safety checks and driver monitoring
   - **Testing**: Extensive validation with emergency scenarios

2. **Feature Interactions**: Conflicts between existing and new features
   - **Mitigation**: Priority system and conflict detection
   - **Testing**: Systematic feature interaction matrix testing

3. **Performance Impact**: Real-time performance degradation
   - **Mitigation**: Profiling and optimization
   - **Testing**: Performance benchmarking

### Medium-Risk Areas
1. **Parameter Conflicts**: Existing vs new parameter interactions
2. **UI Complexity**: Too many options confusing users
3. **Maintenance Burden**: Increased code complexity

## Success Metrics

### Technical Metrics
- All features pass safety validation tests
- Performance impact < 5% CPU overhead
- Zero critical safety regressions
- Feature activation/deactivation < 100ms latency

### User Experience Metrics
- Intuitive parameter configuration
- Clear feature status indication
- Smooth feature transitions
- Comprehensive documentation

## Timeline and Milestones

| Week | Milestone | Deliverables |
|------|-----------|--------------|
| 1-2  | Foundation Setup | Component extraction, architecture extensions |
| 3-4  | Core Implementation 1 | MADS and Smart Offset |
| 5-6  | Core Implementation 2 | GAC, Curve Speed, DLP |
| 7    | Integration | Safety systems, conflict resolution |
| 8    | UI Integration | Settings panel, parameter validation |
| 9    | Testing | Comprehensive validation suite |
| 10   | Documentation | User guides, technical documentation |

## Conclusion

This integration plan leverages the mature implementations from SunnyPilot while respecting DragonPilot's architectural patterns and safety-first approach. The phased implementation strategy minimizes risk while ensuring each feature is properly integrated and validated.

The plan builds upon DragonPilot's existing strengths (ACM, AEM) and extends them with complementary features that enhance both safety and user experience. The modular approach allows for independent feature development and testing while maintaining system integrity.

---

*Document Version: 1.0*  
*Created: 2025-07-10*  
*Author: Claude Code Integration Analysis*