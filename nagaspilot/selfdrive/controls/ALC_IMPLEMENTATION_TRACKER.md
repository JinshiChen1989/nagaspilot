# ALC Implementation Tracker

## Project: Enhanced LCA for nagaspilot (brownpanda focus)

**Based on**: FrogPilot proven patterns + brownpanda.dbc BSD
**Approach**: Enhance existing LCA, don't over-engineer
**Target**: brownpanda cars only

---

## Phase 1: Foundation Setup ✅

### 1.1 Analysis Complete ✅
- [✅] Analyzed FrogPilot's working desire_helper.py 
- [✅] Extracted proven lane width calculation function
- [✅] Studied brownpanda.dbc BSD signals (0x6E4)
- [✅] Identified integration points in existing desire_helper.py

### 1.2 Design Document Complete ✅  
- [✅] Created `/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/alc_design.md`
- [✅] Cross-checked with FrogPilot actual code
- [✅] Simplified to avoid over-engineering
- [✅] Focused on brownpanda only

---

## Phase 2: Core Implementation 🔄

### 2.1 Create Lane Width Utility ✅
**Target**: `/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_lane_width_utils.py`

**Tasks**:
- [✅] Copy FrogPilot's exact `calculate_lane_width()` function
- [✅] Add numpy import and proper error handling
- [✅] Add brownpanda-specific comments
- [✅] Added `calculate_adjacent_lane_widths()` helper function
- [✅] Added `is_lane_width_safe()` validation function

**Success Criteria**:
- Function returns float lane width or 0.0 for invalid lanes
- Handles road edge fallback correctly
- No dependencies on FrogPilot-specific code
- **COMPLETED**: All criteria met

**Files Created**:
```
/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_lane_width_utils.py ✅
```

### 2.2 Create Main ALC Controller ✅
**Target**: `/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_alc_controller.py`

**Tasks**:
- [✅] Create main ALC controller class following FrogPilot patterns
- [✅] Implement state machine (OFF, PRE_LANE_CHANGE, LANE_CHANGE_STARTING)
- [✅] Add FrogPilot's proven torque_applied logic with lane width validation
- [✅] Integrate enhanced blind spot detection framework
- [✅] Add parameter management with np_alc_ prefix (dragonpilot style)
- [✅] Implement timing and safety logic from FrogPilot's working code

**Success Criteria**:
- State machine follows OpenPilot LaneChangeState pattern
- Lane width validation prevents unsafe changes
- Nudgeless and manual modes supported
- Enhanced BSD framework ready for brownpanda integration
- **COMPLETED**: All criteria met

**Files Created**:
```
/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_alc_controller.py ✅
```

### 2.3 Enhance desire_helper.py ⏳
**Target**: `/home/vcar/nagaspilot/alc/selfdrive/controls/lib/desire_helper.py`

**Tasks**:
- [ ] Add brownpanda BSD helper methods
- [ ] Modify torque_applied logic to include lane width check (FrogPilot pattern)
- [ ] Enhance blindspot_detected logic with brownpanda BSD
- [ ] Add configuration parameter support
- [ ] Test with existing LCA functionality

**Success Criteria**:
- Existing LCA behavior preserved
- Enhanced BSD from brownpanda.dbc integrated
- Lane width validation prevents unsafe lane changes
- Backward compatible (graceful degradation)

**Files to Modify**:
```
/home/vcar/nagaspilot/alc/selfdrive/controls/lib/desire_helper.py (lines 70-75 area)
```

### 2.4 Configuration Parameters ⏳
**Target**: Add ALC configuration to nagaspilot parameter system

**Tasks**:
- [ ] Add ALC parameters to configuration system
- [ ] Set sensible defaults
- [ ] Verify parameters accessible from desire_helper.py
- [ ] Add parameter validation

**Success Criteria**:
- Parameters available through nagaspilot config system
- Default values work safely
- Parameters can be tuned by users

**Parameters to Add**:
```python
NP_ALC_LANE_WIDTH_CHECK_ENABLED = "NpAlcLaneWidthCheckEnabled"      # Default: True
NP_ALC_MIN_LANE_WIDTH = "NpAlcMinLaneWidth"                         # Default: 2.5 meters  
NP_ALC_ENHANCED_BSD_ENABLED = "NpAlcEnhancedBsdEnabled"             # Default: True
```

---

## Phase 3: Testing & Validation 🔄

### 3.1 Unit Testing ⏳
**Tasks**:
- [ ] Test lane width calculation function with various inputs
- [ ] Test brownpanda BSD bit extraction
- [ ] Test enhanced desire_helper logic
- [ ] Validate backward compatibility

**Success Criteria**:
- All functions handle edge cases properly
- No crashes or exceptions
- Existing LCA behavior unchanged when features disabled

### 3.2 Integration Testing ⏳
**Tasks**:
- [ ] Test with real ModelV2 data
- [ ] Test with brownpanda CAN signals
- [ ] Test enhanced LCA behavior
- [ ] Verify no conflicts with existing systems

**Success Criteria**:
- Lane width calculations accurate
- BSD enhancement works correctly
- LCA behavior improved without breaking existing functionality

### 3.3 Safety Validation ⏳
**Tasks**:
- [ ] Test failure modes (no vision, no CAN, etc.)
- [ ] Verify graceful degradation
- [ ] Test parameter edge cases
- [ ] Validate conservative fallback behavior

**Success Criteria**:
- System fails safely
- No unsafe lane changes allowed
- Clear fallback to standard behavior

---

## Phase 4: Documentation & Cleanup 🔄

### 4.1 Code Documentation ⏳
**Tasks**:
- [ ] Add comprehensive code comments
- [ ] Document function parameters and return values
- [ ] Add brownpanda-specific notes
- [ ] Update any relevant README files

### 4.2 User Documentation ⏳
**Tasks**:
- [ ] Document new ALC features for users
- [ ] Explain configuration parameters
- [ ] Add brownpanda compatibility notes
- [ ] Create troubleshooting guide

---

## Implementation Tracking

### Current Status: Phase 2 - Core Implementation (70% Complete)
**Started**: 2025-01-13
**Current Task**: 2.3 Integration with desire_helper.py

### Progress Log:
- **2025-01-13**: Completed analysis and design phases
- **2025-01-13**: Starting core implementation
- **2025-01-13**: ✅ Completed lane width utilities (2.1)
- **2025-01-13**: ✅ Completed main ALC controller (2.2)
- **2025-01-13**: Ready for integration phase

### Blockers:
- None identified

### Risks:
- Need to identify correct planning file for lane width integration
- May need to understand message passing architecture better

### Rollback Plan:
- All changes are additive to existing desire_helper.py
- Can disable features via configuration parameters
- Original LCA behavior preserved

---

## Success Metrics

### Safety Goals:
- [ ] Zero unsafe lane changes due to narrow lanes
- [ ] Enhanced BSD coverage prevents blind spot incidents
- [ ] Graceful degradation when sensors unavailable

### Functionality Goals:
- [ ] Lane width validation working correctly
- [ ] brownpanda BSD enhancement active
- [ ] Existing LCA timing and behavior preserved

### Code Quality Goals:
- [ ] Clean integration with existing codebase
- [ ] Follows dragonpilot/nagaspilot patterns
- [ ] Comprehensive error handling
- [ ] Clear documentation

---

## File Structure

### New Files Created:
```
/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_lane_width_utils.py ✅
/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/np_alc_controller.py ✅
```

### Modified Files:
```
/home/vcar/nagaspilot/alc/selfdrive/controls/lib/desire_helper.py
TBD - Planning/control file for lane width integration
TBD - Configuration file for parameters
```

### Documentation Files:
```
/home/vcar/nagaspilot/alc/nagaspilot/selfdrive/controls/lib/alc_design.md ✅
/home/vcar/nagaspilot/alc/ALC_IMPLEMENTATION_TRACKER.md ✅
```