'''
====================================================================
NAGASPILOT V-TSC (VISION TURN SPEED CONTROL) - INTELLIGENT CURVE SPEED MANAGEMENT
====================================================================

MIT Non-Commercial License
Copyright (c) 2019, dragonpilot
Adapted from SunnyPilot implementation for NagasPilot

OVERVIEW:
V-TSC implements vision-based turn speed control that automatically adjusts vehicle
speed when approaching and navigating curves based on predicted lateral acceleration
from the camera vision system, improving safety and comfort during cornering.

CORE FUNCTIONALITY:
- Vision-based curvature prediction and analysis
- State machine for turn navigation phases (Entering, Turning, Leaving)
- Lateral acceleration based speed management
- Predictive speed adjustments for upcoming curves
- Integration with existing longitudinal control systems

OPERATIONAL LOGIC:
┌─────────────────────────────────────────────────────────────────┐
│ DISABLED: No predicted turn or feature disabled                │
│ ENTERING: Substantial turn predicted, adapting speed smoothly   │
│ TURNING: Active cornering with acceleration management          │
│ LEAVING: Road straightens, allowing speed recovery             │
└─────────────────────────────────────────────────────────────────┘

SAFETY FEATURES:
- Minimum speed enforcement (20 km/h operation threshold)
- Gas pedal override for immediate driver control
- Balanced lateral acceleration limits (2.5 m/s²)
- Multiple prediction sources with graceful fallbacks
- State transition validation and error handling

INTEGRATION POINTS:
- OpenPilot longitudinal control for cruise management
- ModelV2 for vision-based path prediction
- LateralPlan for driving path polynomial fitting
- CarState for vehicle dynamics and steering feedback

EFFICIENCY BENEFITS:
- Smooth speed transitions during curve approach
- Reduced harsh braking on unexpected turns
- Improved passenger comfort through predictive control
- Optimized for highway and winding road scenarios

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
- Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
- Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import numpy as np
import time
import math
from enum import Enum

from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
# Inlined constants (removed np_common dependency)
SPEED_THRESHOLD_CREEP = 2.0  # m/s - minimum speed for active control

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

# V-TSC Operation Parameters
NP_VTSC_MIN_V = 20 * CV.KPH_TO_MS  # Do not operate under 20 km/h

# Lateral Acceleration Thresholds (m/s²)
NP_VTSC_ENTERING_PRED_LAT_ACC_TH = 1.3     # Predicted lat acc to trigger entering state (PROVEN SunnyPilot value)
NP_VTSC_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Abort entering if predictions drop (PROVEN SunnyPilot value)
NP_VTSC_TURNING_LAT_ACC_TH = 1.6           # Current lat acc to trigger turning state (PROVEN SunnyPilot value)
NP_VTSC_LEAVING_LAT_ACC_TH = 1.3           # Current lat acc to trigger leaving state (PROVEN SunnyPilot value)
NP_VTSC_FINISH_LAT_ACC_TH = 1.1            # Current lat acc to finish turn cycle (PROVEN SunnyPilot value)

# Vision Evaluation Parameters
NP_VTSC_EVAL_STEP = 5.0        # meters, resolution of curvature evaluation
NP_VTSC_EVAL_START = 20.0      # meters, distance ahead to start evaluation  
NP_VTSC_EVAL_LENGTH = 150.0    # meters, distance ahead to stop evaluation (PROVEN SunnyPilot value)
NP_VTSC_EVAL_RANGE = np.arange(NP_VTSC_EVAL_START, NP_VTSC_EVAL_LENGTH, NP_VTSC_EVAL_STEP)

# Control Parameters
# Lateral acceleration limits by driving personality (follows OpenPilot style)
NP_VTSC_A_LAT_REG_COMFORT = 2.0    # Comfort driving (gentle)
NP_VTSC_A_LAT_REG_NORMAL = 2.5     # Normal driving (balanced) - default
NP_VTSC_A_LAT_REG_SPORT = 3.0      # Sport driving (aggressive)
NP_VTSC_NO_OVERSHOOT_TIME_HORIZON = 4.0  # Time horizon for velocity target (s)

# Smooth deceleration lookup for ENTERING state
NP_VTSC_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.0]    # min decel values (PROVEN SunnyPilot values)
NP_VTSC_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.0]     # lat acc breakpoints (PROVEN SunnyPilot values)

# Acceleration lookup for TURNING state  
NP_VTSC_TURNING_ACC_V = [0.5, 0.0, -0.4]  # acceleration values (PROVEN SunnyPilot values)
NP_VTSC_TURNING_ACC_BP = [1.5, 2.3, 3.0]  # lat acc breakpoints (PROVEN SunnyPilot values)

NP_VTSC_LEAVING_ACC = 0.5      # Comfortable acceleration for leaving turns (m/s²)
NP_VTSC_MIN_LANE_PROB = 0.6    # Minimum lane probability for lane-based prediction

# Trajectory size constant (from OpenPilot)
TRAJECTORY_SIZE = 33

# Debug flag
NP_VTSC_DEBUG = False

# ========================================================================
# ENUMERATIONS
# ========================================================================

class VTSCState(Enum):
    """V-TSC state machine enumeration following NagasPilot patterns"""
    DISABLED = 0
    ENTERING = 1  
    TURNING = 2
    LEAVING = 3

# ========================================================================
# UTILITY FUNCTIONS
# ========================================================================

def _debug_log(msg):
    """Debug logging function"""
    if NP_VTSC_DEBUG:
        cloudlog.debug(f"[NP_VTSC]: {msg}")

def eval_curvature(poly, x_vals):
    """
    Calculate curvature values from polynomial coefficients at given x positions
    Uses standard curvature formula: |y''| / (1 + y'^2)^1.5
    """
    def curvature(x):
        # Calculate curvature using polynomial derivatives
        a = abs(2 * poly[1] + 6 * poly[0] * x) / (1 + (3 * poly[0] * x ** 2 + 2 * poly[1] * x + poly[2]) ** 2) ** 1.5
        return a
    
    return np.vectorize(curvature)(x_vals)

def eval_lat_acc(v_ego, x_curv):
    """
    Calculate lateral acceleration from vehicle speed and curvature vector
    Lateral acceleration = v² * curvature
    """
    def lat_acc(curv):
        a = v_ego ** 2 * curv
        return a
    
    return np.vectorize(lat_acc)(x_curv)

def _description_for_state(state):
    """Convert state enum to human readable string"""
    if state == VTSCState.DISABLED:
        return 'DISABLED'
    elif state == VTSCState.ENTERING:
        return 'ENTERING'
    elif state == VTSCState.TURNING:
        return 'TURNING'
    elif state == VTSCState.LEAVING:
        return 'LEAVING'
    return f'UNKNOWN_{state}'

# ========================================================================
# V-TSC CONTROLLER IMPLEMENTATION
# ========================================================================

class VTSC:
    """Vision Turn Speed Controller - NagasPilot Implementation"""
    
    # ========================================================================
    # INITIALIZATION & CONFIGURATION
    # ========================================================================
    
    def __init__(self, CP):
        """Initialize V-TSC with default state following NagasPilot patterns"""
        # Parameter management (standardized interface)
        self.params = Params()
        self.enabled = self.params.get_bool("dp_lon_vtsc")
        
        # Vehicle configuration
        self._CP = CP
        self._last_params_update = 0.0
        
        # Control state variables
        self._op_enabled = False
        self._gas_pressed = False
        self._v_cruise_setpoint = 0.0
        self._v_ego = 0.0
        self._a_ego = 0.0
        self._a_target = 0.0
        self._v_overshoot = 0.0
        self._state = VTSCState.DISABLED
        
        # Vision analysis variables
        self._current_lat_acc = 0.0
        self._max_v_for_current_curvature = 0.0
        self._max_pred_lat_acc = 0.0
        self._v_overshoot_distance = 200.0
        self._lat_acc_overshoot_ahead = False
        
        self._reset_state()
        
        if self.enabled:
            cloudlog.info("NP V-TSC Controller initialized and enabled")

    # ========================================================================
    # STANDARDIZED INTERFACE (following NagasPilot patterns)
    # ========================================================================
    
    def is_enabled(self):
        """Check if V-TSC controller is enabled via parameter"""
        return self.enabled and self.params.get_bool("dp_lon_vtsc")
        
    @property
    def state(self):
        """Get current V-TSC state"""
        return self._state
    
    @state.setter
    def state(self, value):
        """Set V-TSC state with logging and reset handling"""
        if value != self._state:
            _debug_log(f'State transition: {_description_for_state(self._state)} -> {_description_for_state(value)}')
            if value == VTSCState.DISABLED:
                self._reset_state()
        self._state = value

    @property
    def a_target(self):
        """Get target acceleration (returns ego acceleration when not active)"""
        return self._a_target if self.is_active else self._a_ego

    @property 
    def v_turn(self):
        """Get target velocity for turn (returns cruise setpoint when not active)"""
        if not self.is_active:
            return self._v_cruise_setpoint
            
        # Return overshoot velocity if overshoot predicted, otherwise calculate from acceleration
        return self._v_overshoot if self._lat_acc_overshoot_ahead \
            else self._v_ego + self._a_target * NP_VTSC_NO_OVERSHOOT_TIME_HORIZON

    @property
    def current_lat_acc(self):
        """Get current lateral acceleration"""
        return self._current_lat_acc

    @property 
    def max_pred_lat_acc(self):
        """Get maximum predicted lateral acceleration"""
        return self._max_pred_lat_acc

    @property
    def is_active(self):
        """Check if V-TSC is currently active (not disabled)"""
        return self._state != VTSCState.DISABLED

    # ========================================================================
    # STATE MANAGEMENT
    # ========================================================================

    def _reset_state(self):
        """Reset internal state variables to defaults"""
        self._current_lat_acc = 0.0
        self._max_v_for_current_curvature = 0.0
        self._max_pred_lat_acc = 0.0
        self._v_overshoot_distance = 200.0
        self._lat_acc_overshoot_ahead = False

    def _update_params(self):
        """Update parameters from storage with rate limiting"""
        tm = time.time()
        if tm > self._last_params_update + 5.0:
            self.enabled = self.params.get_bool("dp_lon_vtsc")
            self._last_params_update = tm
            
    def _get_lateral_comfort_limit(self) -> float:
        """Get lateral acceleration limit based on driving personality (follows OpenPilot style)"""
        try:
            # Get driving personality from OpenPilot parameter (0=Comfort, 1=Normal, 2=Sport)
            personality = self.params.get("LongitudinalPersonality", return_default=True)
            if personality is not None:
                personality_int = int(personality)
                if personality_int == 0:    # Comfort
                    return NP_VTSC_A_LAT_REG_COMFORT
                elif personality_int == 2:  # Sport  
                    return NP_VTSC_A_LAT_REG_SPORT
                else:                       # Normal (default for 1 or invalid)
                    return NP_VTSC_A_LAT_REG_NORMAL
        except (ValueError, TypeError, AttributeError):
            pass
        return NP_VTSC_A_LAT_REG_NORMAL  # Default to Normal

    def _update_calculations(self, sm):
        """Update vision analysis and curvature calculations"""
        if not self.is_enabled():
            return
            
        try:
            # Get path polynomial from model data
            path_poly = self._extract_path_polynomial(sm)
            
            # Calculate current curvature from steering angle
            current_curvature = abs(
                sm['carState'].steeringAngleDeg * CV.DEG_TO_RAD / (self._CP.steerRatio * self._CP.wheelbase)
            )
            self._current_lat_acc = current_curvature * self._v_ego ** 2
            
            # Get user-configurable lateral acceleration limit
            lat_acc_limit = self._get_lateral_comfort_limit()
            
            # Calculate maximum safe velocity for current curvature with bounds checking
            if current_curvature > 1e-6:  # Avoid division by tiny numbers
                safe_v = math.sqrt(lat_acc_limit / current_curvature)
                # Bounds check: reasonable speed range 5-50 m/s (18-180 km/h)
                self._max_v_for_current_curvature = max(5.0, min(safe_v, 50.0))
            else:
                self._max_v_for_current_curvature = V_CRUISE_MAX * CV.KPH_TO_MS
            
            # Predict maximum lateral acceleration ahead
            pred_curvatures = eval_curvature(path_poly, NP_VTSC_EVAL_RANGE)
            max_pred_curvature = np.amax(pred_curvatures)
            self._max_pred_lat_acc = self._v_ego ** 2 * max_pred_curvature
            
            # Check for overshoot conditions
            max_curvature_for_vego = lat_acc_limit / max(self._v_ego, 0.1) ** 2
            lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
            self._lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0
            
            if self._lat_acc_overshoot_ahead:
                # Bounds checking for overshoot speed calculation
                if max_pred_curvature > 1e-6:  # Avoid division by tiny numbers
                    safe_overshoot_v = math.sqrt(lat_acc_limit / max_pred_curvature)
                    # Bounds check: reasonable speed range
                    safe_overshoot_v = max(5.0, min(safe_overshoot_v, 50.0))
                    self._v_overshoot = min(safe_overshoot_v, self._v_cruise_setpoint)
                else:
                    self._v_overshoot = self._v_cruise_setpoint
                self._v_overshoot_distance = max(
                    float(lat_acc_overshoot_idxs[0] * NP_VTSC_EVAL_STEP + NP_VTSC_EVAL_START), 
                    NP_VTSC_EVAL_STEP
                )
                _debug_log(f'High LatAcc ahead. Dist: {self._v_overshoot_distance:.2f}m, v: {self._v_overshoot * CV.MS_TO_KPH:.2f}kph')
                
        except Exception as e:
            cloudlog.error(f"NP V-TSC calculation error: {e}")
            self._reset_state()

    def _extract_path_polynomial(self, sm):
        """Extract path polynomial from available data sources with priority order"""
        path_poly = None
        model_data = sm['modelV2'] if sm.valid.get('modelV2', False) else None
        lat_planner_data = sm['lateralPlan'] if sm.valid.get('lateralPlan', False) else None
        
        # Priority 1: Lane lines (more stable than driving path)
        if model_data is not None and len(model_data.laneLines) == 4 and len(model_data.laneLines[0].t) == TRAJECTORY_SIZE:
            path_poly = self._extract_lane_based_polynomial(model_data)
            
        # Priority 2: Lateral planner driving path
        if path_poly is None and lat_planner_data is not None and len(lat_planner_data.psis) == CONTROL_N \
           and lat_planner_data.dPathPoints[0] > 0:
            yData = list(lat_planner_data.dPathPoints)
            path_poly = np.polyfit(lat_planner_data.psis, yData[0:CONTROL_N], 3)
            
        # Priority 3: Straight line fallback
        if path_poly is None:
            path_poly = np.array([0., 0., 0., 0.])
            
        return path_poly

    def _extract_lane_based_polynomial(self, model_data):
        """Extract polynomial from lane lines with quality filtering"""
        try:
            ll_x = model_data.laneLines[1].x  # left and right x coordinates are the same
            lll_y = np.array(model_data.laneLines[1].y)  # left lane line y
            rll_y = np.array(model_data.laneLines[2].y)  # right lane line y
            l_prob = model_data.laneLineProbs[1]
            r_prob = model_data.laneLineProbs[2]
            lll_std = model_data.laneLineStds[1]
            rll_std = model_data.laneLineStds[2]
            
            # Apply quality filters
            width_pts = rll_y - lll_y
            
            # Reduce reliance on lanes that are too far apart
            prob_mods = []
            for t_check in [0.0, 1.5, 3.0]:
                width_at_t = np.interp(t_check * (self._v_ego + 7), ll_x, width_pts)
                prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
            mod = min(prob_mods)
            l_prob *= mod
            r_prob *= mod
            
            # Reduce reliance on uncertain lane lines
            l_std_mod = np.interp(lll_std, [0.15, 0.3], [1.0, 0.0])
            r_std_mod = np.interp(rll_std, [0.15, 0.3], [1.0, 0.0])
            l_prob *= l_std_mod
            r_prob *= r_std_mod
            
            # Use lane-based path only if both lanes have sufficient probability
            if l_prob > NP_VTSC_MIN_LANE_PROB and r_prob > NP_VTSC_MIN_LANE_PROB:
                c_y = width_pts / 2 + lll_y  # Center lane calculation
                return np.polyfit(ll_x, c_y, 3)
                
        except Exception as e:
            _debug_log(f"Lane polynomial extraction error: {e}")
            
        return None

    def _state_transition(self):
        """Handle state machine transitions with safety checks"""
        if not self.is_enabled():
            return
            
        # Global disable conditions
        if not self._op_enabled or not self.enabled or self._gas_pressed:
            self.state = VTSCState.DISABLED
            return
            
        # State-specific transition logic
        if self.state == VTSCState.DISABLED:
            self._handle_disabled_state()
        elif self.state == VTSCState.ENTERING:
            self._handle_entering_state()  
        elif self.state == VTSCState.TURNING:
            self._handle_turning_state()
        elif self.state == VTSCState.LEAVING:
            self._handle_leaving_state()

    def _handle_disabled_state(self):
        """Handle transitions from DISABLED state (SunnyPilot logic)"""
        # Don't enter if speed too low
        if self._v_ego <= NP_VTSC_MIN_V:
            return
        # Enter if predictions meet threshold  
        elif self._max_pred_lat_acc >= NP_VTSC_ENTERING_PRED_LAT_ACC_TH:
            self.state = VTSCState.ENTERING

    def _handle_entering_state(self):
        """Handle transitions from ENTERING state"""
        # Transition to turning if experiencing sufficient lateral acceleration
        if self._current_lat_acc >= NP_VTSC_TURNING_LAT_ACC_TH:
            self.state = VTSCState.TURNING
        # Abort if predicted lateral acceleration drops significantly  
        elif self._max_pred_lat_acc < NP_VTSC_ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VTSCState.DISABLED

    def _handle_turning_state(self):
        """Handle transitions from TURNING state"""
        # Transition to leaving when lateral acceleration decreases
        if self._current_lat_acc <= NP_VTSC_LEAVING_LAT_ACC_TH:
            self.state = VTSCState.LEAVING

    def _handle_leaving_state(self):
        """Handle transitions from LEAVING state"""
        # Return to turning if lateral acceleration increases again
        if self._current_lat_acc >= NP_VTSC_TURNING_LAT_ACC_TH:
            self.state = VTSCState.TURNING
        # Finish turn cycle when lateral acceleration drops sufficiently
        elif self._current_lat_acc < NP_VTSC_FINISH_LAT_ACC_TH:
            self.state = VTSCState.DISABLED

    def _update_solution(self):
        """Calculate target acceleration based on current state"""
        if not self.is_enabled():
            self._a_target = self._a_ego
            return
            
        try:
            if self.state == VTSCState.DISABLED:
                a_target = self._a_ego
                
            elif self.state == VTSCState.ENTERING:
                # Smooth deceleration based on predicted lateral acceleration
                a_target = np.interp(self._max_pred_lat_acc, 
                                   NP_VTSC_ENTERING_SMOOTH_DECEL_BP, 
                                   NP_VTSC_ENTERING_SMOOTH_DECEL_V)
                                   
                # Handle overshoot scenario with calculated deceleration
                if self._lat_acc_overshoot_ahead:
                    a_target = min(
                        (self._v_overshoot ** 2 - self._v_ego ** 2) / (2 * self._v_overshoot_distance),
                        a_target
                    )
                _debug_log(f'Entering: Overshoot={self._lat_acc_overshoot_ahead}, a_target={a_target:.2f}')
                
            elif self.state == VTSCState.TURNING:
                # Comfortable acceleration based on current lateral acceleration
                a_target = np.interp(self._current_lat_acc,
                                   NP_VTSC_TURNING_ACC_BP,
                                   NP_VTSC_TURNING_ACC_V)
                                   
            elif self.state == VTSCState.LEAVING:
                # Fixed comfortable acceleration for speed recovery
                a_target = NP_VTSC_LEAVING_ACC
                
            else:
                a_target = self._a_ego
                
            self._a_target = a_target
            
        except Exception as e:
            cloudlog.error(f"NP V-TSC solution update error: {e}")
            self._a_target = self._a_ego

    # ========================================================================
    # MAIN UPDATE INTERFACE
    # ========================================================================

    def update(self, sm, enabled, v_ego, a_ego, v_cruise_setpoint):
        """Main update function called from longitudinal planner"""
        if not self.is_enabled():
            self.state = VTSCState.DISABLED
            return
            
        try:
            # Update vehicle state
            self._op_enabled = enabled
            self._gas_pressed = sm['carState'].gasPressed
            self._v_ego = v_ego
            self._a_ego = a_ego
            self._v_cruise_setpoint = v_cruise_setpoint
            
            # Update parameters and perform calculations
            self._update_params()
            self._update_calculations(sm)
            self._state_transition()
            self._update_solution()
            
        except Exception as e:
            cloudlog.error(f"NP V-TSC update error: {e}")
            self.state = VTSCState.DISABLED

    # ========================================================================
    # STANDARDIZED INTERFACE
    # ========================================================================

    def get_debug_info(self):
        """Standardized debug information following NagasPilot patterns"""
        return {
            "enabled": self.is_enabled(),
            "active": self.is_active,
            "state": _description_for_state(self.state),
            "current_lat_acc": self._current_lat_acc,
            "max_pred_lat_acc": self._max_pred_lat_acc,
            "v_ego": self._v_ego,
            "v_turn": self.v_turn,
            "a_target": self._a_target,
            "overshoot_ahead": self._lat_acc_overshoot_ahead
        }
