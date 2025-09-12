#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT ADVANCED LANE CHANGE (ALC) CONTROLLER
====================================================================

OVERVIEW:
Advanced Lane Change system that replaces LCA completely. Reuses existing
dragonpilot UI structure (dp_lat_alc_* parameters) with FrogPilot's proven
lane change algorithms. Single-file implementation, no over-engineering.

CORE FUNCTIONALITY:
- FrogPilot's proven lane width calculation (integrated directly)
- Enhanced blind spot detection for brownpanda
- Reuses existing dragonpilot UI (just changed LCA -> ALC)
- Compatible with existing desire_helper pattern

PARAMETER MAPPING:
- dp_lat_alc_speed (was dp_lat_lca_speed) - Speed threshold in km/h
- dp_lat_alc_auto_sec (was dp_lat_lca_auto_sec) - Auto delay in seconds
- Additional ALC enhancements via FrogPilot's lane width validation

ALGORITHM BASIS:
Based on FrogPilot's working code:
- ~/FrogPilot/frogpilot/common/frogpilot_utilities.py:84-100 (lane width)
- ~/FrogPilot/selfdrive/controls/lib/desire_helper.py:84-86 (logic)
"""

import time
import numpy as np
from enum import IntEnum
from typing import Dict, Any, Optional
# Use constants directly instead of importing conversions
class CV:
    KPH_TO_MS = 1.0 / 3.6
    MS_TO_KPH = 3.6
# Simplified - no params needed for now

# Simple logging placeholder
class SimpleLogger:
    def info(self, msg): print(f"[ALC] {msg}")
    def warning(self, msg): print(f"[ALC WARNING] {msg}")
    def debug(self, msg): pass  # Silent for now
    def error(self, msg): print(f"[ALC ERROR] {msg}")

np_logger = SimpleLogger()

# ========================================================================
# ALC STATE DEFINITIONS (FOLLOWING OPENPILOT PATTERN)
# ========================================================================

class AlcState(IntEnum):
    """ALC states following OpenPilot LaneChangeState pattern"""
    OFF = 0
    PRE_LANE_CHANGE = 1  
    LANE_CHANGE_STARTING = 2
    LANE_CHANGE_FINISHING = 3

class AlcDirection(IntEnum):
    """Lane change directions"""
    NONE = 0
    LEFT = 1
    RIGHT = 2

# ========================================================================
# FROGPILOT'S PROVEN LANE WIDTH CALCULATION (INTEGRATED)
# ========================================================================

def calculate_lane_width(lane, current_lane, road_edge=None):
    """
    FrogPilot's exact lane width calculation algorithm
    Copied directly from ~/FrogPilot/frogpilot/common/frogpilot_utilities.py:84-100
    """
    try:
        current_x = np.asarray(current_lane.x)
        current_y = np.asarray(current_lane.y)
        
        lane_y_interp = np.interp(current_x, np.asarray(lane.x), np.asarray(lane.y))
        distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
        
        if road_edge is None:
            return float(distance_to_lane)
        
        road_edge_y_interp = np.interp(current_x, np.asarray(road_edge.x), np.asarray(road_edge.y))
        distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))
        
        # FrogPilot safety: return 0.0 if road edge closer than lane line
        if distance_to_road_edge < distance_to_lane:
            return 0.0
        
        return float(distance_to_lane)
        
    except Exception as e:
        np_logger.warning(f"Lane width calculation error: {e}")
        return 0.0

# ========================================================================
# MAIN ALC CONTROLLER (REUSING DRAGONPILOT UI STRUCTURE)
# ========================================================================

class NpAlcController:
    """
    NagasPilot Advanced Lane Change Controller
    
    Replaces LCA completely while reusing existing dragonpilot UI structure:
    - Uses dp_lat_alc_speed and dp_lat_alc_auto_sec parameters
    - FrogPilot's proven algorithms integrated directly
    - Enhanced with lane width validation
    - Compatible with existing desire_helper pattern
    """
    
    def __init__(self, dp_lat_alc_speed=60, dp_lat_alc_auto_sec=2.0):
        """Initialize ALC controller with dragonpilot parameters"""
        
        # Simplified initialization
        
        # Core dragonpilot ALC parameters (reusing existing UI structure)
        self.alc_speed_kmh = float(dp_lat_alc_speed)  # Speed threshold in km/h
        self.alc_speed_ms = self.alc_speed_kmh * CV.KPH_TO_MS  # Convert to m/s
        self.alc_auto_sec = float(dp_lat_alc_auto_sec)  # Auto delay in seconds
        
        # ALC enhancement parameters (simplified)
        self.min_lane_width = 2.5  # Minimum safe lane width in meters
        self.lane_width_enabled = True  # Always enabled for safety
        
        # State tracking (following original desire_helper pattern)
        self.state = AlcState.OFF
        self.direction = AlcDirection.NONE
        self.auto_timer_start = 0.0
        self.change_completed = False
        
        # Lane width state (FrogPilot enhancement integrated)
        self.lane_width_left = 0.0
        self.lane_width_right = 0.0
        self.lane_available_left = True  # Default true for backward compatibility
        self.lane_available_right = True
        
        # Enhanced BSD state
        self.bsd_left = False
        self.bsd_right = False
        
        # Statistics
        self.changes_completed = 0
        
        # Log initialization
        enabled_str = "enabled" if self.alc_speed_kmh > 0 else "disabled"
        np_logger.info(f"ALC Controller initialized - {enabled_str}, speed: {self.alc_speed_kmh}km/h, auto: {self.alc_auto_sec}s")
    
    
    def _update_lane_widths(self, model_data: Dict[str, Any]) -> None:
        """Update lane widths using FrogPilot's algorithm (ALC enhancement)"""
        if not self.lane_width_enabled:
            # Backward compatibility - assume lanes available when disabled
            self.lane_available_left = True
            self.lane_available_right = True
            return
            
        try:
            model_v2 = model_data.get('modelV2')
            if not model_v2 or not hasattr(model_v2, 'laneLines'):
                return
            
            lane_lines = model_v2.laneLines
            road_edges = getattr(model_v2, 'roadEdges', None)
            
            if not lane_lines or len(lane_lines) < 4:
                return
            
            # Calculate using FrogPilot's method
            if road_edges and len(road_edges) > 0:
                self.lane_width_left = calculate_lane_width(
                    lane_lines[0], lane_lines[1], road_edges[0]
                )
            else:
                self.lane_width_left = calculate_lane_width(
                    lane_lines[0], lane_lines[1], None
                )
            
            if road_edges and len(road_edges) > 1:
                self.lane_width_right = calculate_lane_width(
                    lane_lines[3], lane_lines[2], road_edges[1]
                )
            else:
                self.lane_width_right = calculate_lane_width(
                    lane_lines[3], lane_lines[2], None
                )
            
            # Validate lane availability (ALC enhancement)
            self.lane_available_left = (self.lane_width_left >= self.min_lane_width)
            self.lane_available_right = (self.lane_width_right >= self.min_lane_width)
            
        except Exception as e:
            np_logger.warning(f"Lane width update error: {e}")
            # Fail safe - assume lanes available
            self.lane_available_left = True
            self.lane_available_right = True
    
    def _check_enhanced_blindspot(self, carstate) -> bool:
        """Enhanced blind spot detection using all 4 BSD sensors"""
        # Use all 4 BSD sensors (front and rear on each side)
        # carstate already combines _0x6E2_carState and _0x6E4_adasState sources
        self.bsd_left = carstate.leftBlindspot  # FL + RL sensors
        self.bsd_right = carstate.rightBlindspot  # FR + RR sensors
        
        # Return blindspot status for current direction
        if self.direction == AlcDirection.LEFT:
            return self.bsd_left
        elif self.direction == AlcDirection.RIGHT:
            return self.bsd_right
        return False
    
    def update(self, carstate, model_data: Dict[str, Any], dt: float, c_time: float, left_edge_detected: bool = False, right_edge_detected: bool = False) -> None:
        """
        Main ALC update function - enhances carstate BSD directly for transparent integration
        
        Args:
            carstate: Current car state (will be modified to include ALC enhancements)
            model_data: Model data for lane width calculation
            dt: Delta time (for compatibility)
            c_time: Current time (for auto timer)
            left_edge_detected: Road edge detection from DragonPilot
            right_edge_detected: Road edge detection from DragonPilot
        """
        # Update lane widths (ALC enhancement)
        self._update_lane_widths(model_data)
        
        # Basic conditions following original desire_helper pattern
        v_ego = carstate.vEgo
        one_blinker = carstate.leftBlinker != carstate.rightBlinker
        below_alc_speed = True if self.alc_speed_ms == 0.0 else v_ego < self.alc_speed_ms
        
        # ALC is disabled if speed is 0 (matching original behavior)
        alc_enabled = (self.alc_speed_kmh > 0)
        
        # Determine direction
        if carstate.leftBlinker:
            self.direction = AlcDirection.LEFT
            lane_available = self.lane_available_left
            lane_width = self.lane_width_left
        elif carstate.rightBlinker:
            self.direction = AlcDirection.RIGHT
            lane_available = self.lane_available_right
            lane_width = self.lane_width_right
        else:
            self.direction = AlcDirection.NONE
            lane_available = True  # Default for backward compatibility
            lane_width = 0.0
        
        # Enhanced safety checks: blindspot + road edge
        blindspot_detected = self._check_enhanced_blindspot(carstate)
        
        # Road edge detection (from existing DragonPilot implementation)
        road_edge_detected = False
        if self.direction == AlcDirection.LEFT and left_edge_detected:
            road_edge_detected = True
        elif self.direction == AlcDirection.RIGHT and right_edge_detected:
            road_edge_detected = True
        
        # Torque applied logic (following original desire_helper pattern)
        torque_applied = False
        
        if carstate.steeringPressed:
            # Manual steering input (traditional mode)
            if ((self.direction == AlcDirection.LEFT and carstate.steeringTorque > 0) or
                (self.direction == AlcDirection.RIGHT and carstate.steeringTorque < 0)):
                torque_applied = True
        else:
            # Auto mode (following original auto_sec logic)
            if self.alc_auto_sec > 0.0 and lane_available:
                if blindspot_detected or road_edge_detected:
                    # Reset timer if any safety issue detected
                    self.auto_timer_start = c_time
                else:
                    # Check if enough time has passed
                    if (c_time - self.auto_timer_start) >= self.alc_auto_sec:
                        torque_applied = True
        
        # State machine (following original desire_helper pattern)
        if not alc_enabled or below_alc_speed:
            self.state = AlcState.OFF
            self.direction = AlcDirection.NONE
            
        elif not one_blinker:
            self.state = AlcState.OFF
            self.direction = AlcDirection.NONE
            self.change_completed = False  # Reset on blinker off
            
        elif torque_applied and lane_available and not blindspot_detected and not road_edge_detected:
            if self.state != AlcState.LANE_CHANGE_STARTING:
                self.state = AlcState.LANE_CHANGE_STARTING
                self.changes_completed += 1
                direction_str = "left" if self.direction == AlcDirection.LEFT else "right"
                np_logger.info(f"ALC: Starting lane change {direction_str} - width: {lane_width:.2f}m")
                
        elif one_blinker and self.state == AlcState.OFF:
            self.state = AlcState.PRE_LANE_CHANGE
            # Start auto timer on blinker activation
            if self.alc_auto_sec > 0.0:
                self.auto_timer_start = c_time
        
        # Enhance carstate BSD with ALC lane width and road edge detection
        # This makes ALC enhancements transparent to DesireHelper
        original_left_bsd = carstate.leftBlindspot
        original_right_bsd = carstate.rightBlindspot
        
        # Add lane width gating to BSD detection
        if not self.lane_available_left:
            carstate.leftBlindspot = True  # Block left lane change if lane too narrow
        if not self.lane_available_right:
            carstate.rightBlindspot = True  # Block right lane change if lane too narrow
            
        # Add road edge detection to BSD
        if left_edge_detected:
            carstate.leftBlindspot = True  # Block left lane change if road edge detected
        if right_edge_detected:
            carstate.rightBlindspot = True  # Block right lane change if road edge detected
        
        # Log ALC enhancements when they override original BSD
        if carstate.leftBlindspot != original_left_bsd or carstate.rightBlindspot != original_right_bsd:
            reasons = []
            if not self.lane_available_left: reasons.append(f"left_lane_width:{self.lane_width_left:.1f}m")
            if not self.lane_available_right: reasons.append(f"right_lane_width:{self.lane_width_right:.1f}m")
            if left_edge_detected: reasons.append("left_road_edge")
            if right_edge_detected: reasons.append("right_road_edge")
            if reasons:
                np_logger.debug(f"ALC enhanced BSD: {','.join(reasons)}")
    
    def update_params(self, dp_lat_alc_speed, dp_lat_alc_auto_sec):
        """Update dragonpilot parameters"""
        old_speed = self.alc_speed_kmh
        self.alc_speed_kmh = float(dp_lat_alc_speed) 
        self.alc_speed_ms = self.alc_speed_kmh * CV.KPH_TO_MS
        self.alc_auto_sec = float(dp_lat_alc_auto_sec)
        
        # Update ALC enhancement parameters (simplified)
        self.min_lane_width = 2.5  # Fixed for safety
        self.lane_width_enabled = True  # Always enabled
        
        if old_speed != self.alc_speed_kmh:
            enabled_str = "enabled" if self.alc_speed_kmh > 0 else "disabled"
            np_logger.info(f"ALC parameters updated - {enabled_str}, speed: {self.alc_speed_kmh}km/h, auto: {self.alc_auto_sec}s")
    
    def get_debug_info(self) -> Dict[str, Any]:
        """Get debug information"""
        return {
            'alc_enabled': self.alc_speed_kmh > 0,
            'alc_speed_kmh': self.alc_speed_kmh,
            'alc_auto_sec': self.alc_auto_sec,
            'state': self.state.name if hasattr(self.state, 'name') else str(self.state),
            'direction': self.direction.name if hasattr(self.direction, 'name') else str(self.direction),
            'lane_width_left': round(self.lane_width_left, 2),
            'lane_width_right': round(self.lane_width_right, 2),
            'lane_available_left': self.lane_available_left,
            'lane_available_right': self.lane_available_right,
            'lane_width_enabled': self.lane_width_enabled,
            'min_lane_width': self.min_lane_width,
            'changes_completed': self.changes_completed,
            'bsd_left': self.bsd_left,
            'bsd_right': self.bsd_right,
            'algorithm_source': 'frogpilot_dragonpilot_hybrid'
        }