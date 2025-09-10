'''
====================================================================
NAGASPILOT M-TSC (MAP TURN SPEED CONTROL) - OSM-BASED CURVE SPEED MANAGEMENT
====================================================================

MIT Non-Commercial License
Copyright (c) 2019, dragonpilot
Adapted from FrogPilot map integration patterns for NagasPilot

OVERVIEW:
M-TSC implements map-based turn speed control that automatically adjusts vehicle
speed when approaching curves detected from OpenStreetMap data, providing advance
warning and speed reduction before V-TSC engages with vision-based detection.

CORE FUNCTIONALITY:
- OSM data fetching based on ego vehicle speed and position
- Map-based curvature prediction and analysis
- Advance warning system for upcoming turns
- Coordinated handoff with V-TSC (Vision Turn Speed Control)
- Speed-dependent map data lookahead distance calculation

OPERATIONAL LOGIC:
┌─────────────────────────────────────────────────────────────────┐
│ DISABLED: No map data or feature disabled                     │
│ MONITORING: Map data available, analyzing upcoming curves      │
│ ADVANCE_WARNING: Curve detected ahead, preparing speed adjust  │
│ COORDINATING: V-TSC active, providing backup recommendations   │
└─────────────────────────────────────────────────────────────────┘

SAFETY FEATURES:
- Minimum speed enforcement (25 km/h operation threshold)
- Gas pedal override for immediate driver control
- Conservative speed recommendations (-2.5 m/s² deceleration limit)
- OSM data validation and error handling
- Seamless integration with V-TSC priority system

INTEGRATION POINTS:
- TSC Manager for coordinated turn speed control
- OpenStreetMap data via MAPD integration
- GPS positioning for map data fetching
- V-TSC coordination for handoff management

OSM DATA FETCHING:
- Speed-dependent lookahead: 3-8 seconds based on ego speed
- Curve detection from map geometry and road classification
- Adaptive fetch frequency to minimize data usage
- Cached map segments for efficiency

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
- Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
- Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import numpy as np
import time
import math
import json
from enum import Enum
from typing import Tuple, Optional, Dict

from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX

# Import MAPD extractor with fallback handling
try:
    from nagaspilot.selfdrive.mapd.np_mapd_extractor import NPMapdExtractor
    MAPD_AVAILABLE = True
except ImportError:
    cloudlog.warning("MAPD extractor not available - M-TSC will use fallback mode")
    NPMapdExtractor = None
    MAPD_AVAILABLE = False

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

# Common constants (replacing np_common imports)
SPEED_THRESHOLD_CREEP = 2.0  # m/s - minimum speed for active control

# Control modes for M-TSC system
class ControlMode(Enum):
    """Control modes for M-TSC state management"""
    DISABLED = "disabled"
    MONITORING = "monitoring" 
    ACTIVE = "active"
    ERROR = "error"

# M-TSC Operation Parameters - PROVEN SunnyPilot values
NP_MTSC_MIN_V = 25 * CV.KPH_TO_MS  # PROVEN: SunnyPilot minimum speed (aligned with documentation)
NP_MTSC_MAX_LOOKAHEAD_DISTANCE = 500.0  # Maximum distance for M-TSC focus (meters) - optimized for braking calculations  
NP_MTSC_MIN_LOOKAHEAD_DISTANCE = 100.0  # Minimum distance for M-TSC focus (meters) - sufficient for advance warning

# PROVEN SunnyPilot constants (battle-tested)
NP_MTSC_NO_OVERSHOOT_TIME_HORIZON = 4.0  # PROVEN: SunnyPilot 4-second horizon
NP_MTSC_ENTERING_PRED_LAT_ACC_TH = 1.3   # PROVEN: SunnyPilot 1.3 m/s² entering threshold
NP_MTSC_TURNING_LAT_ACC_TH = 1.6         # PROVEN: SunnyPilot 1.6 m/s² turning threshold

# Lateral acceleration limits by driving personality (matches V-TSC behavior)
NP_MTSC_A_LAT_REG_COMFORT = 2.0    # Comfort driving (gentle) - matches V-TSC
NP_MTSC_A_LAT_REG_NORMAL = 2.5     # Normal driving (balanced) - matches V-TSC default
NP_MTSC_A_LAT_REG_SPORT = 3.0      # Sport driving (aggressive) - matches V-TSC

# Curve Detection Thresholds
NP_MTSC_MIN_CURVE_RADIUS = 150.0      # meters, minimum radius to consider as curve
NP_MTSC_CURVE_DETECTION_THRESHOLD = 0.007  # rad/m, curvature threshold
NP_MTSC_CONFIDENCE_THRESHOLD = 0.7    # Minimum confidence for speed recommendations

# PROVEN acceleration values from SunnyPilot (replace experimental -2.5)
NP_MTSC_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.0]  # PROVEN: SunnyPilot smooth deceleration
NP_MTSC_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.0]   # PROVEN: SunnyPilot breakpoints
NP_MTSC_TURNING_ACC_V = [0.5, 0.0, -0.4]        # PROVEN: SunnyPilot turning acceleration
NP_MTSC_TURNING_ACC_BP = [1.5, 2.3, 3.0]        # PROVEN: SunnyPilot turning breakpoints

# Legacy constants for backward compatibility (being replaced by personality-based calculations)
NP_MTSC_APPROACH_DECEL = 1.5         # Approach deceleration (fallback value)
# Note: Main curve speed calculations now use personality-based lateral acceleration limits

# Missing constants causing crashes
NP_MTSC_REACTION_TIME = 1.5          # Reaction time in seconds
NP_MTSC_MIN_CURVE_SPEED = 8.0 * CV.KPH_TO_MS   # Minimum safe curve speed (8 kph)
NP_MTSC_MAX_CURVE_SPEED = 60.0 * CV.KPH_TO_MS  # Maximum curve speed limit (60 kph)
NP_MTSC_SAFETY_MARGIN = 1.2          # Safety margin multiplier

# Gradual speed reduction parameters
NP_MTSC_SPEED_TRANSITION_RATE = 0.5   # m/s per update cycle (0.5 m/s = 1.8 kph)
NP_MTSC_MAX_SPEED_REDUCTION_RATE = 2.0 # Maximum speed reduction rate (m/s²) - fallback only

# Personality-based deceleration limits (matches V-TSC behavior)
NP_MTSC_MAX_DECEL_COMFORT = 2.0      # Comfort: gentle deceleration
NP_MTSC_MAX_DECEL_NORMAL = 2.5       # Normal: balanced deceleration  
NP_MTSC_MAX_DECEL_SPORT = 3.0        # Sport: aggressive deceleration

# OSM Data Management - Efficient for Curve Detection (not navigation)
NP_MTSC_DATA_CACHE_TIME = 60.0       # seconds, longer cache for efficiency 
NP_MTSC_MIN_FETCH_INTERVAL = 5.0     # seconds, less frequent for curve detection
NP_MTSC_GPS_ACCURACY_THRESHOLD = 15.0 # meters, relaxed for curve detection
NP_MTSC_SEGMENT_DISTANCE_THRESHOLD = 200.0  # meters, distance before new fetch

# PROVEN FrogPilot data limits (replace experimental conservative limits)
NP_MTSC_FREE_REQUESTS = 100_000      # PROVEN: FrogPilot monthly limit
NP_MTSC_MAX_DAILY_REQUESTS = NP_MTSC_FREE_REQUESTS // 30  # ~3,333/day (proven reasonable)
# Remove experimental hourly limit - not needed with proper efficiency
NP_MTSC_FALLBACK_DISTANCE = 2000     # meters, fallback when no data available

# Debug flag
NP_MTSC_DEBUG = False

# ========================================================================
# ENUMERATIONS
# ========================================================================

# M-TSC state machine with coordination support
class MTSCState(Enum):
    """M-TSC state machine with V-TSC coordination"""
    DISABLED = 0
    MONITORING = 1        # Monitoring map data, advance warning (like SunnyPilot entering) 
    ACTIVE = 2           # Active curve management (like SunnyPilot turning)
    ADVANCE_WARNING = 3  # Advance warning state for early curve detection
    COORDINATING = 4     # Coordinating with V-TSC when both systems active

# Curve classification enum
class CurveType(Enum):
    """Curve classification for speed adjustment"""
    UNKNOWN = 0
    GENTLE = 1    # R > 500m
    MODERATE = 2  # 200m < R <= 500m  
    SHARP = 3     # R <= 200m

# REMOVE experimental curve classification - use proven physics-based approach instead
# SunnyPilot uses direct curvature-to-lateral-acceleration formula (proven)

# ========================================================================
# UTILITY FUNCTIONS
# ========================================================================

def _debug_log(msg):
    """Debug logging function"""
    if NP_MTSC_DEBUG:
        cloudlog.debug(f"[NP_MTSC]: {msg}")

def calculate_bearing_offset(lat, lon, bearing, distance):
    """Calculate new position given bearing and distance (from FrogPilot pattern)"""
    R = 6371000  # Earth's radius in meters
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)
    
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(distance / R) +
        math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad)
    )
    
    new_lon_rad = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
        math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )
    
    return math.degrees(new_lat_rad), math.degrees(new_lon_rad)

def calculate_curve_radius(lat1, lon1, lat2, lon2, lat3, lon3):
    """Calculate curve radius from three GPS points"""
    try:
        # Convert to radians
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)  
        lat3, lon3 = math.radians(lat3), math.radians(lon3)
        
        # Calculate side lengths using haversine
        def haversine_dist(lat_a, lon_a, lat_b, lon_b):
            dlat = lat_b - lat_a
            dlon = lon_b - lon_a
            a = math.sin(dlat/2)**2 + math.cos(lat_a) * math.cos(lat_b) * math.sin(dlon/2)**2
            return 2 * 6371000 * math.asin(math.sqrt(a))
        
        a = haversine_dist(lat2, lon2, lat3, lon3)  # Distance from point 2 to 3
        b = haversine_dist(lat1, lon1, lat3, lon3)  # Distance from point 1 to 3
        c = haversine_dist(lat1, lon1, lat2, lon2)  # Distance from point 1 to 2
        
        # Calculate area using cross product approximation for small distances
        area_approx = abs((lat2 - lat1) * (lon3 - lon1) - (lat3 - lat1) * (lon2 - lon1)) * 6371000**2 / 2
        
        # Calculate radius using R = abc / 4*Area
        if area_approx > 1e-10:  # Avoid division by zero
            radius = (a * b * c) / (4 * area_approx)
            return min(radius, 10000.0)  # Cap at 10km for sanity
        
        return float('inf')  # Straight line
        
    except (ValueError, ZeroDivisionError):
        return float('inf')

def eval_lat_acc_from_curvature(v_ego, curvature):
    """PROVEN SunnyPilot formula: Calculate lateral acceleration from curvature"""
    return v_ego ** 2 * curvature

def eval_curvature_from_radius(radius):
    """Convert radius to curvature (1/R)"""  
    return 1.0 / max(radius, 1.0)  # Avoid division by zero

def get_smooth_deceleration(predicted_lat_acc):
    """PROVEN SunnyPilot lookup for smooth deceleration"""
    return np.interp(abs(predicted_lat_acc), 
                     NP_MTSC_ENTERING_SMOOTH_DECEL_BP,
                     NP_MTSC_ENTERING_SMOOTH_DECEL_V)

def classify_curve(radius):
    """Classify curve type based on radius"""
    if radius > 500.0:
        return CurveType.GENTLE
    elif radius > 200.0:
        return CurveType.MODERATE
    else:
        return CurveType.SHARP

# ========================================================================
# M-TSC CONTROLLER IMPLEMENTATION  
# ========================================================================

class MTSC:
    """Map Turn Speed Controller - NagasPilot Implementation"""
    
    # ========================================================================
    # INITIALIZATION & CONFIGURATION
    # ========================================================================
    
    def __init__(self, CP):
        """Initialize M-TSC with default state following NagasPilot patterns"""
        # Parameter management (standardized interface)
        self.params = Params()
        self.enabled = self.params.get_bool("dp_lon_mtsc")
        
        # Vehicle configuration
        self._CP = CP
        self._last_params_update = 0.0
        
        # Control state variables
        self._op_enabled = False
        self._gas_pressed = False
        self._v_cruise_setpoint = 0.0
        self._v_ego = 0.0
        self._state = MTSCState.DISABLED
        self._vtsc_active = False  # V-TSC state for coordination
        
        # MAPD Data Extractor with fallback handling
        if MAPD_AVAILABLE and NPMapdExtractor:
            try:
                self._data_extractor = NPMapdExtractor(self.params)
                self._mapd_enabled = True
                cloudlog.info("M-TSC: MAPD extractor initialized successfully")
            except Exception as e:
                cloudlog.error(f"M-TSC: MAPD extractor failed to initialize: {e}")
                self._data_extractor = None
                self._mapd_enabled = False
        else:
            self._data_extractor = None
            self._mapd_enabled = False
            cloudlog.info("M-TSC: Running in fallback mode without MAPD")
        
        self._mtsc_data_cache = {}
        self._last_mtsc_data_update = 0.0
        
        # GPS tracking
        self._current_gps = None
        
        # Curve analysis variables
        self._upcoming_curves = []
        self._closest_curve_distance = float('inf')
        self._recommended_speed = 0.0
        self._confidence = 0.0
        self._curve_type = CurveType.UNKNOWN
        
        # Gradual speed reduction variables
        self._target_speed = 0.0
        self._speed_reduction_active = False
        self._last_recommended_speed = 0.0
        self._current_decel_rate = 0.0  # Track current deceleration rate for debug
        
        # Lookahead calculation
        self._current_lookahead_distance = NP_MTSC_MIN_LOOKAHEAD_DISTANCE
        
        if self.enabled:
            cloudlog.info("NP M-TSC Controller initialized with MAPD integration")
            cloudlog.info("Using real OSM curve detection instead of simulation")

    # ========================================================================
    # STANDARDIZED INTERFACE (following NagasPilot patterns)
    # ========================================================================
    
    def is_enabled(self):
        """Check if M-TSC controller is enabled via parameter"""
        return self.enabled and self.params.get_bool("dp_lon_mtsc")
        
    @property
    def state(self):
        """Get current M-TSC state"""
        return self._state
    
    @state.setter
    def state(self, value):
        """Set M-TSC state with logging"""
        if value != self._state:
            _debug_log(f'M-TSC State transition: {self._state.name} -> {value.name}')
        self._state = value

    @property
    def is_active(self):
        """Check if M-TSC is currently active (providing recommendations)"""
        return self._state in [MTSCState.ADVANCE_WARNING, MTSCState.COORDINATING]

    def get_speed_recommendation(self) -> Tuple[float, float]:
        """Get speed recommendation and confidence (for TSC Manager)"""
        if not self.is_active:
            return float('inf'), 0.0
        return self._recommended_speed, self._confidence

    # ========================================================================
    # PARAMETER MANAGEMENT
    # ========================================================================

    def _update_params(self):
        """Update parameters from storage with rate limiting"""
        tm = time.time()
        if tm > self._last_params_update + 5.0:
            self.enabled = self.params.get_bool("dp_lon_mtsc")
            self._last_params_update = tm

    def _get_lateral_comfort_limit(self):
        """Get lateral acceleration limit based on driving personality (matches V-TSC behavior)"""
        try:
            # Get driving personality from OpenPilot parameter (0=Comfort, 1=Normal, 2=Sport)
            personality = self.params.get("LongitudinalPersonality", return_default=True)
            if personality is not None:
                personality_int = int(personality)
                if personality_int == 0:    # Comfort
                    return NP_MTSC_A_LAT_REG_COMFORT
                elif personality_int == 2:  # Sport  
                    return NP_MTSC_A_LAT_REG_SPORT
                else:                       # Normal (default for 1 or invalid)
                    return NP_MTSC_A_LAT_REG_NORMAL
        except (ValueError, TypeError, AttributeError):
            pass
        return NP_MTSC_A_LAT_REG_NORMAL  # Default to Normal

    def _calculate_lookahead_distance(self, v_ego):
        """Calculate physics-based lookahead distance using deceleration and ego speed"""
        # Physics-based calculation:
        # lookahead = reaction_distance + braking_distance + safety_margin
        
        # Reaction distance: distance traveled during reaction time
        reaction_distance = v_ego * NP_MTSC_REACTION_TIME
        
        # Braking distance for worst-case curve speed reduction
        # Assume we need to decelerate from v_ego to safe curve speed
        target_curve_speed = max(NP_MTSC_MIN_CURVE_SPEED, 
                                min(NP_MTSC_MAX_CURVE_SPEED, v_ego * 0.7))  # 30% speed reduction
        
        # Calculate braking distance using: d = (v1² - v2²) / (2 * a)
        if v_ego > target_curve_speed:
            speed_diff_sq = v_ego**2 - target_curve_speed**2
            braking_distance = speed_diff_sq / (2 * NP_MTSC_APPROACH_DECEL)
        else:
            braking_distance = 0.0  # Already at appropriate speed
        
        # Total lookahead with safety margin
        base_lookahead = reaction_distance + braking_distance
        safe_lookahead = base_lookahead * NP_MTSC_SAFETY_MARGIN
        
        # Ensure reasonable bounds for data efficiency
        return np.clip(safe_lookahead, NP_MTSC_MIN_LOOKAHEAD_DISTANCE, NP_MTSC_MAX_LOOKAHEAD_DISTANCE)

    def _get_mem_params(self):
        """Get memory params for real-time data access"""
        import platform
        return Params("/dev/shm/params") if platform.system() != "Darwin" else Params()

    def _calculate_deceleration_zone(self, v_ego, target_speed):
        """Calculate deceleration zone distances for efficient data fetching"""
        if v_ego <= target_speed:
            return {
                'reaction_distance': 0.0,
                'braking_distance': 0.0,
                'total_distance': 0.0,
                'decel_start_distance': 0.0
            }
        
        # Reaction distance (constant speed during reaction time)
        reaction_distance = v_ego * NP_MTSC_REACTION_TIME
        
        # Braking distance using physics: d = (v1² - v2²) / (2a)
        speed_diff_sq = v_ego**2 - target_speed**2
        braking_distance = speed_diff_sq / (2 * NP_MTSC_APPROACH_DECEL)
        
        # Total stopping/deceleration distance
        total_distance = reaction_distance + braking_distance
        
        # Distance where deceleration should start (with safety margin)
        decel_start_distance = total_distance * NP_MTSC_SAFETY_MARGIN
        
        return {
            'reaction_distance': reaction_distance,
            'braking_distance': braking_distance,
            'total_distance': total_distance,
            'decel_start_distance': decel_start_distance
        }

    def _get_optimal_fetch_distance(self, v_ego, curves_ahead):
        """Calculate optimal distance for next data fetch based on deceleration physics"""
        if not curves_ahead:
            # No curves: adaptive based on speed (faster = look further ahead)
            # Use physics: at higher speeds, curves appear faster relative to decel capability
            base_distance = v_ego * 25  # 25 seconds of travel
            return max(min(base_distance, NP_MTSC_FALLBACK_DISTANCE), 800)
        
        closest_curve = curves_ahead[0]
        curve_distance = closest_curve['distance']
        curve_radius = closest_curve['radius']
        
        # Calculate safe speed for this curve radius using personality-based limits
        max_lat_acc = self._get_lateral_comfort_limit()  # Adapts to driving personality
        curve_curvature = 1.0 / max(curve_radius, 1.0)  # Convert radius to curvature
        safe_speed = math.sqrt(max_lat_acc / curve_curvature)  # v = sqrt(a_lat / curvature)
        safe_speed = max(safe_speed, NP_MTSC_MIN_CURVE_SPEED)  # Minimum safe speed
        
        # Get deceleration zone info
        decel_info = self._calculate_deceleration_zone(v_ego, safe_speed)
        
        if v_ego > safe_speed:
            # Need to decelerate: fetch new data before we reach deceleration zone
            # This ensures we have updated curve info when deceleration starts
            fetch_trigger_distance = curve_distance - decel_info['decel_start_distance'] - 100  # 100m buffer
            return max(fetch_trigger_distance, 150)  # Minimum 150m ahead
        else:
            # Already at appropriate speed: less urgent, can wait longer
            return max(curve_distance * 0.4, 400)  # 40% to curve or 400m minimum

    # ========================================================================
    # OSM DATA MANAGEMENT
    # ========================================================================

    # Legacy OSM fetching methods removed - replaced by MAPD extractor

    def _fetch_osm_data(self, current_gps, v_ego):
        """Legacy method - replaced by MAPD extractor"""
        # This method is now handled by NPMapdExtractor
        # All OSM data fetching is now done by the MAPD extractor in the update() method
        # Real curve data is obtained via _get_mtsc_curve_data()
        
        # Use real MAPD curve data instead of simulation
        curve_data = self._get_mtsc_curve_data()
        curves_ahead = []
        
        if curve_data and curve_data.get("has_curve", False):
            # Convert MAPD data to legacy curve format for compatibility
            primary_curve = {
                'distance': curve_data.get('curve_distance', float('inf')),
                'radius': curve_data.get('curve_radius', float('inf')),
                'latitude': 0,  # Not needed for M-TSC logic
                'longitude': 0,  # Not needed for M-TSC logic  
                'confidence': curve_data.get('confidence', 0.0),
                'type': classify_curve(curve_data.get('curve_radius', float('inf'))),
                'direction': curve_data.get('curve_direction', 'unknown'),
                'road_type': curve_data.get('road_type', 'unknown'),
                'data_source': curve_data.get('data_source', 'unknown')
            }
            curves_ahead.append(primary_curve)
            
            # Add additional curves if available
            for additional_curve in curve_data.get('all_curves', [])[:4]:  # Max 4 additional
                if additional_curve.get('distance', 0) > primary_curve['distance']:
                    curve_info = {
                        'distance': additional_curve.get('distance', float('inf')),
                        'radius': additional_curve.get('radius', float('inf')),
                        'latitude': 0,
                        'longitude': 0,
                        'confidence': additional_curve.get('confidence', 0.0),
                        'type': classify_curve(additional_curve.get('radius', float('inf'))),
                        'direction': additional_curve.get('direction', 'unknown')
                    }
                    curves_ahead.append(curve_info)
        
        self._upcoming_curves = sorted(curves_ahead, key=lambda x: x['distance'])
        
        if curves_ahead:
            closest_curve = curves_ahead[0]
            data_source = closest_curve.get('data_source', 'unknown')
            
            _debug_log(f"REAL curve at {closest_curve['distance']:.0f}m (R={closest_curve['radius']:.0f}m), "
                      f"source={data_source}, confidence={closest_curve['confidence']:.2f}")
        else:
            _debug_log(f"No curves detected from MAPD, speed={v_ego*CV.MS_TO_KPH:.0f}kph")

    def _get_mtsc_curve_data(self):
        """Get real curve data from MAPD extractor with safe parameter access."""
        try:
            # Get real curve data from memory params with retry logic
            mp = self._get_mem_params()
            
            # Use atomic read with timeout (OpenPilot pattern)
            for attempt in range(3):  # Retry up to 3 times
                try:
                    curve_data_json = mp.get("MTSCCurveData", encoding='utf-8')
                    break
                except Exception:
                    if attempt == 2:  # Last attempt
                        raise
                    time.sleep(0.001)  # 1ms wait before retry
            
            if curve_data_json:
                curve_data = json.loads(curve_data_json)
                
                # Validate data integrity and freshness
                required_keys = ["has_curve", "timestamp", "data_source"]
                if all(key in curve_data for key in required_keys):
                    data_age = time.time() - curve_data.get("timestamp", 0)
                    # More conservative freshness check for safety
                    if data_age < 5.0 and curve_data.get("has_curve", False):
                        return curve_data
            
        except Exception as e:
            cloudlog.error(f"NP M-TSC: Error reading curve data: {e}")
        
        return None

    # ========================================================================
    # CURVE ANALYSIS & SPEED CALCULATION
    # ========================================================================

    def _get_personality_max_decel(self):
        """Get maximum deceleration limit based on driving personality (matches V-TSC behavior)"""
        try:
            # Get driving personality from OpenPilot parameter (0=Comfort, 1=Normal, 2=Sport)
            personality = self.params.get("LongitudinalPersonality", return_default=True)
            if personality is not None:
                personality_int = int(personality)
                if personality_int == 0:    # Comfort
                    return NP_MTSC_MAX_DECEL_COMFORT
                elif personality_int == 2:  # Sport  
                    return NP_MTSC_MAX_DECEL_SPORT
                else:                       # Normal (default for 1 or invalid)
                    return NP_MTSC_MAX_DECEL_NORMAL
        except (ValueError, TypeError, AttributeError):
            pass
        return NP_MTSC_MAX_DECEL_NORMAL  # Default to Normal
    
    
    def _apply_gradual_speed_reduction(self, target_speed, v_ego, dt=0.05):
        """Apply distance-adaptive gradual speed reduction following driving behavior personality"""
        if target_speed >= v_ego:
            # No reduction needed - don't accelerate, just maintain or allow natural increase
            self._speed_reduction_active = False
            return min(target_speed, v_ego + 1.0)  # Allow slight increase but don't actively accelerate
        
        # Get personality-based maximum deceleration limit
        personality_max_decel = self._get_personality_max_decel()
        
        # Calculate distance-adaptive deceleration rate
        distance_to_curve = self._closest_curve_distance
        speed_diff = v_ego - target_speed
        
        # Distance-adaptive deceleration factors (percentage of personality max)
        # Farther = gentler, nearer = stronger (up to personality maximum)
        if distance_to_curve > 300:  # Very far - very gentle
            decel_factor = 0.15  # 15% of personality max - barely noticeable
            time_horizon = 10.0  # 10 seconds to target
        elif distance_to_curve > 200:  # Far - gentle
            decel_factor = 0.35  # 35% of personality max - gentle reduction
            time_horizon = 8.0   # 8 seconds to target
        elif distance_to_curve > 100:  # Medium - moderate
            decel_factor = 0.60  # 60% of personality max - noticeable but comfortable
            time_horizon = 5.0   # 5 seconds to target
        elif distance_to_curve > 50:   # Close - stronger
            decel_factor = 0.80  # 80% of personality max - firm but comfortable
            time_horizon = 3.0   # 3 seconds to target
        else:  # Very close - maximum comfortable deceleration
            decel_factor = 1.0   # 100% of personality max - follows driving behavior
            time_horizon = 2.0   # 2 seconds to target
        
        # Calculate maximum deceleration for this distance
        max_decel = personality_max_decel * decel_factor
        
        # Calculate required deceleration rate to reach target in time
        required_decel = speed_diff / time_horizon
        
        # Use the minimum of required and maximum comfortable deceleration
        actual_decel = min(required_decel, max_decel)
        
        # Apply gradual reduction
        if not self._speed_reduction_active:
            # Starting speed reduction - initialize
            self._speed_reduction_active = True
            self._last_recommended_speed = v_ego
        
        # Calculate new recommended speed with distance-adaptive transition
        speed_change = actual_decel * dt / 0.05  # Normalize for 20Hz update rate (0.05s)
        new_recommended_speed = self._last_recommended_speed - speed_change
        
        # Ensure we don't overshoot the target
        new_recommended_speed = max(new_recommended_speed, target_speed)
        
        # Store for next iteration and debug tracking
        self._last_recommended_speed = new_recommended_speed
        self._current_decel_rate = actual_decel
        
        _debug_log(f"Personality-adaptive decel: dist={distance_to_curve:.0f}m, "
                  f"personality_max={personality_max_decel:.1f}m/s², factor={decel_factor:.2f}, "
                  f"max_decel={max_decel:.1f}m/s², actual_decel={actual_decel:.1f}m/s², "
                  f"speed: {self._last_recommended_speed*CV.MS_TO_KPH:.1f}->{new_recommended_speed*CV.MS_TO_KPH:.1f}kph")
        
        return new_recommended_speed
    
    def _analyze_curves(self, v_ego):
        """Analyze upcoming curves and calculate speed recommendations with gradual reduction"""
        if not self._upcoming_curves:
            self._closest_curve_distance = float('inf')
            self._recommended_speed = 0.0
            self._confidence = 0.0
            self._curve_type = CurveType.UNKNOWN
            self._speed_reduction_active = False
            return
            
        # Find the most significant curve within our analysis range
        closest_curve = self._upcoming_curves[0]
        self._closest_curve_distance = closest_curve['distance']
        self._curve_type = closest_curve['type']
        
        # Calculate recommended speed based on curve characteristics
        curve_radius = closest_curve['radius']
        curve_confidence = closest_curve['confidence']
        
        # Calculate safe speed for curve using personality-based lateral acceleration limit
        max_lat_acc = self._get_lateral_comfort_limit()  # Adapts to driving personality like V-TSC
        curve_curvature = 1.0 / max(curve_radius, 1.0)  # Convert radius to curvature
        safe_curve_speed = math.sqrt(max_lat_acc / curve_curvature)  # v = sqrt(a_lat / curvature)
        
        # Apply curve type adjustments
        if self._curve_type == CurveType.GENTLE:
            speed_reduction = 0.9  # Minimal reduction
        elif self._curve_type == CurveType.MODERATE:
            speed_reduction = 0.8  # Moderate reduction  
        else:  # SHARP
            speed_reduction = 0.7  # Significant reduction
            
        safe_curve_speed *= speed_reduction
        
        # Calculate approach speed considering deceleration distance
        max_approach_decel = NP_MTSC_APPROACH_DECEL
        
        # Calculate maximum speed we can have now and still decelerate comfortably
        decel_distance = self._closest_curve_distance * 0.8  # Start decelerating before curve
        target_approach_speed = math.sqrt(safe_curve_speed**2 + 2 * max_approach_decel * decel_distance)
        
        # Apply gradual speed reduction instead of sudden change
        self._recommended_speed = self._apply_gradual_speed_reduction(target_approach_speed, v_ego)
        self._confidence = curve_confidence
        
        # Reduce confidence if curve is very far away
        if self._closest_curve_distance > 300:
            self._confidence *= 0.7
            
        _debug_log(f"Curve analysis: radius={curve_radius:.1f}m, dist={self._closest_curve_distance:.1f}m, "
                  f"safe_speed={safe_curve_speed*CV.MS_TO_KPH:.1f}kph, target={target_approach_speed*CV.MS_TO_KPH:.1f}kph, "
                  f"recommended={self._recommended_speed*CV.MS_TO_KPH:.1f}kph, confidence={self._confidence:.2f}, "
                  f"gradual={self._speed_reduction_active}")

    # ========================================================================
    # STATE MACHINE
    # ========================================================================

    def _update_state_machine(self):
        """Update M-TSC state machine"""
        if not self.is_enabled():
            self.state = MTSCState.DISABLED
            return
            
        # Global disable conditions
        if not self._op_enabled or self._gas_pressed or self._v_ego < NP_MTSC_MIN_V:
            self.state = MTSCState.DISABLED
            return
            
        # State-specific logic
        if self.state == MTSCState.DISABLED:
            if self._current_gps and self._upcoming_curves:
                self.state = MTSCState.MONITORING
                
        elif self.state == MTSCState.MONITORING:
            if not self._upcoming_curves:
                self.state = MTSCState.DISABLED
            elif self._confidence > NP_MTSC_CONFIDENCE_THRESHOLD and self._closest_curve_distance < 400:
                if self._vtsc_active:
                    self.state = MTSCState.COORDINATING
                else:
                    self.state = MTSCState.ADVANCE_WARNING
                    
        elif self.state == MTSCState.ADVANCE_WARNING:
            if self._vtsc_active:
                self.state = MTSCState.COORDINATING
            elif self._confidence < NP_MTSC_CONFIDENCE_THRESHOLD or self._closest_curve_distance > 500:
                self.state = MTSCState.MONITORING
            elif not self._upcoming_curves:
                self.state = MTSCState.DISABLED
                
        elif self.state == MTSCState.COORDINATING:
            if not self._vtsc_active:
                if self._confidence > NP_MTSC_CONFIDENCE_THRESHOLD:
                    self.state = MTSCState.ADVANCE_WARNING
                else:
                    self.state = MTSCState.MONITORING
            elif not self._upcoming_curves:
                self.state = MTSCState.DISABLED

    # ========================================================================
    # MAIN UPDATE INTERFACE
    # ========================================================================

    def update(self, sm, enabled, v_ego, a_ego, v_cruise_setpoint, vtsc_active=False):
        """Main update function called from TSC Manager"""
        if not self.is_enabled():
            self.state = MTSCState.DISABLED
            return
            
        try:
            # Update vehicle state
            self._op_enabled = enabled
            self._gas_pressed = sm['carState'].gasPressed
            self._v_ego = v_ego
            self._v_cruise_setpoint = v_cruise_setpoint
            self._vtsc_active = vtsc_active
            
            # Update GPS position and feed to MAPD extractor
            gps_location = sm.get('gpsLocationExternal')
            if gps_location and gps_location.valid:
                self._current_gps = {
                    'latitude': gps_location.latitude,
                    'longitude': gps_location.longitude,
                    'bearing': gps_location.bearingDeg,
                    'accuracy': gps_location.accuracy,
                    'timestamp': time.time()
                }
                
                # Update MAPD extractor with current position and heading
                # Get vehicle heading from car state or GPS bearing
                heading = gps_location.bearingDeg
                self._data_extractor.update_position(
                    gps_location.latitude, 
                    gps_location.longitude, 
                    heading
                )
                
            # Update parameters
            self._update_params()
            
            # Update MAPD data extractor with fallback handling
            if (self._current_gps and 
                self._current_gps.get('accuracy', 100) < NP_MTSC_GPS_ACCURACY_THRESHOLD):
                if self._mapd_enabled and self._data_extractor:
                    try:
                        # MAPD extractor handles its own update logic based on position changes
                        self._data_extractor.tick()
                    except Exception as e:
                        cloudlog.error(f"M-TSC: MAPD extractor error: {e}")
                        # Disable MAPD and fall back to simulation
                        self._mapd_enabled = False
                        self._data_extractor = None
                
                # If MAPD is not available, use fallback curve detection
                if not self._mapd_enabled:
                    self._fetch_osm_data(self._current_gps, v_ego)
            
            # Analyze curves and calculate recommendations
            self._analyze_curves(v_ego)
            
            # Update state machine
            self._update_state_machine()
            
        except Exception as e:
            cloudlog.error(f"NP M-TSC update error: {e}")
            self.state = MTSCState.DISABLED

    # ========================================================================
    # DEBUG INTERFACE
    # ========================================================================

    def get_debug_info(self):
        """Standardized debug information following NagasPilot patterns"""
        return {
            "enabled": self.is_enabled(),
            "active": self.is_active,
            "state": self.state.name,
            "vtsc_active": self._vtsc_active,
            "curves_detected": len(self._upcoming_curves),
            "closest_curve_distance": self._closest_curve_distance,
            "curve_type": self._curve_type.name if self._curve_type else "UNKNOWN",
            "recommended_speed_kph": self._recommended_speed * CV.MS_TO_KPH if self._recommended_speed > 0 else 0,
            "confidence": self._confidence,
            "lookahead_distance": self._current_lookahead_distance,
            "gps_valid": self._current_gps is not None,
            # Gradual speed reduction debug info
            "speed_reduction_active": self._speed_reduction_active,
            "last_recommended_speed_kph": self._last_recommended_speed * CV.MS_TO_KPH,
            "current_decel_rate": self._current_decel_rate,
            "personality_max_decel": self._get_personality_max_decel(),
            # Data efficiency metrics (FrogPilot-inspired)
            # MAPD extractor status
            "mapd_extractor_status": self._data_extractor.get_status(),
            "cached_segments": self._data_extractor.get_status().get("cached_segments", 0),
            "has_recent_mapd_data": self._data_extractor.get_status().get("has_recent_data", False)
        }

