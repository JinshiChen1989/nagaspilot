#!/usr/bin/env python3
"""
np_TripController - NagasPilot Trip Tracking Controller
Provides distance tracking and Trip A/B functionality with np_ prefixed parameters.
Creates TotalDistance, TotalDrives parameters for UI compatibility.
Adds np_ prefixed parameters for nagaspilot integration and intervention tracking.
"""

import time
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from cereal import messaging

# Update frequency - low rate since we're just adding Trip A/B tracking
UPDATE_RATE_HZ = 0.5  # 2 second intervals

class NpTripController:
    """
    np_TripController - NagasPilot trip tracking controller.
    Provides distance tracking and Trip A/B functionality with np_ prefixed parameters.
    Creates TotalDistance, TotalDrives parameters that UI depends on.
    Adds np_ prefixed parameters for nagaspilot integration and intervention tracking.
    """
    
    def __init__(self):
        """Initialize distance tracking, Trip A/B session tracking and intervention monitoring"""
        self.params = Params()
        
        # Core distance tracking (provides TotalDistance, TotalDrives for UI)
        self.sm = messaging.SubMaster(['carState', 'controlsState'])
        self.last_engaged_state = False
        
        # Initialize tracking state for all metrics
        self.total_distance = self.get_float_param("np_trip_total_distance", 0.0)
        self.total_drives = self.get_int_param("np_trip_total_drives", 0)
        self.uptime_onroad = self.get_float_param("np_trip_uptime_onroad", 0.0)
        self.total_onroad_time = self.get_float_param("np_trip_total_onroad_time", 0.0)
        
        # Current session tracking (resettable rolling counter)
        self.current_uptime_onroad = 0.0  # Reset each session
        self.current_total_onroad_time = 0.0  # Reset each session
        self.last_update_time = time.time()
        self.session_start_time = time.time()
        self.drive_started = False
        self.was_engaged = False
        
        # Control flag
        self.running = True
        
    def get_float_param(self, key, default):
        """Safely get float parameter"""
        try:
            param = self.params.get(key)
            return float(param.decode('utf8')) if param else default
        except:
            return default
            
    def get_int_param(self, key, default):
        """Safely get int parameter"""
        try:
            param = self.params.get(key)
            return int(param.decode('utf8')) if param else default
        except:
            return default
        
    def update_distance_tracking(self):
        """Core tracking for distance, time, and all metrics (replaces system daemons)"""
        self.sm.update()
        
        if not (self.sm.updated['carState'] and self.sm.updated['controlsState']):
            return
            
        car_state = self.sm['carState']
        controls_state = self.sm['controlsState']
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt <= 0:
            return
            
        # Track total onroad time whenever moving (> 1 m/s)  
        if car_state.vEgo > 1.0:
            self.total_onroad_time += dt  # Lifetime total
            self.current_total_onroad_time += dt  # Current session
            
        # Track uptime when openpilot is engaged (replaces UptimeOnroad from hardwared)
        if controls_state.enabled:
            self.uptime_onroad += dt  # Lifetime engaged
            self.current_uptime_onroad += dt  # Current session engaged
            if not self.was_engaged:
                self.was_engaged = True
                print("np_TripController: OpenPilot engaged - tracking onroad time")
        else:
            if self.was_engaged:
                self.was_engaged = False
                print("np_TripController: OpenPilot disengaged")
        
        # Only track distance when moving (> 1 m/s = ~3.6 km/h)
        if car_state.vEgo > 1.0:
            # Mark start of new drive if not already started
            if not self.drive_started:
                self.drive_started = True
                self.total_drives += 1
                self.params.put("np_trip_total_drives", self.total_drives)
                print(f"np_TripController: Drive {self.total_drives} started")
            
            # Calculate distance: v * t (same as original distanced)
            distance_delta = car_state.vEgo * dt
            self.total_distance += distance_delta
        else:
            # Reset drive state when stopped for a while
            if self.drive_started and dt > 30.0:  # 30 seconds stopped
                self.drive_started = False
                print("np_TripController: Drive ended")
        
        # Update parameters every few cycles to avoid excessive writes
        if int(current_time) % 5 == 0:  # Every 5 seconds
            # Calculate engagement ratios
            lifetime_ratio = (self.uptime_onroad / self.total_onroad_time * 100.0) if self.total_onroad_time > 0 else 0.0
            current_ratio = (self.current_uptime_onroad / self.current_total_onroad_time * 100.0) if self.current_total_onroad_time > 0 else 0.0
            
            # Write lifetime parameters
            self.params.put("np_trip_total_distance", self.total_distance)
            self.params.put("np_trip_uptime_onroad", self.uptime_onroad)
            self.params.put("np_trip_total_onroad_time", self.total_onroad_time)
            self.params.put("np_trip_lifetime_engagement_ratio", lifetime_ratio)
            
            # Write current session parameters
            self.params.put("np_trip_current_uptime_onroad", self.current_uptime_onroad)
            self.params.put("np_trip_current_total_onroad_time", self.current_total_onroad_time)
            self.params.put("np_trip_current_engagement_ratio", current_ratio)
            
            # Also create compatibility parameters for any legacy code
            self.params.put("TotalDistance", str(self.total_distance))
            self.params.put("UptimeOnroad", str(self.uptime_onroad))
            self.params.put("TotalDrives", self.total_drives)
        
        self.last_update_time = current_time
        
    def update_trip_sessions(self):
        """Update np_ prefixed Trip A/B session parameters"""
        try:
            # Add np_ prefixed versions for nagaspilot compatibility  
            self.params.put("np_trip_total_distance", self.total_distance)
            self.params.put("np_trip_uptime_onroad", self.uptime_onroad)
            self.params.put("np_trip_total_onroad_time", self.total_onroad_time)
            self.params.put("np_trip_total_drives", self.total_drives)
            
        except Exception as e:
            print(f"np_TripController: Error updating trip sessions: {e}")
    
    def track_interventions(self):
        """Track interventions with np_ prefix (detects OpenPilot disengagements)"""
        try:
            self.sm.update()
            if not (self.sm.updated['carState'] and self.sm.updated['controlsState']):
                return
                
            controls_state = self.sm['controlsState']
            current_engaged = controls_state.enabled
            
            # Detect disengagement (intervention)
            if self.last_engaged_state and not current_engaged:
                self.increment_interventions()
                
            self.last_engaged_state = current_engaged
            
        except Exception as e:
            print(f"np_TripController: Intervention tracking error: {e}")
    
    def increment_interventions(self):
        """Increment np_ prefixed intervention counters"""
        # Get current counts from params
        try:
            lifetime_str = self.params.get("np_trip_lifetime_interventions")
            current_str = self.params.get("np_trip_current_interventions")
            
            lifetime_count = int(lifetime_str.decode('utf8')) if lifetime_str else 0
            current_count = int(current_str.decode('utf8')) if current_str else 0
            
            # Increment both counters
            lifetime_count += 1
            current_count += 1
            
            # Write back to params
            self.params.put("np_trip_lifetime_interventions", lifetime_count)
            self.params.put("np_trip_current_interventions", current_count)
            
            print(f"np_TripController: Intervention detected - Current: {current_count}, Lifetime: {lifetime_count}")
            
        except Exception as e:
            print(f"np_TripController: Error incrementing interventions: {e}")
    
    def reset_current_session(self):
        """Reset current session counters (rolling counter)"""
        # Reset current session tracking
        self.current_uptime_onroad = 0.0
        self.current_total_onroad_time = 0.0
        
        # Reset parameters
        self.params.put("np_trip_current_interventions", 0)
        self.params.put("np_trip_current_uptime_onroad", 0.0)
        self.params.put("np_trip_current_total_onroad_time", 0.0)
        self.params.put("np_trip_current_engagement_ratio", 0.0)
        
        print("np_TripController: Current session reset - interventions, engagement tracking cleared")
    
    def get_openpilot_control_ratio(self):
        """Calculate how much OpenPilot controlled vs driver took over"""
        try:
            if self.total_onroad_time <= 0:
                return 0.0
                
            # OpenPilot control ratio = (time engaged / total driving time) * 100%
            openpilot_ratio = (self.uptime_onroad / self.total_onroad_time) * 100.0
            manual_ratio = 100.0 - openpilot_ratio
            
            print(f"np_TripController: Control Ratio - OpenPilot: {openpilot_ratio:.1f}%, Manual: {manual_ratio:.1f}%")
            print(f"np_TripController: Times - OP Engaged: {self.uptime_onroad:.1f}s, Total Onroad: {self.total_onroad_time:.1f}s")
            
            return openpilot_ratio
            
        except Exception as e:
            print(f"np_TripController: Error calculating control ratio: {e}")
            return 0.0
    
    def run(self):
        """Main loop - add Trip A/B support to system trip tracking"""
        rk = Ratekeeper(UPDATE_RATE_HZ)
        
        # Initialize intervention tracking
        self.params.put("np_trip_lifetime_interventions", 0)
        self.params.put("np_trip_current_interventions", 0)
        
        print("np_TripController: Started - adding np_ Trip A/B support to system tracking")
        
        while self.running:
            # Core distance tracking (provides TotalDistance, TotalDrives for UI)
            self.update_distance_tracking()
            
            # Add np_ prefixed parameters for nagaspilot compatibility
            self.update_trip_sessions()
            
            # Track interventions
            self.track_interventions()
            
            # Report control ratio every 60 seconds
            current_time = time.time()
            if int(current_time) % 60 == 0 and self.total_onroad_time > 30:  # At least 30s of driving
                self.get_openpilot_control_ratio()
            
            rk.keep_time()
    
    def stop(self):
        """Stop the helper"""
        self.running = False

def main():
    """Main function for standalone operation"""
    controller = NpTripController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()
        print("np_TripController stopped")

if __name__ == "__main__":
    main()