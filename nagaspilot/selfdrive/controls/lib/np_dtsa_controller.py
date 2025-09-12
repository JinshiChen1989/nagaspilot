"""
NagasPilot Dynamic Torque Steering Adjustment (DTSA) Controller
BYD torque control features for BrownPanda brand vehicles

Follows DragonPilot patterns for clean integration with existing brownpanda CarController.
Supports BYD_ATTO3 and BYD_DOLPHIN models only.
"""

import numpy as np
from opendbc.car import CarParams
from cereal import messaging
from common.conversions import Conversions as CV
from common.params import Params
from common.filter import FirstOrderFilter
from common.numpy_fast import interp


class DynamicSteerActuatorDelay:
    """Dynamic Torque Steering Adjustment (DTSA) delay component - BYD implementation
    
    Enhanced BYD-specific algorithm with friction compensation and torque error tracking
    """
    
    def __init__(self, enable_dtsa=False):
        self.enable_dtsa = enable_dtsa
        
        # BYD DTSA filters and state
        self.dtsa_filter = FirstOrderFilter(0, 0.5, 0.01, False)
        self.eps_torque_error_filter = FirstOrderFilter(0, 0.3, 0.01, False)
        
        # DTSA state variables
        self.dtsa_value = 0.0
        self.eps_torque_error = 0.0
        self.last_steering_torque = 0.0
        
        # BYD DTSA parameters
        self.dtsa_base_delay = 0.02  # Base delay in seconds
        self.dtsa_max_delay = 0.5    # Maximum delay in seconds
        self.torque_error_threshold = 0.15  # Torque error threshold
        self.friction_compensation_factor = 0.12  # BYD friction factor
    
    def update(self, current_torque_diff, current_steer_torque, vehicle_speed=0.0):
        """Calculate dynamic delay using BYD's enhanced DTSA algorithm"""
        if not self.enable_dtsa:
            return 0.5  # Default delay when DTSA disabled
        
        # Calculate EPS torque error (BYD pattern)
        torque_change_rate = abs(current_steer_torque - self.last_steering_torque)
        self.eps_torque_error = self.eps_torque_error_filter.update(torque_change_rate)
        self.last_steering_torque = current_steer_torque
        
        # BYD DTSA algorithm with enhanced friction compensation
        if abs(current_steer_torque) > self.torque_error_threshold:
            # Dynamic delay based on torque difference and EPS error
            torque_factor = min(current_torque_diff * 2.0, 1.0)
            eps_error_factor = min(self.eps_torque_error * 5.0, 1.0)
            
            # Speed-dependent friction compensation (BYD specific)
            speed_factor = interp(vehicle_speed, [0, 10, 30, 60], [1.2, 1.0, 0.85, 0.7])
            friction_compensation = self.friction_compensation_factor * speed_factor
            
            # Calculate DTSA value
            base_dtsa = self.dtsa_base_delay * (1.0 + torque_factor + eps_error_factor)
            dtsa_raw = base_dtsa + friction_compensation
            
            # Clamp to reasonable limits
            dtsa_clamped = np.clip(dtsa_raw, self.dtsa_base_delay, self.dtsa_max_delay)
            
        else:
            # Minimal steering input - use default delay
            dtsa_clamped = self.dtsa_max_delay
        
        # Apply filter for smooth transitions
        self.dtsa_value = self.dtsa_filter.update(dtsa_clamped)
        return self.dtsa_value
    
    def get_debug_info(self):
        """Get DTSA debug information for logging"""
        return {
            'dtsa_value': self.dtsa_value,
            'eps_torque_error': self.eps_torque_error,
            'enabled': self.enable_dtsa
        }


class NPDTSAController:
    """NagasPilot DTSA Controller - Model-agnostic following DragonPilot patterns"""
    
    def __init__(self, CP, car_controller_params):
        self.CP = CP
        self.params = car_controller_params
        self.user_params = Params()
        
        # Check if DTSA is supported by this car (car file controls this)
        self.dtsa_supported = getattr(car_controller_params, 'DTSA_ENABLE', False)
        
        # DTSA enabled by user parameter (disabled by default)
        dtsa_enable_key = f"DTSA_Enable_{CP.carFingerprint.name}" if self.dtsa_supported else None
        self.dtsa_enabled = self.user_params.get_bool(dtsa_enable_key, False) if dtsa_enable_key else False
        
        if not self.dtsa_enabled or not self.dtsa_supported:
            return  # Skip initialization - not enabled or not supported by this car
            
        # Initialize DTSA features using DragonPilot parameter pattern
        self.enable_dtsa_delay = self.user_params.get_bool(f"DTSA_Delay_{CP.carFingerprint.name}", False)
        self.dtsa_delay_controller = DynamicSteerActuatorDelay(self.enable_dtsa_delay) if self.enable_dtsa_delay else None
        
        # Load car-specific DTSA parameters from car file
        self.dtsa_max_torque = getattr(car_controller_params, 'DTSA_MAX_TORQUE', car_controller_params.STEER_MAX)
        self.dtsa_response_factor = getattr(car_controller_params, 'DTSA_RESPONSE_FACTOR', 1.0)
        
        # Frame counter for periodic updates (DragonPilot pattern)
        self._frame = 0
        
        # Messaging for DTSA delay feedback
        if self.enable_dtsa_delay:
            self.sm = messaging.SubMaster(['carControl', 'carState'])
    
    def update_live_parameters(self):
        """Update DTSA parameters from live tuning (DragonPilot pattern)"""
        if not self.dtsa_enabled or not self.dtsa_supported:
            return
            
        # Update every 250 frames (~2.5s at 100Hz) - DragonPilot pattern
        if self._frame % 250 == 0:
            # Check if user disabled DTSA
            dtsa_enable_key = f"DTSA_Enable_{self.CP.carFingerprint.name}"
            self.dtsa_enabled = self.user_params.get_bool(dtsa_enable_key, False)
        
        self._frame += 1
    
    def apply_dtsa_enhancements(self, base_torque, CS):
        """Apply DTSA enhancements following DragonPilot patterns"""
        if not self.dtsa_enabled or not self.dtsa_supported:
            return base_torque  # Pass through unmodified
        
        enhanced_torque = base_torque
        
        # Dynamic Torque Steering Adjustment (DTSA) delay component
        if self.enable_dtsa_delay and self.dtsa_delay_controller:
            self.sm.update(0)
            current_torque_diff = abs(self.sm['carControl'].actuatorsOutput.steer - enhanced_torque)
            
            # Get vehicle speed for DTSA algorithm
            vehicle_speed = CS.vEgo * CV.MS_TO_KPH if hasattr(CS, 'vEgo') else 0.0
            
            # Update DTSA delay using enhanced algorithm
            dtsa_delay_value = self.dtsa_delay_controller.update(current_torque_diff, 
                                                               self.sm['carControl'].actuatorsOutput.steer,
                                                               vehicle_speed)
            
            # Apply DTSA-adjusted torque scaling with car-specific response factor
            base_scale = self.dtsa_response_factor
            if dtsa_delay_value < 0.3:  # High responsiveness mode
                torque_scale = base_scale * 1.05 * (1.0 - dtsa_delay_value * 0.2)
            else:  # Smooth/comfort mode
                torque_scale = base_scale * 0.95 * (1.0 - (dtsa_delay_value - 0.3) * 0.4)
                
            enhanced_torque *= np.clip(torque_scale, 0.7, 1.1)
        
        # Apply car-specific DTSA limits (from car file)
        enhanced_torque = np.clip(enhanced_torque, -self.dtsa_max_torque, self.dtsa_max_torque)
        
        return enhanced_torque
    
    def get_dtsa_delay_value(self):
        """Get current DTSA delay value for debugging"""
        if not self.dtsa_enabled or not self.dtsa_supported or not self.enable_dtsa_delay or not self.dtsa_delay_controller:
            return 0.0
        return self.dtsa_delay_controller.dtsa_value
    
    def get_dtsa_debug_info(self):
        """Get comprehensive DTSA debug information"""
        if not self.dtsa_enabled or not self.dtsa_supported or not self.enable_dtsa_delay or not self.dtsa_delay_controller:
            return None
        return self.dtsa_delay_controller.get_debug_info()
    
    def log_debug_info(self):
        """Log DTSA debug information"""
        if not self.dtsa_enabled or not self.dtsa_supported or self._frame % 50 != 0:
            return
            
        if self.enable_dtsa_delay and self.dtsa_delay_controller:
            debug_info = self.dtsa_delay_controller.get_debug_info()
            print(f"DTSA Delay: {debug_info['dtsa_value']:.3f}, EPS Error: {debug_info['eps_torque_error']:.3f}, Model: {self.CP.carFingerprint.name}")