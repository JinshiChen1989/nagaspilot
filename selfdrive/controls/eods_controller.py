#!/usr/bin/env python3
"""
EODS (Emergency Object Detection System) Controller
NagasPilot Enhanced Object Detection and Safety System

Integrates with YOLOv8 daemon to provide emergency response actions:
- Emergency object detection (person, horse, cow, elephant, etc.)
- Sophisticated threat assessment based on distance and object type
- DCP integration for emergency braking and speed control
- Real-time emergency response with configurable parameters

Architecture:
- Subscribes to yolov8Detections messages from YOLOv8 daemon
- Processes emergency class detections (EODS_CLASSES)
- Calculates threat levels and emergency distances
- Integrates with DCP foundation for control actions
- Publishes emergency alerts and actions

Author: NagasPilot Development Team
Version: 1.0.0
Date: 2025-08-02
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import IntEnum

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.numpy_fast import clip

class EmergencyAction(IntEnum):
    """Emergency action types in order of severity"""
    MONITOR = 0      # Continue monitoring, no action
    SLOW_DOWN = 1    # Reduce speed gradually
    HARD_BRAKE = 2   # Apply strong braking
    EMERGENCY_STOP = 3  # Maximum emergency braking

class ThreatLevel(IntEnum):
    """Threat level classification"""
    NONE = 0         # No threat
    LOW = 1          # Distant object, monitor
    MEDIUM = 2       # Moderate distance, prepare
    HIGH = 3         # Close distance, action required
    CRITICAL = 4     # Immediate threat, emergency action
    EMERGENCY = 5    # Maximum threat, emergency stop

@dataclass
class EmergencyObject:
    """Emergency object detection data"""
    class_name: str
    confidence: float
    distance: float
    position_x: float
    position_y: float
    position_z: float
    threat_level: ThreatLevel
    timestamp: float

@dataclass
class EmergencyResponse:
    """Emergency response decision"""
    action: EmergencyAction
    target_speed: Optional[float]  # Target speed for slow down actions
    brake_force: Optional[float]   # Brake force for braking actions
    alert_level: int              # Alert level for UI
    message: str                  # Human readable message

class NpEODSController:
    """
    Enhanced Object Detection System Controller
    
    Provides sophisticated emergency object detection and response
    integrating with NagasPilot's DCP foundation system.
    """
    
    def __init__(self):
        self.params = Params()
        
        # Load EODS parameters
        self._load_eods_parameters()
        
        # Emergency object classes (from YOLOv8 daemon)
        self.emergency_classes = {
            'person': {'severity': 5, 'min_distance': 3.0},
            'horse': {'severity': 5, 'min_distance': 5.0},
            'cow': {'severity': 5, 'min_distance': 5.0},
            'elephant': {'severity': 5, 'min_distance': 8.0},
            'cat': {'severity': 3, 'min_distance': 2.0},
            'dog': {'severity': 3, 'min_distance': 2.0},
        }
        
        # Emergency response state
        self.current_emergency_objects: List[EmergencyObject] = []
        self.last_emergency_action = EmergencyAction.MONITOR
        self.emergency_start_time = 0.0
        self.emergency_active = False
        
        # DCP integration
        self.dcp_enabled = False
        self.last_dcp_override = 0.0
        
        # Performance tracking
        self.processing_times = []
        self.detection_counts = []
        
        cloudlog.info("EODS Controller initialized with emergency object detection")
    
    def _load_eods_parameters(self):
        """Load EODS configuration parameters"""
        
        # Emergency distances (meters)
        self.emergency_distance = float(self.params.get("np_eods_emergency_distance", "10.0"))
        self.slow_distance = float(self.params.get("np_eods_slow_distance", "20.0"))
        self.monitor_distance = float(self.params.get("np_eods_monitor_distance", "35.0"))
        
        # Confidence thresholds
        self.confidence_threshold = float(self.params.get("np_eods_confidence_threshold", "0.8"))
        self.high_confidence_threshold = float(self.params.get("np_eods_high_confidence_threshold", "0.9"))
        
        # Response timing (seconds)
        self.response_delay = float(self.params.get("np_eods_response_delay", "0.2"))
        self.emergency_timeout = float(self.params.get("np_eods_emergency_timeout", "5.0"))
        
        # Speed control parameters
        self.max_emergency_decel = float(self.params.get("np_eods_max_decel", "6.0"))  # m/s²
        self.slow_down_factor = float(self.params.get("np_eods_slow_factor", "0.7"))  # 70% of current speed
        
        # UI alert parameters
        self.enable_alerts = self.params.get_bool("np_eods_enable_alerts", True)
        self.alert_threshold = float(self.params.get("np_eods_alert_threshold", "25.0"))  # meters
        
        cloudlog.info(f"EODS parameters loaded - Emergency: {self.emergency_distance}m, "
                     f"Slow: {self.slow_distance}m, Confidence: {self.confidence_threshold}")
    
    def _assess_threat_level(self, detection: Dict) -> ThreatLevel:
        """
        Assess threat level based on object class, distance, and confidence
        
        Args:
            detection: Detection data from YOLOv8 daemon
            
        Returns:
            ThreatLevel: Calculated threat level
        """
        class_name = detection.get('className', '')
        confidence = detection.get('confidence', 0.0)
        
        # Get 3D position if available
        position_3d = detection.get('position3D')
        if not position_3d:
            return ThreatLevel.NONE
        
        # Calculate distance
        distance = np.sqrt(
            position_3d.x ** 2 + 
            position_3d.y ** 2 + 
            position_3d.z ** 2
        )
        
        # Check confidence threshold
        if confidence < self.confidence_threshold:
            return ThreatLevel.NONE
        
        # Get emergency class info
        emergency_info = self.emergency_classes.get(class_name)
        if not emergency_info:
            return ThreatLevel.NONE
        
        # Base severity from class
        base_severity = emergency_info['severity']
        min_distance = emergency_info['min_distance']
        
        # Distance-based threat assessment
        if distance < min_distance:
            # Too close - maximum threat
            threat_level = ThreatLevel.EMERGENCY
        elif distance < self.emergency_distance:
            # Emergency distance
            threat_level = ThreatLevel.CRITICAL
        elif distance < self.slow_distance:
            # Slow down distance
            threat_level = ThreatLevel.HIGH if base_severity >= 4 else ThreatLevel.MEDIUM
        elif distance < self.monitor_distance:
            # Monitor distance
            threat_level = ThreatLevel.MEDIUM if base_severity >= 4 else ThreatLevel.LOW
        else:
            # Too far for action
            threat_level = ThreatLevel.LOW
        
        # Confidence adjustment
        if confidence >= self.high_confidence_threshold:
            # High confidence - increase threat by 1 level (capped)
            threat_level = min(ThreatLevel.EMERGENCY, threat_level + 1)
        elif confidence < (self.confidence_threshold + 0.1):
            # Low confidence - decrease threat by 1 level
            threat_level = max(ThreatLevel.NONE, threat_level - 1)
        
        return ThreatLevel(threat_level)
    
    def _determine_emergency_action(self, emergency_objects: List[EmergencyObject], 
                                  current_speed: float) -> EmergencyResponse:
        """
        Determine emergency action based on detected objects and current state
        
        Args:
            emergency_objects: List of emergency objects detected
            current_speed: Current vehicle speed (m/s)
            
        Returns:
            EmergencyResponse: Emergency action to take
        """
        if not emergency_objects:
            return EmergencyResponse(
                action=EmergencyAction.MONITOR,
                target_speed=None,
                brake_force=None,
                alert_level=0,
                message="No emergency objects detected"
            )
        
        # Find highest threat level
        max_threat = max(obj.threat_level for obj in emergency_objects)
        closest_object = min(emergency_objects, key=lambda obj: obj.distance)
        
        # Determine action based on highest threat
        if max_threat >= ThreatLevel.EMERGENCY:
            # Emergency stop
            return EmergencyResponse(
                action=EmergencyAction.EMERGENCY_STOP,
                target_speed=0.0,
                brake_force=self.max_emergency_decel,
                alert_level=3,
                message=f"EMERGENCY STOP: {closest_object.class_name} at {closest_object.distance:.1f}m"
            )
        
        elif max_threat >= ThreatLevel.CRITICAL:
            # Hard brake
            target_speed = max(0.0, current_speed * 0.3)  # Reduce to 30% of current speed
            brake_force = min(self.max_emergency_decel * 0.8, 4.0)  # 80% of max brake force
            
            return EmergencyResponse(
                action=EmergencyAction.HARD_BRAKE,
                target_speed=target_speed,
                brake_force=brake_force,
                alert_level=2,
                message=f"HARD BRAKE: {closest_object.class_name} at {closest_object.distance:.1f}m"
            )
        
        elif max_threat >= ThreatLevel.HIGH:
            # Slow down
            target_speed = current_speed * self.slow_down_factor
            
            return EmergencyResponse(
                action=EmergencyAction.SLOW_DOWN,
                target_speed=target_speed,
                brake_force=2.0,  # Moderate braking
                alert_level=1,
                message=f"SLOW DOWN: {closest_object.class_name} at {closest_object.distance:.1f}m"
            )
        
        else:
            # Monitor only
            return EmergencyResponse(
                action=EmergencyAction.MONITOR,
                target_speed=None,
                brake_force=None,
                alert_level=0 if max_threat <= ThreatLevel.LOW else 1,
                message=f"Monitoring: {closest_object.class_name} at {closest_object.distance:.1f}m"
            )
    
    def _integrate_with_dcp(self, emergency_response: EmergencyResponse) -> Dict:
        """
        Integrate emergency response with DCP foundation system
        
        Args:
            emergency_response: Emergency response to execute
            
        Returns:
            Dict: DCP integration data
        """
        current_time = time.time()
        
        # Create DCP override data
        dcp_data = {
            'eods_active': True,
            'emergency_action': emergency_response.action.value,
            'emergency_override': False,
            'target_speed': None,
            'decel_override': None,
            'timestamp': current_time
        }
        
        # Set DCP overrides based on action
        if emergency_response.action == EmergencyAction.EMERGENCY_STOP:
            dcp_data['emergency_override'] = True
            dcp_data['target_speed'] = 0.0
            dcp_data['decel_override'] = self.max_emergency_decel
            self.last_dcp_override = current_time
            
        elif emergency_response.action == EmergencyAction.HARD_BRAKE:
            dcp_data['emergency_override'] = True
            dcp_data['target_speed'] = emergency_response.target_speed
            dcp_data['decel_override'] = emergency_response.brake_force
            self.last_dcp_override = current_time
            
        elif emergency_response.action == EmergencyAction.SLOW_DOWN:
            dcp_data['target_speed'] = emergency_response.target_speed
            dcp_data['decel_override'] = emergency_response.brake_force
            
        # Reset override after timeout
        if (current_time - self.last_dcp_override) > self.emergency_timeout:
            dcp_data['emergency_override'] = False
        
        return dcp_data
    
    def process_detections(self, yolov8_msg, current_speed: float) -> Tuple[EmergencyResponse, Dict]:
        """
        Main processing function for EODS controller
        
        Args:
            yolov8_msg: YOLOv8 detection message
            current_speed: Current vehicle speed (m/s)
            
        Returns:
            Tuple[EmergencyResponse, Dict]: Emergency response and DCP data
        """
        start_time = time.time()
        
        # Extract emergency objects from detections
        emergency_objects = []
        
        for detection in yolov8_msg.detections:
            # Filter for EODS consumer and emergency classes
            if (detection.consumer == "EODS" and 
                detection.className in self.emergency_classes):
                
                # Assess threat level
                threat_level = self._assess_threat_level(detection.__dict__)
                
                if threat_level > ThreatLevel.NONE:
                    # Get position data
                    pos_3d = detection.position3D
                    distance = np.sqrt(pos_3d.x**2 + pos_3d.y**2 + pos_3d.z**2)
                    
                    emergency_obj = EmergencyObject(
                        class_name=detection.className,
                        confidence=detection.confidence,
                        distance=distance,
                        position_x=pos_3d.x,
                        position_y=pos_3d.y,
                        position_z=pos_3d.z,
                        threat_level=threat_level,
                        timestamp=start_time
                    )
                    emergency_objects.append(emergency_obj)
        
        # Update current emergency objects
        self.current_emergency_objects = emergency_objects
        
        # Determine emergency action
        emergency_response = self._determine_emergency_action(
            emergency_objects, current_speed
        )
        
        # Integrate with DCP if needed
        dcp_data = self._integrate_with_dcp(emergency_response)
        
        # Update emergency state
        if emergency_response.action > EmergencyAction.MONITOR:
            if not self.emergency_active:
                self.emergency_start_time = start_time
                self.emergency_active = True
        else:
            self.emergency_active = False
        
        self.last_emergency_action = emergency_response.action
        
        # Performance tracking
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.detection_counts.append(len(emergency_objects))
        
        # Keep only recent performance data
        if len(self.processing_times) > 100:
            self.processing_times = self.processing_times[-50:]
            self.detection_counts = self.detection_counts[-50:]
        
        # Log emergency actions
        if emergency_response.action > EmergencyAction.MONITOR:
            cloudlog.warning(f"EODS: {emergency_response.message}")
        
        return emergency_response, dcp_data
    
    def get_performance_stats(self) -> Dict:
        """Get EODS controller performance statistics"""
        if not self.processing_times:
            return {}
        
        return {
            'avg_processing_time_ms': np.mean(self.processing_times) * 1000,
            'max_processing_time_ms': np.max(self.processing_times) * 1000,
            'avg_detections_per_frame': np.mean(self.detection_counts),
            'total_processed_frames': len(self.processing_times),
            'emergency_active': self.emergency_active,
            'last_emergency_action': self.last_emergency_action.name,
            'emergency_duration': time.time() - self.emergency_start_time if self.emergency_active else 0.0
        }
    
    def reset_emergency_state(self):
        """Reset emergency state (for testing or manual override)"""
        self.emergency_active = False
        self.current_emergency_objects = []
        self.last_emergency_action = EmergencyAction.MONITOR
        self.emergency_start_time = 0.0
        cloudlog.info("EODS emergency state reset")

def main():
    """
    Test function for EODS controller
    """
    print("EODS Controller Test Mode")
    
    # Initialize controller
    eods = NpEODSController()
    
    # Performance stats
    stats = eods.get_performance_stats()
    print(f"Processing Stats: {stats}")

if __name__ == "__main__":
    main()