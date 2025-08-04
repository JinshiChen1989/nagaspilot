#!/usr/bin/env python3
import math
import time
from typing import SupportsFloat

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose
from openpilot.selfdrive.modeld.model_capabilities import ModelCapabilities
from openpilot.selfdrive.nagaspilot import get_model_generation
# SSD/HOD Monitoring System Enhancements - Phase 4 Implementation
from openpilot.selfdrive.controls.lib.nagaspilot.np_ssd_controller import SimpleSSDTimer  # Stand Still Duration timer
from openpilot.selfdrive.controls.lib.nagaspilot.np_hod_controller import SimpleHODTimer  # Hand Off Duration timer

# SOC Safety Controller - Phase 5 Implementation  
from openpilot.selfdrive.controls.lib.nagaspilot.np_soc_controller import NpSOCController  # Smart Offset Controller

# APSL Accelerator Pedal Speed Learning Controller (DCP Filter Architecture)
from openpilot.selfdrive.controls.lib.nagaspilot.np_apsl_controller import APSLFilter

# BPSL Brake Pedal Speed Learning Controller (DCP Filter Architecture) 
from openpilot.selfdrive.controls.lib.nagaspilot.np_bpsl_controller import BPSLFilter

# DCP Profile System - Core cruise control with filter architecture
from openpilot.selfdrive.controls.lib.nagaspilot.dcp_profile import DCPProfile

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    # DLP (Dynamic Lane Profile) support detection
    custom_model, model_gen = get_model_generation(self.params)
    model_capabilities = ModelCapabilities.get_by_gen(model_gen)
    self.model_use_lateral_planner = custom_model and model_capabilities & ModelCapabilities.LateralPlannerSolution

    # Conditional service subscription based on DLP support
    base_services = ['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                     'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                     'driverMonitoringState', 
                     'onroadEvents', 'driverAssistance']
    
    if self.model_use_lateral_planner:
      base_services.extend(['lateralPlanDEPRECATED'])
      cloudlog.info("controlsd: DLP lateral planner enabled")
    else:
      cloudlog.info("controlsd: DLP lateral planner disabled")
      
    self.sm = messaging.SubMaster(base_services, poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState', 'npControlsState'])

    self.steer_limited_by_controls = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)

    # Unified Lateral Control Mode (replaces ALKA enable toggle)
    # 0=Off, 1=Lane (ALKA), 2=Laneless, 3=DLP
    self.dlp_mode = int(self.params.get("np_dlp_mode", "0"))
    self.prev_dlp_mode = self.dlp_mode  # Track mode changes for logging
    self.alka_active = False
    
    # Log initial mode
    mode_names = {0: "Off", 1: "Lanekeep", 2: "Laneless", 3: "DLP"}
    cloudlog.info(f"Unified Lateral Control initialized: {mode_names.get(self.dlp_mode, 'Unknown')}")
    
    # NagasPilot Controllers - Minimal Integration (6 controllers kept separate as requested)
    self.nagaspilot_enabled = self.params.get_bool("np_enabled", False)
    if self.nagaspilot_enabled:
      self.ssd_timer = SimpleSSDTimer()      # Stand Still Duration timer
      self.hod_timer = SimpleHODTimer()      # Hand Off Duration timer  
      self.soc_controller = NpSOCController() # Smart Offset Controller
      self.dcp_profile = DCPProfile()        # Dynamic Cruise Profile
      self.apsl_controller = APSLFilter()    # Accelerator Pedal Speed Learning
      self.bpsl_controller = BPSLFilter()    # Brake Pedal Speed Learning
      cloudlog.info("NagasPilot enabled: 6 controllers initialized")
    else:
      cloudlog.info("NagasPilot disabled: Using base OpenPilot behavior")

  def update_nagaspilot_controllers(self, CS, enabled, current_time):
    """Update all NagasPilot controllers if enabled"""
    if not self.nagaspilot_enabled:
      return  # Skip if NagasPilot disabled
    
    # Update each controller if individually enabled
    if hasattr(self, 'ssd_timer') and self.params.get_bool("np_ssd_enabled", False):
      self.ssd_timer.update(CS, enabled, current_time)
    
    if hasattr(self, 'hod_timer') and self.params.get_bool("np_hod_enabled", False):
      self.hod_timer.update(CS, enabled, current_time)
      
    if hasattr(self, 'soc_controller') and self.params.get_bool("np_soc_enabled", False):
      self.soc_controller.update(CS, enabled, current_time)
      
    if hasattr(self, 'dcp_profile') and int(self.params.get("np_dcp_mode", "0")) > 0:
      # DCP updates handled in longitudinal control section
      pass
      
    if hasattr(self, 'apsl_controller') and self.params.get_bool("np_apsl_enabled", False):
      self.apsl_controller.update(CS, enabled)
      
    if hasattr(self, 'bpsl_controller') and self.params.get_bool("np_bpsl_enabled", False):
      self.bpsl_controller.update(CS, enabled)

  def get_lateral_mode_active(self, CS):
    """
    Unified lateral control activation logic (cruise-independent)
    Returns: (alka_active, use_lateral_planner)
    
    Enhanced safety validations:
    - Speed thresholds (following OpenPilot patterns)
    - Steering fault detection 
    - Manual override detection
    - Reverse gear protection
    - Standstill safety
    """
    # Enhanced safety checks for ALL modes (no cruise dependency)
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    reverse_gear = CS.gearShifter == car.CarState.GearShifter.reverse
    steering_faults = CS.steerFaultTemporary or CS.steerFaultPermanent
    
    # NDLOB: Check if lateral should be maintained despite brake press
    # This works across ALL DLP profiles (Lane, Laneless, DLP)
    ndlob_override = False
    if hasattr(self, 'LP') and CS.brakePressed:
        ndlob_override = self.LP.should_maintain_lateral_on_brake(self.sm)
        if ndlob_override:
            cloudlog.info(f"[NDLOB] Override active - maintaining lateral control despite brake")
    
    # Combined basic safety (enhanced from original ALKA logic)
    # Include NDLOB override for brake-based disengagement
    brake_disengagement = CS.brakePressed and not ndlob_override
    basic_safety = not standstill and not reverse_gear and not steering_faults and not brake_disengagement
    
    # Additional safety for advanced modes (DLP/Laneless)
    # NDLOB override applies to advanced modes too
    advanced_safety = basic_safety and self.model_use_lateral_planner
    
    # Get and validate current mode
    try:
      dlp_mode = int(self.params.get("np_dlp_mode", "0"))
    except (ValueError, TypeError) as e:
      # Invalid parameter - default to off for safety and track the failure
      cloudlog.error(f"CRITICAL: Parameter validation failure for np_dlp_mode: {e}")
      cloudlog.warning("Invalid np_dlp_mode parameter, defaulting to Off")
      # Track parameter corruption for system health monitoring
      if not hasattr(self, 'parameter_failures'):
        self.parameter_failures = {}
      self.parameter_failures['np_dlp_mode'] = {'error': str(e), 'timestamp': time.time()}
      dlp_mode = 0
    
    # Validate mode range
    if dlp_mode < 0 or dlp_mode > 3:
      cloudlog.warning(f"Invalid np_dlp_mode value: {dlp_mode}, defaulting to Off")
      dlp_mode = 0
    
    if dlp_mode == 0:
      # Off - no lateral assists
      return False, False
    elif dlp_mode == 1:
      # Lanekeep - cruise-independent basic lane keeping
      return basic_safety, False
    elif dlp_mode == 2:
      # Laneless - advanced lane keeping without strict lanes
      return False, advanced_safety
    elif dlp_mode == 3:
      # DLP - full dynamic lane profile system
      return False, advanced_safety
    else:
      # Fallback - should not reach here due to validation above
      return False, False

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']
    
    # NagasPilot Controllers Update - Minimal Integration
    import time
    current_time = time.monotonic()
    
    # Update all NagasPilot controllers with single call
    self.update_nagaspilot_controllers(CS, True, current_time)  # enabled=True in state_control
    
    # Get simplified status for legacy compatibility (if NagasPilot enabled)
    if self.nagaspilot_enabled:
      ssd_state = {'timeout_reached': hasattr(self, 'ssd_timer') and self.ssd_timer.get_status(CS, current_time).get('timeout_reached', False)}
    else:
      ssd_state = {'timeout_reached': False}
    hod_state = {'timeout_reached': hod_status['timeout_reached'], 'awarenessStatus': hod_status['awarenessStatus']}
    
    # Update trip tracking - minimal implementation for trip parameters
    if hasattr(CS, 'vEgo'):
        self.trip_distance += abs(CS.vEgo) * 0.01  # DT_CTRL = 0.01s
        if self.trip_distance > 1000:  # Every 1km
            current = self.params.get_int("np_trip_lifetime_distance", 0)
            self.params.put_int("np_trip_lifetime_distance", current + 1000)
            self.trip_distance = 0

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    
    # Unified Lateral Control (cruise-independent)
    self.alka_active, dynamic_lateral_planner = self.get_lateral_mode_active(CS)
    
    # Update DLP activation based on unified mode
    if dynamic_lateral_planner:
      self.model_use_lateral_planner = dynamic_lateral_planner
    
    # Mode change detection and logging
    current_dlp_mode = int(self.params.get("np_dlp_mode", "0"))
    if current_dlp_mode != self.prev_dlp_mode:
      mode_names = {0: "Off", 1: "Lanekeep", 2: "Laneless", 3: "DLP"}
      cloudlog.info(f"Lateral control mode changed: {mode_names.get(self.prev_dlp_mode, 'Unknown')} → {mode_names.get(current_dlp_mode, 'Unknown')}")
      self.prev_dlp_mode = current_dlp_mode
    
    # Normal operation with HOD timeout check
    lat_active = self.sm['selfdriveState'].active or self.alka_active
    
    # HOD (Hand Off Duration) integration: Disable lateral control when timeout reached
    # Purpose: Progressive timeout system disables steering assistance after configured duration
    # Implementation: Uses timeout_reached flag from HOD timer to affect lateral control
    hod_timeout_reached = hod_state.get('timeout_reached', False)
    
    CC.latActive = lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill) and not hod_timeout_reached
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    # NagasPilot DCP Integration - Minimal Implementation
    if self.nagaspilot_enabled and hasattr(self, 'dcp_profile') and int(self.params.get("np_dcp_mode", "0")) > 0:
      try:
        base_target_speed = CS.vCruise * CV.KPH_TO_MS
        driving_context = {'v_ego': CS.vEgo, 'cruise_enabled': CC.enabled}
        dcp_result = self.dcp_profile.get_target_speed_with_filters(base_target_speed, driving_context)
        modified_target_speed = dcp_result.get('final_speed', base_target_speed)
        if abs(modified_target_speed - base_target_speed) > 0.1:
          speed_ratio = modified_target_speed / max(base_target_speed, 0.1)
          modified_accel_target = long_plan.aTarget * min(speed_ratio, 2.0)
        else:
          modified_accel_target = long_plan.aTarget
      except Exception as e:
        modified_accel_target = long_plan.aTarget
        cloudlog.error(f"DCP processing error: {e}")
    else:
      modified_accel_target = long_plan.aTarget
    
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, modified_accel_target, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    if not CC.latActive:
      new_desired_curvature = self.curvature
    elif self.model_use_lateral_planner and self.sm.updated.get('lateralPlanDEPRECATED'):
      # DLP (Dynamic Lane Profile) mode - use lateral planner curvature
      lat_plan = self.sm['lateralPlanDEPRECATED']
      if lat_plan.mpcSolutionValid:
        new_desired_curvature = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures)
      else:
        # Fallback to model curvature when DLP solution is invalid
        new_desired_curvature = model_v2.action.desiredCurvature
    else:
      # Original nagaspilot behavior - use model curvature directly
      new_desired_curvature = model_v2.action.desiredCurvature
      
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_controls, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited)  # TODO what if not available
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      # SSD (Stand Still Duration) integration: Disable auto-resume when timeout reached
      # Purpose: Prevent indefinite auto-resume during extended standstill periods
      # Implementation: Adds SSD timeout check to existing resume condition logic
      CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1 and not ssd_state.get('timeout_reached', False)

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_controls = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_controls = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # npControlsState
    dat = messaging.new_message('npControlsState')
    dat.valid = True
    ncs = dat.npControlsState
    ncs.alkaActive = self.alka_active
    
    # DCP Foundation Status (@1-@15)
    try:
      # Core DCP status
      ncs.npDcpMode = int(self.params.get("np_dcp_mode", "1"))
      ncs.npDcpStatus = ncs.npDcpMode > 0  # Active if not OFF
      ncs.npDcpPersonality = int(self.params.get("np_dcp_personality", "1"))
      ncs.npDcpSafetyFallback = self.params.get_bool("np_dcp_safety_fallback", True)
      ncs.npDcpFallbackActive = ncs.npDcpMode == 0  # Fallback active when mode is OFF
      
      # DCP bias settings
      ncs.npDcpHighwayBias = float(self.params.get("np_dcp_highway_bias", "0.8"))
      ncs.npDcpUrbanBias = float(self.params.get("np_dcp_urban_bias", "0.3"))
      
      # Foundation readiness
      ncs.npDcpFoundationReady = True  # DCP system is implemented
      
      # Filter layer status (basic implementation for now)
      ncs.npDcpFilterLayersActive = False  # Will be updated when speed controllers are implemented
      ncs.npDcpActiveFiltersCount = 0      # No active filters yet
      ncs.npDcpBaseSpeed = 0.0             # Will be populated when filter architecture provides data
      ncs.npDcpFinalSpeed = 0.0            # Will be populated when filter architecture provides data
      
      # MPC mode and error tracking
      ncs.npDcpMpcMode = "unknown"         # Will be updated when longitudinal plan integration is complete
      ncs.npDcpErrorCount = 0              # No errors currently tracked
      
    except (ValueError, TypeError) as e:
      cloudlog.warning(f"Error populating DCP status: {e}")
      # Set safe defaults on error
      ncs.npDcpMode = 0
      ncs.npDcpStatus = False
      ncs.npDcpSafetyFallback = True
      ncs.npDcpFallbackActive = True
      ncs.npDcpFoundationReady = False
      
    # DLP Foundation Status (@16-@25)
    try:
      # Get real-time DLP status from lateral_planner
      if hasattr(self, 'lateral_planner') and hasattr(self.lateral_planner, 'dynamic_lane_profile'):
        # Validate DLP mode is within expected range [0-3]
        dlp_mode = max(0, min(3, self.lateral_planner.dynamic_lane_profile))
        ncs.npDlpMode = dlp_mode
        ncs.npDlpStatus = self.lateral_planner.dynamic_lane_profile_status
        ncs.npDlpFallbackActive = (dlp_mode == 0)
        ncs.npDlpFoundationReady = True
        
        # Real-time lane confidence and vision status
        if hasattr(self.lateral_planner, 'LP') and hasattr(self.lateral_planner.LP, 'lll_prob'):
          # Add bounds checking to ensure lane confidence stays in valid range [0.0, 1.0]
          lane_conf = (self.lateral_planner.LP.lll_prob + self.lateral_planner.LP.rll_prob) / 2
          ncs.npDlpLaneConfidence = max(0.0, min(1.0, lane_conf))
        else:
          ncs.npDlpLaneConfidence = 0.0
          
        ncs.npDlpVisionCurve = self.lateral_planner.vision_curve_laneless
        ncs.npDlpModeAuto = (dlp_mode == 3)  # Use validated mode
        ncs.npDlpEnhancementActive = (dlp_mode > 0)
        ncs.npDlpPathOffset = getattr(self.lateral_planner.LP, 'path_offset', 0.0) if hasattr(self.lateral_planner, 'LP') else 0.0
      else:
        # Fallback to parameter-based status if lateral_planner not available
        dlp_mode = int(self.params.get("np_dlp_mode", "0"))
        ncs.npDlpMode = dlp_mode
        ncs.npDlpStatus = dlp_mode > 0
        ncs.npDlpFallbackActive = dlp_mode == 0
        ncs.npDlpFoundationReady = True
        ncs.npDlpLaneConfidence = 0.0
        ncs.npDlpVisionCurve = False
        ncs.npDlpModeAuto = (dlp_mode == 3)
        ncs.npDlpEnhancementActive = False
        ncs.npDlpPathOffset = 0.0
      
    except (ValueError, TypeError) as e:
      cloudlog.warning(f"Error populating DLP status: {e}")
      # Set safe defaults on error
      ncs.npDlpMode = 0
      ncs.npDlpStatus = False
      ncs.npDlpFallbackActive = True
      ncs.npDlpFoundationReady = False
      
    # Speed Controllers @26-@43 (Phase 2 DCP Filter Layers)
    # VTSC (Vision Turn Speed Controller) @26-@31
    try:
      vtsc_enabled = self.params.get_bool("np_vtsc_enabled", False)
      ncs.npVtscEnabled = vtsc_enabled
      
      # VTSC status - Simplified
      if self.nagaspilot_enabled and vtsc_enabled:
        ncs.npVtscActive = CC.enabled
        ncs.npVtscTargetSpeed = 0.0
        ncs.npVtscCurrentCurvature = 0.0
        ncs.npVtscDistanceToCurve = 0.0
        ncs.npVtscState = 1 if CC.enabled else 0
      else:
        ncs.npVtscActive = False
        ncs.npVtscTargetSpeed = 0.0
        ncs.npVtscCurrentCurvature = 0.0
        ncs.npVtscDistanceToCurve = 0.0
        ncs.npVtscState = 1 if vtsc_enabled else 0
      
    except (ValueError, TypeError, AttributeError) as e:
      cloudlog.warning(f"Error populating VTSC status: {e}")
      # ⚠️ CRITICAL: Status population errors should be tracked for system health monitoring
      # TODO: Add error rate tracking and alert thresholds for integration issues
      # Set safe defaults on error
      ncs.npVtscEnabled = False
      ncs.npVtscActive = False
      ncs.npVtscTargetSpeed = 0.0
      ncs.npVtscCurrentCurvature = 0.0
      ncs.npVtscDistanceToCurve = 0.0
      ncs.npVtscState = 0
      
    # VCSC (Vertical Comfort Speed Controller) @32-@37 (Kalman Filter Enhanced)
    try:
      vcsc_enabled = self.params.get_bool("np_vcsc_enabled", False)
      ncs.npVcscEnabled = vcsc_enabled
      
      # VCSC status - Simplified
      if self.nagaspilot_enabled and vcsc_enabled:
        ncs.npVcscActive = CC.enabled
        ncs.npVcscTargetSpeed = 0.0
        ncs.npVcscComfortScore = 0.0
        ncs.npVcscConfidence = 0.0
        ncs.npVcscSpeedReduction = 0.0
      else:
        ncs.npVcscActive = False
        ncs.npVcscTargetSpeed = 0.0
        ncs.npVcscComfortScore = 0.0
        ncs.npVcscConfidence = 0.0
        ncs.npVcscSpeedReduction = 0.0
        ncs.npVcscComfortScore = 0.0
        ncs.npVcscConfidence = 1.0
        ncs.npVcscSpeedReduction = 0.0
        
    except (ValueError, TypeError, AttributeError) as e:
      cloudlog.warning(f"Error populating VCSC status: {e}")
      # Set safe defaults on error
      ncs.npVcscEnabled = False
      ncs.npVcscActive = False
      ncs.npVcscTargetSpeed = 0.0
      ncs.npVcscComfortScore = 0.0
      ncs.npVcscConfidence = 1.0
      ncs.npVcscSpeedReduction = 0.0

    # MTSC status - Simplified
    mtsc_enabled = self.params.get_bool("np_mtsc_enabled", False)
    ncs.npMtscEnabled = mtsc_enabled
    if self.nagaspilot_enabled and mtsc_enabled:
      ncs.npMtscActive = CC.enabled
      ncs.npMtscTargetSpeed = 0.0
    else:
      ncs.npMtscActive = False
      ncs.npMtscTargetSpeed = 0.0

    # PDA (Parallel Drive Avoidance) @41-@43  
    try:
      pda_enabled = self.params.get_bool("np_pda_enabled", False)
      ncs.npPdaEnabled = pda_enabled
      
      # Get PDA status from DCP filter manager if available
      if hasattr(self, 'dcp_profile') and hasattr(self.dcp_profile, 'filter_manager'):
        pda_filter = None
        for filter_layer in self.dcp_profile.filter_manager.filters:
          if filter_layer.name == "PDA":
            pda_filter = filter_layer
            break
        
        if pda_filter and pda_enabled:
          pda_status = pda_filter.get_status()
          ncs.npPdaActive = pda_status['active_overtaking'] and pda_status['dcp_active']
          ncs.npPdaTargetSpeed = 0.0  # Will be calculated from acceleration_multiplier
        else:
          ncs.npPdaActive = False
          ncs.npPdaTargetSpeed = 0.0
      else:
        # Fallback when DCP not available
        ncs.npPdaActive = pda_enabled and CC.enabled
        ncs.npPdaTargetSpeed = 0.0
        
    except (ValueError, TypeError, AttributeError) as e:
      cloudlog.warning(f"Error populating PDA status: {e}")
      # Set safe defaults on error
      ncs.npPdaEnabled = False
      ncs.npPdaActive = False
      ncs.npPdaTargetSpeed = 0.0
    
    
    # Monitoring System Enhancements @51-@56 - Phase 4 Message Protocol Population
    # Purpose: Publish SSD/HOD telemetry data via npControlsState for external monitoring
    # Architecture: Clean function calls with all logic encapsulated in SSD/HOD classes
    try:
      # SSD (Stand Still Duration) status fields @51-@54
      # All condition checking and timeout logic handled internally by SimpleSSDTimer
      ncs.npSsdEnabled = ssd_status['enabled']          # Feature toggle state
      ncs.npSsdActive = ssd_status['active']            # Currently in standstill and timing
      ncs.npSsdTimeRemaining = ssd_status['time_remaining']  # Seconds until timeout
      ncs.npSsdDurationLevel = ssd_status['duration_level']  # User duration setting (0-3)
      
      # HOD (Hand Off Duration) status fields @55-@56
      # All condition checking and awareness logic handled internally by SimpleHODTimer
      ncs.npHodEnabled = hod_status['enabled']          # Feature toggle state
      ncs.npHodActive = hod_status['active']            # Currently hands-off and timing
      
      # SOC (Smart Offset Controller) status fields @57-@59 - Phase 5 Safety Enhancement
      # All TTA calculation and offset logic handled internally by SmartOffsetController
      ncs.npSocEnabled = soc_status['enabled']          # Feature toggle state
      ncs.npSocActive = soc_status['active']            # Currently applying lateral offset
      ncs.npSocOffset = soc_status['offset']            # Current lateral offset (+ = left, - = right)
      
      # APSL (Accelerator Pedal Speed Learning) status fields - MADS Compatible
      # Speed learning logic handled through DCP filter architecture
      ncs.npApslEnabled = apsl_status['enabled']                    # Feature toggle state
      ncs.npApslLearningMode = apsl_status['learning_mode']         # Currently learning from pedal input
      ncs.npApslPedalPressed = apsl_status['last_pedal_position'] > 0.05  # Pedal above threshold
      ncs.npApslLearnedSpeed = apsl_status.get('learned_target_speed', 0.0) or 0.0  # Learned target speed (m/s)
      
      # BPSL (Brake Pedal Speed Learning) status fields - Dual-Pedal System
      # Brake learning logic handled through DCP filter architecture  
      ncs.npBpslEnabled = bpsl_status['enabled']                    # Feature toggle state
      ncs.npBpslLearningMode = bpsl_status['learning_mode']         # Currently learning from brake input
      ncs.npBpslBrakePressed = bpsl_status['last_brake_position'] > 0.05  # Brake above threshold
      ncs.npBpslLearnedSpeed = bpsl_status.get('learned_target_speed', 0.0) or 0.0  # Learned target speed (m/s)
      ncs.npBpslSystemBrakeActive = bpsl_status['system_brake_active']  # System vs manual brake detection
      
    except (ValueError, TypeError, AttributeError) as e:
      cloudlog.warning(f"Error populating SSD/HOD/SOC/APSL/BPSL status: {e}")
      # Safe defaults on error - ensure system stability
      ncs.npSsdEnabled = False
      ncs.npSsdActive = False
      ncs.npSsdTimeRemaining = 0.0
      ncs.npSsdDurationLevel = 0
      ncs.npHodEnabled = False
      ncs.npHodActive = False
      ncs.npSocEnabled = False
      ncs.npSocActive = False
      ncs.npSocOffset = 0.0
      ncs.npApslEnabled = False
      ncs.npApslLearningMode = False
      ncs.npApslPedalPressed = False
      ncs.npApslLearnedSpeed = 0.0
      ncs.npBpslEnabled = False
      ncs.npBpslLearningMode = False
      ncs.npBpslBrakePressed = False
      ncs.npBpslLearnedSpeed = 0.0
      ncs.npBpslSystemBrakeActive = False
    
    self.pm.send('npControlsState', dat)

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    # Driver monitoring timeout integration - warning pattern
    dm_force_decel = self.sm['driverMonitoringState'].awarenessStatus < 0.
    soft_disabling = self.sm['selfdriveState'].state == State.softDisabling
    
    # HOD (Hand Off Duration) driver monitoring integration - Phase 4 Implementation  
    # Purpose: Progressive warning system affects driver monitoring awareness level
    # Implementation: awarenessStatus from HOD timer influences deceleration behavior
    # Values: 1.0=normal, 0.5=warning, 0.0=trigger deceleration
    hod_force_decel = hod_state.get('awarenessStatus', 1.0) <= 0.0
    
    cs.forceDecel = bool(dm_force_decel or hod_force_decel or soft_disabling)

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
