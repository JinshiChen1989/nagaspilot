"""
BrownPanda Car Controller
========================

Converts openpilot control commands to vehicle-specific CAN messages for the BrownPanda platform.
Supports both traditional acceleration/brake control and direct motor torque control for EVs.

Message Structure:
  - 106 longCmd:     Longitudinal control (accel, brake, torque) @ 100Hz
  - 107 latCommand:  Lateral control (steering angle, torque) @ 100Hz
  - 108 dispCommand: Display and HUD commands @ 50Hz

Author: BrownPanda Integration Team
"""

# ═══════════════════════════════════════════════════════════════════════════
#                                 IMPORTS
# ═══════════════════════════════════════════════════════════════════════════

import numpy as np
from opendbc.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit

# Import Params with fallback for different nagaspilot configurations
try:
  from openpilot.common.params import Params
except ImportError:
  try:
    from common.params import Params
  except ImportError:
    # Fallback for minimal configurations
    class Params:
      def get_bool(self, key, default=False):
        return default

# ═══════════════════════════════════════════════════════════════════════════
#                               CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════

# Vehicle Physics Constants
DEFAULT_VEHICLE_MASS = 1500.0      # kg - typical sedan mass
DEFAULT_WHEEL_RADIUS = 0.31        # m - typical tire radius
DEFAULT_FINAL_DRIVE_RATIO = 3.5    # gear reduction ratio
DEFAULT_MOTOR_EFFICIENCY = 0.9     # 90% motor efficiency

# Torque Application Modes
TORQUE_MODE_MOTOR = "motor"        # Torque applied at motor (before gearbox)
TORQUE_MODE_WHEEL = "wheel"        # Torque applied at wheel (after gearbox)

# Torque Limits (Nm) - Must be within DBC range [-5000|15475]
# Motor torque limits (typical EV motor)
MAX_MOTOR_TORQUE = 350.0           # Maximum motor torque (safe limit)
MIN_MOTOR_TORQUE = -200.0          # Maximum regen braking torque (safe limit)

# Wheel torque limits (high-performance vehicles can reach 5000 Nm)
MAX_WHEEL_TORQUE = 5000.0          # Maximum wheel torque (high-performance limit)
MIN_WHEEL_TORQUE = -2000.0         # Maximum wheel regen torque (high-performance limit)

# Message Frequencies
FREQ_CONTROL = 100                 # Hz - Critical control messages
FREQ_DISPLAY = 50                  # Hz - Display updates
FREQ_STATUS = 10                   # Hz - Status messages

# Debug Logging
DEBUG_LOG_INTERVAL = 50            # frames (0.5s at 100Hz)


class CarController(CarControllerBase):
  """
  BrownPanda Car Controller

  Converts openpilot control commands into BrownPanda-compatible CAN messages.
  Supports dual control modes: acceleration-based and direct motor torque.
  """

  def __init__(self, dbc_names, CP):
    """Initialize the car controller with vehicle-specific parameters"""
    super().__init__(dbc_names, CP)

    # Core Components
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = Params()
    self.frame = 0
    self.CP = CP  # Store CP reference for helper functions

    # Control State Memory (for rate limiting)
    self.last_steer_angle = 0.0
    self.last_steer_torque = 0.0
    self.last_accel = 0.0
    self.last_brake = 0.0

    # Vehicle Configuration
    self.SPEED_MAX = getattr(getattr(CP, 'carControlLimits', None), 'SPEED_MAX', 140.0)

  # ═══════════════════════════════════════════════════════════════════════════
  #                          PHYSICS HELPER FUNCTIONS
  # ═══════════════════════════════════════════════════════════════════════════

  def calculate_motor_torque(self, desired_accel, CP):
    """Convert acceleration command to motor torque using vehicle physics"""
    if desired_accel == 0:
      return 0.0

    # Vehicle parameters with fallbacks
    vehicle_mass = getattr(CP, 'mass', DEFAULT_VEHICLE_MASS)
    wheel_radius = getattr(CP, 'wheelRadius', DEFAULT_WHEEL_RADIUS)
    final_drive_ratio = getattr(CP, 'finalDriveRatio', DEFAULT_FINAL_DRIVE_RATIO)
    motor_efficiency = DEFAULT_MOTOR_EFFICIENCY

    # Physics: F = ma, T = F×r, T_motor = T_wheel / (ratio × efficiency)
    wheel_force = vehicle_mass * desired_accel
    wheel_torque = wheel_force * wheel_radius
    motor_torque_raw = wheel_torque / final_drive_ratio
    motor_torque = motor_torque_raw / motor_efficiency

    # Apply safety limits and DBC signal range
    # DBC range: SG_ cmdTorque : 16|12@1+ (5,-5000) [-5000|15475] "NM"
    motor_torque_limited = np.clip(motor_torque, MIN_MOTOR_TORQUE, MAX_MOTOR_TORQUE)

    # Ensure within DBC signal range (-5000 to +15475 Nm)
    return np.clip(motor_torque_limited, -5000.0, 15475.0)

  def get_speed_dependent_limits(self, CS, CP, limit_type):
    """Get speed-dependent control limits with fallbacks"""
    control_limits = getattr(CP, 'carControlLimits', None)
    if not control_limits:
      return self._get_default_limits(limit_type)

    limit_config = getattr(control_limits, limit_type, None)
    if not limit_config:
      return self._get_default_limits(limit_type)

    return np.interp(CS.out.vEgo, limit_config[0], limit_config[1])

  def _get_default_limits(self, limit_type):
    """Fallback limits when configuration is missing"""
    defaults = {
      'STEER_ANGLE_RATE': 30.0,
      'STEER_ANGLE_MAX': 180.0,
      'STEER_TORQUE_MAX': 100.0,
      'STEER_TORQUE_RATE': 20.0,
      'ACCEL_MAX': 50.0,
      'BRAKE_MAX': 50.0,
      'ACCEL_RATE': 10.0,
      'BRAKE_RATE': 15.0,
    }
    return defaults.get(limit_type, 50.0)



  # ═══════════════════════════════════════════════════════════════════════════
  #                              MAIN CONTROL LOOP
  # ═══════════════════════════════════════════════════════════════════════════

  def update(self, CC, CS, now_nanos):
    """
    Main control loop - called every 10ms (100Hz) by openpilot

    Processes openpilot commands and generates vehicle CAN messages
    Returns actuator confirmation and CAN message list
    """
    # Initialize
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    hand_off_enabled = self.params.get_bool("np_lat_hand_off_enable")

    # ═══════════════════════════════════════════════════════════════════════════
    #                            STEERING CONTROL
    # ═══════════════════════════════════════════════════════════════════════════

    steering_angle, steering_torque, steering_shake = self._process_steering_control(
      CC, CS, actuators, hud_control, hand_off_enabled)

    # ═══════════════════════════════════════════════════════════════════════════
    #                         LONGITUDINAL CONTROL
    # ═══════════════════════════════════════════════════════════════════════════

    (accel_pedal, brake_pedal, brake_pressure, speed_target,
     desired_accel, torque_command) = self._process_longitudinal_control(
      CC, CS, actuators)

    # ═══════════════════════════════════════════════════════════════════════════
    #                           CAN MESSAGE GENERATION
    # ═══════════════════════════════════════════════════════════════════════════

    # Generate all CAN messages
    can_sends.extend(self._generate_steering_message(steering_angle, steering_torque,
                                                   steering_shake, CC))
    can_sends.extend(self._generate_longitudinal_message(accel_pedal, brake_pedal,
                                                       brake_pressure, speed_target,
                                                       desired_accel, torque_command))
    can_sends.extend(self._generate_display_message(CC, CS, hud_control, hand_off_enabled))

    # Update frame counter and return
    self.frame += 1
    return CC.actuators.as_builder(), can_sends

  # ═══════════════════════════════════════════════════════════════════════════
  #                           CONTROL PROCESSING FUNCTIONS
  # ═══════════════════════════════════════════════════════════════════════════

  def _process_steering_control(self, CC, CS, actuators, hud_control, hand_off_enabled):
    """Process steering control commands with rate limiting and safety checks"""

    if CC.latActive:
      # Get desired commands
      desired_angle = actuators.steeringAngleDeg
      desired_torque = actuators.torque

      # Apply rate limiting
      steer_angle_rate = self.get_speed_dependent_limits(CS, self.CP, 'STEER_ANGLE_RATE')
      max_angle = self.get_speed_dependent_limits(CS, self.CP, 'STEER_ANGLE_MAX')
      max_torque = self.get_speed_dependent_limits(CS, self.CP, 'STEER_TORQUE_MAX')
      steer_torque_rate = self.get_speed_dependent_limits(CS, self.CP, 'STEER_TORQUE_RATE')

      # Rate limit and clamp angle
      limited_angle = rate_limit(desired_angle, self.last_steer_angle,
                               -steer_angle_rate * DT_CTRL, steer_angle_rate * DT_CTRL)
      limited_angle = np.clip(limited_angle, -max_angle, max_angle)

      # Rate limit and clamp torque
      limited_torque = rate_limit(desired_torque, self.last_steer_torque,
                                -steer_torque_rate * DT_CTRL, steer_torque_rate * DT_CTRL)
      limited_torque = np.clip(limited_torque, -max_torque, max_torque)

      # Steering shake for alerts (disabled in hand-off mode)
      steer_shake = 0 if hand_off_enabled else (1 if hud_control.audibleAlert else 0)

    else:
      # Inactive steering - hold current position
      limited_angle = CS.out.steeringAngleDeg if hasattr(CS.out, 'steeringAngleDeg') else 0.0
      limited_torque = 0.0
      steer_shake = 0

    # Save state for next cycle
    self.last_steer_angle = limited_angle
    self.last_steer_torque = limited_torque

    return limited_angle, limited_torque, steer_shake

  def _process_longitudinal_control(self, CC, CS, actuators):
    """Process longitudinal control with acceleration-to-torque conversion"""

    if CC.longActive:
      # Get desired acceleration
      desired_accel = actuators.accel

      # Get speed-dependent limits
      max_accel = self.get_speed_dependent_limits(CS, self.CP, 'ACCEL_MAX')
      max_brake = self.get_speed_dependent_limits(CS, self.CP, 'BRAKE_MAX')
      accel_rate = self.get_speed_dependent_limits(CS, self.CP, 'ACCEL_RATE')
      brake_rate = self.get_speed_dependent_limits(CS, self.CP, 'BRAKE_RATE')

      # Get scaling factor
      multipliers = getattr(self.CP, 'multipliers', None)
      scale_factor = multipliers.ACCEL_BRAKE_SCALE if multipliers else 1.0

      # Convert acceleration to pedal commands
      if desired_accel > 0:
        accel_pedal = min(desired_accel * scale_factor, max_accel)
        brake_pedal = 0.0
        brake_pressure = 0.0
      else:
        accel_pedal = 0.0
        brake_pedal = min(abs(desired_accel) * scale_factor, max_brake)
        brake_pressure = min(abs(desired_accel) * 2.0, 15.0)

      # Apply rate limiting
      accel_pedal = rate_limit(accel_pedal, self.last_accel,
                             -accel_rate * DT_CTRL, accel_rate * DT_CTRL)
      brake_pedal = rate_limit(brake_pedal, self.last_brake,
                             -brake_rate * DT_CTRL, brake_rate * DT_CTRL)

      # Calculate motor torque
      torque_command = self.calculate_motor_torque(desired_accel, self.CP)

      # Speed target
      control_limits = getattr(self.CP, 'carControlLimits', None)
      speed_max = getattr(control_limits, 'SPEED_MAX', 140.0) if control_limits else 140.0
      speed_target = min(getattr(CC.cruiseControl, 'speedTarget', CS.out.vEgo), speed_max)

      # Debug logging
      if self.frame % DEBUG_LOG_INTERVAL == 0:
        try:
          from openpilot.common.swaglog import cloudlog
          cloudlog.debug(f"[TorqueConv] Accel: {desired_accel:.2f}m/s² → Torque: {torque_command:.1f}Nm")
        except ImportError:
          # Fallback for different nagaspilot configurations
          pass

    else:
      # Inactive longitudinal control
      accel_pedal = brake_pedal = brake_pressure = speed_target = desired_accel = torque_command = 0.0

    # Save state for next cycle
    self.last_accel = accel_pedal
    self.last_brake = brake_pedal

    return accel_pedal, brake_pedal, brake_pressure, speed_target, desired_accel, torque_command

  # ═══════════════════════════════════════════════════════════════════════════
  #                           CAN MESSAGE GENERATION
  # ═══════════════════════════════════════════════════════════════════════════

  def _generate_steering_message(self, steering_angle, steering_torque, steering_shake, CC):
    """Generate latCommand CAN message (ID 107) @ 100Hz"""
    # Validate and clamp all signals to DBC ranges
    steer_values = {
      "cmdSteerAngle": np.clip(steering_angle, -3276.8, 3276.7),    # [-3276.8|3276.7] "deg"
      "cmdSteerTorque": np.clip(steering_torque, -327.68, 327.67),  # [-327.68|327.67] "Nm"
      "cmdSteerShake": np.clip(int(steering_shake), 0, 3),          # [0|3] ""
      "cmdSteerEnable": 1 if CC.latActive else 0,                  # [0|1] ""
      "cmdSteerMode": 1 if CC.latActive else 0,                    # [0|1] ""
      "cmdSteerAlert": 3 if hud_control.visualAlert else 0,       # [0|3] "" - Dynamic alert level
    }
    return [self.packer.make_can_msg("latCommand", Bus.pt, steer_values)]

  def _generate_longitudinal_message(self, accel_pedal, brake_pedal, brake_pressure,
                                   speed_target, desired_accel, torque_command):
    """Generate longCmd CAN message (ID 106) @ 100Hz"""
    # Validate and clamp all signals to DBC ranges
    long_values = {
      "cmdPedalAcc": np.clip(accel_pedal, 0, 100),           # [0|100] "%"
      "cmdPedalBrk": np.clip(brake_pedal, 0, 100),           # [0|100] "%"
      "cmdBrakePressure": np.clip(brake_pressure, 0, 1023.984375),  # [0|1023.984375] "bar"
      "cmdSpeedAccel": np.clip(desired_accel, -10, 2.75),   # [-10|2.75] "m/s2"
      "cmdSpeedTarget": np.clip(speed_target, 0, 655.35),    # [0|655.35] "km/h"
      "cmdTorque": torque_command,  # Already validated in calculate_motor_torque()
    }
    return [self.packer.make_can_msg("longCmd", Bus.pt, long_values)]

  def _generate_display_message(self, CC, CS, hud_control, hand_off_enabled):
    """Generate dispCommand CAN message (ID 108) @ 50Hz"""
    if self.frame % 2 != 0:  # Only send every other frame
      return []

    # Calculate display speed
    multipliers = getattr(self.CP, 'multipliers', None)
    speed_ratio = multipliers.GPS_TO_METER_SPEED_RATIO if multipliers else 3.6
    hud_speed = min(CS.out.vEgo * speed_ratio, self.SPEED_MAX) if hasattr(CS.out, 'vEgo') else 0.0

    # Validate and clamp all signals to DBC ranges
    display_values = {
      "cmdDispCruiseEnable": 1 if CC.enabled else 0,              # [0|1] ""
      "cmdDispLaneEnable": 1 if CC.latActive else 0,              # [0|1] ""
      "cmdDispSoundAlert": 0 if hand_off_enabled else (1 if hud_control.audibleAlert else 0),  # [0|1] ""
      "cmdDispDepartLeft": 1 if hud_control.leftLaneDepart else 0,   # [0|1] "" - Lane departure from model
      "cmdDispDepartRight": 1 if hud_control.rightLaneDepart else 0, # [0|1] "" - Lane departure from model
      "cmdDispLane1": 1 if hud_control.leftLaneVisible else 0,    # [0|1] "" - Left lane from vision model
      "cmdDispLane2": 1 if hud_control.lanesVisible else 0,       # [0|1] "" - Current lanes from vision model
      "cmdDispLane3": 1 if hud_control.lanesVisible else 0,       # [0|1] "" - Current lanes from vision model
      "cmdDispLane4": 1 if hud_control.rightLaneVisible else 0,   # [0|1] "" - Right lane from vision model
      "cmdDispRoadEdgeLeft": 1 if hud_control.leftLaneVisible and CC.latActive else 0,   # [0|1] "" - Left road edge from model
      "cmdDispRoadEdgeRight": 1 if hud_control.rightLaneVisible and CC.latActive else 0, # [0|1] "" - Right road edge from model
      "cmdDispSpeed": np.clip(hud_speed, 0, 655.35),              # [0|655.35] "km/h"
      "cmdDispLeadC": 1 if hud_control.leadVisible else 0,        # [0|1] "" - Center lead from radar/vision model
      "cmdDispLeadL": 1 if hasattr(CS.out, 'leftBlindspot') and CS.out.leftBlindspot else 0,   # [0|1] "" - Left lead vehicle
      "cmdDispLeadR": 1 if hasattr(CS.out, 'rightBlindspot') and CS.out.rightBlindspot else 0, # [0|1] "" - Right lead vehicle
    }
    return [self.packer.make_can_msg("dispCommand", Bus.pt, display_values)]

# ═══════════════════════════════════════════════════════════════════════════
#                                END OF FILE
# ═══════════════════════════════════════════════════════════════════════════
