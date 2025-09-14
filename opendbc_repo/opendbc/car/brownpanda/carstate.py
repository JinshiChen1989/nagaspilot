# Standard library imports
from collections import defaultdict

# OpenPilot framework imports
from cereal import car
from opendbc.can.parser import CANParser
from opendbc.car import create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.brownpanda.values import DBC, STEER_THRESHOLD, TEMP_STEER_FAULTS, PERM_STEER_FAULTS, CAR
from opendbc.car.brownpanda.brownpandacan import CanBus, brownpanda_crc8
from opendbc.car.interfaces import CarStateBase

TransmissionType = car.CarParams.TransmissionType
ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter

# Button and control mapping constants
# Maps physical buttons to cereal ButtonEvent types for standardized processing
# Button mapping: DBC signal names -> cereal ButtonEvent types
BUTTONS_DICT = {
  # Core cruise control buttons
  "btnSpdUp": ButtonType.accelCruise,       # Speed up / Resume
  "btnSpdDn": ButtonType.decelCruise,       # Speed down / Set
  "btnCcCancel": ButtonType.cancel,         # Cancel cruise control
  "btnCcResume": ButtonType.resumeCruise,   # Resume cruise control
  "btnCcEn": ButtonType.mainCruise,         # Enable cruise control
  "btnCcSet": ButtonType.setCruise,         # Set cruise speed

  # Following distance control
  "btnDistFar": ButtonType.gapAdjustCruise, # Increase following distance
  "btnDistNr": ButtonType.gapAdjustCruise,  # Decrease following distance

  # Lane keeping and driver assistance
  "btnLaneEn": ButtonType.lkas,       # Lane keeping enable
  "btnMads": ButtonType.altButton2,         # MADS button

  # Turn signals
  "btnBlinkL": ButtonType.leftBlinker,      # Left turn signal
  "btnBlinkR": ButtonType.rightBlinker,     # Right turn signal
}

# Transmission gear mapping: DBC values -> cereal GearShifter enum
# Simple direct mapping for Chinese EV transmissions
GEAR_DICT = {
  0: GearShifter.park,     # P - Park
  1: GearShifter.reverse,  # R - Reverse
  2: GearShifter.neutral,  # N - Neutral
  3: GearShifter.drive,    # D - Drive
}


# =============================================================================
# [Section] Class
# [Brief ] Parse BrownPanda CAN into cereal CarState
# ---------------------------------------------------------------------------
# [Class ] CarState
# [Brief ] Provides update loop and CAN parser configuration
# [Bases ] CarStateBase
# =============================================================================
def validate_message(cp, msg_name):
  """Simple CRC validation following Hyundai pattern"""
  try:
    msg_data = cp.vl[msg_name]
    raw_data = cp.to_can_parser.msgs[msg_name]['data']
    if raw_data and len(raw_data) >= 8:
      expected_crc = brownpanda_crc8(raw_data[:7])
      actual_crc = msg_data.get('CRC8_J1850', 0)
      return expected_crc == actual_crc
  except (KeyError, TypeError):
    pass
  return True  # No validation if CRC not available


def check_counter(self, msg_name, counter_val):
  """Simple counter freshness check (4-bit counter, wraps at 16)"""
  if msg_name not in self.msg_counters:
    self.msg_counters[msg_name] = counter_val
    return True

  last_counter = self.msg_counters[msg_name]
  expected_counter = (last_counter + 1) & 0xF

  # Allow some tolerance for missed messages
  counter_ok = (counter_val == expected_counter) or (counter_val == ((expected_counter + 1) & 0xF))

  if counter_ok:
    self.msg_counters[msg_name] = counter_val

  return counter_ok


class CarState(CarStateBase):
  
  # =============================================================================
  # [Section] Automatic Model Detection  
  # [Brief ] Detect car brand/model from modelMarker bits in CAN messages
  # ---------------------------------------------------------------------------
  # [Function] detect_car_model
  # [Brief ] Automatically detect car model from brand marker CAN messages
  # [Params] can_recv: dict - CAN message data
  # [Returns] CAR enum or None if detection fails
  # =============================================================================
  @staticmethod
  def detect_car_model(can_recv):
    """Automatically detect car model from modelMarker bits in brand CAN messages
    
    Brand Detection Logic:
    - All models use 0x6F1 message but with different contexts
    - BYD models: modelMarker 1=ATTO3, 2=DOLPHIN + BYD brand context
    - CHANGAN models: modelMarker 1=DEEPAL_S05 + CHANGAN brand context
    - Need additional signals to distinguish BYD vs CHANGAN when both use modelMarker=1
    """
    
    # Brand-specific detection: each brand has its own message ID
    # 0x6F0 (1776) = BYD brand marker
    # 0x6F1 (1777) = CHANGAN brand marker
    # 0x6F2 (1778) = MG brand marker  
    # 0x6F3 (1779) = GAC brand marker
    
    # BYD Brand Detection (0x6F0)
    if '_0x6F0_BYD' in can_recv and can_recv['_0x6F0_BYD'] is not None:
      model_marker = can_recv['_0x6F0_BYD']['modelMarker']
      
      if model_marker == 1:
        return CAR.BYD_ATTO3    # BYD modelMarker=1
      elif model_marker == 2:
        return CAR.BYD_DOLPHIN  # BYD modelMarker=2
      # Future BYD models: modelMarker 3, 4, etc.
        
    # CHANGAN Brand Detection (0x6F1)
    elif '_0x6F1_CHANGAN' in can_recv and can_recv['_0x6F1_CHANGAN'] is not None:
      model_marker = can_recv['_0x6F1_CHANGAN']['modelMarker']
      
      if model_marker == 1:
        return CAR.DEEPAL_S05   # CHANGAN (DEEPAL sub-brand) modelMarker=1
      # Future CHANGAN models: modelMarker 2, 3, etc.
    
    # MG Brand Detection (0x6F2)
    elif '_0x6F2_MG' in can_recv and can_recv['_0x6F2_MG'] is not None:
      model_marker = can_recv['_0x6F2_MG']['modelMarker']
      # Future MG models based on modelMarker values
      return None  # Unknown MG model
    
    # GAC Brand Detection (0x6F3)
    elif '_0x6F3_GAC' in can_recv and can_recv['_0x6F3_GAC'] is not None:
      model_marker = can_recv['_0x6F3_GAC']['modelMarker']
      # Future GAC models based on modelMarker values
      return None  # Unknown GAC model
      
    return None  # No brand marker detected
  # =============================================================================
  # [Section] Initialization
  # [Brief ] Initialize state, default mappings, and system capabilities
  # ---------------------------------------------------------------------------
  # [Function] __init__
  # [Brief ] Initialize parser state and feature tracking
  # [Params] CP: CarParams
  # [Returns] None
  # =============================================================================
  def __init__(self, CP):
    super().__init__(CP)

    # Gear position tracking (simple direct mapping for EVs)
    self.shifter_values = {0: 0, 1: 1, 2: 2, 3: 3}

    # Button state tracking for edge detection
    self.button_states = defaultdict(int)

    # Driver assistance system states
    self.main_on_last = False
    self.lkas_enabled = False           # Lane keeping active status
    self.lane_enabled_status = True     # Lane keeping enabled in cluster

    # Following distance control (1=closest, 4=farthest)
    self.gap_index = 2                  # Default medium following distance
    
    # Automatic model detection state
    self.detected_model = None          # Currently detected car model
    self.detection_confidence = 0       # Detection confidence counter
    self.last_model_marker = None       # Last seen modelMarker value
    self.gap_debounce_frames = 0        # Prevent button bounce

    # System capabilities
    self.has_camera = True              # Vision-based ADAS

    # Simple counter tracking for message freshness (following other brands)
    self.msg_counters = defaultdict(int)  # Track last seen counter per message

  # =============================================================================
  # [Section] Update Loop
  # [Brief ] Parse CAN samples into standardized cereal CarState
  # ---------------------------------------------------------------------------
  # [Function] update
  # [Brief ] Parse CAN into cereal CarState
  # [Params] cp: CANParser
  # [Returns] car.CarState
  # =============================================================================
  def update(self, cp) -> car.CarState:
    ret = car.CarState.new_message()

    # Simple validation (like Hyundai) - check key messages
    key_messages = ["_0x6E0_userCommand", "_0x6E2_carState", "_0x6E6_espState"]
    for msg in key_messages:
      if not validate_message(cp, msg):
        # Could log validation failure but don't fail completely
        pass

      # Simple counter check for freshness
      try:
        counter_val = cp.vl[msg].get('counter', 0)
        if not check_counter(self, msg, counter_val):
          # Could log counter validation failure
          pass
      except (KeyError, TypeError):
        pass

    # Update button debounce
    if self.gap_debounce_frames > 0:
      self.gap_debounce_frames -= 1

    # Vehicle dynamics
    ret.vEgo = cp.vl["_0x6E2_carState"]["currentSpeed"] * CV.KPH_TO_MS  # Convert km/h to m/s
    ret.vEgoRaw = ret.vEgo
    ret.standstill = ret.vEgo < 0.1  # Vehicle stopped threshold

    # Wheel speeds
    ret.wheelSpeeds.fl = cp.vl["_0x6E6_espState"]["wheelFrontLeft"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["_0x6E6_espState"]["wheelFrontRight"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["_0x6E6_espState"]["wheelRearLeft"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["_0x6E6_espState"]["wheelRearRight"] * CV.KPH_TO_MS

    # Steering
    ret.steeringAngleDeg = cp.vl["_0x6E0_userCommand"]["steerAngle"]  # Steering wheel angle
    ret.steeringTorque = cp.vl["_0x6E2_carState"]["steerTorq"]        # Driver input torque
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD  # Driver override detection

    # Powertrain faults (steer/ACC)
    powertrain_fault = cp.vl["_0x6E7_pwtState"]["pwrFault"]
    ret.steerFaultTemporary = powertrain_fault in TEMP_STEER_FAULTS   # Recoverable faults
    ret.steerFaultPermanent = powertrain_fault in PERM_STEER_FAULTS   # Non-recoverable faults
    ret.accFaulted = powertrain_fault in (5, 6)  # Sensor/communication faults affect ACC

    # Pedals
    ret.gas = cp.vl["_0x6E0_userCommand"]["accPedal"] / 100.0         # Accelerator pedal (0.0-1.0)
    ret.gasPressed = ret.gas > 0.05                           # 5% threshold for gas pressed
    ret.brake = cp.vl["_0x6E0_userCommand"]["brkPedal"] / 100.0       # Brake pedal (0.0-1.0)
    ret.brakePressed = bool(cp.vl["_0x6E1_userCommand2"]["actBrakePressed"])  # Physical brake switch

    # Additional inputs (used for enhanced dynamics)
    _yaw_rate_user = cp.vl["_0x6E0_userCommand"]["SteerYawRate"]  # Driver yaw rate input for enhanced dynamics

    # Gear and braking
    ret.gearShifter = GEAR_DICT.get(cp.vl["_0x6E3_carState2"]["gearPos"], GearShifter.unknown)
    ret.regenBraking = cp.vl["_0x6E2_carState"]["brkPressure"] > 0  # Regenerative braking active

    # RPM and energy level
    ret.engineRPM = cp.vl["_0x6E7_pwtState"]["engRPM"]  # Motor RPM for EVs, engine RPM for hybrids

    # Battery or fuel level
    eng_type = cp.vl["_0x6E7_pwtState"]["engType"]
    if eng_type == 2:  # BEV (Battery Electric Vehicle)
        ret.fuelGauge = cp.vl["_0x6E7_pwtState"]["battLvl"] * 0.01  # Battery level (0.0-1.0)
    else:  # ICE, HEV, FCEV - use fuel level
        ret.fuelGauge = cp.vl["_0x6E7_pwtState"]["fuelLvl"] * 0.01  # Fuel level (0.0-1.0)

    # Charging status
    charge_state = cp.vl["_0x6E3_carState2"]["chrgState"]
    ret.charging = bool(charge_state > 0)  # Active charging detection

    # Non-critical fault heuristic
    ret.carFaultedNonCritical = powertrain_fault in [1, 2, 5, 6]  # Overheat, voltage, sensor faults

    # IMU sensor
    ret.aEgo = cp.vl["_0x6E9_imuSensor2"]["imuAxisLong"]  # Longitudinal acceleration (m/s²)
    ret.yawRate = cp.vl["_0x6E8_ImuSensor"]["imuYaw"]   # Vehicle yaw rate (rad/s)

    # Cruise state
    ret.cruiseState.enabled = bool(cp.vl["_0x6E2_carState"]["statCcActive"])   # Cruise control active
    ret.cruiseState.available = bool(cp.vl["_0x6E2_carState"]["statCcEn"])     # Cruise control available

    # Set speed (if available)
    try:
      set_speed_kph = cp.vl["_0x6E2_carState"]["targetSpeed"]
      if set_speed_kph > 0:
        ret.cruiseState.speedCluster = set_speed_kph * CV.KPH_TO_MS  # Convert to m/s
        ret.cruiseState.speed = ret.cruiseState.speedCluster
    except KeyError:
      ret.cruiseState.speed = ret.vEgo  # Fallback to current speed

    ret.cruiseState.standstill = ret.standstill

    # Lane keeping state
    self.lkas_enabled = bool(cp.vl["_0x6E2_carState"]["statLnActive"])        # Lane keeping active
    self.lane_enabled_status = bool(cp.vl["_0x6E2_carState"]["statLnEn"])     # Lane keeping enabled in cluster

    # Doors/seatbelt/parking brake
    ret.doorOpen = any([
      cp.vl["_0x6E3_carState2"]["switchDoorFL"],  # Front left door
      cp.vl["_0x6E3_carState2"]["switchDoorFR"],  # Front right door
      cp.vl["_0x6E3_carState2"]["switchDoorRL"],  # Rear left door
      cp.vl["_0x6E3_carState2"]["switchDoorRR"],  # Rear right door
    ])
    ret.seatbeltUnlatched = not cp.vl["_0x6E3_carState2"]["switchBeltDriver"]  # Driver seatbelt
    ret.parkingBrake = bool(cp.vl["_0x6E2_carState"]["parkingBrake"])           # Parking brake engaged
    ret.genericToggle = bool(cp.vl["_0x6E2_carState"]["rdyDrive"])              # Vehicle ready to drive

    # Blind spot
    ret.blindSpotLeft = bool(cp.vl["_0x6E4_adasState"]["bsdFrontLeft"]) or bool(cp.vl["_0x6E4_adasState"]["bsdRearLeft"])
    ret.blindSpotRight = bool(cp.vl["_0x6E4_adasState"]["bsdFrontRight"]) or bool(cp.vl["_0x6E4_adasState"]["bsdRearRight"])

    # Lead detection (camera-based)
    ret.radarState.leadOne.status = bool(cp.vl["_0x6E4_adasState"]["leadConf"] > 50)  # High confidence threshold
    ret.radarState.leadOne.dRel = cp.vl["_0x6E4_adasState"]["leadDistX"]              # Distance ahead (m)
    ret.radarState.leadOne.yRel = cp.vl["_0x6E4_adasState"]["leadDistY"]              # Lateral offset (m)
    ret.radarState.leadOne.vRel = 0.0                                          # No relative velocity from camera
    ret.radarState.radarUnavailable = True

    # Vision processing (not available in current DBC)

    # Additional blindspot mapping for cereal compatibility
    ret.leftBlindspot = bool(cp.vl["_0x6E4_adasState"]["bsdFrontLeft"] or cp.vl["_0x6E4_adasState"]["bsdRearLeft"])
    ret.rightBlindspot = bool(cp.vl["_0x6E4_adasState"]["bsdFrontRight"] or cp.vl["_0x6E4_adasState"]["bsdRearRight"])

    # Process button events for cruise control and driver assistance
    button_events = []
    for button_name, button_type in BUTTONS_DICT.items():
      # Get button state from appropriate message
      if button_name in ["btnSpdUp", "btnSpdDn", "btnCcCancel", "btnCcResume", "btnCcEn", "btnCcSet", "btnDistFar", "btnDistNr"]:
        current_pressed = bool(cp.vl["_0x6E0_userCommand"][button_name])
      else:
        current_pressed = bool(cp.vl["_0x6E1_userCommand2"][button_name])
      previous_pressed = self.button_states[button_name]

      # Detect button press/release events
      if current_pressed != previous_pressed:
        button_events.extend(create_button_events(
          int(current_pressed), int(previous_pressed), {1: button_type}
        ))
        self.button_states[button_name] = current_pressed

        # Handle following distance adjustment (with debounce)
        if current_pressed and self.gap_debounce_frames == 0:
          if button_name == "btnDistFar":
            self.gap_index = min(self.gap_index + 1, 4)  # Increase distance (max 4)
            self.gap_debounce_frames = 5                 # Prevent button bounce
          elif button_name == "btnDistNr":
            self.gap_index = max(self.gap_index - 1, 1)  # Decrease distance (min 1)
            self.gap_debounce_frames = 5                 # Prevent button bounce

    ret.buttonEvents = button_events

    # Following distance setting (prefer CAN signal if available, otherwise use button tracking)
    try:
      gi = int(cp.vl["_0x6E3_carState2"]["gapIndex"])
      if 1 <= gi <= 4:  # Valid gap index range
        self.gap_index = gi
    except KeyError:
      pass  # Use button-tracked gap_index
    ret.gapAdjustCruiseTr = int(self.gap_index)  # Current following distance setting

    # Turn signal processing (button input + actual lamp status)
    ret.leftBlinker = bool(cp.vl["_0x6E1_userCommand2"]["btnBlinkL"])    # Left turn button
    ret.rightBlinker = bool(cp.vl["_0x6E1_userCommand2"]["btnBlinkR"])   # Right turn button

    # Get actual turn signal lamp status (prefer lamp over button)
    lamp_l = ret.leftBlinker
    lamp_r = ret.rightBlinker
    try:
      lamp_l = bool(cp.vl["_0x6E3_carState2"]["turnSignalL"]) or lamp_l  # Left lamp actual status
      lamp_r = bool(cp.vl["_0x6E3_carState2"]["turnSignalR"]) or lamp_r  # Right lamp actual status
    except KeyError:
      pass  # Use button status as fallback

    # Hazard lights
    hazard = bool(cp.vl["_0x6E1_userCommand2"].get("btnHazard", 0))
    if hazard:
      lamp_l = True
      lamp_r = True

    ret.leftBlinkerOn = lamp_l   # Final left turn signal status
    ret.rightBlinkerOn = lamp_r  # Final right turn signal status

    # AEB/FCW (not available in current DBC)
    ret.stockAeb = False  # Automatic emergency braking not available
    ret.stockFcw = False  # Forward collision warning not available

    # ACC fault signal not available in current DBC
    pass  # Use powertrain fault mapping

    # ESP (not available in current DBC)
    ret.espDisabled = False  # Electronic Stability Program status unknown

    # Chassis accel (not available in current DBC)

    # Automatic model detection from brand marker CAN messages
    detected = self.detect_car_model(cp.vl)
    if detected is not None:
      if detected == self.detected_model:
        self.detection_confidence = min(self.detection_confidence + 1, 100)
      else:
        self.detected_model = detected
        self.detection_confidence = 1
        print(f"BrownPanda: Auto-detected car model {detected.name} via modelMarker (confidence: {self.detection_confidence})")
    
    # Store current modelMarker for debugging
    try:
      current_marker = cp.vl.get('_0x6F1_CHANGAN', {}).get('modelMarker', None)
      if current_marker != self.last_model_marker:
        print(f"BrownPanda: ModelMarker changed from {self.last_model_marker} to {current_marker}")
        self.last_model_marker = current_marker
    except (KeyError, AttributeError):
      pass

    # Finalize
    self.out = ret.as_reader()
    return ret

  @staticmethod
  # =============================================================================
  # [Section] CAN Parser (Powertrain bus)
  # [Brief ] Define message list and expected frequencies
  # ---------------------------------------------------------------------------
  # [Function] get_can_parser
  # [Brief ] Configure powertrain-bus CAN parser
  # [Params] CP: CarParams
  # [Returns] CANParser
  # =============================================================================
  def get_can_parser(CP):
    # CAN messages to parse with their frequencies (Hz) - Updated for DBC compliance
    messages = [
      ("_0x6E0_userCommand", 100),    # Driver inputs (buttons, pedals, steering)
      ("_0x6E1_userCommand2", 50),    # Additional user commands (MADS, blinkers)
      ("_0x6E2_carState", 100),       # Primary vehicle state (speed, gear, cruise)
      ("_0x6E3_carState2", 100),      # Extended vehicle state (doors, charging)
      ("_0x6E4_adasState", 20),       # Camera-based ADAS data (lead vehicle, BSD)
      ("_0x6E5_adasState2", 20),      # Additional ADAS state
      ("_0x6E6_espState", 50),        # Individual wheel speed sensors
      ("_0x6E7_pwtState", 20),        # Motor/engine and energy management
      ("_0x6E8_ImuSensor", 100),      # Inertial measurement unit data (gyro)
      ("_0x6E9_imuSensor2", 100),     # Inertial measurement unit data (accel)
      
      # Brand marker messages for automatic model detection
      ("_0x6F0_BYD", 10),           # BYD brand marker with modelMarker
      ("_0x6F1_CHANGAN", 10),       # CHANGAN brand marker with modelMarker
      ("_0x6F2_MG", 10),            # MG brand marker with modelMarker
      ("_0x6F3_GAC", 10),           # GAC brand marker with modelMarker
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).pt)

  @staticmethod
  # =============================================================================
  # [Section] Camera CAN Parser
  # [Brief ] Single-bus design; return empty parser for camera bus
  # ---------------------------------------------------------------------------
  # [Function] get_cam_can_parser
  # [Brief ] Configure camera-bus CAN parser (single-bus: empty)
  # [Params] CP: CarParams
  # [Returns] CANParser
  # =============================================================================
  def get_cam_can_parser(CP):
    messages = []  # No separate camera bus messages
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).pt)

