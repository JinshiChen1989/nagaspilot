import numpy as np
from collections import defaultdict

from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.brownpanda.values import DBC, BrownPandaFlags
from opendbc.car.interfaces import CarStateBase

TransmissionType = structs.CarParams.TransmissionType
ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

BUTTONS_DICT = {
  "btnCcEn": ButtonType.mainCruise,
  "btnSpdUp": ButtonType.accelCruise,
  "btnSpdDn": ButtonType.decelCruise,
  "btnDistFar": ButtonType.gapAdjustCruise,
  "btnDistNr": ButtonType.gapAdjustCruise,
  "btnLaneEn": ButtonType.lkas,
  "btnBlinkL": ButtonType.leftBlinker,
  "btnBlinkR": ButtonType.rightBlinker,
  "btnCcResume": ButtonType.resumeCruise,
  "btnCcCancel": ButtonType.cancel,
}

GEAR_DICT = {
  0: GearShifter.park,
  1: GearShifter.reverse,
  2: GearShifter.neutral,
  3: GearShifter.drive,
}



class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    
    self.shifter_values = can_define.dv["carState"]["gearPos"]

    self.button_states = defaultdict(int)
    self.main_on_last = False
    self.lkas_enabled = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgo = cp.vl["carState"]["vehSpd"] * CV.KPH_TO_MS
    ret.vEgoRaw = ret.vEgo
    ret.standstill = ret.vEgo < 0.1

    # Wheel speeds
    ret.wheelSpeeds = structs.CarState.WheelSpeeds.new_message()
    ret.wheelSpeeds.fl = cp.vl["wheelSensor"]["wheelFrontLeft"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["wheelSensor"]["wheelFrontRight"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["wheelSensor"]["wheelRearLeft"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["wheelSensor"]["wheelRearRight"] * CV.KPH_TO_MS

    # Steering
    ret.steeringAngleDeg = cp.vl["userState"]["strAngle"]
    ret.steeringTorque = cp.vl["userState"]["strTorq"]
    ret.steeringPressed = bool(cp.vl["userState"]["actSteerPressed"])

    # Pedals
    ret.gas = cp.vl["carState"]["accPedal"] / 100.0
    ret.gasPressed = ret.gas > 0.05
    ret.brake = cp.vl["carState"]["brkPedal"] / 100.0
    ret.brakePressed = bool(cp.vl["userState"]["actBrakePressed"])

    # Gear
    ret.gearShifter = GEAR_DICT.get(cp.vl["carState"]["gearPos"], GearShifter.unknown)

    # Gas pedal
    ret.regenBraking = cp.vl["carState"]["brkPressure"] > 0

    # Engine/Motor
    ret.engineRPM = cp.vl["carState"]["engSpd"]
    
    # IMU data for improved vehicle dynamics
    ret.aEgo = cp.vl["imuSensor"]["imuAxisLong"]  # Longitudinal acceleration from IMU
    ret.aLateral = cp.vl["imuSensor"]["imuAxisLat"]  # Lateral acceleration from IMU

    # Cruise control
    ret.cruiseState.enabled = False
    ret.cruiseState.available = True
    ret.cruiseState.speed = ret.vEgo
    ret.cruiseState.standstill = ret.standstill

    # Safety
    ret.doorOpen = any([
      cp.vl["bodyState"]["switchDoorFL"],
      cp.vl["bodyState"]["switchDoorFR"],
      cp.vl["bodyState"]["switchDoorRL"],
      cp.vl["bodyState"]["switchDoorRR"],
    ])
    ret.seatbeltUnlatched = not cp.vl["bodyState"]["switchBeltDriver"]

    # Blind spot monitoring
    ret.blindSpotLeft = bool(cp.vl["radarState"]["blindSpotFL"]) or bool(cp.vl["radarState"]["blindSpotRL"])
    ret.blindSpotRight = bool(cp.vl["radarState"]["blindSpotFR"]) or bool(cp.vl["radarState"]["blindSpotRR"])

    # Lead car
    ret.radarState.leadOne.status = bool(cp.vl["radarState"]["leadConf"] > 50)
    ret.radarState.leadOne.dRel = cp.vl["radarState"]["leadDistX"]
    ret.radarState.leadOne.yRel = cp.vl["radarState"]["leadDistY"]
    ret.radarState.leadOne.vRel = 0.0  # Not available in DBC

    # Button events
    button_events = []
    for button_name, button_type in BUTTONS_DICT.items():
      button_pressed = bool(cp.vl["userState"][button_name])
      if button_pressed != self.button_states[button_name]:
        button_events.extend(create_button_events(self.button_states[button_name], button_pressed, {button_type}))
        self.button_states[button_name] = button_pressed

    ret.buttonEvents = button_events

    # Stock longitudinal
    ret.stockAeb = False
    ret.stockFcw = False

    # Update carState
    self.out = ret.as_reader()
    return ret

  @staticmethod
  def get_can_parsers(CP):
    messages = [
      ("pandaMessage", 100),      # 96: Panda status and vehicle info
      ("userState", 100),         # 97: User inputs (buttons, steering)
      ("carState", 100),          # 98: Vehicle state (speed, pedals, gear)
      ("wheelSensor", 50),        # 99: Individual wheel speeds
      ("radarState", 20),         # 100: Radar and blind spot data
      ("bodyState", 10),          # 101: Door/belt status
      ("imuSensor", 100),         # 105: IMU acceleration data
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, Bus.pt),
    }