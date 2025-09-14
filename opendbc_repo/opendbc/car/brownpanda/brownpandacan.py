from cereal import car
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC8J1850

# BrownPanda CRC8 calculation using centralized J1850 implementation
def brownpanda_crc8(data):
  """Calculate CRC8_J1850 checksum for BrownPanda CAN messages

  Uses the centralized CRC8J1850 lookup table from opendbc.car.crc
  Following the same pattern as Chrysler implementation

  Args:
    data: bytes of CAN message data (7 bytes, excluding CRC field)

  Returns:
    int: CRC8 value (0-255)
  """
  crc = 0xFF  # J1850 standard init value
  for i in range(len(data)):
    crc ^= data[i]
    crc = CRC8J1850[crc]
  return crc


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP if fingerprint is None else None, fingerprint)
    # Single-bus architecture: all virtual buses map to the same physical bus
    self._pt = self.offset
    self._radar = self.offset
    self._lkas = self.offset

  @property
  def pt(self) -> int:
    return self._pt

  @property
  def radar(self) -> int:
    return self._radar

  @property
  def lkas(self) -> int:
    return self._lkas


VisualAlert = car.CarControl.HUDControl.VisualAlert


def create_lat_command(packer, CAN: CanBusBase, CC, steering_angle: float, steering_torque: float, counter: int = 0):
  # Simple steering command with counter and CRC
  values = {
    "cmdSteerAngle": max(min(steering_angle, 819.1), -819.2),
    "cmdSteerTorque": max(min(steering_torque, 20.47), -20.48),
    "cmdRackTorque": max(min(steering_torque * 10, 3276.7), -3276.8),
    "cmdSteerMode": 1 if CC.latActive else 0,
    "cmdSteerEnable": 1 if CC.latActive else 0,
    "cmdSteerShake": 1 if getattr(CC.hudControl, 'visualAlert', None) == VisualAlert.steerRequired else 0,
    "cmdDispLnActive": 1 if CC.latActive else 0,
    "cmdDispLnEn": 1 if CC.latActive else 0,
    "cmdDispDepartLeft": 1 if getattr(CC.hudControl, 'leftLaneDepart', False) else 0,
    "cmdDispDepartRight": 1 if getattr(CC.hudControl, 'rightLaneDepart', False) else 0,
    "cmdDispTurnLeft": 1 if getattr(CC.hudControl, 'leftBlinkerOn', False) else 0,
    "cmdDispTurnRight": 1 if getattr(CC.hudControl, 'rightBlinkerOn', False) else 0,
    "counter": counter & 0xF,
  }

  # Chrysler-style: pack, calculate CRC on message data, repack
  dat = packer.make_can_msg("_0x6EC_latCommand", CAN.pt, values)[1]
  values["CRC8_J1850"] = brownpanda_crc8(dat[:7])
  return packer.make_can_msg("_0x6EC_latCommand", CAN.pt, values)


def create_long_command(packer, CAN: CanBusBase, CC, CS, accel: float, torque_command: float, counter: int = 0):
  # Simple longitudinal command
  speed_target = getattr(getattr(CC, 'cruiseControl', None), 'speedTarget', None)
  if speed_target is None:
    speed_target = getattr(CS.out, 'vEgo', 0.0) * 3.6

  values = {
    "cmdSpeedTarget": max(min(speed_target, 163.83), 0.0),
    "cmdMotorTorque": max(min(torque_command, 8191), -8192),
    "cmdBrakePressure": max(min(abs(min(accel, 0.0)) / 0.015625, 63.984375), 0.0),
    "cmdCruiseEn": 1 if CC.enabled else 0,
    "cmdRegenEn": 1 if accel < 0 else 0,
    "cmdRegenKer": 1 if accel < -1.0 else 0,
    "counter": counter & 0xF,
  }

  dat = packer.make_can_msg("_0x6EA_longCommand", CAN.pt, values)[1]
  values["CRC8_J1850"] = brownpanda_crc8(dat[:7])
  return packer.make_can_msg("_0x6EA_longCommand", CAN.pt, values)


def create_long_command2(packer, CAN: CanBusBase, CC, CS, accel: float, stopping_counter: int, params, counter: int = 0):
  # Advanced longitudinal features
  speed_accel = max(accel, 0.0) if CC.longActive else 0.0
  speed_decel = abs(min(accel, 0.0)) if CC.longActive else 0.0

  values = {
    "cmdSpeedAccel": max(min(speed_accel, -0.05), -12.8),
    "cmdSpeedDecel": max(min(speed_decel, -0.05), -12.8),
    "cmdAccelTune": 7,
    "cmdDecelTune": 7,
    "cmdPedalGas": max(min(speed_accel * 25.0, 102.0), 0.0),
    "cmdPedalBrk": max(min(speed_decel * 25.0, 102.0), 0.0),
    "cmdBrakeComp": max(min(abs(min(accel, 0.0)) * getattr(params, 'REGEN_BRAKE_MAX', 0.3), 1.0), 0.0),
    "counter": counter & 0xF,
  }

  dat = packer.make_can_msg("_0x6EB_longCommand2", CAN.pt, values)[1]
  values["CRC8_J1850"] = brownpanda_crc8(dat[:7])
  return packer.make_can_msg("_0x6EB_longCommand2", CAN.pt, values)


def create_vision_info(packer, CAN: CanBusBase, CC, CS, counter: int = 0):
  # Minimal spare command
  values = {
    "counter": counter & 0xF,
  }

  dat = packer.make_can_msg("_0x6EF_spareCommand", CAN.pt, values)[1]
  values["CRC8_J1850"] = brownpanda_crc8(dat[:7])
  return packer.make_can_msg("_0x6EF_spareCommand", CAN.pt, values)


def create_disp_command(packer, CAN: CanBusBase, CC, CS, counter: int = 0):
  # Simple display command
  hud = CC.hudControl
  lead_distance = getattr(CS.out.radarState.leadOne, 'dRel', 0.0)
  lead_lat_offset = getattr(CS.out.radarState.leadOne, 'yRel', 0.0)
  sound_alert = 1 if (getattr(hud, 'visualAlert', None) in (VisualAlert.fcw, VisualAlert.steerRequired)) else 0

  values = {
    "cmdDispSoundAlert": sound_alert,
    "cmdDispDistraction": 1 if getattr(hud, 'visualAlert', None) == VisualAlert.steerRequired else 0,
    "cmdSteerAlert": 2 if getattr(hud, 'visualAlert', None) == VisualAlert.steerRequired else 0,
    "cmdDispLeadNumber": 1 if getattr(hud, 'leadVisible', False) else 0,
    "cmdDispLeadLong": max(min(lead_distance, 1310.71), -1310.72),
    "cmdDispLeadLat": max(min(lead_lat_offset, 20.47), -20.48),
    "counter": counter & 0xF,
  }

  dat = packer.make_can_msg("_0x6ED_dispCommand", CAN.pt, values)[1]
  values["CRC8_J1850"] = brownpanda_crc8(dat[:7])
  return packer.make_can_msg("_0x6ED_dispCommand", CAN.pt, values)