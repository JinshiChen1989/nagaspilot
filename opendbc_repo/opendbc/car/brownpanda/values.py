from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, Device

Ecu = structs.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1        # 100Hz control rate
  STEER_MAX = 1500      # max steering torque
  STEER_DELTA_UP = 15   # torque increase rate
  STEER_DELTA_DOWN = 25 # torque decrease rate
  STEER_ERROR_MAX = 350 # max delta between cmd and actual

  ACCEL_MAX = 2.0       # m/s^2
  ACCEL_MIN = -3.5      # m/s^2

  def __init__(self, CP):
    pass


class BrownPandaFlags(IntFlag):
  # Powertrain types
  EV = 1
  HYBRID = 2
  ICE = 4

  # Steering control
  ANGLE_CONTROL = 8
  TORQUE_CONTROL = 16

  # Features
  LONG_CONTROL = 32
  STOP_AND_GO = 64
  LANE_CHANGE_ASSIST = 128
  TRAFFIC_SIGN_DETECTION = 256
  BLIND_SPOT_MONITOR = 512
  FORWARD_COLLISION_WARNING = 1024

  # Sensors
  CAMERA_RADAR = 2048
  LIDAR = 4096
  ULTRASONIC = 8192

  # Regional
  LEFT_HAND_DRIVE = 16384
  RIGHT_HAND_DRIVE = 32768


class Footnote(Enum):
  BYD_CHARGING = CarFootnote(
    "BYD ATTO3: Autonomous features disabled during charging.",
    Column.FSR_LONGITUDINAL)

  DEEPAL_PERFORMANCE = CarFootnote(
    "DEEPAL S05: Performance mode affects steering sensitivity.",
    Column.FSR_STEERING)

  BROWNPANDA_SPEED = CarFootnote(
    "BrownPanda vehicles: Full features available above 5 mph.",
    Column.FSR_LONGITUDINAL)


@dataclass
class BrownPandaCarDocs(CarDocs):
  package: str = "BrownPanda Autonomous System"

  def init_make(self, CP: structs.CarParams):
    if CP.flags & BrownPandaFlags.EV:
      harness = CarHarness.ford_q4
    elif CP.flags & BrownPandaFlags.HYBRID:
      harness = CarHarness.hyundai_k
    else:
      harness = CarHarness.toyota_a

    parts = [harness]
    if CP.flags & BrownPandaFlags.CAMERA_RADAR:
      parts.append(Device.threex_angled_mount)
    if CP.flags & BrownPandaFlags.LIDAR:
      parts.append(Device.comma_power_v2)

    self.car_parts = CarParts(parts)


def dbc_dict(pt_dbc, radar_dbc):
  return {Bus.pt: pt_dbc, Bus.radar: radar_dbc}


@dataclass
class BrownPandaPlatformConfig(PlatformConfig):
  dbc_dict: dict = field(default_factory=lambda: dbc_dict('brownpanda', None))

  def init(self):
    pass


class CAR(Platforms):
  BYD_ATTO3 = BrownPandaPlatformConfig(
    [
      BrownPandaCarDocs("BYD ATTO3 2022-24", "BrownPanda Pro",
                        video="https://www.youtube.com/watch?v=brownpanda_atto3",
                        footnotes=[Footnote.BYD_CHARGING, Footnote.BROWNPANDA_SPEED],
                        min_steer_speed=5. * CV.MPH_TO_MS,
                        min_enable_speed=5. * CV.MPH_TO_MS),
      BrownPandaCarDocs("BYD ATTO3 Extended Range 2023-24", "BrownPanda Pro+",
                        min_steer_speed=3. * CV.MPH_TO_MS),
    ],
    CarSpecs(
      mass=1680,        # Compact EV SUV class
      wheelbase=2.72,   # Standard SUV dimensions
      steerRatio=16.0,  # Balanced steering response
      centerToFrontRatio=0.4,
      tireStiffnessFactor=0.8,
      minSteerSpeed=0.5,  # 1.8 km/h - allow very low speed steering
      minEnableSpeed=-1,  # Enable from complete stop (like VW)
    ),
    dbc_dict('brownpanda', None),
    flags=BrownPandaFlags.EV |
          BrownPandaFlags.ANGLE_CONTROL |
          BrownPandaFlags.STOP_AND_GO |
          BrownPandaFlags.LANE_CHANGE_ASSIST |
          BrownPandaFlags.CAMERA_RADAR |
          BrownPandaFlags.BLIND_SPOT_MONITOR |
          BrownPandaFlags.FORWARD_COLLISION_WARNING,
  )

  DEEPAL_S05 = BrownPandaPlatformConfig(
    [
      BrownPandaCarDocs("DEEPAL S05 2023-24", "BrownPanda Advanced",
                        video="https://www.youtube.com/watch?v=brownpanda_deepal",
                        footnotes=[Footnote.DEEPAL_PERFORMANCE, Footnote.BROWNPANDA_SPEED],
                        min_steer_speed=3. * CV.MPH_TO_MS,
                        min_enable_speed=3. * CV.MPH_TO_MS),
      BrownPandaCarDocs("DEEPAL S05 Performance 2024", "BrownPanda Sport",
                        min_steer_speed=2. * CV.MPH_TO_MS),
    ],
    CarSpecs(
      mass=1750,        # Performance EV SUV + LIDAR equipment
      wheelbase=2.72,   # Same platform as ATTO3
      steerRatio=16.0,  # Consistent with BYD platform
      centerToFrontRatio=0.42,
      tireStiffnessFactor=0.85,  # Higher performance tires
      minSteerSpeed=3 * CV.KPH_TO_MS,
      minEnableSpeed=3 * CV.KPH_TO_MS,
    ),
    dbc_dict('brownpanda', None),
    flags=BrownPandaFlags.EV |
          BrownPandaFlags.ANGLE_CONTROL |
          BrownPandaFlags.STOP_AND_GO |
          BrownPandaFlags.LANE_CHANGE_ASSIST |
          BrownPandaFlags.CAMERA_RADAR |
          BrownPandaFlags.BLIND_SPOT_MONITOR |
          BrownPandaFlags.FORWARD_COLLISION_WARNING |
          BrownPandaFlags.TRAFFIC_SIGN_DETECTION |
          BrownPandaFlags.LIDAR,
  )

  BYD_DOLPHIN = BrownPandaPlatformConfig(
    [
      BrownPandaCarDocs("BYD DOLPHIN 2023-24", "BrownPanda Compact",
                        video="https://www.youtube.com/watch?v=brownpanda_dolphin",
                        footnotes=[Footnote.BYD_CHARGING, Footnote.BROWNPANDA_SPEED],
                        min_steer_speed=5. * CV.MPH_TO_MS,
                        min_enable_speed=5. * CV.MPH_TO_MS),
      BrownPandaCarDocs("BYD DOLPHIN Premium 2024", "BrownPanda Compact+",
                        min_steer_speed=3. * CV.MPH_TO_MS),
    ],
    CarSpecs(
      mass=1405,        # Honda Jazz/Fit + EV battery (1200kg + 205kg)
      wheelbase=2.70,   # Subcompact EV class, Honda Jazz equivalent
      steerRatio=15.0,
      centerToFrontRatio=0.40,
      tireStiffnessFactor=0.70,
      minSteerSpeed=5 * CV.KPH_TO_MS,
      minEnableSpeed=5 * CV.KPH_TO_MS,
    ),
    dbc_dict('brownpanda', None),
    flags=BrownPandaFlags.EV |
          BrownPandaFlags.ANGLE_CONTROL |
          BrownPandaFlags.STOP_AND_GO |
          BrownPandaFlags.LANE_CHANGE_ASSIST |
          BrownPandaFlags.CAMERA_RADAR |
          BrownPandaFlags.BLIND_SPOT_MONITOR |
          BrownPandaFlags.FORWARD_COLLISION_WARNING,
  )

  FORD_RANGER = BrownPandaPlatformConfig(
    [
      BrownPandaCarDocs("FORD RANGER 2023-24", "BrownPanda Truck",
                        video="https://www.youtube.com/watch?v=brownpanda_ranger",
                        footnotes=[Footnote.BROWNPANDA_SPEED],
                        min_steer_speed=10. * CV.MPH_TO_MS,
                        min_enable_speed=10. * CV.MPH_TO_MS),
      BrownPandaCarDocs("FORD RANGER Raptor 2024", "BrownPanda Off-Road",
                        min_steer_speed=5. * CV.MPH_TO_MS),
    ],
    CarSpecs(
      mass=2100,        # Mid-size pickup truck
      wheelbase=3.27,   # Long truck wheelbase
      steerRatio=17.0,  # Higher ratio for truck stability
      centerToFrontRatio=0.45,  # Front-heavy truck balance
      tireStiffnessFactor=0.90,  # All-terrain tire capability
      minSteerSpeed=10 * CV.KPH_TO_MS,  # Higher minimum for truck safety
      minEnableSpeed=10 * CV.KPH_TO_MS,
    ),
    dbc_dict('brownpanda', None),
    flags=BrownPandaFlags.ICE |
          BrownPandaFlags.TORQUE_CONTROL |
          BrownPandaFlags.LONG_CONTROL |
          BrownPandaFlags.CAMERA_RADAR |
          BrownPandaFlags.BLIND_SPOT_MONITOR |
          BrownPandaFlags.FORWARD_COLLISION_WARNING |
          BrownPandaFlags.ULTRASONIC,
  )


FW_VERSIONS = {}




DBC = {
  CAR.BYD_ATTO3: dbc_dict('brownpanda', None),
  CAR.DEEPAL_S05: dbc_dict('brownpanda', None),
  CAR.BYD_DOLPHIN: dbc_dict('brownpanda', None),
  CAR.FORD_RANGER: dbc_dict('brownpanda', None),
}