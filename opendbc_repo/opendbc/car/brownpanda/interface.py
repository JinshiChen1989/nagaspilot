from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.brownpanda.carcontroller import CarController
from opendbc.car.brownpanda.carstate import CarState
from opendbc.car.brownpanda.values import CAR, BrownPandaFlags, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, np_params, docs) -> structs.CarParams:
    ret.brand = "brownpanda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]

    platform_config = candidate
    ret.flags = platform_config.flags if hasattr(platform_config, 'flags') else 0

    # Auto-update UI with detected model name
    if hasattr(candidate, 'name'):
      Params().put("np_device_model_selected", candidate.name)

    # Load vehicle specifications
    if hasattr(platform_config, 'specs'):
      specs = platform_config.specs
      ret.mass = specs.mass
      ret.wheelbase = specs.wheelbase
      ret.steerRatio = specs.steerRatio
      ret.centerToFrontRatio = getattr(specs, 'centerToFrontRatio', 0.4)
      ret.tireStiffnessFactor = getattr(specs, 'tireStiffnessFactor', 0.7)
      ret.rotationalInertia = specs.mass * (specs.wheelbase * 0.5) ** 2
      ret.minSteerSpeed = getattr(specs, 'minSteerSpeed', 5.0)
      ret.minEnableSpeed = getattr(specs, 'minEnableSpeed', 5.0)
    else:
      # Fallback defaults
      ret.mass = 1700.
      ret.wheelbase = 2.72
      ret.steerRatio = 16.0
      ret.centerToFrontRatio = 0.42
      ret.tireStiffnessFactor = 0.7
      ret.rotationalInertia = 1600 * (2.7 * 0.5) ** 2
      ret.minSteerSpeed = 5.0
      ret.minEnableSpeed = 5.0

    ret.centerToFront = ret.wheelbase * ret.centerToFrontRatio
    ret.dashcamOnly = False

    # Control type configuration
    if ret.flags & BrownPandaFlags.TORQUE_CONTROL:
      ret.steerControlType = SteerControlType.torque
    else:
      ret.steerControlType = SteerControlType.angle

    # Actuator delays - EV responds faster than ICE
    if ret.flags & BrownPandaFlags.EV:
      ret.steerActuatorDelay = 0.08  # Fast EV steering response
    elif ret.flags & BrownPandaFlags.HYBRID:
      ret.steerActuatorDelay = 0.12  # Moderate hybrid response
    else:
      ret.steerActuatorDelay = 0.15  # Slower ICE response

    ret.steerLimitTimer = 0.4  # Standard steering limit timer

    # Longitudinal control
    ret.openpilotLongitudinalControl = bool(ret.flags & BrownPandaFlags.LONG_CONTROL)
    ret.stoppingControl = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)
    ret.startingState = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)

    # EV-specific parameters - EVs can stop/start more precisely
    if ret.flags & BrownPandaFlags.EV:
      ret.vEgoStopping = 0.3  # EV precise low-speed control
      ret.vEgoStarting = 0.3
      ret.stopAccel = -2.5    # EV regenerative braking
    else:
      ret.vEgoStopping = 0.5  # ICE less precise at low speeds
      ret.vEgoStarting = 0.5
      ret.stopAccel = -2.0    # Standard ICE braking

    ret.stoppingDecelRate = 0.8
    ret.maxSteeringAngleDeg = 1080

    # Longitudinal tuning per model
    if candidate == CAR.BYD_ATTO3:
      ret.longitudinalTuning.kf = 1.0
      ret.longitudinalTuning.kpBP = [0., 5., 15., 30.]
      ret.longitudinalTuning.kpV = [1.6, 1.1, 0.7, 0.4]
      ret.longitudinalTuning.kiBP = [0., 5., 15., 30.]
      ret.longitudinalTuning.kiV = [0.20, 0.15, 0.10, 0.06]
      ret.longitudinalActuatorDelayLowerBound = 0.15
      ret.longitudinalActuatorDelayUpperBound = 0.25
    elif candidate == CAR.DEEPAL_S05:
      ret.longitudinalTuning.kf = 1.2
      ret.longitudinalTuning.kpBP = [0., 8., 20., 35.]
      ret.longitudinalTuning.kpV = [2.0, 1.4, 0.9, 0.6]
      ret.longitudinalTuning.kiBP = [0., 8., 20., 35.]
      ret.longitudinalTuning.kiV = [0.30, 0.20, 0.14, 0.10]
      ret.longitudinalActuatorDelayLowerBound = 0.10
      ret.longitudinalActuatorDelayUpperBound = 0.18
    else:
      ret.longitudinalTuning.kf = 1.0
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.longitudinalActuatorDelayLowerBound = 0.20
      ret.longitudinalActuatorDelayUpperBound = 0.30

    # Lateral tuning
    if ret.steerControlType == SteerControlType.torque:
      ret.lateralTuning.init('torque')
      ret.lateralTuning.torque.useSteeringAngle = True
      ret.lateralTuning.torque.kf = 1.0
      ret.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0

      if ret.flags & BrownPandaFlags.EV:
        ret.lateralTuning.torque.kp = 1.2
        ret.lateralTuning.torque.ki = 0.12
        ret.lateralTuning.torque.friction = 0.08
        ret.lateralParams.torqueBP = [0., 8., 15., 25., 40.]
        ret.lateralParams.torqueV = [0., 60., 120., 250., 400.]
      else:
        ret.lateralTuning.torque.kp = 1.0
        ret.lateralTuning.torque.ki = 0.1
        ret.lateralTuning.torque.friction = 0.1
        ret.lateralParams.torqueBP = [0., 5., 10., 20., 30.]
        ret.lateralParams.torqueV = [0., 50., 100., 200., 300.]
    else:
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kf = 0.00008
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.]

      if ret.flags & BrownPandaFlags.EV:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.2, 0.35, 0.5]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.01, 0.02, 0.03]
      else:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.15, 0.25, 0.4]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.008, 0.015, 0.025]

    # Features
    ret.autoResumeSng = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)
    ret.enableBsm = bool(ret.flags & BrownPandaFlags.BLIND_SPOT_MONITOR)
    ret.enableApgs = False
    ret.notCar = False
    ret.enableDsu = False
    ret.enableGasInterceptor = False
    ret.experimentalLongitudinalAvailable = ret.openpilotLongitudinalControl
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret