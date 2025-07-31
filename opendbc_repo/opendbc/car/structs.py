import os
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

class NPFlags:
  # Longitudinal Control Flags (0-15)
  ACM = 1                    # Adaptive Cruise Module
  ACM_DOWNHILL = 2          # ACM downhill mode
  AEM = 4                   # Adaptive Engagement Module
  
  # Lateral Control Flags (16-31)
  LateralALKA = 16          # Lateral Always Keep Lane Assist
  
  # Car-Specific Flags (32-255)
  ExtRadar = 32             # External radar support
  

# Replace DPFlags with NPFlags
DPFlags = NPFlags
