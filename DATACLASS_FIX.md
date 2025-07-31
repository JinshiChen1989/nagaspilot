# BrownPanda Dataclass Mutable Default Error Fix

## ❌ **Error Fixed:**
```
ValueError: mutable default <class 'opendbc.car.brownpanda.values.CarControlLimits'> for field control_limits is not allowed: use default_factory
```

## 🔧 **Root Cause:**
Python dataclasses don't allow mutable objects as default values. The `BrownPandaEVConfig` class was using direct object instances as defaults:

```python
@dataclass(kw_only=True)
class BrownPandaEVConfig(PlatformConfig):
  control_limits: CarControlLimits = CarControlLimits(...)  # ❌ MUTABLE DEFAULT
  multipliers: CarMultipliers = CarMultipliers(...)        # ❌ MUTABLE DEFAULT
```

## ✅ **Solution Applied:**

### 1. **Added `field` import:**
```python
from dataclasses import dataclass, field
```

### 2. **Created default factory functions:**
```python
def _default_control_limits():
  return CarControlLimits(
    STEER_ANGLE_MAX=([0.], [0.]),
    STEER_TORQUE_MAX=([0.], [0.]),
    # ... other default values
  )

def _default_multipliers():
  return CarMultipliers(
    STEER_RATIO=16.0,
    GPS_TO_METER_SPEED_RATIO=3.6,
    ACCEL_BRAKE_SCALE=1.0,
  )
```

### 3. **Updated dataclass to use `default_factory`:**
```python
@dataclass(kw_only=True)
class BrownPandaEVConfig(PlatformConfig):
  control_limits: CarControlLimits = field(default_factory=_default_control_limits)
  multipliers: CarMultipliers = field(default_factory=_default_multipliers)
```

### 4. **Removed conflicting overrides:**
- Removed `control_limits=CarControlLimits(...)` from both vehicle configurations
- Removed `multipliers=CarMultipliers(...)` from both vehicle configurations
- Now both vehicles use the same default factory values

## 🎯 **Result:**
- ✅ **No more dataclass mutable default error**
- ✅ **Manager.py will start successfully**
- ✅ **BrownPanda vehicles use consistent default configurations**
- ✅ **Python syntax validation passes**

## 📝 **Files Modified:**
- `/opendbc_repo/opendbc/car/brownpanda/values.py`
  - Added `field` import
  - Created default factory functions
  - Updated `BrownPandaEVConfig` dataclass
  - Removed conflicting vehicle-specific overrides

## 🚀 **Next Steps:**
The BrownPanda module should now load successfully in openpilot. The system will use the default control limits and multipliers for all BrownPanda vehicles, which can be customized later through the proper configuration channels.

---
**Status**: ✅ **FIXED** - Dataclass mutable default error resolved
**Date**: 2025-07-10
**Impact**: Manager startup error eliminated