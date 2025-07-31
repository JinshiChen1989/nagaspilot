from opendbc.car import CanBusBase


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP if fingerprint is None else None, fingerprint)
    
    # Brown Panda uses a simple single-bus architecture
    self._pt = self.offset
    self._radar = self.offset
    self._lkas = self.offset

  @property
  def pt(self) -> int:
    """Powertrain bus"""
    return self._pt

  @property
  def radar(self) -> int:
    """Radar bus"""
    return self._radar

  @property
  def lkas(self) -> int:
    """Lane keeping assist bus"""
    return self._lkas