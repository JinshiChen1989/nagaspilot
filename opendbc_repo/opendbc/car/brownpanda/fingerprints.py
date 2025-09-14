from opendbc.car.structs import CarParams
from opendbc.car.brownpanda.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.BYD_ATTO3: [{
    # Core vehicle messages (0x6E0-0x6EF) - all models should have these
    1760: 8, 1761: 8, 1762: 8, 1763: 8, 1764: 8, 1765: 8, 1766: 8, 1767: 8, 1768: 8, 1769: 8, 1770: 8, 1771: 8, 1772: 8, 1773: 8, 1774: 8, 1775: 8,
    # BYD brand marker
    1776: 8, 1777: 8, 1778: 8, 1779: 8
  }],
  CAR.BYD_DOLPHIN: [{
    # Core vehicle messages (0x6E0-0x6EF) - all models should have these
    1760: 8, 1761: 8, 1762: 8, 1763: 8, 1764: 8, 1765: 8, 1766: 8, 1767: 8, 1768: 8, 1769: 8, 1770: 8, 1771: 8, 1772: 8, 1773: 8, 1774: 8, 1775: 8,
    # BYD brand marker
    1776: 8, 1777: 8, 1778: 8, 1779: 8
  }],
  CAR.DEEPAL_S05: [{
    # Core vehicle messages (0x6E0-0x6EF) - all models should have these
    1760: 8, 1761: 8, 1762: 8, 1763: 8, 1764: 8, 1765: 8, 1766: 8, 1767: 8, 1768: 8, 1769: 8, 1770: 8, 1771: 8, 1772: 8, 1773: 8, 1774: 8, 1775: 8,
    # CHANGAN brand marker
    1776: 8, 1777: 8, 1778: 8, 1779: 8
  }],
}

FW_VERSIONS = {
  CAR.BYD_ATTO3: {
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'BYD_ATTO3_CAM_V1.0\x00\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'BYD_ATTO3_ECU_V1.0\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.BYD_DOLPHIN: {
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'BYD_DOLPHIN_CAM_V1.0\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'BYD_DOLPHIN_ECU_V1.0\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.DEEPAL_S05: {
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'DEEPAL_S05_CAM_V1.0\x00\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'DEEPAL_S05_ECU_V1.0\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}