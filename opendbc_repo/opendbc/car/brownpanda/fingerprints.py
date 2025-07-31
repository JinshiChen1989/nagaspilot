from opendbc.car.brownpanda.values import CAR

# Simple authenticator detection - just check if the CAN message exists
AUTHENTICATOR_MESSAGES = {
  1808: CAR.BYD_ATTO3,     # BYD ATTO3 authenticator
  1809: CAR.BYD_DOLPHIN,   # BYD DOLPHIN authenticator  
  1856: CAR.DEEPAL_S05,    # DEEPAL S05 authenticator
  2033: CAR.FORD_RANGER,   # FORD RANGER authenticator
}

def get_car_from_authenticator(can_msgs):
  """
  Simple brownpanda-style authenticator detection.
  Just check if authenticator CAN message is present - no complex parsing.
  """
  for msg in can_msgs:
    if msg.address in AUTHENTICATOR_MESSAGES:
      return AUTHENTICATOR_MESSAGES[msg.address], {}
  
  return None, None

FW_VERSIONS = {}

FINGERPRINTS = {
  CAR.BYD_ATTO3: [
    {96: 6, 97: 8, 98: 8, 99: 8, 100: 6, 101: 2, 105: 6, 1808: 8},  # Core + Authenticator
  ],
  CAR.BYD_DOLPHIN: [
    {96: 6, 97: 8, 98: 8, 99: 8, 100: 6, 101: 2, 105: 6, 1809: 8},  # Core + Authenticator
  ],
  CAR.DEEPAL_S05: [
    {96: 6, 97: 8, 98: 8, 99: 8, 100: 6, 101: 2, 105: 6, 1856: 8},  # Core + Authenticator
  ],
  CAR.FORD_RANGER: [
    {96: 6, 97: 8, 98: 8, 99: 8, 100: 6, 101: 2, 105: 6, 2033: 8},  # Core + Authenticator
  ],
}