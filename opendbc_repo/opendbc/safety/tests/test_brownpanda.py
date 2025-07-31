#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda


class TestBrownpanda(common.PandaSafetyTest):
  TX_MSGS = [[0x6A, 0], [0x6B, 0], [0x6C, 0], [0x6D, 0]]  # Brownpanda CAN IDs from DBC
  FWD_BUS_LOOKUP = {}

  def setUp(self):
    self.packer = CANPackerPanda("brownpanda")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.body, 0)  # Using body safety model as fallback
    self.safety.init_tests()

  def _car_state_msg(self, vehicle_speed, gear_position):
    values = {"vehicleSpeed": vehicle_speed, "gearPosition": gear_position}
    return self.packer.make_can_msg_panda("carState", 0, values)

  def _user_state_msg(self, steer_angle, steer_torque):
    values = {"steerAngle": steer_angle, "steerTorque": steer_torque}
    return self.packer.make_can_msg_panda("userState", 0, values)

  def _steer_command_msg(self, steer_angle, steer_torque):
    values = {"cmdSteerAngle": steer_angle, "cmdSteerTorque": steer_torque}
    return self.packer.make_can_msg_panda("steerCommand", 0, values)

  def _act_command_msg(self, pedal_acc, pedal_brk):
    values = {"cmdPedalAcc": pedal_acc, "cmdPedalBrk": pedal_brk}
    return self.packer.make_can_msg_panda("actCommand", 0, values)

  def test_rx_hook(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_vehicle_moving())

    # Test basic message reception
    self.assertTrue(self._rx(self._car_state_msg(0, 0)))
    self.assertTrue(self._rx(self._user_state_msg(0, 0)))

  def test_tx_hook(self):
    # Test that commands are blocked when controls not allowed
    self.assertFalse(self._tx(self._steer_command_msg(0, 0)))
    self.assertFalse(self._tx(self._act_command_msg(0, 0)))
    
    # Enable controls and test again
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._steer_command_msg(0, 0)))
    self.assertTrue(self._tx(self._act_command_msg(0, 0)))

  def test_basic_safety(self):
    # Test basic safety functionality
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.safety.get_controls_allowed())
    
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.get_controls_allowed())


if __name__ == "__main__":
    unittest.main()