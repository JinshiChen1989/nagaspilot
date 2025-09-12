import rclpy
from rclpy.node import Node

from op_core_msgs.msg import ControlsCommand, ControlsDebug

import cereal.messaging as messaging


class ControlsToROS(Node):
  def __init__(self):
    super().__init__('controlsd_to_ros')
    self.pub_cmd = self.create_publisher(ControlsCommand, 'controls_command', 20)
    self.pub_dbg = self.create_publisher(ControlsDebug, 'controls_debug', 20)
    self.timer = self.create_timer(0.02, self.step)  # 50 Hz typical
    self.sm = messaging.SubMaster(['carControl', 'controlsState'])

  def step(self):
    self.sm.update(0)
    if self.sm.valid['carControl']:
      cc = self.sm['carControl']
      msg = ControlsCommand()
      msg.enabled = bool(cc.enabled)
      msg.lat_active = bool(cc.latActive)
      msg.long_active = bool(cc.longActive)
      msg.curvature = float(cc.actuators.curvature)
      msg.accel = float(cc.actuators.accel)
      msg.torque = float(cc.actuators.torque)
      msg.steering_angle_deg = float(cc.actuators.steeringAngleDeg)
      msg.left_blinker = bool(cc.leftBlinker)
      msg.right_blinker = bool(cc.rightBlinker)
      self.pub_cmd.publish(msg)
    if self.sm.valid['controlsState']:
      cs = self.sm['controlsState']
      dbg = ControlsDebug()
      dbg.long_control_state = int(cs.longControlState)
      # Lateral union mapping
      lat = cs.lateralControlState
      which = lat.which()
      dbg.lateral_mode = which
      # common fields if present
      try:
        dbg.active = bool(getattr(lat, which).active)
      except Exception:
        dbg.active = False
      # Populate per-mode fields
      if which == 'pidState':
        s = lat.pidState
        dbg.steering_angle_deg = float(s.steeringAngleDeg)
        dbg.output = float(s.output)
        dbg.saturated = bool(s.saturated)
        dbg.angle_error = float(s.angleError)
        dbg.p = float(s.p)
        dbg.i = float(s.i)
        dbg.f = float(s.f)
      elif which == 'torqueState':
        s = lat.torqueState
        dbg.output = float(s.output)
        dbg.saturated = bool(s.saturated)
        dbg.error = float(s.error)
        dbg.error_rate = float(s.errorRate)
        dbg.p = float(s.p)
        dbg.i = float(s.i)
        dbg.d = float(s.d)
        dbg.f = float(s.f)
        dbg.desired_lateral_accel = float(s.desiredLateralAccel)
        dbg.actual_lateral_accel = float(s.actualLateralAccel)
      elif which == 'angleState':
        s = lat.angleState
        dbg.steering_angle_deg = float(s.steeringAngleDeg)
        dbg.output = float(s.output)
        dbg.saturated = bool(s.saturated)
      elif which == 'debugState':
        s = lat.debugState
        dbg.steering_angle_deg = float(s.steeringAngleDeg)
        dbg.output = float(s.output)
        dbg.saturated = bool(s.saturated)
      # add curvature info
      dbg.curvature = float(cs.curvature)
      dbg.desired_curvature = float(cs.desiredCurvature)
      self.pub_dbg.publish(dbg)


def main():
  rclpy.init()
  node = ControlsToROS()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()
