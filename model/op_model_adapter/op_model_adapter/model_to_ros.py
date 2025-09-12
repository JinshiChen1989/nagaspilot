import rclpy
from rclpy.node import Node

from op_core_msgs.msg import ModelAction

import cereal.messaging as messaging
from cereal import log


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


class ModelToROS(Node):
  def __init__(self):
    super().__init__('model_to_ros')
    self.pub_action = self.create_publisher(ModelAction, 'model_action', 20)
    self.timer = self.create_timer(0.01, self.step)  # 100 Hz
    self.sm = messaging.SubMaster(['modelV2'])

  def step(self):
    self.sm.update(0)
    if not self.sm.valid['modelV2']:
      return
    mv2 = self.sm['modelV2']
    act = mv2.action
    msg = ModelAction()
    msg.desired_curvature = float(act.desiredCurvature)
    msg.desired_acceleration = float(act.desiredAcceleration)
    msg.should_stop = bool(act.shouldStop)
    msg.lane_change_state = int(mv2.meta.laneChangeState)
    msg.lane_change_direction = int(mv2.meta.laneChangeDirection)
    self.pub_action.publish(msg)


def main():
  rclpy.init()
  node = ModelToROS()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

