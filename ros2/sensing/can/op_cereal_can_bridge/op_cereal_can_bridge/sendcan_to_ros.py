import rclpy
from rclpy.node import Node

from op_core_msgs.msg import CanData, CanDataArray

import cereal.messaging as messaging


class SendcanToROS(Node):
  def __init__(self):
    super().__init__('sendcan_to_ros')
    self.pub = self.create_publisher(CanDataArray, 'sendcan', 100)
    self.timer = self.create_timer(0.01, self.step)  # 100 Hz
    self.sm = messaging.SubMaster(['sendcan'])

  def step(self):
    self.sm.update(0)
    if not self.sm.valid['sendcan']:
      return
    evt = self.sm['sendcan']
    frames = []
    for f in evt:
      cd = CanData()
      cd.address = int(f.address)
      cd.dat = list(bytes(f.dat))
      cd.src = int(f.src)
      frames.append(cd)
    if frames:
      arr = CanDataArray()
      arr.frames = frames
      self.pub.publish(arr)


def main():
  rclpy.init()
  node = SendcanToROS()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

