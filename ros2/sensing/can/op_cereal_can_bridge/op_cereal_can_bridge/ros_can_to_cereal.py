import rclpy
from rclpy.node import Node

from op_core_msgs.msg import CanDataArray

import cereal.messaging as messaging
from cereal import log


class ROSCanToCereal(Node):
  def __init__(self):
    super().__init__('ros_can_to_cereal')
    self.sub = self.create_subscription(CanDataArray, 'can', self.on_can, 100)
    self.pm = messaging.PubMaster(['can'])

  def on_can(self, arr: CanDataArray):
    try:
      msg = messaging.new_message('can', len(arr.frames))
      for i, f in enumerate(arr.frames):
        msg.can[i].address = int(f.address)
        msg.can[i].dat = bytes(f.dat)
        msg.can[i].src = int(f.src)
      self.pm.send('can', msg)
    except Exception:
      # keep node alive on transient publish issues
      pass


def main():
  rclpy.init()
  node = ROSCanToCereal()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

