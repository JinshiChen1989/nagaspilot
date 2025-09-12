import os
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node

try:
  import can  # python-can
except Exception:  # pragma: no cover
  can = None

try:
  from op_core_msgs.msg import CanData, CanDataArray, PandaState
except Exception:
  # Fallback placeholders to allow static analysis without ROS build
  CanData = None  # type: ignore
  CanDataArray = None  # type: ignore
  PandaState = None  # type: ignore


class SocketCANBridge(Node):
  def __init__(self):
    super().__init__('op_can_bridge')

    # Parameters
    self.declare_parameter('ifaces', ['can0'])
    self.declare_parameter('iface_to_bus', {})  # e.g., {"can0": 0, "can1": 1}
    self.declare_parameter('freshness_sec', 1.0)
    self.declare_parameter('enable_tx', False)  # gate TX by default

    self.ifaces: List[str] = list(self.get_parameter('ifaces').get_parameter_value().string_array_value)
    self.iface_to_bus: Dict[str, int] = dict(self.get_parameter('iface_to_bus').get_parameter_value().string_value) if False else {}
    self.freshness_sec: float = float(self.get_parameter('freshness_sec').value)
    self.enable_tx: bool = bool(self.get_parameter('enable_tx').value)

    # Default mapping if none provided
    if not self.iface_to_bus:
      self.iface_to_bus = {name: idx for idx, name in enumerate(self.ifaces)}

    # Publishers/Subscribers
    self.pub_can = self.create_publisher(CanDataArray, 'can', 100)
    self.pub_panda = self.create_publisher(PandaState, 'panda_states', 10)
    self.sub_sendcan = self.create_subscription(CanDataArray, 'sendcan', self.on_sendcan, 100)

    # SocketCAN channels
    self.buses: Dict[str, any] = {}
    if can is None:
      self.get_logger().warning('python-can not available; running in dry mode (no CAN I/O).')
    else:
      for iface in self.ifaces:
        try:
          bus = can.Bus(interface='socketcan', channel=iface)
          self.buses[iface] = bus
          self.get_logger().info(f'Opened SocketCAN iface {iface}')
        except Exception as e:
          self.get_logger().error(f'Failed to open {iface}: {e}')

    # Timers
    self.create_timer(0.01, self.rx_step)   # 100 Hz receive publish
    self.create_timer(1.0, self.health_step)  # 1 Hz health

    # Stats
    self.tx_cnt = 0
    self.rx_cnt = 0
    self.tx_lost_cnt = 0
    self.rx_lost_cnt = 0

  def on_sendcan(self, msg: CanDataArray) -> None:
    if not self.enable_tx:
      return
    now = self.get_clock().now().nanoseconds
    # No per-frame timestamp in CanDataArray; rely on ROS header or freshness param overall
    if (can is None) or (not self.buses):
      return
    # Fan-out by src mapping
    for frame in msg.frames:
      try:
        iface = None
        for k, v in self.iface_to_bus.items():
          if v == frame.src:
            iface = k
            break
        if iface is None or iface not in self.buses:
          continue
        data_bytes = bytes(frame.dat)
        can_msg = can.Message(arbitration_id=int(frame.address), data=data_bytes, is_extended_id=(int(frame.address) > 0x7FF))
        self.buses[iface].send(can_msg)
        self.tx_cnt += 1
      except Exception:
        self.tx_lost_cnt += 1

  def rx_step(self) -> None:
    if (can is None) or (not self.buses):
      return
    frames: List[CanData] = []
    for iface, bus in self.buses.items():
      try:
        # Non-blocking read; drain small bursts
        while True:
          msg = bus.recv(timeout=0.0)
          if msg is None:
            break
          cd = CanData()
          cd.address = int(msg.arbitration_id)
          cd.dat = list(bytes(msg.data))
          cd.src = int(self.iface_to_bus.get(iface, 0))
          frames.append(cd)
      except Exception:
        self.rx_lost_cnt += 1
    if frames:
      arr = CanDataArray()
      arr.frames = frames
      self.pub_can.publish(arr)
      self.rx_cnt += len(frames)

  def health_step(self) -> None:
    # Publish minimal panda state emulation
    try:
      ps = PandaState()
      ps.controls_allowed = False
      ps.panda_type = 0
      ps.ignition = (self.rx_cnt > 0)
      ps.total_rx_cnt = self.rx_cnt
      ps.total_tx_cnt = self.tx_cnt
      ps.total_rx_lost_cnt = self.rx_lost_cnt
      ps.total_tx_lost_cnt = self.tx_lost_cnt
      self.pub_panda.publish(ps)
    except Exception:
      pass


def main():
  rclpy.init()
  node = SocketCANBridge()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

