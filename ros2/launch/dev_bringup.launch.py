from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
      package='op_can_bridge',
      executable='op_can_bridge_node',
      name='op_can_bridge',
      output='screen',
      parameters=[{
        'ifaces': ['can0'],
        'iface_to_bus': {'can0': 0},
        'freshness_sec': 1.0,
        'enable_tx': False,
      }]
    ),
    Node(
      package='op_cereal_can_bridge',
      executable='ros_can_to_cereal',
      name='ros_can_to_cereal',
      output='screen',
    ),
    Node(
      package='op_cereal_can_bridge',
      executable='sendcan_to_ros',
      name='sendcan_to_ros',
      output='screen',
    ),
    Node(
      package='op_model_adapter',
      executable='model_to_ros',
      name='model_to_ros',
      output='screen',
    ),
    Node(
      package='op_controls_adapter',
      executable='controlsd_to_ros',
      name='controlsd_to_ros',
      output='screen',
    ),
  ])
