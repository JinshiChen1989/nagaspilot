from setuptools import setup

package_name = 'op_cereal_can_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nagasware',
    maintainer_email='dev@nagasware',
    description='Bridges between cereal CAN topics and ROS 2 (rclpy) topics for SocketCAN integration.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sendcan_to_ros = op_cereal_can_bridge.sendcan_to_ros:main',
            'ros_can_to_cereal = op_cereal_can_bridge.ros_can_to_cereal:main',
        ],
    },
)

