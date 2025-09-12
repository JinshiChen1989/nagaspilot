from setuptools import setup

package_name = 'op_can_bridge'

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
    description='SocketCAN bridge for RK3588 using ROS 2 rclpy, mapping to op_core_msgs.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'op_can_bridge_node = op_can_bridge.bridge_node:main',
        ],
    },
)

