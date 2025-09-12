from setuptools import setup

package_name = 'op_controls_adapter'

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
    description='Adapter node to publish ControlsCommand from existing carControl (cereal) into ROS 2 using rclpy.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controlsd_to_ros = op_controls_adapter.controlsd_to_ros:main',
        ],
    },
)

