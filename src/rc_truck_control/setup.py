from setuptools import setup

package_name = 'rc_truck_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='todo@example.com',
    description='ROS 2 control nodes for an RC truck running on a Raspberry Pi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_node = rc_truck_control.throttle_node:main',
            'servo_node = rc_truck_control.servo_node:main',
            'sensor_node = rc_truck_control.sensor_node:main',
        ],
    },
)
