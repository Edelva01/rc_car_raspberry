from setuptools import setup

package_name = 'rc_truck_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edelva01',
    maintainer_email='Edelva01@users.noreply.github.com',
    description='Hardware interface nodes for the RC truck.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esc_pwm_node = rc_truck_hw.esc_pwm_node:main',
            'servo_pwm_node = rc_truck_hw.servo_pwm_node:main',
            'range_sensor_node = rc_truck_hw.range_sensor_node:main',
            'camera_node = rc_truck_hw.camera_node:main',
        ],
    },
)
