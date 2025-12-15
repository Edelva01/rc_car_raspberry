from setuptools import setup

package_name = 'rc_truck_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edelva01',
    maintainer_email='Edelva01@users.noreply.github.com',
    description='Perception pipeline nodes for the RC truck.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_pipeline_node = rc_truck_perception.vision_pipeline_node:main',
            'obstacle_map_node = rc_truck_perception.obstacle_map_node:main',
        ],
    },
)
