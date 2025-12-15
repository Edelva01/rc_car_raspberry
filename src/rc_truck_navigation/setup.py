from setuptools import setup

package_name = 'rc_truck_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/navigation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edelva01',
    maintainer_email='Edelva01@users.noreply.github.com',
    description='Navigation and behavior nodes for the RC truck.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_manager_node = rc_truck_navigation.behavior_manager_node:main',
            'path_planner_node = rc_truck_navigation.path_planner_node:main',
        ],
    },
)
