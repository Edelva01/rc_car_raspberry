from setuptools import setup

package_name = 'rc_truck_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/bringup.launch.py', 'launch/simulation.launch.py']),
        (f'share/{package_name}/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edelva01',
    maintainer_email='Edelva01@users.noreply.github.com',
    description='Bringup launch files for the RC truck.',
    license='MIT',
)