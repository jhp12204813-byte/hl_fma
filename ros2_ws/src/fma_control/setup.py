from glob import glob

from setuptools import find_packages, setup

package_name = 'fma_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/tools', ['../fma_perception/tools/replay_c920_lane.py']),
        ('share/' + package_name + '/config', ['../../../config/c920_bev_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idp2',
    maintainer_email='idp2@todo.todo',
    description='Control, command arbitration, and safety components for the FMA autonomous vehicle.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'competition_lane = fma_control.lane_stop_test_node:competition_main',
        'lane_stop_test = fma_control.lane_stop_test_node:main',
        'lane_follow = fma_control.lane_follow_node:main',
        'keyboard_teleop = fma_control.keyboard_teleop_node:main',
        'command_arbiter = fma_control.command_arbiter_node:main',
        'ramp_controller = fma_control.ramp_controller_node:main',
    ]},
)
