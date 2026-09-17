from setuptools import find_packages, setup
from glob import glob

package_name = 'fma_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idp2',
    maintainer_email='idp2@todo.todo',
    description='Perception components for the FMA autonomous vehicle.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'c920_camera_node = fma_perception.c920_camera_node:main',
        'lane_detector = fma_perception.lane_detector_node:main',
        'lane_debug_snapshot = fma_perception.lane_debug_snapshot:main',
        'lane_media_replay = fma_perception.lane_media_replay:main',
        'obstacle_detection = fma_perception.obstacle_detection_node:main',
        'signal_car_detection = fma_perception.signal_car_detection_node:main',
    ]},
)
