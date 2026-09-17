from setuptools import find_packages, setup

package_name = 'fma_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idp2',
    maintainer_email='idp2@todo.todo',
    description='Odometry and GPS waypoint management for the FMA autonomous vehicle.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'f9p_front = fma_localization.f9p_front_node:main',
        'gps_controller = fma_localization.gps_controller_node:main',
        'dense_gps_controller = fma_localization.dense_gps_controller_node:main',
    ]},
)
