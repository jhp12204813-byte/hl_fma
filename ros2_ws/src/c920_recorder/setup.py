from setuptools import setup
setup(name='c920_recorder', version='0.1.0', packages=['c920_recorder'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/c920_recorder']),
                  ('share/c920_recorder', ['package.xml', 'plugin.xml', 'README.md', 'VALIDATION.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='idp2', maintainer_email='idp2@example.com',
      description='Independent ROS RGB recorder for Logitech C920', license='Apache-2.0',
      entry_points={'console_scripts': ['record = c920_recorder.runner:main',
                                       'verify = c920_recorder.verification:main']})
