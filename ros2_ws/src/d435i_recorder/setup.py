from setuptools import setup
setup(name='d435i_recorder', version='0.1.0', packages=['d435i_recorder'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/d435i_recorder']),
                  ('share/d435i_recorder', ['package.xml', 'plugin.xml', 'README.md', 'VALIDATION.md', 'OVERFLOW_DIAGNOSIS.md', 'AB_MEASUREMENT.md', 'VISION_REPLAY.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='idp2', maintainer_email='idp2@example.com',
      description='Standalone D435i data recorder', license='Apache-2.0',
      entry_points={'console_scripts': ['record = d435i_recorder.runner:main',
                                       'vision_replay = d435i_recorder.vision_replay:main']})
