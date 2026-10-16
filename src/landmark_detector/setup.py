import os
from glob import glob
from setuptools import setup

package_name = 'landmark_detector'

setup(
    name=package_name,
    version='0.0.1',
    # landmark_detector.scripts is a subpackage and must be listed here, otherwise
    # `ros2 run landmark_detector sample_color` fails with ModuleNotFoundError.
    packages=[package_name, package_name + '.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhuqiben',
    maintainer_email='Zhuqiben@users.noreply.github.com',
    description='Landmark colour detector (LifecycleNode)',
    license='MIT',
    url='https://github.com/Zhuqiben/beacon',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = landmark_detector.detector_node:main',
            'lifecycle_autostart = landmark_detector.lifecycle_autostart:main',
            'sample_color = landmark_detector.scripts.sample_color:main',
        ],
    },
)
