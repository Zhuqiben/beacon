import os
from glob import glob
from setuptools import setup

package_name = 'ekf_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhuqiben',
    maintainer_email='Zhuqiben@users.noreply.github.com',
    description='Online EKF-SLAM for landmark navigation',
    license='MIT',
    url='https://github.com/Zhuqiben/beacon',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = ekf_slam.slam_node:main',
        ],
    },
)
