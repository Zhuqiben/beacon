import os
from glob import glob
from setuptools import setup

package_name = 'visualizer'

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
    description='Detection and map visualisation',
    license='MIT',
    url='https://github.com/Zhuqiben/beacon',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viz_node = visualizer.viz_node:main',
        ],
    },
)
