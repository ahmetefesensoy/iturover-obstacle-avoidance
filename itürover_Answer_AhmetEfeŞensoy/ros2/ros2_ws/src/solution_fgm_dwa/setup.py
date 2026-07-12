import os
from glob import glob
from setuptools import setup

package_name = 'solution_fgm_dwa'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmet Efe Sensoy',
    maintainer_email='ahmetefesensoy2011@gmail.com',
    description='FGM + DWA reaktif kontrolcü (ROS 2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_robot = solution_fgm_dwa.reactive_robot:main',
        ],
    },
)
