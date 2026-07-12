import os
from glob import glob
from setuptools import setup

package_name = 'solution_duz_fgm'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını kur
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmet Efe Sensoy',
    maintainer_email='ahmetefesensoy2011@gmail.com',
    description='Düz Follow the Gap Method (FGM) + disparity extender reaktif kontrolcü (ROS 2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run solution_duz_fgm reactive_robot
            'reactive_robot = solution_duz_fgm.reactive_robot:main',
        ],
    },
)
