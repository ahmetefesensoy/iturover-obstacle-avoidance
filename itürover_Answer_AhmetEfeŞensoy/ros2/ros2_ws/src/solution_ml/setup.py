import os
from glob import glob
from setuptools import setup

package_name = 'solution_ml'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ahmet Efe Sensoy',
    maintainer_email='ahmetefesensoy2011@gmail.com',
    description='Behavioural Cloning + DQN ile eğitilmiş MLP politikasıyla reaktif sürüş; BARN veri toplama; Nav2 tabanlı expert navigasyon (ROS 2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ml_robot = solution_ml.ml_robot:main',
            'collect_data = solution_ml.collect_data:main',
            'dqn_finetune = solution_ml.dqn_finetune:main',
            'odom_to_tf = solution_ml.odom_to_tf:main',
            'shutdown_timer = solution_ml.shutdown_timer:main',
            'barn_goal_publisher = solution_ml.barn_goal_publisher:main',
        ],
    },
)
