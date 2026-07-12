from setuptools import setup

package_name = 'reactive_robot_common'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ahmet Efe Sensoy',
    maintainer_email='ahmetefesensoy2011@gmail.com',
    description='Solution\'lar arasında paylaşılan saf-Python yardımcılar (feature_utils, nn, barn_sim).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
