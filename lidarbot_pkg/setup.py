import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidarbot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nectar A. Gonzalez Negron',
    maintainer_email='nectargonzalez@outlook.com',
    description='Autonomous lidar robot program for the NRC 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'encoder_node = lidarbot_pkg.encoder_node:main',
            'lidar_node = lidarbot_pkg.lidar_node:main',
            'localization_node = lidarbot_pkg.localization_node:main',
            'navigation_node = lidarbot_pkg.navigation_node:main',
        ],
    },
)
