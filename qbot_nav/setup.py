#setup.py

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'qbot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Install config folder (SLAM params)
        (os.path.join('share', package_name, 'config'), glob(
            os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdul-rafi',
    maintainer_email='rafiabdul7128@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_server = qbot_nav.navigation_server:main',
            'qbot_controller = qbot_nav.qbot_controller:main',
            'slam_pose_publisher = qbot_nav.slam_pose_publisher:main',
        ],
    },
)