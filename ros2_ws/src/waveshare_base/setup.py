from setuptools import setup
import os
from glob import glob

package_name = 'waveshare_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='ROS2 driver for Waveshare Robotaxi chassis',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = waveshare_base.base_node:main',
            'odom_publisher_node = waveshare_base.odom_publisher_node:main',
            'encoder_odom_node = waveshare_base.encoder_odom_node:main'
        ],
    },
) 