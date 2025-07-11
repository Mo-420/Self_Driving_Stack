from setuptools import setup

package_name = 'waveshare_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'odom_publisher_node = waveshare_base.odom_publisher_node:main'
        ],
    },
) 