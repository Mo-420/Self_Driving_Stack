from setuptools import setup

package_name = 'waveshare_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='Sensor driver nodes for WaveShare Robotaxi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno085_node = waveshare_sensors.bno085_node:main',
        ],
    },
) 