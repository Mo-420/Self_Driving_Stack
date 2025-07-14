from setuptools import setup

package_name = 'waveshare_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='SLAM-Toolbox launches for WaveShare Robotaxi',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
) 