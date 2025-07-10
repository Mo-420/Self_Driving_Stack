from setuptools import setup

package_name = 'waveshare_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='Ultrasonic safety node for Waveshare Robotaxi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_safety_node = waveshare_safety.ultrasonic_safety_node:main'
        ],
    },
) 