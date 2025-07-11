from setuptools import setup

package_name = 'waveshare_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'python-socketio'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='Navigation behaviours for Waveshare Robotaxi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wander_node = waveshare_navigation.wander_node:main',
            'goal_follower_node = waveshare_navigation.goal_follower_node:main',
            'rules_of_road_node = waveshare_navigation.rules_of_road_node:main',
            'route_receiver_node = waveshare_navigation.route_receiver_node:main',
            'telemetry_sender_node = waveshare_navigation.telemetry_sender_node:main',
        ],
    },
) 