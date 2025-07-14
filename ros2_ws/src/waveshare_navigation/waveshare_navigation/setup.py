from setuptools import setup

package_name = 'waveshare_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
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
            'path_sequencer_node = waveshare_navigation.path_sequencer_node:main',
            'arbiter_node = waveshare_navigation.arbiter_node:main',
            'teleop_socketio_node = waveshare_navigation.teleop_socketio_node:main',
        ],
    },
) 