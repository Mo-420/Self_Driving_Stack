from setuptools import setup

package_name = 'waveshare_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='Perception nodes (YOLO sign detection, lane segmentation) for Robotaxi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_sign_node = waveshare_perception.yolo_sign_node:main',
            'yolo_traffic_light_node = waveshare_perception.yolo_traffic_light_node:main',
            'lane_segmentation_node = waveshare_perception.lane_segmentation_node:main',
            'cam_socketio_bridge = waveshare_perception.cam_socketio_bridge:main',
            'yolo_crosswalk_node = waveshare_perception.yolo_crosswalk_node:main',
            'semantic_seg_node = waveshare_perception.semantic_seg_node:main',
        ],
    },
) 