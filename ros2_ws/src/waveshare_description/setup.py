from setuptools import setup

package_name = 'waveshare_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='URDF and robot description for WaveShare Robotaxi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
) 