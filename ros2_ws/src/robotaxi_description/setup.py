from setuptools import setup

package_name = 'robotaxi_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/urdf', ['urdf/robotaxi.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/test_track.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='URDF and Gazebo worlds for Robotaxi simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
) 