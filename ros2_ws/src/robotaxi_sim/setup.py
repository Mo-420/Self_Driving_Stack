from setuptools import setup, find_packages

package_name = 'robotaxi_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotaxi',
    maintainer_email='robotaxi@example.com',
    description='Launch Gazebo with robotaxi and start full stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
) 