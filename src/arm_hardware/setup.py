import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Bridge from JointTrajectory to servo motors via a microcontroller.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hardware_bridge = arm_hardware.hardware_bridge:main',
        ],
    },
)
