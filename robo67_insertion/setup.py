from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robo67_insertion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robo67',
    maintainer_email='team@robo67.local',
    description='Classical vision + force peg-in-hole insertion for the Franka Panda.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'socket_detector = robo67_insertion.nodes.socket_detector_node:main',
            'calibration = robo67_insertion.nodes.calibration_node:main',
            'insertion_orchestrator = robo67_insertion.nodes.insertion_orchestrator_node:main',
            'd405_servo = robo67_insertion.nodes.d405_servo_node:main',
        ],
    },
)
