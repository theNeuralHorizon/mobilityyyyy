from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'artpark_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kshitij',
    maintainer_email='kshitij.betwal@gmail.com',
    description='Perception nodes: AprilTag vote, floor logo edge sampler, obstacle monitor.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag_handler=artpark_perception.apriltag_handler:main',
            'floor_logo_detector=artpark_perception.floor_logo_detector:main',
            'obstacle_monitor=artpark_perception.obstacle_monitor:main',
        ],
    },
)
