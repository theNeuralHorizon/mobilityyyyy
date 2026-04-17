from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'artpark_decision'

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
    description='State machine + edge sampler + tile tracker.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_machine=artpark_decision.state_machine:main',
            'tile_tracker=artpark_decision.tile_tracker:main',
        ],
    },
)
