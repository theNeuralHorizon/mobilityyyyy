from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'artpark_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kshitij',
    maintainer_email='kshitij.betwal@gmail.com',
    description='Run logger: scorecard CSV + thought JSONL + images.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'logger_node=artpark_logger.logger_node:main',
        ],
    },
)
