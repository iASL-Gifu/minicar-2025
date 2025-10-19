from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'back_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Back control node for minicar',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'back_control_node = back_control.back_control_node:main',
        ],
    },
)
