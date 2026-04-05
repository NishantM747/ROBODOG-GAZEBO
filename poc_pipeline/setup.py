from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'poc_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
     	glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaravb',
    maintainer_email='emphiasarmlorn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'dog_command_node = poc_pipeline.dog_command_node:main',
        'serial_bridge_node = poc_pipeline.serial_bridge_node:main',
        ],
    },
)
