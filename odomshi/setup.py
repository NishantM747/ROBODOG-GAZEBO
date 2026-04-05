from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'odomshi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required ament index entry
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install config files (THIS is what you need)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nishant',
    maintainer_email='nishantmiglani123@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_publisher = odomshi.odom:main',
            'imu_publisher = odomshi.imu:main',
            'relay = odomshi.relay:main'
        ],
    },
)
