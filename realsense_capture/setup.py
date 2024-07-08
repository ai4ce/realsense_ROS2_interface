import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'realsense_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Irving Fang',
    maintainer_email='irving.fang@nyu.edu',
    description='Interfaces the robot/joystick with the realsense ROS wrapper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_image_server = realsense_capture.realsense_image_server:main',
            'realsense_image_client = realsense_capture.realsense_image_client:main',
        ],
    },
)
