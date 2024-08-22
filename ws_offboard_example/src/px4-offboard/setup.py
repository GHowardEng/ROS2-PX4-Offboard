import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GH',
    maintainer_email='granthoward@uvic.ca',
    description='Example package for PX4 offboard control via ROS2',
    license='License Declaration',
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
        ],
    },
)
