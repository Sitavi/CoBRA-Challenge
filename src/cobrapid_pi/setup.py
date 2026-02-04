from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cobrapid_pi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PIDteam',
    maintainer_email='PIDteam@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = cobrapid_pi.publisher_member_function:main',
                'apriltag = cobrapid_pi.apriltag_node:main',
                'camera_tag_pose_broadcaster = cobrapid_pi.camera_tag_pose_broadcaster:main',
                'control = cobrapid_pi.control:main',
        ],
    },
)
