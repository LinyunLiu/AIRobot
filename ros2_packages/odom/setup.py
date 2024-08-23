from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'ekf'), glob('ekf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airobot',
    maintainer_email='airobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapToOdomTF = odom.mapToOdomTF:main',
            'initialPoseSetter = odom.initialPoseSetter:main',
            'teleop_cmd_vel = odom.teleop_cmd_vel:main',
            'testOdom = odom.testOdom:main',
            'cmd_vel_controller = odom.cmd_vel_controller:main',
            'LaserToBase = odom.LaserToBase:main',
            'bno = odom.bno:main',
            'left_wheel = odom.leftWheel:main',
            'right_wheel = odom.rightWheel:main',
            'left_wheel2 = odom.leftWheel2:main',
            'right_wheel2 = odom.rightWheel2:main',
            'odom_publisher = odom.odomPublisher:main',
            'odom_publisher2 = odom.odomPublisher2:main',
            'teleopWheel = odom.teleopWheel:main',
            'icp = odom.icp:main',
        ],
    },
)
