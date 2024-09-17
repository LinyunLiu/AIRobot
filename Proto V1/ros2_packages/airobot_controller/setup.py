from setuptools import find_packages, setup

package_name = 'airobot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',],
    zip_safe=True,
    maintainer='airobot',
    maintainer_email='airobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_publisher = airobot_controller.teleop_publisher:main',
            'motor_subscriber = airobot_controller.motor_subscriber:main',
            'motor_subscriber2 = airobot_controller.motor_subscriber2:main',
        ],
    },
)
