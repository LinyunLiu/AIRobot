# AIRobot

A teleop robot without much AI

# Required External Repositries 


# How to install ros2 packages
-create workspace folder 
```bash
mkdir ros2_ws
mkdir ros2_ws/src
#copy folders of 
cd ros2_ws
colcon build 
```

put below in the bottom of ~/.bashrc 

    source [path_to]/ros2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=waffle

# ro2 tips

- create workspace folder 

        mkdir ros2_ws
        mkdir ros2_ws/src
        cd ros2_ws
        colcon build 
- create package using python (there is option for c++ but not used)

        cd src
        ros2 pkg create airobot_controller --build-type ament_python --dependencies rclpy

- in .bashrc source setup.bash path

        source [path_to]/ros2_ws/install/setup.bash # make sure it is the right path

- when create new file 

        put in setup.py >> items of 'consol_scripts'
            "name = pkgname.pyfile:main", 
    ex)
    ```python
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


                
# at ros2_ws need it when ever new ros2 python file is created
# --symlink-install let your package auto-sync when edit the .py file 
colcon build --symlink-install 

# build specific package 
colcon build --symlink-install --packages-select <Package_Name>

# if setup.py install is deprecated
sudo apt install python3-pip
pip3 list |grep setuptools
#if the version is not 58.2.0
pip3 install setuptools==58.2.0

# if you use another package in a package
put below in package.xml 
	<depend>packageName</depend>
	ex)
		<depend>geometry_msgs</depend>

# investigate
ros2 topic list
ros2 topic info <topicName>
ros2 interface show <Type from above>

# check frames
ros2 run tf2_tools view_frames

# To add 'launch' dir (or any dir with different name)
	in package.xml
		<depend>launch</depend>
		<depend>launch_ros</depend>
	in setup.py
		import os
		from glob import glob 
		data_files=[
			...
			(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
		],

