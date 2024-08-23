# AIRobot

A teleop robot without much AI

# ROS2

<details><summary>Required External Repositries </summary>

RPLIDAR A1 https://index.ros.org/r/rplidar_ros/ 

- GitHub page Readme.md is the instruction for ROS and ROS2. Follow ros.org.

Lidar Odometry
https://github.com/MAPIRlab/rf2o_laser_odometry

BNO055 (IMU)
https://github.com/process1183/ros2_bno055

</details>
<details><summary>Required ROS2 packages</summary>

- nav2 https://docs.nav2.org/ and turtlebot3
    ```bash
    sudo apt install -y ros-humble-navigation2
    sudo apt install -y ros-humble-nav2-bringup
    sudo apt install -y ros-humble-turtlebot3*
    ```
</details>
<details><summary>How to install ros2 packages</summary>

- create workspace folder 
    ```bash
    mkdir ros2_ws
    mkdir ros2_ws/src
    #copy folders of 
    cd ros2_ws
    colcon build 
    ```
- put below in the bottom of ~/.bashrc 

        source [path_to]/ros2_ws/install/setup.bash
        export TURTLEBOT3_MODEL=waffle # if you are using turtlebot3 packages

</details>
<details><summary>ROS2 tips </summary>
- ROS2 humble Tutorial: https://www.youtube.com/watch?v=Gg25GfA456o

- create workspace folder 
    ```bash
    mkdir ros2_ws
    mkdir ros2_ws/src
    cd ros2_ws
    colcon build
    ``` 
- create package that is using python 
    ```bash
    cd src
    ros2 pkg create airobot_controller --build-type ament_python --dependencies rclpy
    ```
- in .bashrc source setup.bash path

        source [path_to]/ros2_ws/install/setup.bash # make sure it is the right path

- when create new file 

        put in setup.py >> items of 'consol_scripts'
            "name = pkgname.pyfile:main", 
    ex) portion of setup.py
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


                
- at ros2_ws need it when ever new ros2 python file is created

    '--symlink-install' let your package auto-sync when edit the .py file 

    ```bash
    colcon build --symlink-install 
    ```
- build specific package 

    ```bash
    colcon build --symlink-install --packages-select <Package_Name>
    ``` 

- ERROR FIXING: if setup.py install is deprecated
    ```bash
    sudo apt install python3-pip
    pip3 list |grep setuptools
    #if the version is not 58.2.0
    pip3 install setuptools==58.2.0
    ```
- if you use another package in a package

    put \<depend>packageName\</depend> in package.xml 
    

    ex)
    ```python
    <depend>geometry_msgs</depend>
    ```
- ros2 investigate
    ```bash
    ros2 topic list
    ros2 topic info <topicName>
    ros2 interface show <Type from above>
    ```
- check frames
    ```bash
    ros2 run tf2_tools view_frames
    ```
- To add 'launch' dir (or any dir with different name)

	in package.xml

    \<depend>launch\</depend>

    \<depend>launch_ros\</depend>

	in setup.py

    ```python
    import os
    from glob import glob 
    data_files=[
        ...
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    ```

</details>
<details><summary>Run rviz2 with template</summary>
    
    ros2 run rviz2 rviz2 -d <path to .rviz file>

</details>
<details><summary>RpLiDAR A1</summary>
    
    ros2 launch rplidar_ros rplidar_a1_launch.py 

</details>
<details><summary>Cheak Connections between Frames </summary>
    
    ros2 run tf2_tools view_frames

</details>
<details><summary>SLAM</summary>
    
    ros2 launch odom slam_launch.py 

- To save the map from SLAM
    
        ros2 run nav2_map_server map_saver_cli -f ~/map

</details>
<details><summary>Nav2 Turtlebot3 simulation </summary>

- tutorial: https://roboticsbackend.com/ros2-nav2-tutorial/
- run each one in different terminal 
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True 

    ros2 run turtlebot3_teleop teleop_keyboard
    ```

- Nav2 using turtlebot packages
    ```bash
    ros2 launch odom nav2_launch.py 
    ros2 run odom cmd_vel_controller
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=[path_to]/hallway.yaml
    ```
</details>