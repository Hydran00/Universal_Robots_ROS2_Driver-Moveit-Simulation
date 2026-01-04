# Universal_Robots_ROS2_Driver-Moveit-Simulation
*Warning: This is no longer mantained!*  
This is a repository that unifies different official repository for ROS2 and the Universal Robot robots. Simulation with Ignition Gazebo is integrated. Both real robot and simulated ones work with Moveit2 (including moveit servo).

https://user-images.githubusercontent.com/93198865/223504480-c2a297b7-8528-48a2-80bd-3883b3e9e8fe.mp4


## Requirements:

1. [ROS2 Humble](https://docs.ros.org/en/humble/index.html). (I'm using Ubuntu 22.04) 
2. [Gazebo Ignition](https://gazebosim.org/docs) (Fortress Version)
3. ROS2 control and controllers:  
    ```sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers```
4. [MoveIt2](https://moveit.ros.org/install-moveit2/binary/)
5. Moveit2 Servo:  
    ```sudo apt-get install ros-humble-moveit-servo```


## Installation:
Launch moveit, moveit_servo and collisions.   
```
cd <your_ros2_ws>
git clone https://github.com/Hydran00/Universal_Robots_ROS2_Driver-Moveit-Simulation.git
mv Universal_Robots_ROS2_Driver-Moveit-Simulation src
colcon build
```

**IMPORTANT**: If you cannot build because you get error of the missing "SuppressWarning" library for gazebo, you have to comment the include lines in those files in the ros installation folders. Check the path of those files in the compilation error output.  

## Launch environment with real robot: 
Connect to the robot using drivers:  
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 launch_rviz:=false
```  
Open a new terminal and launch moveit, moveit_servo and collisions:  
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```

## Launch environment with simulated robot in Gazebo:  
``ros2 launch ur_simulation_ignition ur_sim_moveit.launch.py``

**You can obviously change the robot type (ur3e, ur5e,...)**

## Enabling servo mode:
To use moveit servo you have to check if the forward_position_controller is **active**. You can do it with:  
``ros2 control list_controllers``  
If it is **inactive** you can activate them using:
  - Real robot:  
``ros2 control switch_controllers --activate forward_position_controller --deactivate scaled_joint_trajectory_controller --strict``
  - Simulation:  
``ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller --strict``  
After doing that you will be able to control the robot by publishing messages through these topics:  
``/servo_node/delta_joint_cmds``   
``/servo_node/delta_twist_cmds``  

## Control robot in real time with moveit servo:
I added two ways of controlling robot similarly to the Moveit2 tutorial (take a look at the [documentation](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html?highlight=servo)).  
In a new terminal type:  
- Joypad control:  
  ``ros2 run joystick_controller joystick_controller``  
- Keyboard control:  
  ``ros2 run servo_interface ur_servo_keyboard_input_node``  
These two nodes publish messages in the two previously mentioned topics according to your input.
## Tips for recreating your own environment:
- If you don't want to use my planning scene you can:
  - Disable my collisions:  
  comment the part when I load the `` collision_node``  in the launch file `` ur_moveit.launch.py``  under ``ur_moveit_config/launch`` 
  - Create you own collisions:  
  modify `` collision_loader/src/add_collisions.cpp``  according to your preferences.
  - Change robot's base position and orientation:  
   modify tag ``origin`` in ``ur_description/urdf/ur.urdf.xacro`` 
  - Change or remove gripper:
  modify the end of ``ur_description/urdf/ur_macro.xacro`` and collisions in ``Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/ur_macro.srdf.xacro``



