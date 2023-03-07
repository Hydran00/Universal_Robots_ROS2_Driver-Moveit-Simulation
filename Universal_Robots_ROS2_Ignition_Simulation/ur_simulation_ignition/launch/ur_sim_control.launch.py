# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
import xacro,os
from launch import LaunchDescription,LaunchContext
from launch.substitutions import PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler
)
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    ur_type = LaunchConfiguration('ur_type').perform(context)
    
    safety_limits = LaunchConfiguration('safety_limits').perform(context)
    safety_pos_margin = LaunchConfiguration("safety_pos_margin").perform(context)
    safety_k_position = LaunchConfiguration("safety_k_position").perform(context)
    # General arguments

    runtime_config_package = LaunchConfiguration("runtime_config_package").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    #description_package =LaunchConfiguration("description_package").perform(context)
    description_package = get_package_share_directory('ur_description')
    description_file = LaunchConfiguration("description_file").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    start_joint_controller = LaunchConfiguration("start_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    #initial_joint_controller = PathJoinSubstitution(
    #    [FindPackageShare(runtime_config_package), "config", controllers_file]
    #)
    #
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    
    initial_joint_controllers = os.path.join(get_package_share_directory(runtime_config_package), "config", controllers_file)

    print("AAAA "+str(initial_joint_controllers))
    rviz_config_file = os.path.join((description_package), "rviz", "view_robot.rviz")

    '''
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    '''
    
    xacro_path = os.path.join(description_package,"urdf",description_file)
    #PathJoinSubstitution([FindPackageShare(description_package),"urdf",description_file])
    robot_description_content = xacro.process_file(xacro_path, mappings={"safety_limits":safety_limits,"safety_pos_margin":safety_pos_margin,
                                                                        "safety_k_position":safety_k_position,
                                                                        "name":"ur","ur_type":ur_type,
                                                                        "prefix":'',"sim_ignition":"true","simulation_controllers":initial_joint_controllers})
    robot_description_content = robot_description_content.toprettyxml(indent=' ')
    
    #parsing xacro file and saving sdf
    with open('robot_description.sdf', 'w+') as f:
        f.write(robot_description_content)
        print(f.read())
        #loading sdf from xacro file
        ignition_robot_spawner = Node(
            package="ros_ign_gazebo",
            executable="create",
            output="screen",
            arguments=[
                " -file ",
                "robot_description.sdf" #POSITION OF ROBOT IS SET IN ur_descritpion/urdf/ur.urdf.xacro
            ]
        )
    #add table to the simulation
    table_sdf_path = os.path.join(get_package_share_directory('ur_simulation_ignition'),'table','table.sdf')
    ignition_table_spawner = Node(
            package="ros_ign_gazebo",
            executable="create",
            output="screen",
            arguments=[
                " -file ",
                table_sdf_path
         ]
    )

    robot_description = {"robot_description": robot_description_content}

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d ", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )
    
    forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['forward_position_controller', "-c", "/controller_manager"],
    )
    '''
    forward_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['forward_position_controller', "-c", "/controller_manager", "--stopped"],
    )'''

    # Ignition nodes
    '''ignition_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=['-name', 'ur5e' ,'-z', '1.4',
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )
    '''

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_ign_gazebo"), "/launch/ign_gazebo.launch.py"]
        ),
        launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
    )
    
    # static Robot state publisher
    static_transform_publisher=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.79",
                        "3.14", "0.0", "3.14",
                        "world", "base_link"])
    
    nodes_to_start = [
        static_transform_publisher,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        ignition_launch_description,
        ignition_robot_spawner,
        ignition_table_spawner,
        forward_controller_spawner
        #forward_controller_spawner_stopped
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_ignition",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    '''
        # Initialize Arguments
    safety_limits = LaunchConfiguration("safety_limits")
    ur_type = LaunchConfiguration("ur_type")
    
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")

    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    '''
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
