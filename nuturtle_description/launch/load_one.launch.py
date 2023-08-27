from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.actions import Shutdown, SetLaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='color',
            default_value='purple',
            choices=['purple', 'red', 'green', 'blue'],
            description='controls the color of the base of the robot'
            ),

        SetLaunchConfiguration(
            name='rviz_config',
            value=['config/basic_', LaunchConfiguration('color'), '.rviz']
            ),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='controls whether rviz is launched'
            ),

        DeclareLaunchArgument(
            name='link',
            default_value='base_link',
            choices=['base_link', 'base_scan', 'caster_back_link', 'imu_link',
                     'wheel_left_link', 'wheel_right_link', 'base_footprint'],
            description='set the fixed frame in rviz'
            ),

        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description='controls whether the joint_state_publisher is used'
            ),

        Node(
            package="rviz2",
            namespace=LaunchConfiguration('color'),
            executable="rviz2",
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare(
                "nuturtle_description"), LaunchConfiguration('rviz_config')]),
                '-f', PathJoinSubstitution([LaunchConfiguration('color'),
                                            LaunchConfiguration('link')])],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
            ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration('color'),
            parameters=[
                {"robot_description":
                    Command([ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"),
                            "urdf/turtlebot3_burger.urdf.xacro"]),
                        " color:=",
                        LaunchConfiguration('color')
                        ]), 'frame_prefix': PathJoinSubstitution(
                        [LaunchConfiguration('color'), ' '])}
                        ]
            ),

        Node(
            package="joint_state_publisher",
            namespace=LaunchConfiguration('color'),
            executable="joint_state_publisher",
            condition=LaunchConfigurationEquals('use_jsp', 'true')
            )

    ])
