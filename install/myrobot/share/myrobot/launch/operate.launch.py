from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_name = 'myrobot'
    urdf_file = PathJoinSubstitution([
        FindPackageShare('myrobot'),
        'urdf',
        'robot.urdf.xacro'
    ])
    robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_file
        ]),
        value_type=str
    )
    controller_file = PathJoinSubstitution([
        FindPackageShare('myrobot'),
        'config',
        'controller.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description}]),
        Node(
            package=package_name,
            executable='send_move',
            name='send_move',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description}, 
                controller_file]),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50','--param-file', controller_file],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller', '--controller-manager-timeout', '50','--param-file', controller_file],
                    output='screen'
                )
            ]
        ),

    ])