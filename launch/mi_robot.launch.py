import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'dle_pista.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_description = Command(['xacro ', xacro_path])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v4', world_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                        'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'mi_robot', '-z', '0.1']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'rvizconfig.rviz')],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='mi_robot_diferencial',
            executable='cinematica_ruedas.py',
            name='cinematica_ruedas',
            output='screen',
            parameters=[{'use_sim_time': True,
                        'wheel_radius': 0.05,
                        'wheel_separation': 0.25}],
        ),
    ])