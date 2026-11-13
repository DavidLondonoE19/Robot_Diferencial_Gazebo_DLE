import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import ExecuteProcess
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'dle_pista.sdf')
    world_path_2 = os.path.join(pkg, 'worlds', 'pista_competencia.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_description = Command(['xacro ', xacro_path])

    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop', choices=['teleop', 'auto']),
        DeclareLaunchArgument('pista', default_value='propia', choices=['propia', 'competencia']),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(pkg)),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v4', world_path], output='screen',
               condition=LaunchConfigurationEquals('pista', 'propia')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v4', world_path_2], output='screen',
                       condition=LaunchConfigurationEquals('pista', 'competencia')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                        'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'mi_robot', '-z', '0.1'],
            condition=LaunchConfigurationEquals('pista', 'propia')
        ),
        Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', 'robot_description', '-name', 'mi_robot',
                            '-x', '-0.60', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
                    condition=LaunchConfigurationEquals('pista', 'competencia')
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
                        'wheel_radius': 0.0245,
                        'wheel_separation': 0.134}],
        ),
        Node(
            package='mi_robot_diferencial',
            executable='lidar_aeb.py',
            name='lidar_aeb',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='twist_mux', executable='twist_mux', output='screen',
            parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml'), {'use_sim_time': True}],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')]
        ),
        Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            prefix='xterm -e', output='screen',
            remappings=[('cmd_vel', 'cmd_vel_teleop')],
            condition=LaunchConfigurationEquals('modo', 'teleop')
        ),

        
        
    ])