import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, PythonExpression

# Configuración por pista: world, spawn (detrás de la línea de meta), línea de meta para el
# conteo de vueltas y parámetros del seguimiento de pared que cambian respecto a los de wall_follow.py
PISTAS = {
    'propia': {
        'world': 'pista_propia.sdf',
        'spawn': ['-x', '0.80', '-y', '0.50', '-z', '0.1', '-Y', '-1.5708'],
        'meta': {'meta_eje': 'eje_y', 'meta_valor': 0.0, 'meta_min': 0.45, 'meta_max': 1.15,
                 'meta_sentido': -1, 'distancia_min_vuelta': 10.0},
        # Carriles de 0.37-0.6 m, esquina de 90° y "bolitas": pared exterior más cerca y giro en esquina
        'wall_follow': {'d_des': 0.20, 'k_esquina': 4.0, 'd_esquina': 0.50},
    },
    'competencia': {
        'world': 'pista_competencia.sdf',
        'spawn': ['-x', '-0.60', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
        'meta': {'meta_eje': 'eje_x', 'meta_valor': 0.0, 'meta_min': -0.4, 'meta_max': 0.4,
                 'meta_sentido': 1, 'distancia_min_vuelta': 20.0},
        'wall_follow': {},
    },
}


def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    joy_config = os.path.join(pkg, 'config', 'joy_teleop.yaml')

    robot_description = Command(['xacro ', xacro_path])

    def si(**args):
        """Condición verdadera si todos los argumentos del launch tienen el valor dado."""
        expr = []
        for i, (nombre, valor) in enumerate(args.items()):
            expr += [" and " if i else "", "'", LaunchConfiguration(nombre), "' == '" + valor + "'"]
        return IfCondition(PythonExpression(expr))

    por_pista = []
    for nombre, cfg in PISTAS.items():
        por_pista += [
            ExecuteProcess(cmd=['gz', 'sim', '-r', '-v4', os.path.join(pkg, 'worlds', cfg['world'])],
                           output='screen', condition=si(pista=nombre)),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'mi_robot'] + cfg['spawn'],
                condition=si(pista=nombre)
            ),
            Node(
                package='mi_robot_diferencial',
                executable='cinematica_ruedas.py',
                name='cinematica_ruedas',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'wheel_radius': 0.0245,
                             # Vía efectiva del skid-steer: B_ef = chi*B = 1.320*0.134 m,
                             # calibrada con calibrar_odometria.py (giro en el sitio vs /ground_truth)
                             'wheel_separation': 0.1769,
                             **cfg['meta']}],
                condition=si(pista=nombre)
            ),
            Node(
                package='mi_robot_diferencial',
                executable='wall_follow.py',
                name='wall_follow',
                output='screen',
                parameters=[{'use_sim_time': True, **cfg['wall_follow']}],
                condition=si(modo='auto', pista=nombre)
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop', choices=['teleop', 'auto']),
        DeclareLaunchArgument('pista', default_value='propia', choices=['propia', 'competencia']),
        DeclareLaunchArgument('control', default_value='teclado', choices=['teclado', 'mando'],
                              description='Dispositivo de teleoperación (solo con modo:=teleop)'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(pkg)),
        *por_pista,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                        'use_sim_time': True}]
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
            executable='lidar_aeb.py',
            name='lidar_aeb',
            output='screen',
            parameters=[{'use_sim_time': True, 'mode': 'corridor'}],
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
            condition=si(modo='teleop', control='teclado')
        ),
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
            parameters=[joy_config],
            condition=si(modo='teleop', control='mando')
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config],
            remappings=[('cmd_vel', 'cmd_vel_joy')],
            condition=si(modo='teleop', control='mando')
        ),
    ])
