# mi_robot_diferencial

Robot móvil diferencial simulado en Gazebo Sim, con ROS 2 Jazzy. Incluye modelo URDF/Xacro parametrizado, LiDAR y cámara, pista propia, odometría del plugin DiffDrive y un nodo que calcula la odometría a partir de la velocidad de las ruedas y la compara con el plugin.

**Curso:** Robótica, Universidad EIA. **Autor:** <David Londoño Escandon>.

## Requisitos
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim (Harmonic)
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard \
                 ros-jazzy-rviz2 ros-jazzy-robot-state-publisher python3-numpy
```

## Compilar
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws_dif/src && cd ~/ros2_ws_dif/src
git clone <https://github.com/DavidLondonoE19/Robot_Diferencial_Gazebo_DLE> mi_robot_diferencial
cd ~/ros2_ws_dif
colcon build --symlink-install
source install/setup.bash
```

## Ejecutar
Un solo launch levanta Gazebo (mundo `dle_pista`), el spawn, el bridge, `robot_state_publisher`, RViz2 y el nodo de cinemática:
```bash
ros2 launch mi_robot_diferencial mi_robot.launch.py
```
Teleoperación, en otra terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> **Importante:** cada terminal nueva debe ejecutar `source install/setup.bash`. Si aparece `Package 'mi_robot_diferencial' not found`, falta ese paso.

Al terminar, cierra el launch con `Ctrl+C`: el nodo imprime Q, W y el resumen del error (RMS, máximo y final).

## Contenido
- `urdf/`: modelo Xacro del robot (chasis, dos ruedas motrices, caster, LiDAR, cámara, plugins).
- `worlds/dle_pista.sdf`: pista con curvas, estrechamiento, obstáculo y muros con colisión.
- `config/`: `bridge.yaml` (ros_gz_bridge) y configuración de RViz2.
- `launch/mi_robot.launch.py`: arranque completo.
- `scripts/cinematica_ruedas.py`: odometría propia por cinemática de ruedas.

## Topics principales
`/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan`, `/image_raw`, `/camera_info`, `/clock`, `/odom_cinematica`, `/error_odometria`, `/error_odometria_pos`.

## Cinemática
`w = Q ξ`, `ξ = W w` con `W = Qᵀ(QQᵀ)⁻¹`, `η̇ = J(φ) ξ`, integración de Euler. Con r = 0.05 m y separación 0.25 m: `v = 0.025 (wD + wI)`, `ω = 0.2 (wD − wI)`.

## Resultados (recorrido completo, error vs. `/odom` del plugin)
| Corrida | Distancia | RMS posición | Máx. posición | RMS orientación |
|---|---|---|---|---|
| 1 | 14.28 m | 1.35 mm | 6.00 mm | 0.051° |
| 2 | 13.65 m | 1.93 mm | 8.00 mm | 0.105° |