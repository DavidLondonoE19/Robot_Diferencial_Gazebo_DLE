#!/usr/bin/env python3
"""
aeb_node.py — Frenado Automático de Emergencia (AEB) con LiDAR
Robot diferencial en Gazebo (ROS 2 Jazzy + ros_gz)

Arquitectura (el nodo actúa como "compuerta" entre el teleop y el robot):

    teleop_twist_keyboard --/cmd_vel_teleop--> [aeb_node] --/cmd_vel--> DiffDrive
                                                   ^    ^
                                           /scan --+    +-- /odom (v medida)

Teleop con remap:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop

Idea física:
    Para un obstáculo estático visto en el haz i (rango r_i, ángulo th_i), la tasa de
    cambio del rango es  r_dot_i = -v*cos(th_i).  La rotación (w) NO cambia el rango,
    solo mueve el punto tangencialmente.
    iTTC_i = r_i / max(-r_dot_i, 0)      ->  frenar si min(iTTC) < umbral

    Umbral mínimo físicamente necesario (latencia + desaceleración constante a):
        d_frenado = v*t_lat + v^2/(2a)   y   d = v*TTC   ->   TTC_min = t_lat + v/(2a)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        # %% Parámetros (ajustables con --ros-args -p nombre:=valor)
        self.declare_parameter('mode', 'ittc')          # 'ittc' | 'corridor'
        self.declare_parameter('ttc_threshold', 0.5)    # [s] umbral base, valor máximo en t_lat + v/(2a)
        self.declare_parameter('a_max', 1.0)            # [m/s^2] desaceleración del robot
        self.declare_parameter('t_latency', 0.15)       # [s] latencia sensor->actuador
        # Del URDF, la margen se debe al radio de las ruedas más su ubicación (i.e. separación / 2) = 0.0915 m 
        self.declare_parameter('half_width', 0.1)      # [m] Debe ser mayor a chasis_ancho/2 = 0.0765, margen + desviación estándar del Lidar (0.010)
        # Con base a la colisión definida en X del URDF, (x0+cx)*m_scale más la mitad de la medida del chasis = 0.0863 m
        self.declare_parameter('front_offset', 0.0863)    # [m] LiDAR -> parachoques delantero
        # Con base a la colisión definida en X del URDF, (x0+cx)*m_scale menos la mitad de la medida del chasis = 0.1123 m
        self.declare_parameter('rear_offset', 0.1123)     # [m] LiDAR -> parachoques trasero
        self.declare_parameter('v_min', 0.01)           # [m/s] v_min a la que aplica AEB
        self.declare_parameter('rate', 30.0)            # [Hz] lazo de control o loop rate del código
        self.declare_parameter('cmd_in_topic', '/cmd_vel_mux')
        # %% Estado
        self.r = None          # rangos válidos
        self.th = None         # ángulos correspondientes
        self.v_meas = 0.0      # velocidad lineal medida (/odom)
        self.cmd = Twist()     # último comando del teleop

        # %% Comunicación
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_in_topic').value, self.cmd_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_active = self.create_publisher(Bool, '/aeb/active', 10)
        self.pub_ttc = self.create_publisher(Float32, '/aeb/ttc', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self.loop)
        self.get_logger().info('AEB listo. Modo: %s' % self.get_parameter('mode').value)

    # ------------------------------------------------------------------ callbacks
    def scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        th = msg.angle_min + np.arange(r.size) * msg.angle_increment
        # Descartar inf, nan y lecturas fuera de rango del sensor
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.r, self.th = r[valid], th[valid]

    def odom_cb(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x

    def cmd_cb(self, msg: Twist):
        self.cmd = msg

    # ------------------------------------------------------------------ TTC
    def compute_ttc(self, v: float) -> float:
        """Tiempo mínimo a colisión si el robot avanza con velocidad lineal v."""
        if self.r is None or self.r.size == 0 or abs(v) < self.get_parameter('v_min').value:
            return math.inf

        if self.get_parameter('mode').value == 'ittc':
            # Modelo por haz: cada lectura "se acerca" a tasa v*cos(th)
            closing = v * np.cos(self.th)            # = -r_dot
            mask = closing > 0.0                     # solo haces que se acercan
            if not mask.any():
                return math.inf
            return float(np.min(self.r[mask] / closing[mask]))

        # Modo 'corridor': solo puntos dentro del ancho del robot (huella)
        x = self.r * np.cos(self.th)
        y = self.r * np.sin(self.th)
        hw = self.get_parameter('half_width').value
        if v > 0:
            mask = (x > 0.0) & (np.abs(y) < hw)
            d = x[mask] - self.get_parameter('front_offset').value
        else:
            mask = (x < 0.0) & (np.abs(y) < hw)
            d = -x[mask] - self.get_parameter('rear_offset').value
        if not mask.any():
            return math.inf
        return float(np.min(np.clip(d, 0.0, None)) / abs(v))

    def threshold(self, v: float) -> float:
        """Umbral dinámico: nunca menor que t_lat + |v|/(2a)."""
        a = self.get_parameter('a_max').value
        t_lat = self.get_parameter('t_latency').value
        return max(self.get_parameter('ttc_threshold').value, t_lat + abs(v) / (2.0 * a))

    # ------------------------------------------------------------------ lazo
    def loop(self):
        v_cmd = self.cmd.linear.x
        ttc_meas = self.compute_ttc(self.v_meas)   # ¿ya voy hacia algo?
        ttc_cmd = self.compute_ttc(v_cmd)          # ¿el comando me lleva hacia algo?

        emergency = ttc_meas < self.threshold(self.v_meas)
        unsafe_cmd = ttc_cmd < self.threshold(v_cmd)

        # Se permite el comando lineal solo si es seguro y, en emergencia,
        # solo si va en sentido contrario al movimiento actual (alejarse).
        allow_linear = (not unsafe_cmd) and not (emergency and v_cmd * self.v_meas > 0.0)

        out = Twist()
        out.linear.x = v_cmd if allow_linear else 0.0
        out.angular.z = self.cmd.angular.z   # el giro no cambia el rango (ver docstring)
        self.pub_cmd.publish(out)

        active = emergency or (unsafe_cmd and abs(v_cmd) > 0.0)
        self.pub_active.publish(Bool(data=bool(active)))
        self.pub_ttc.publish(Float32(data=float(min(ttc_meas, ttc_cmd, 10.0))))  # tope para rqt_plot

        if active:
            self.get_logger().warn('AEB ACTIVO  TTC=%.2f s' % min(ttc_meas, ttc_cmd),
                                   throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())   # dejar el robot quieto al salir
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
