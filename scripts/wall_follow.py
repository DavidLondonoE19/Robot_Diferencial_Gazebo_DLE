#!/usr/bin/env python3
"""
wall_follow.py — Seguimiento de pared con LiDAR (modo autónomo)
Robot diferencial skid-steer (MicroROS Yahboom) en Gazebo (ROS 2 Jazzy + ros_gz)

Arquitectura:
    /scan --> [wall_follow] --/cmd_vel_auto--> twist_mux --/cmd_vel_mux--> [lidar_aeb] --/cmd_vel--> DiffDrive

Método de dos rayos (reporte, requisito 4):
    b: rayo perpendicular a la pared (+90° pared izquierda, -90° pared derecha)
    a: rayo adelantado un ángulo theta respecto a b
        alpha   = atan((a cos(theta) - b) / (a sin(theta)))
        D_t     = b cos(alpha)
        D_t+1   = D_t + l sin(alpha)          (l: distancia de anticipación)
        e       = D_des - D_t+1
    PID sobre e -> omega.  v se reduce con |omega| (curvatura) y con la distancia libre al frente.

Extensiones (justificadas en el pitch):
    - Carril angosto: si el ancho medido (izq + der) es menor que 2*D_des + margen, la referencia
      pasa a ser el centro del carril (ancho/2). Así el robot no roza la pared opuesta en el
      estrechamiento de 0.40 m.
    - Esquina (opcional, k_esquina > 0): si la holgura al frente es menor que d_esquina, se suma
      un giro hacia el lado libre. En la pista de competencia se deja en 0: no hay esquinas
      cerradas y, frente a un obstáculo, el robot debe detenerse (prueba del AEB), no esquivarlo.
    - Velocidad limitada por tiempo al frente: v <= (d_frente_libre - x_frente) / T_frente, con
      T_frente mayor que el umbral del AEB, para que el AEB nunca tenga que actuar en las curvas.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class SeguidorPared:
    """Controlador puro (sin ROS) para poder probarlo y sintonizarlo fuera de Gazebo."""

    def __init__(self, p):
        self.p = p
        self.e_ant = None
        self.de_filt = 0.0
        self.integral = 0.0

    def reiniciar(self):
        self.e_ant = None
        self.de_filt = 0.0
        self.integral = 0.0

    # ------------------------------------------------------------------ lectura de rayos
    @staticmethod
    def rango_en(ranges, angle_min, angle_inc, r_min, r_max, ang, ventana, r_clip):
        """Promedio de los rayos válidos alrededor del ángulo pedido (rad)."""
        n = len(ranges)
        i = int(round((ang - angle_min) / angle_inc)) % n
        idx = [(i + k) % n for k in range(-ventana, ventana + 1)]
        v = [ranges[j] for j in idx
             if math.isfinite(ranges[j]) and r_min <= ranges[j] <= r_max]
        return min(float(np.mean(v)), r_clip) if v else r_clip

    # ------------------------------------------------------------------ ley de control
    def calcular(self, ranges, angle_min, angle_inc, r_min, r_max, dt):
        p = self.p
        s = 1.0 if p['lado'] == 'izq' else -1.0
        th = math.radians(p['theta_deg'])
        rd = lambda ang: self.rango_en(ranges, angle_min, angle_inc, r_min, r_max,
                                       ang, p['ventana'], p['r_clip'])

        b = rd(s * math.pi / 2)
        a = rd(s * (math.pi / 2 - th))
        alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th))
        d_t = b * math.cos(alpha)
        d_t1 = d_t + p['l'] * math.sin(alpha)

        # Referencia: centro del carril cuando es más angosto que 2*D_des + margen
        ancho = rd(math.pi / 2) + rd(-math.pi / 2)
        d_obj = p['d_des']
        if ancho < 2.0 * p['d_des'] + p['margen_carril']:
            d_obj = 0.5 * ancho

        e = d_obj - d_t1

        # PID (derivada filtrada, integral con anti-windup)
        if self.e_ant is None or dt <= 0.0:
            de = 0.0
        else:
            de = (e - self.e_ant) / dt
        self.e_ant = e
        self.de_filt += p['filtro_d'] * (de - self.de_filt)
        self.integral = float(np.clip(self.integral + e * max(dt, 0.0),
                                      -p['i_max'], p['i_max']))
        u = p['kp'] * e + p['kd'] * self.de_filt + p['ki'] * self.integral

        # Pared izquierda: e > 0 (muy cerca) -> girar a la derecha (omega < 0)
        w = -s * u

        # Espacio libre al frente dentro de la huella del robot (misma geometría que el AEB
        # en modo corredor): puntos con x > 0 y |y| < semiancho.
        frente, y_obs = self.libre_al_frente(ranges, angle_min, angle_inc, r_min, r_max,
                                             p['semiancho'], p['r_clip'])
        holgura = max(0.0, frente - p['x_frente'])

        # Esquina u obstáculo al frente: girar hacia el lado libre. Si el obstáculo está
        # centrado (pared perpendicular), se gira hacia el lado contrario a la pared seguida.
        if holgura < p['d_esquina']:
            lado_giro = -math.copysign(1.0, y_obs) if abs(y_obs) > 0.03 else -s
            w += lado_giro * p['k_esquina'] * (p['d_esquina'] - holgura)

        w = float(np.clip(w, -p['w_max'], p['w_max']))

        # Velocidad: menor en curva (|w| grande) y limitada por tiempo al frente
        v = p['v_max'] * (1.0 - abs(w) / p['w_max'])
        v = min(v, holgura / p['t_frente'])
        v = float(np.clip(v, p['v_min'], p['v_max']))

        return v, w, {'e': e, 'alpha': alpha, 'd': d_t, 'frente': frente, 'ancho': ancho}

    @staticmethod
    def libre_al_frente(ranges, angle_min, angle_inc, r_min, r_max, semiancho, r_clip):
        """Distancia al punto más cercano dentro del corredor del robot y su coordenada y."""
        r = np.asarray(ranges, dtype=float)
        ang = angle_min + np.arange(r.size) * angle_inc
        ok = np.isfinite(r) & (r >= r_min) & (r <= r_max)
        x = r[ok] * np.cos(ang[ok])
        y = r[ok] * np.sin(ang[ok])
        m = (x > 0.0) & (np.abs(y) < semiancho)
        if not m.any():
            return r_clip, 0.0
        i = int(np.argmin(x[m]))
        cerca = m.copy()
        cerca[m] = x[m] < x[m][i] + 0.10
        return min(float(x[m][i]), r_clip), float(np.mean(y[cerca]))


PARAMS = {
    'lado': 'izq',              # pared seguida: 'izq' (interior en sentido antihorario) | 'der'
    'd_des': 0.30,              # [m] distancia deseada a la pared
    'theta_deg': 45.0,          # [°] ángulo entre los rayos a y b
    'l': 0.40,                  # [m] distancia de anticipación
    'kp': 3.0,
    'ki': 0.0,
    'kd': 0.35,
    'i_max': 0.2,
    'filtro_d': 0.5,            # filtro exponencial de la derivada (0..1)
    'v_max': 0.80,              # [m/s] (límite del robot: 1.0)
    'v_min': 0.12,              # [m/s]
    'w_max': 3.5,               # [rad/s] (límite del robot: 4.0)
    'ventana': 2,               # rayos vecinos promediados a cada lado (0.8° c/u)
    'r_clip': 2.0,              # [m] rangos mayores se recortan (pared perdida)
    'margen_carril': 0.10,      # [m]
    'semiancho': 0.10,          # [m] semiancho del corredor frontal (igual al del AEB)
    'd_esquina': 0.35,          # [m] holgura al frente que activa el giro de esquina
    'k_esquina': 0.0,           # [rad/s/m] 0 = no esquiva: frente a un obstáculo frena y actúa el AEB
    'x_frente': 0.09,           # [m] LiDAR -> parachoques delantero
    't_frente': 0.80,           # [s] tiempo mínimo al frente (mayor que el umbral del AEB)
}


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow')

        # %% Parámetros (ajustables con --ros-args -p nombre:=valor; ver PARAMS)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('cmd_topic', '/cmd_vel_auto')

        # %% Estado
        self.ctrl = SeguidorPared(self._leer_params())
        self.t_ant = None      # stamp del último /scan [s]

        # %% Comunicación
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.pub_e = self.create_publisher(Float32, '/wall/error', 10)
        self.pub_d = self.create_publisher(Float32, '/wall/distancia', 10)
        self.get_logger().info(
            'Seguimiento de pared listo: pared %s, D_des=%.2f m, l=%.2f m, theta=%.0f°'
            % (PARAMS['lado'], self.ctrl.p['d_des'], self.ctrl.p['l'], self.ctrl.p['theta_deg']))

    # ------------------------------------------------------------------ callbacks
    def _leer_params(self):
        return {k: self.get_parameter(k).value for k in PARAMS}

    def scan_cb(self, msg: LaserScan):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.t_ant is None else t - self.t_ant
        self.t_ant = t
        self.ctrl.p = self._leer_params()
        v, w, info = self.ctrl.calcular(list(msg.ranges), msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max, dt)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)
        self.pub_e.publish(Float32(data=float(info['e'])))
        self.pub_d.publish(Float32(data=float(info['d'])))


def main():
    rclpy.init()
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
