#!/usr/bin/env python3
"""
calibrar_odometria.py — Calibración de la odometría del skid-steer (vía efectiva y radio)
Robot diferencial skid-steer (MicroROS Yahboom) en Gazebo (ROS 2 Jazzy + ros_gz)

En un skid-steer las ruedas deben deslizar lateralmente para girar, así que el robot gira
menos de lo que indican los encoders. El modelo diferencial se conserva reemplazando la vía
geométrica B por una vía efectiva:

    B_ef = chi * B,     chi = dtheta_encoders / dtheta_real       (chi > 1)

donde dtheta_encoders = (r / B) * integral(wD - wI) dt  y  dtheta_real sale de /ground_truth.
El radio efectivo se calibra en línea recta:  r_ef = r * d_real / d_encoders.

Procedimiento (el robot debe estar en un lugar libre, p. ej. el spawn de la pista):
    1. Giro en el sitio a cada velocidad de 'w_giro' durante 'vueltas' vueltas reales.
    2. Recta a 'v_recta' durante 'd_recta' metros reales.
    3. Imprime chi por velocidad, B_ef promedio y r_ef.

Uso (con la simulación en modo teleop, sin tocar el teclado):
    ros2 run mi_robot_diferencial calibrar_odometria.py --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calibrador(Node):
    def __init__(self):
        super().__init__('calibrar_odometria')

        # %% Parámetros (ajustables con --ros-args -p nombre:=valor)
        self.declare_parameter('wheel_radius', 0.049 / 2)       # [m] radio geométrico
        self.declare_parameter('wheel_separation', 0.134)       # [m] vía geométrica B (centro a centro)
        self.declare_parameter('joints_left', ['joint_rueda_del_izq', 'joint_rueda_tra_izq'])
        self.declare_parameter('joints_right', ['joint_rueda_del_der', 'joint_rueda_tra_der'])
        self.declare_parameter('w_giro', [1.0, 2.0, 3.0])        # [rad/s] velocidades de giro en el sitio
        self.declare_parameter('vueltas', 3.0)                  # vueltas reales por cada velocidad
        self.declare_parameter('v_recta', 0.4)                  # [m/s] velocidad de la prueba en recta
        self.declare_parameter('d_recta', 3.0)                  # [m] distancia real de la prueba en recta
        self.declare_parameter('cmd_topic', '/cmd_vel_teleop')  # entra por twist_mux y pasa por el AEB
        g = lambda n: self.get_parameter(n).value
        self.r, self.B = float(g('wheel_radius')), float(g('wheel_separation'))
        self.jl, self.jr = g('joints_left'), g('joints_right')
        self.w_giro = [float(w) for w in g('w_giro')]
        self.vueltas, self.v_recta, self.d_recta = float(g('vueltas')), float(g('v_recta')), float(g('d_recta'))

        # %% Estado
        self.t_js = None
        self.int_dif = 0.0      # integral de (wD - wI) dt  -> giro según encoders
        self.int_sum = 0.0      # integral de (wD + wI) dt  -> avance según encoders
        self.gt = None          # (x, y, yaw) de /ground_truth
        self.yaw_unw = 0.0      # yaw real acumulado (sin envolver)
        self.yaw_ant = None
        self.yaw_ref = None     # rumbo inicial, para mantener la recta

        # %% Comunicación
        self.pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.create_subscription(JointState, '/joint_states', self.cb_js, 50)
        self.create_subscription(Odometry, '/ground_truth', self.cb_gt, 50)

        # Secuencia de pruebas: giros en el sitio y una recta
        self.fases = [('giro', w) for w in self.w_giro] + [('recta', self.v_recta)]
        self.i = -1
        self.pausa_hasta = None
        self.res_giro = []
        self.res_recta = None
        self.create_timer(0.02, self.lazo)
        self.get_logger().info('Calibración: giros a %s rad/s (%.0f vueltas c/u) y recta de %.1f m'
                               % (self.w_giro, self.vueltas, self.d_recta))

    # ------------------------------------------------------------------ callbacks
    def cb_js(self, msg):
        try:
            iR = [msg.name.index(j) for j in self.jr]
            iL = [msg.name.index(j) for j in self.jl]
        except ValueError:
            return
        if len(msg.velocity) != len(msg.name):
            return
        # Skid-steer: rueda equivalente por lado = promedio de las dos ruedas de ese lado
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wD = float(np.mean([msg.velocity[i] for i in iR]))
        wI = float(np.mean([msg.velocity[i] for i in iL]))
        if self.t_js is not None and 0.0 < t - self.t_js < 0.5:
            dt = t - self.t_js
            self.int_dif += (wD - wI) * dt
            self.int_sum += (wD + wI) * dt
        self.t_js = t

    def cb_gt(self, msg):
        p = msg.pose.pose.position
        th = yaw(msg.pose.pose.orientation)
        # Acumular el yaw sin envolver para contar vueltas completas
        if self.yaw_ant is not None:
            d = math.atan2(math.sin(th - self.yaw_ant), math.cos(th - self.yaw_ant))
            self.yaw_unw += d
        self.yaw_ant = th
        if self.yaw_ref is None:
            self.yaw_ref = th
        self.gt = (p.x, p.y, th)

    # ------------------------------------------------------------------ fases
    def iniciar_fase(self):
        self.i += 1
        self.int_dif = self.int_sum = 0.0
        self.yaw0 = self.yaw_unw
        self.p0 = self.gt[:2]

    # ------------------------------------------------------------------ lazo
    def lazo(self):
        if self.gt is None or self.t_js is None:
            return
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.pausa_hasta is not None:
            parar = Twist()
            if self.i + 1 < len(self.fases) and self.fases[self.i + 1][0] == 'recta':
                e_th = math.atan2(math.sin(self.yaw_ref - self.gt[2]), math.cos(self.yaw_ref - self.gt[2]))
                parar.angular.z = 2.0 * e_th
            self.pub.publish(parar)
            if ahora < self.pausa_hasta:
                return
            self.pausa_hasta = None
            if self.i + 1 >= len(self.fases):
                self.reporte()
                raise SystemExit
            self.iniciar_fase()
        if self.i < 0:
            self.iniciar_fase()

        tipo, val = self.fases[self.i]
        cmd = Twist()
        if tipo == 'giro':
            cmd.angular.z = val
            real = abs(self.yaw_unw - self.yaw0)
            if real >= self.vueltas * 2.0 * math.pi:
                enc = abs(self.r / self.B * self.int_dif)
                chi = enc / real
                self.res_giro.append((val, real, enc, chi))
                self.get_logger().info('w=%.1f rad/s: real=%.1f°, encoders=%.1f°, chi=%.3f'
                                       % (val, math.degrees(real), math.degrees(enc), chi))
                self.pausa_hasta = ahora + 1.5
                cmd = Twist()
        else:
            cmd.linear.x = val
            e_th = math.atan2(math.sin(self.yaw_ref - self.gt[2]), math.cos(self.yaw_ref - self.gt[2]))
            cmd.angular.z = 2.0 * e_th
            real = math.hypot(self.gt[0] - self.p0[0], self.gt[1] - self.p0[1])
            if real >= self.d_recta:
                enc = abs(self.r / 2.0 * self.int_sum)
                self.res_recta = (real, enc)
                self.get_logger().info('recta: real=%.3f m, encoders=%.3f m' % (real, enc))
                self.pausa_hasta = ahora + 1.5
                cmd = Twist()
        self.pub.publish(cmd)

    # ------------------------------------------------------------------ reporte
    def reporte(self):
        chis = [c for (_, _, _, c) in self.res_giro]
        chi = float(np.mean(chis))
        r_ef = self.r * self.res_recta[0] / self.res_recta[1] if self.res_recta else self.r
        lineas = ['', '================ RESULTADO DE LA CALIBRACIÓN ================',
                  ' w [rad/s]   giro real [°]   giro encoders [°]   chi']
        for w, real, enc, c in self.res_giro:
            lineas.append('   %4.1f        %8.1f          %8.1f        %.3f' % (w, math.degrees(real), math.degrees(enc), c))
        lineas += ['chi promedio = %.3f  (desv. %.3f)' % (chi, float(np.std(chis))),
                   'B geométrica = %.4f m  ->  B efectiva = %.4f m' % (self.B, chi * self.B),
                   'r geométrico = %.4f m  ->  r efectivo = %.4f m' % (self.r, r_ef),
                   'Usar en cinematica_ruedas: wheel_separation:=%.4f  wheel_radius:=%.4f' % (chi * self.B, r_ef),
                   '=============================================================']
        self.get_logger().info('\n'.join(lineas))


def main():
    rclpy.init()
    nodo = Calibrador()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        nodo.pub.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
