#!/usr/bin/env python3

# Modelo general:
#   w = Q * xi          (Q es 2x3, una fila por rueda)
#   xi = W * w          con W = Q^T (Q Q^T)^-1   (pq m < n)
#   eta_dot = J(phi) * xi
#   eta_{k+1} = eta_k + eta_dot * dt   -- integracion
#
# w = [wD, wI]   xi = [u, v, r]   eta = [x, y, phi]


import math
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def wrap(a):
    """Normaliza un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fila_Q(a, phi, theta_B, dx, dy):
    """Fila de Q de una llanta (modelo generalizado).
    a: radio, phi: orientación del rodillo, theta_B: orientación de la llanta
    respecto al cuerpo, (dx, dy): posición de la llanta en {B}."""
    c, s = math.cos(theta_B), math.sin(theta_B)
    rot = np.array([[c, s], [-s, c]])
    pos = np.array([[1.0, 0.0, -dy], [0.0, 1.0, dx]])
    rod = np.array([[1.0 / a, math.tan(phi) / a]])
    return (rod @ rot @ pos).flatten()


def construir_Q_W(r, b):
    """Diferencial con rueda loca: a1 = a2 = r, phi = 0, theta_B = 0, dx = 0,
    dy = -b (derecha) y +b (izquierda), con b = separación / 2.
    Filas de Q en el orden del reporte: [D, I]."""
    Q = np.vstack([fila_Q(r, 0.0, 0.0, 0.0, -b),    # llanta derecha   (y = -b)
                   fila_Q(r, 0.0, 0.0, 0.0, +b)])   # llanta izquierda (y = +b)
    W = Q.T @ np.linalg.inv(Q @ Q.T)                 # caso m < n
    return Q, W


def J(phi):
    """Jacobiano de clase: eta_dot = J(phi) xi."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def paso_euler(x, y, th, xi, dt):
    """Un paso de Euler (clase): eta_{k+1} = eta_k + J(phi_k) xi_k dt."""
    d = J(th) @ xi
    return x + d[0] * dt, y + d[1] * dt, wrap(th + d[2] * dt)

class CinematicaRuedas(Node):
    def __init__(self):
        super().__init__("cinematica_ruedas")
        self.declare_parameter("wheel_radius", 0.05)        # r 
        self.declare_parameter("wheel_separation", 0.25)    # 2b
        self.declare_parameter("joint_left", "joint_rueda_izq")
        self.declare_parameter("joint_right", "joint_rueda_der")
        self.declare_parameter("wheel_sign", 1)             # +1: giro positivo = avanzar
        self.declare_parameter("use_joint_velocity", False) # False: w = d(posición)/dt
        self.declare_parameter("init_from_odom", True)      # arranca desde la pose de /odom
        self.declare_parameter("max_dt", 0.5)               # descarta saltos de tiempo [s]

        p = lambda n: self.get_parameter(n).value
        self.r = float(p("wheel_radius"))
        self.L = float(p("wheel_separation"))
        self.j_izq, self.j_der = p("joint_left"), p("joint_right")
        self.sign = float(p("wheel_sign"))
        self.usar_vel = bool(p("use_joint_velocity"))
        self.init_odom = bool(p("init_from_odom"))
        self.max_dt = float(p("max_dt"))

        self.Q, self.W = construir_Q_W(self.r, self.L / 2.0)
        self.get_logger().info(f"Q (filas [D, I]) =\n{np.round(self.Q, 4)}")
        self.get_logger().info(f"W = Q^T (Q Q^T)^-1 =\n{np.round(self.W, 4)}")
        self.get_logger().info(
            f"r={self.r} m, separación={self.L} m, integración=Euler, "
            f"w desde {'velocity' if self.usar_vel else 'd(position)/dt'}")

        
        self.x = self.y = self.th = 0.0
        self.xi = np.zeros(3)
        self.listo = not self.init_odom
        self.t_prev, self.pos_prev = None, None
        self.t_js = None
        
        self.n = 0
        self.s_pos2 = self.s_th2 = 0.0
        self.max_pos = self.max_th = 0.0
        self.e_pos = self.e_th = 0.0
        self.dist = 0.0
        self.odom_prev = None


        self.create_subscription(JointState, "/joint_states", self.cb_joint, 50)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom_cinematica", 10)
        self.pub_err = self.create_publisher(Vector3Stamped, "/error_odometria", 10)
        self.pub_err_pos = self.create_publisher(Float64, "/error_odometria_pos", 10)

    def _tiempo(self, stamp):
        """Tiempo del mensaje [s]; si el stamp viene en cero usa el reloj del nodo."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    # -- /joint_states: w -> xi -> eta_dot -> integración ---------------------
    def cb_joint(self, msg):
        try:
            iD, iI = msg.name.index(self.j_der), msg.name.index(self.j_izq)
        except ValueError:
            self.get_logger().warn("No encuentro los joints de las ruedas en /joint_states",
                                   throttle_duration_sec=5.0)
            return
        t = self._tiempo(msg.header.stamp)
        hay_pos = len(msg.position) == len(msg.name)

        if not self.listo:                            
            self.t_prev = t
            if hay_pos:
                self.pos_prev = np.array([msg.position[iD], msg.position[iI]])
            return
        if self.t_prev is None:                       
            self.t_prev = t
            if hay_pos:
                self.pos_prev = np.array([msg.position[iD], msg.position[iI]])
            return
        dt = t - self.t_prev
        if dt <= 1e-9:
            return
        if dt > self.max_dt:                          
            self.t_prev = t
            if hay_pos:
                self.pos_prev = np.array([msg.position[iD], msg.position[iI]])
            return

        if self.usar_vel and len(msg.velocity) == len(msg.name):
            w = np.array([msg.velocity[iD], msg.velocity[iI]]) * self.sign
        else:
            if not hay_pos or self.pos_prev is None:
                return
            pos = np.array([msg.position[iD], msg.position[iI]])
            w = (pos - self.pos_prev) / dt * self.sign
            self.pos_prev = pos
        self.t_prev, self.t_js = t, t

        self.xi = self.W @ w                          # xi = W w  -> [u, v, r]
        self.x, self.y, self.th = paso_euler(self.x, self.y, self.th, self.xi, dt)

    # /odom publica la propia y calcula el error 
    def cb_odom(self, msg):
        xo, yo = msg.pose.pose.position.x, msg.pose.pose.position.y
        tho = yaw_de_cuaternion(msg.pose.pose.orientation)
        if not self.listo:
            self.x, self.y, self.th = xo, yo, tho
            self.listo = True
            self.get_logger().info(f"Pose inicial tomada de /odom: ({xo:.3f}, {yo:.3f}, {tho:.3f})")
            self.get_logger().info(f"Q (filas [D, I]) =\n{np.round(self.Q, 4)}")
            self.get_logger().info(f"W = Q^T (Q Q^T)^-1 =\n{np.round(self.W, 4)}")
            return
        if self.t_js is None:
            return
        if self.odom_prev is not None:
            self.dist += math.hypot(xo - self.odom_prev[0], yo - self.odom_prev[1])
        self.odom_prev = (xo, yo)

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = msg.header.frame_id or "odom"
        o.child_frame_id = msg.child_frame_id or "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.th / 2.0)
        o.pose.pose.orientation.w = math.cos(self.th / 2.0)
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(self.xi[0]), float(self.xi[2])
        self.pub_odom.publish(o)

        ex, ey, eth = self.x - xo, self.y - yo, wrap(self.th - tho)   
        self.e_pos, self.e_th = math.hypot(ex, ey), eth
        e = Vector3Stamped()
        e.header.stamp = msg.header.stamp
        e.vector.x, e.vector.y, e.vector.z = ex, ey, eth
        self.pub_err.publish(e)
        f = Float64()
        f.data = self.e_pos
        self.pub_err_pos.publish(f)

        self.n += 1
        self.s_pos2 += self.e_pos ** 2
        self.s_th2 += eth ** 2
        self.max_pos = max(self.max_pos, self.e_pos)
        self.max_th = max(self.max_th, abs(eth))
        
    def resumen(self):
        if self.n == 0:
            print("[cinematica_ruedas] Sin muestras para el resumen.")
            return
        rms_pos = math.sqrt(self.s_pos2 / self.n)
        rms_th = math.sqrt(self.s_th2 / self.n)
        rel = 100.0 * self.e_pos / self.dist if self.dist > 1e-6 else float("nan")
        print(f"\nQ =\n{np.round(self.Q, 4)}")
        print(f"W =\n{np.round(self.W, 4)}")
        print("\n===== Resumen de error: odometría propia vs /odom (plugin) =====")
        print(f"muestras: {self.n}   distancia recorrida (plugin): {self.dist:.3f} m")
        print(f"error de posición    RMS: {rms_pos*1000:.2f} mm   máx: {self.max_pos*1000:.2f} mm   "
              f"final: {self.e_pos*1000:.2f} mm  ({rel:.3f} % de la distancia)")
        print(f"error de orientación RMS: {math.degrees(rms_th):.3f} deg   "
              f"máx: {math.degrees(self.max_th):.3f} deg   final: {math.degrees(self.e_th):.3f} deg")
        print("=================================================================")




def main():
    rclpy.init()
    nodo = CinematicaRuedas()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.resumen()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()