#!/usr/bin/env python3

# Modelo general:
#   w = Q * xi          (Q es 2x3, una fila por rueda)
#   xi = W * w          con W = Q^T (Q Q^T)^-1   (pq m < n)
#   eta_dot = J(phi) * xi
#   eta_{k+1} = eta_k + eta_dot * dt   -- integracion
#
# w = [wD, wI]   xi = [u, v, r]   eta = [x, y, phi]



#QUE HAY QUE CAMBIAR SI DAN OTRO ROBOT: construir_Q_W, W = Qᵀ(QQᵀ)⁻¹, cb_joint, Los twist publicados. Usan xi[0] y xi[2], o sea u y r, y suponen que v = 0


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
#ángulo entre −π y π, para que φ cno crezca infiniamente

def yaw_de_cuaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
#/odom entrega la orientación como cuaternión. La función saca φ del cuaternion.

def fila_Q(a, phi, theta_B, dx, dy): #construye una fila de Q para la rueda que le metamos a la funcion
    """Fila de Q de una llanta (modelo generalizado).
    a: radio, phi: orientación del rodillo, theta_B: orientación de la llanta
    respecto al cuerpo, (dx, dy): posición de la llanta en {B}."""
    c, s = math.cos(theta_B), math.sin(theta_B) # c y s son el coseno y el seno del ángulo theta_B
    rot = np.array([[c, s], [-s, c]])
    pos = np.array([[1.0, 0.0, -dy], [0.0, 1.0, dx]])
    rod = np.array([[1.0 / a, math.tan(phi) / a]])
    return (rod @ rot @ pos).flatten()


def construir_Q_W(r, b): # construye Q y W para el robot diferencial con rueda loca
    """Diferencial con rueda loca: a1 = a2 = r, phi = 0, theta_B = 0, dx = 0,
    dy = -b (derecha) y +b (izquierda), con b = separación / 2.
    Filas de Q en el orden del reporte: [D, I]."""
    Q = np.vstack([fila_Q(r, 0.0, 0.0, 0.0, -b),    # llanta derecha   (y = -b)
                   fila_Q(r, 0.0, 0.0, 0.0, +b)])   # llanta izquierda (y = +b)
    W = Q.T @ np.linalg.inv(Q @ Q.T)                 # caso m < n
    return Q, W


def J(phi): # [[cos,-sin,0],[sin,cos,0],[0,0,1]]
    """Jacobiano de clase: eta_dot = J(phi) xi."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def paso_euler(x, y, th, xi, dt):
    """Un paso de Euler (clase): eta_{k+1} = eta_k + J(phi_k) xi_k dt."""
    d = J(th) @ xi # Calcula la velocidad en el mundo, d = [ẋ, ẏ, φdot],
    return x + d[0] * dt, y + d[1] * dt, wrap(th + d[2] * dt) # posición nueva = posición vieja + velocidad × tiempo


class CinematicaRuedas(Node):
    def __init__(self):
        super().__init__("cinematica_ruedas")
        self.declare_parameter("wheel_radius", 0.049/2)        # r 
        self.declare_parameter("wheel_separation", 0.134)    # 2b
        self.declare_parameter("joints_left", ["joint_rueda_del_izq", "joint_rueda_tra_izq"])
        self.declare_parameter("joints_right", ["joint_rueda_del_der", "joint_rueda_tra_der"])
        self.declare_parameter("wheel_sign", 1)             # +1: giro positivo = avanzar
        self.declare_parameter("use_joint_velocity", False) # False: w = d(posición)/dt
        self.declare_parameter("init_from_odom", True)      # arranca desde la pose de /odom
        self.declare_parameter("max_dt", 0.5)  
        self.declare_parameter("ref_topic", "/ground_truth")             # descarta saltos de tiempo [s]

        p = lambda n: self.get_parameter(n).value
        self.r = float(p("wheel_radius"))
        self.L = float(p("wheel_separation"))
        self.j_izq, self.j_der = p("joints_left"), p("joints_right")
        self.ref_topic = p("ref_topic")
        self.sign = float(p("wheel_sign"))
        self.usar_vel = bool(p("use_joint_velocity"))
        self.init_odom = bool(p("init_from_odom"))
        self.max_dt = float(p("max_dt"))

        self.Q, self.W = construir_Q_W(self.r, self.L / 2.0) 
        # Calcula Q y W una sola vez. Aquí b = separación/2. Después imprime Q y W en el log.
        self.get_logger().info(f"Q (filas [D, I]) =\n{np.round(self.Q, 4)}")
        self.get_logger().info(f"W = Q^T (Q Q^T)^-1 =\n{np.round(self.W, 4)}")
        self.get_logger().info(
            f"r={self.r} m, separación={self.L} m, integración=Euler, "
            f"w desde {'velocity' if self.usar_vel else 'd(position)/dt'}")

        
        self.x = self.y = self.th = 0.0 #η(t) = [x, y, φ].
        # Con init_from_odom = True, 
        # la primera vez que llega /odom, cb_odom sobrescribe esos tres valores con la pose real del plugin
        self.xi = np.zeros(3) # ξ = [u, v, r] del último instante procesado
        self.listo = not self.init_odom # ¿ya tengo la pose inicial?
        self.t_prev, self.pos_prev = None, None # muestra anterior de /joint_states
        self.t_js = None # última vez que integré
        
        self.n = 0
        self.s_pos2 = self.s_th2 = 0.0
        self.max_pos = self.max_th = 0.0
        self.e_pos = self.e_th = 0.0
        self.dist = 0.0
        self.odom_prev = None


        self.create_subscription(JointState, "/joint_states", self.cb_joint, 50) 
        # se suscribe a joint states y ejecuta cb_jointcada vez que llega un mensaje
        self.create_subscription(Odometry, self.ref_topic, self.cb_odom, 10)
        # igual que la de arriba pero con cb_odom
        self.pub_odom = self.create_publisher(Odometry, "/odom_cinematica", 10) #tu pose calculada con las ruedas
        self.pub_err = self.create_publisher(Vector3Stamped, "/error_odometria", 10) # el error [ex, ey, eth]
        self.pub_err_pos = self.create_publisher(Float64, "/error_odometria_pos", 10) # solo la norma del error de posición
        # crea publicadores

    def _tiempo(self, stamp):
        # Convierte el stamp (hora a la que corresponde un mensaje) a segundos. Si viene en cero, 
        # usa el reloj del nodo como respaldo. Esta función es la que alimenta el dt que vimos antes.
        """Tiempo del mensaje [s]; si el stamp viene en cero usa el reloj del nodo."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    # -- /joint_states: w -> xi -> eta_dot -> integración ---------------------
    def cb_joint(self, msg):
        try:
            iD = [msg.name.index(j) for j in self.j_der]
            iI = [msg.name.index(j) for j in self.j_izq]
        except ValueError:
            self.get_logger().warn("No encuentro los joints de las ruedas en /joint_states",
                                   throttle_duration_sec=5.0)
            return

        # Skid-steer: rueda equivalente por lado = promedio de las dos ruedas de ese lado
        def prom(arr):
            return np.array([np.mean([arr[i] for i in iD]),
                             np.mean([arr[i] for i in iI])])

        t = self._tiempo(msg.header.stamp)
        pos = prom(msg.position) if len(msg.position) == len(msg.name) else None

        if not self.listo or self.t_prev is None:
            self.t_prev, self.pos_prev = t, pos
            return
        dt = t - self.t_prev
        if dt <= 1e-9:
            return
        if dt > self.max_dt:
            self.t_prev, self.pos_prev = t, pos
            return

        if self.usar_vel and len(msg.velocity) == len(msg.name):
            w = prom(msg.velocity) * self.sign
        else:
            if pos is None or self.pos_prev is None:
                return
            w = (pos - self.pos_prev) / dt * self.sign
        self.pos_prev = pos
        self.t_prev, self.t_js = t, t

        self.xi = self.W @ w                          # xi = W w  -> [u, v, r]
        self.x, self.y, self.th = paso_euler(self.x, self.y, self.th, self.xi, dt)

    # /odom publica la propia y calcula el error 
    def cb_odom(self, msg):
        xo, yo = msg.pose.pose.position.x, msg.pose.pose.position.y
        tho = yaw_de_cuaternion(msg.pose.pose.orientation)
        # Leer la pose del plugin

        if not self.listo:
            self.x, self.y, self.th = xo, yo, tho
            self.listo = True
            self.get_logger().info(f"Pose inicial tomada de /odom: ({xo:.3f}, {yo:.3f}, {tho:.3f})")
            self.get_logger().info(f"Q (filas [D, I]) =\n{np.round(self.Q, 4)}")
            self.get_logger().info(f"W = Q^T (Q Q^T)^-1 =\n{np.round(self.W, 4)}")
            return
        # La primera vez que llega /odom, tu pose toma el valor del plugin y se activa listo. 
        # A partir de ahí cb_joint puede empezar a integrar. 
        # Aquí es donde se sobrescriben los ceros que pusimos en el constructor.
        if self.t_js is None:
            return
        # Si cb_joint todavía no ha integrado nada, no hay pose propia que publicar, así que sale.
        if self.odom_prev is not None:
            self.dist += math.hypot(xo - self.odom_prev[0], yo - self.odom_prev[1])
        self.odom_prev = (xo, yo)
        #Suma cuánto se movió el plugin desde el mensaje anterior (la hipotenusa de Δx y Δy). 
        # Solo sirve para expresar el error final como porcentaje de la distancia recorrida.

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
        # Error = tuya − plugin. e_pos es la distancia entre las dos posiciones. 
        # wrap evita un falso error de 2π cuando un ángulo está cerca de ±π.

        self.n += 1
        self.s_pos2 += self.e_pos ** 2
        self.s_th2 += eth ** 2
        self.max_pos = max(self.max_pos, self.e_pos)
        self.max_th = max(self.max_th, abs(eth))

        #n cuenta las muestras. s_pos2 acumula la suma de errores al cuadrado, que luego sirve para el RMS = √(Σe²/n). 
        # max_pos guarda el peor error. Se hace lo mismo con el ángulo.
        
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