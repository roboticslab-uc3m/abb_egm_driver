import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ABBRobotEGM import EGM
import threading
import time

# Factor de suavizado (Cuanto más bajo, más suave)
SMOOTH_FACTOR = 0.02

class EGMDriver(Node):
    def __init__(self):
        super().__init__('abb_egm_driver')

        self.subscription = self.create_subscription(
            Pose, 'robot_command', self.listener_callback, 10)

        self.get_logger().info('🛡️ Driver EGM SEGURO iniciado.')

        # Variables de control
        self.target_pos = None
        self.target_orient = None
        self.current_send_pos = None
        self.current_send_orient = None

        self.running = True
        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

    def listener_callback(self, msg):
        # Conversión M -> MM
        self.target_pos = [msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0]
        self.target_orient = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        # self.get_logger().info('Nueva orden recibida.')

    def interpolate(self, current, target, factor):
        return current + (target - current) * factor

    def run_egm_loop(self):
        with EGM() as egm:
            self.get_logger().info('✅ Esperando al Robot...')

            # CONTADOR DE SEGURIDAD
            startup_counter = 0
            INITIAL_STABILIZATION_CYCLES = 100 # Primeros 100 paquetes solo escuchamos

            while self.running:
                success, state = egm.receive_from_robot()
                if not success: continue

                # Leer posición REAL
                robot_pos = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
                robot_orient = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]

                # FASE 1: ARRANQUE (Primeros instantes)
                # Simplemente copiamos lo que hace el robot para no dar tirones
                if startup_counter < INITIAL_STABILIZATION_CYCLES:
                    self.current_send_pos = robot_pos
                    self.current_send_orient = robot_orient

                    # Si recibimos ordenes de ROS durante el arranque, las ignoramos hasta estar estables
                    # o forzamos que el objetivo sea la posición actual
                    if self.target_pos is None:
                        self.target_pos = robot_pos
                        self.target_orient = robot_orient

                    startup_counter += 1
                    # Enviamos de vuelta LO MISMO que recibimos (Efecto espejo)
                    egm.send_to_robot(cartesian=(robot_pos, robot_orient))
                    continue

                # FASE 2: CONTROL SUAVE
                # Si no hay orden nueva, el objetivo es quedarse donde estamos
                if self.target_pos is None:
                    self.target_pos = self.current_send_pos
                    self.target_orient = self.current_send_orient

                # Interpolación de Posición
                self.current_send_pos[0] = self.interpolate(self.current_send_pos[0], self.target_pos[0], SMOOTH_FACTOR)
                self.current_send_pos[1] = self.interpolate(self.current_send_pos[1], self.target_pos[1], SMOOTH_FACTOR)
                self.current_send_pos[2] = self.interpolate(self.current_send_pos[2], self.target_pos[2], SMOOTH_FACTOR)

                # Interpolación de Orientación (Simplificada: Copia directa si hay orden)
                # Para evitar giros locos, solo actualizamos orientación si hay una orden explícita
                if self.target_orient is not None:
                     self.current_send_orient = self.target_orient
                else:
                     self.current_send_orient = robot_orient

                egm.send_to_robot(cartesian=(self.current_send_pos, self.current_send_orient))

def main(args=None):
    rclpy.init(args=args)
    node = EGMDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        node.egm_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
