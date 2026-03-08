import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Configuración de teclas
msg = """
---------------------------
CONTROL MANUAL DE ROBOT ABB
---------------------------
Moverse:
   w
 a s d    (X / Y)

Subir/Bajar:
   q
   e      (Z)

ESPACIO: Parada de Emergencia (Vuelve a Home)
CTRL-C: Salir
"""

# Mapa de teclas -> Movimiento (x, y, z) en MILÍMETROS por pulsación
MOVE_BINDINGS = {
    'w': ( 0.05,  0.0,  0.0),  # Adelante (+X) 5cm
    's': (-0.05,  0.0,  0.0),  # Atrás (-X)
    'a': ( 0.0,   0.05, 0.0),  # Izquierda (+Y)
    'd': ( 0.0,  -0.05, 0.0),  # Derecha (-Y)
    'q': ( 0.0,   0.0,  0.05), # Arriba (+Z)
    'e': ( 0.0,   0.0, -0.05), # Abajo (-Z)
}

# Posición inicial (Debe coincidir aprox con donde está tu robot ahora)
HOME_X = 0.45
HOME_Y = 0.0
HOME_Z = 0.45

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardCommander(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Pose, 'robot_command', 10)
        self.x = HOME_X
        self.y = HOME_Y
        self.z = HOME_Z
        print(msg)

    def publish_pos(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        # Orientación fija mirando abajo
        pose.orientation.w = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0

        self.publisher_.publish(pose)
        print(f'Moviendo a: X={self.x:.2f} Y={self.y:.2f} Z={self.z:.2f}\r')

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCommander()

    try:
        # Enviamos la posición inicial al arrancar
        node.publish_pos()

        while True:
            key = getKey(settings)

            if key in MOVE_BINDINGS.keys():
                dx, dy, dz = MOVE_BINDINGS[key]
                node.x += dx
                node.y += dy
                node.z += dz
                node.publish_pos()

            elif key == ' ':
                # Reset a casa
                node.x = HOME_X
                node.y = HOME_Y
                node.z = HOME_Z
                print("\n¡RESET A HOME!\n")
                node.publish_pos()

            elif key == '\x03': # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
