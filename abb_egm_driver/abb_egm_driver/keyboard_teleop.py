import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

help = """
---------------------------
MANUAL CONTROL OF ABB ROBOT
---------------------------
Motion:
   w
 a s d    (X / Y)

Up/Down:
   q
   e      (Z)

SPACE: Emergency Stop (Return to Home)
CTRL-C: Exit
"""

MOVE_BINDINGS = {
    'w': ( 0.05,  0.0,  0.0),  # Forward (+X)
    's': (-0.05,  0.0,  0.0),  # Backward (-X)
    'a': ( 0.0,   0.05, 0.0),  # Left (+Y)
    'd': ( 0.0,  -0.05, 0.0),  # Right (-Y)
    'q': ( 0.0,   0.0,  0.05), # Up (+Z)
    'e': ( 0.0,   0.0, -0.05), # Down (-Z)
}

HOME_X = 0.45
HOME_Y = 0.0
HOME_Z = 0.45

HOME_RW = 0.0
HOME_RX = 0.0
HOME_RY = 1.0
HOME_RZ = 0.0

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardCommander(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.publisher = self.create_publisher(Pose, 'command/pose', 10)

        self.x = HOME_X
        self.y = HOME_Y
        self.z = HOME_Z

        self.rw = HOME_RW
        self.rx = HOME_RX
        self.ry = HOME_RY
        self.rz = HOME_RZ

        print(help)

    def publish_pos(self):
        pose = Pose()

        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        pose.orientation.w = self.rw
        pose.orientation.x = self.rx
        pose.orientation.y = self.ry
        pose.orientation.z = self.rz

        self.publisher.publish(pose)
        print(f'Target: X={self.x:.2f} Y={self.y:.2f} Z={self.z:.2f}\r')

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCommander()

    try:
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
                node.x = HOME_X
                node.y = HOME_Y
                node.z = HOME_Z

                node.rw = HOME_RW
                node.rx = HOME_RX
                node.ry = HOME_RY
                node.rz = HOME_RZ

                print("\nRESET TO HOME!\n")
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
