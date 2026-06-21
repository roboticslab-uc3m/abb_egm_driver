import copy
import math
import sys
import termios
import threading
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from PyKDL import Rotation

help = """
---------------------------
MANUAL CONTROL OF ABB ROBOT
---------------------------
Translation:
   w/s : +X/-X
   a/d : +Y/-Y
   q/e : +Z/-Z

Rotation:
   t/g : +RX/-RX
   f/h : +RY/-RY
   r/y : +RZ/-RZ

p: print current pose
j: print current joint configuration
i: print this help message

ENTER: enable/disable tool
SPACE: return to home
ESC:   exit
"""

TRANSLATE_BINDINGS = {
    'w': ( 0.05,  0.0,  0.0),  # Forward (+X)
    's': (-0.05,  0.0,  0.0),  # Backward (-X)
    'a': ( 0.0,   0.05, 0.0),  # Left (+Y)
    'd': ( 0.0,  -0.05, 0.0),  # Right (-Y)
    'q': ( 0.0,   0.0,  0.05), # Up (+Z)
    'e': ( 0.0,   0.0, -0.05), # Down (-Z)
}

ROTATE_BINDINGS = {
    't': ( 0.05,  0.0,  0.0),  # Rotate +RX
    'g': (-0.05,  0.0,  0.0),  # Rotate -RX
    'f': ( 0.0,   0.05, 0.0),  # Rotate +RY
    'h': ( 0.0,  -0.05, 0.0),  # Rotate -RY
    'r': ( 0.0,   0.0,  0.05), # Rotate +RZ
    'y': ( 0.0,   0.0, -0.05), # Rotate -RZ
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

        self.pose_publisher = self.create_publisher(Pose, 'command/pose', 10)
        self.do_publisher = self.create_publisher(Bool, 'command/do', 10)
        self.trajectory_publisher = self.create_publisher(Pose, 'trajectory/movel', 10)

        self.pose_callback = self.create_subscription(Pose, 'state/pose', self.pose_listener_callback, 10)
        self.joint_callback = self.create_subscription(JointState, 'state/joint', self.joint_listener_callback, 10)

        self.state_pose = None
        self.state_joint = None

        self.x = None
        self.y = None
        self.z = None

        self.rw = None
        self.rx = None
        self.ry = None
        self.rz = None

        self.do_state = False

        print(help)

        self.initialized = False

        print('Waiting for initial pose and joint state...')

    def pose_listener_callback(self, msg):
        self.state_pose = copy.copy(msg)

    def joint_listener_callback(self, msg):
        self.state_joint = list(msg.position)

    def publish_pose(self):
        pose = Pose()

        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        pose.orientation.w = self.rw
        pose.orientation.x = self.rx
        pose.orientation.y = self.ry
        pose.orientation.z = self.rz

        self.pose_publisher.publish(pose)
        print(f'Target: X={self.x:.2f} Y={self.y:.2f} Z={self.z:.2f} [m] || QW={self.rw:.2f} QX={self.rx:.2f} QY={self.ry:.2f} QZ={self.rz:.2f}')

    def publish_do(self):
        msg = Bool()
        msg.data = self.do_state
        self.do_publisher.publish(msg)
        print(f'Tool {"ENABLED" if self.do_state else "DISABLED"}')
        self.do_state = not self.do_state # set for next toggle

    def publish_home(self):
        pose = Pose()

        pose.position.x = self.x = HOME_X
        pose.position.y = self.y = HOME_Y
        pose.position.z = self.z = HOME_Z

        pose.orientation.w = self.rw = HOME_RW
        pose.orientation.x = self.rx = HOME_RX
        pose.orientation.y = self.ry = HOME_RY
        pose.orientation.z = self.rz = HOME_RZ

        self.trajectory_publisher.publish(pose)
        print("RESET TO HOME!")

def do_key_action(node, settings):
    while True:
        key = getKey(settings)

        if key in TRANSLATE_BINDINGS.keys():
            dx, dy, dz = TRANSLATE_BINDINGS[key]
            node.x += dx
            node.y += dy
            node.z += dz
            node.publish_pose()

        elif key in ROTATE_BINDINGS.keys():
            drx, dry, drz = ROTATE_BINDINGS[key]
            rot = Rotation.Quaternion(node.rx, node.ry, node.rz, node.rw)
            rot.DoRotX(drx)
            rot.DoRotY(dry)
            rot.DoRotZ(drz)
            node.rx, node.ry, node.rz, node.rw = rot.GetQuaternion()
            node.publish_pose()

        elif key == '\r': # ENTER
            node.publish_do()

        elif key == ' ':
            node.publish_home()

        elif key == 'p':
            if node.state_pose is not None:
                p = node.state_pose.position
                q = node.state_pose.orientation
                print(f'Current pose: X={p.x:.2f} Y={p.y:.2f} Z={p.z:.2f} [m] || QW={q.w:.2f} QX={q.x:.2f} QY={q.y:.2f} QZ={q.z:.2f}')
            else:
                print('No pose state received yet.')

        elif key == 'j':
            if node.state_joint is not None:
                joints_str = ', '.join(map('{:.2f}'.format, map(math.degrees, node.state_joint)))
                print(f'Current joint configuration: [{joints_str}] [deg]')
            else:
                print('No joint state received yet.')

        elif key == 'i':
            print(help)

        elif key == '\x1b': # ESC
            rclpy.shutdown()
            break

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCommander()
    thread = None

    try:
        while not node.initialized:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.state_pose is not None and node.state_joint is not None:
                node.x = node.state_pose.position.x
                node.y = node.state_pose.position.y
                node.z = node.state_pose.position.z

                node.rw = node.state_pose.orientation.w
                node.rx = node.state_pose.orientation.x
                node.ry = node.state_pose.orientation.y
                node.rz = node.state_pose.orientation.z

                node.initialized = True

                print('Initial pose and joint state received. You can now control the robot.')

        thread = threading.Thread(target=do_key_action, args=(node, settings))
        thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if thread is not None:
            thread.join()

        node.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
