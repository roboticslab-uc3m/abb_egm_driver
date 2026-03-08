import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ABBRobotEGM import EGM
import threading
import time

# EMA factor (low-pass filter, lower is smoother)
SMOOTH_FACTOR = 0.02

class EGMDriver(Node):
    def __init__(self):
        super().__init__('abb_egm_driver')

        self.subscription = self.create_subscription(Pose, 'pose', self.listener_callback, 10)

        self.get_logger().info('Starting EGM Driver...')

        self.target_pos = None
        self.target_orient = None
        self.current_send_pos = None
        self.current_send_orient = None

        self.running = True
        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

        self.get_logger().info('EGM Driver is ready and running.')

    def listener_callback(self, msg):
        self.target_pos = [msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0]
        self.target_orient = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    def interpolate(self, current, target, factor):
        return current + (target - current) * factor

    def run_egm_loop(self):
        with EGM() as egm:
            self.get_logger().info('Waiting response from robot...')

            startup_counter = 0
            notify_initial_startup = True
            INITIAL_STABILIZATION_CYCLES = 100

            while self.running:
                success, state = egm.receive_from_robot()
                if not success or state is None or state.cartesian is None:
                    self.get_logger().warning('Failed to receive robot state. Retrying...')
                    continue

                # Current actual pose
                robot_pos = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
                robot_orient = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]

                # PHASE 1: STARTUP (copy whatever the robot is doing to avoid initial jerky motion)
                if startup_counter < INITIAL_STABILIZATION_CYCLES:
                    self.current_send_pos = robot_pos
                    self.current_send_orient = robot_orient

                    # Ignore ROS commands until we have a stable reading from the robot
                    if self.target_pos is None:
                        self.target_pos = robot_pos
                        self.target_orient = robot_orient

                    startup_counter += 1
                    egm.send_to_robot_cart(robot_pos, robot_orient) # type: ignore
                    continue

                if notify_initial_startup:
                    self.get_logger().info('Robot state received. Entering control loop.')
                    notify_initial_startup = False

                # PHASE 2: SMOOTH CONTROL (keep current pose in absence of pending commands)
                if self.target_pos is None:
                    self.target_pos = self.current_send_pos

                if self.target_orient is None:
                    self.target_orient = self.current_send_orient

                # Apply low-pass filter (exponential moving average) to position
                self.current_send_pos = [self.interpolate(self.current_send_pos[i], self.target_pos[i], SMOOTH_FACTOR) for i in range(3)] # type: ignore
                self.current_send_orient = self.target_orient

                egm.send_to_robot_cart(self.current_send_pos, self.current_send_orient) # type: ignore
                time.sleep(0.004)

def main(args=None):
    rclpy.init(args=args)
    node = EGMDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.egm_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
