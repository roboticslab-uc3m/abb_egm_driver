import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor
from ABBRobotEGM import EGM
import threading
import time

# EMA factor (low-pass filter, lower is smoother)
SMOOTH_FACTOR = 0.02

# ROS state publish period (in seconds)
PUBLISH_PERIOD = 0.01

class EGMDriver(Node):
    def __init__(self):
        super().__init__('abb_egm_driver')
        self.get_logger().info('Starting EGM Driver...')

        self.target_pos = None
        self.target_orient = None
        self.current_pos = None
        self.current_orient = None
        self.current_send_pos = None
        self.current_send_orient = None

        smooth_factor_param = self.declare_parameter('smooth_factor', SMOOTH_FACTOR,
                                                      ParameterDescriptor(description='Smoothing factor for low-pass filter (lower is smoother)',
                                                                          additional_constraints='0.0 <= smooth_factor <= 1.0'))

        self.smooth_factor = smooth_factor_param.get_parameter_value().double_value

        if self.smooth_factor < 0.0 or self.smooth_factor > 1.0:
            self.get_logger().warning('Invalid smooth_factor value. It must be between 0.0 and 1.0. Using default value: {}'.format(SMOOTH_FACTOR))
            self.smooth_factor = SMOOTH_FACTOR
        else:
            self.get_logger().info(f'Using smooth_factor: {self.smooth_factor}')

        publish_period_param = self.declare_parameter('publish_period', PUBLISH_PERIOD,
                                                      ParameterDescriptor(description='Period for publishing robot state (in seconds, use <= 0 for no publishing)',
                                                                          read_only=True))

        self.publish_period = publish_period_param.get_parameter_value().double_value

        if self.publish_period <= 0.0:
            self.get_logger().info('Publishing of robot state is disabled (publish_period <= 0).')
        else:
            self.get_logger().info(f'Publishing of robot state is enabled (publish_period: {self.publish_period} seconds).')
            self.publisher = self.create_publisher(Pose, 'state/pose', 10)
            self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.subscription = self.create_subscription(Pose, 'pose', self.listener_callback, 10)

        self.running = True
        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

        self.get_logger().info('EGM Driver is ready and running.')

    def listener_callback(self, msg):
        self.target_pos = [msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0]
        self.target_orient = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    def timer_callback(self):
        if self.current_pos is not None and self.current_orient is not None:
            msg = Pose()
            msg.position.x = self.current_pos[0] / 1000.0
            msg.position.y = self.current_pos[1] / 1000.0
            msg.position.z = self.current_pos[2] / 1000.0
            msg.orientation.w = self.current_orient[0]
            msg.orientation.x = self.current_orient[1]
            msg.orientation.y = self.current_orient[2]
            msg.orientation.z = self.current_orient[3]
            self.publisher.publish(msg)

    def filter(self, current, target):
        return current + (target - current) * self.smooth_factor

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
                self.current_pos = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
                self.current_orient = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]

                # PHASE 1: STARTUP (copy whatever the robot is doing to avoid initial jerky motion)
                if startup_counter < INITIAL_STABILIZATION_CYCLES:
                    self.current_send_pos = self.current_pos
                    self.current_send_orient = self.current_orient

                    # Ignore ROS commands until we have a stable reading from the robot
                    if self.target_pos is None:
                        self.target_pos = self.current_pos
                        self.target_orient = self.current_orient

                    startup_counter += 1
                    egm.send_to_robot_cart(self.current_pos, self.current_orient) # type: ignore
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
                self.current_send_pos = [self.filter(self.current_send_pos[i], self.target_pos[i]) for i in range(3)] # type: ignore
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
