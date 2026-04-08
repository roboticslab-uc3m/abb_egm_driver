import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
from ABBRobotEGM import EGM
from enum import Enum
import math
import threading
import time

class CommandMode(Enum):
    POSE = 'pose'
    JOINT = 'joint'
    CORR = 'corr'

# default EGM communication port
EMG_PORT = 6510

# default 4 ms period for EGM communication (250 Hz)
EGM_PERIOD = 4

# default minimum period for path correction mode
EGM_PATH_CORR_PERIOD = 24

# EMA factor (low-pass filter, lower is smoother)
SMOOTH_FACTOR = 0.02

# ROS state publish period (in milliseconds)
PUBLISH_PERIOD = 10

# default guidance mode for incoming commands (pose or joint)
EGM_MODE = CommandMode.POSE

class EGMDriver(Node):
    def __init__(self):
        super().__init__('abb_egm_driver')
        self.get_logger().info('Starting EGM Driver...')

        egm_port_param = self.declare_parameter('egm_port', EMG_PORT,
                                                ParameterDescriptor(description='Port for EGM communication',
                                                                    read_only=True))

        self.egm_port = egm_port_param.get_parameter_value().integer_value

        if self.egm_port <= 0 or self.egm_port > 65535:
            self.get_logger().warning(f'Invalid EGM port number. It must be between 1 and 65535. Using default port: {EMG_PORT}')
            self.egm_port = EMG_PORT
        else:
            self.get_logger().info(f'Using EGM port: {self.egm_port}')

        self.current_joint_position = None
        self.current_pos = None
        self.current_orient = None

        self.current_send_joint_position = None
        self.current_send_pos = None
        self.current_send_orient = None

        self.target_joint_position = None
        self.target_pos = None
        self.target_orient = None

        smooth_factor_param = self.declare_parameter('smooth_factor', SMOOTH_FACTOR,
                                                      ParameterDescriptor(description='Smoothing factor for low-pass filter (lower is smoother)',
                                                                          additional_constraints='0.0 <= smooth_factor <= 1.0'))

        self.smooth_factor = smooth_factor_param.get_parameter_value().double_value

        if self.smooth_factor < 0.0 or self.smooth_factor > 1.0:
            self.get_logger().warning(f'Invalid smooth_factor value. It must be between 0.0 and 1.0. Using default value: {SMOOTH_FACTOR}')
            self.smooth_factor = SMOOTH_FACTOR
        else:
            self.get_logger().info(f'Using smooth_factor: {self.smooth_factor}')

        publish_period_param = self.declare_parameter('publish_period', PUBLISH_PERIOD,
                                                      ParameterDescriptor(description='Period for publishing robot state (in milliseconds, use <= 0 for no publishing)',
                                                                          read_only=True))

        publish_period = publish_period_param.get_parameter_value().integer_value

        if publish_period <= 0:
            self.get_logger().info('Publishing of robot state is disabled (publish_period <= 0).')
        else:
            self.get_logger().info(f'Publishing of robot state is enabled (publish_period: {publish_period} milliseconds).')
            self.publisher_joint = self.create_publisher(JointState, 'state/joint', 10)
            self.publisher_pose = self.create_publisher(Pose, 'state/pose', 10)
            self.timer = self.create_timer(publish_period * 0.001, self.timer_callback)

        command_mode_param = self.declare_parameter('command_mode', 'pose',
                                                    ParameterDescriptor(description='Control mode for incoming commands (pose, joint or corr)',
                                                                        additional_constraints='command_mode must be either "pose", "joint" or "corr"',
                                                                        read_only=True))

        command_mode_str = command_mode_param.get_parameter_value().string_value.lower()

        if command_mode_str not in ['pose', 'joint', 'corr']:
            self.get_logger().warning(f'Invalid command_mode value. It must be either "pose", "joint" or "corr". Using default mode: pose')
            self.command_mode = EGM_MODE
        else:
            self.command_mode = CommandMode(command_mode_str)
            self.get_logger().info(f'Using command_mode: {self.command_mode.value}')

        command_period_param = self.declare_parameter('command_period', EGM_PATH_CORR_PERIOD if self.command_mode == CommandMode.CORR else EGM_PERIOD,
                                                       ParameterDescriptor(description='Command period for EGM communication (in milliseconds)',
                                                                           additional_constraints='Must be a multiple of 4 (pose and joint mode) or 24 (path correction mode)',
                                                                           read_only=True))

        command_period = command_period_param.get_parameter_value().integer_value

        if command_period <= 0 or command_period % EGM_PERIOD != 0 and self.command_mode in [CommandMode.POSE, CommandMode.JOINT]:
            self.get_logger().warning(f'Command period of {command_period} ms is not a positive multiple of {EGM_PERIOD}, forcing to {EGM_PERIOD} ms.')
            command_period = EGM_PERIOD
        elif command_period <= 0 or command_period % EGM_PATH_CORR_PERIOD != 0 and self.command_mode == CommandMode.CORR:
            self.get_logger().warning(f'Command period of {command_period} ms is not a positive multiple of {EGM_PATH_CORR_PERIOD}, forcing to {EGM_PATH_CORR_PERIOD} ms.')
            command_period = EGM_PATH_CORR_PERIOD

        self.divisor = command_period / EGM_PERIOD
        self.counter = 0

        if self.command_mode == CommandMode.POSE:
            self.subscription = self.create_subscription(Pose, 'command/pose', self.pose_listener_callback, 10)
        elif self.command_mode == CommandMode.JOINT:
            self.subscription = self.create_subscription(Float32MultiArray, 'command/joint', self.joint_listener_callback, 10)
        elif self.command_mode == CommandMode.CORR:
            self.subscription = self.create_subscription(Point, 'command/path_corr', self.corr_listener_callback, 10)
        else:
            self.get_logger().error('Invalid command mode. This should never happen due to parameter validation.')

        self.running = True
        self.initialized = False

        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

        self.get_logger().info('EGM Driver is ready and running.')

    def pose_listener_callback(self, msg):
        self.target_pos = [msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0]
        self.target_orient = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    def joint_listener_callback(self, msg):
        if self.current_joint_position is None:
            self.get_logger().warning('Received joint command before robot state is available. Ignoring command.')
            return

        if len(msg.data) != len(self.current_joint_position):
            self.get_logger().warning(f'Received joint command with incorrect number of joints. Expected {len(self.current_joint_position)}, got {len(msg.data)}.')
            return

        self.target_joint_position = list(map(math.degrees, msg.data))

    def corr_listener_callback(self, msg):
        self.target_pos = [msg.x * 1000.0, msg.y * 1000.0, msg.z * 1000.0]

    def timer_callback(self):
        if self.current_joint_position is not None:
            joint_msg = JointState()
            joint_msg.position = list(map(math.radians, self.current_joint_position))
            self.publisher_joint.publish(joint_msg)

        if self.current_pos is not None and self.current_orient is not None:
            pose_msg = Pose()

            pose_msg.position.x = self.current_pos[0] / 1000.0
            pose_msg.position.y = self.current_pos[1] / 1000.0
            pose_msg.position.z = self.current_pos[2] / 1000.0

            pose_msg.orientation.w = self.current_orient[0]
            pose_msg.orientation.x = self.current_orient[1]
            pose_msg.orientation.y = self.current_orient[2]
            pose_msg.orientation.z = self.current_orient[3]

            self.publisher_pose.publish(pose_msg)

    def filter(self, current, target):
        return current + (target - current) * self.smooth_factor

    def send_command(self, egm):
        if self.initialized and self.counter % self.divisor == 0:
            if self.command_mode == CommandMode.JOINT:
                egm.send_to_robot(self.current_send_joint_position) # type: ignore
            elif self.command_mode == CommandMode.POSE:
                egm.send_to_robot_cart(self.current_send_pos, self.current_send_orient) # type: ignore
            elif self.command_mode == CommandMode.CORR:
                egm.send_to_robot_path_corr(self.current_send_pos) # type: ignore

    def run_egm_loop(self):
        with EGM(port=self.egm_port) as egm:
            self.get_logger().info('Waiting response from robot...')

            startup_counter = 0
            INITIAL_STABILIZATION_CYCLES = 100

            while self.running:
                success, state = egm.receive_from_robot()

                if not success or state is None or state.cartesian is None:
                    self.get_logger().warning('Failed to receive robot state. Retrying...')
                    continue

                self.current_joint_position = state.joint_angles.tolist()
                self.current_pos = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
                self.current_orient = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]

                if startup_counter < INITIAL_STABILIZATION_CYCLES:
                    self.current_send_joint_position = self.current_joint_position
                    self.current_send_pos = self.current_pos
                    self.current_send_orient = self.current_orient

                    # Ignore ROS commands until we have a stable reading from the robot
                    self.target_joint_position = self.current_joint_position
                    self.target_pos = self.current_pos
                    self.target_orient = self.current_orient

                    startup_counter += 1
                    continue

                if not self.initialized:
                    self.get_logger().info('Robot state received. Entering control loop.')
                    self.initialized = True

                # Apply low-pass filter (exponential moving average) to position
                self.current_send_joint_position = [self.filter(self.current_send_joint_position[i], self.target_joint_position[i]) for i in range(len(self.current_send_joint_position))] # type: ignore
                self.current_send_pos = [self.filter(self.current_send_pos[i], self.target_pos[i]) for i in range(3)] # type: ignore
                self.current_send_orient = self.target_orient

                self.counter += 1
                self.send_command(egm)
                time.sleep(EGM_PERIOD * 0.001) # convert ms to seconds

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
        node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
