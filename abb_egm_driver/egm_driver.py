import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from ABBRobotEGM import EGM
from enum import Enum
import math
import threading
import PyKDL as kdl
import abb_egm_driver.motion3 as m3

from abb_egm_driver.parameters import egm_driver

class Mode(Enum):
    POSE = 'pose'
    JOINT = 'joint'
    CORR = 'corr'

# default 4 ms period for EGM communication (250 Hz)
EGM_PERIOD = 4

# default minimum period for path correction mode (ms)
EGM_PATH_CORR_PERIOD = 24

# fixed array size of 10 for incoming data from RAPID
DATA_LENGTH = 40

class EGMDriver(Node):
    def __init__(self):
        super().__init__('abb_egm_driver')
        self.get_logger().info('Starting EGM Driver...')

        self.param_listener = egm_driver.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.current_joint = None
        self.current_pos = None
        self.current_orient = None

        self.current_send_joint = None
        self.current_send_pos = None
        self.current_send_orient = None
        self.current_send_corr = [0.0, 0.0, 0.0]

        self.target_joint = None
        self.target_pos = None
        self.target_orient = None
        self.target_corr = [0.0, 0.0, 0.0]

        self.send_do = None

        self.data_in = None
        self.data_out = None

        self.lin_trajectory = None
        self.joint_profiles = None
        self.processing_trajectory = None
        self.trajectory_start_time = None

        self.get_logger().info(f'Using EGM port: {self.params.egm_port}.')
        self.get_logger().info(f'Using smooth_factor: {self.params.smooth_factor}.')

        self.get_logger().info(f'Using publish_period: {self.params.publish_period} ms.')
        self.publisher_joint = self.create_publisher(JointState, 'state/joint', 10)
        self.publisher_pose = self.create_publisher(Pose, 'state/pose', 10)
        self.publisher_data = self.create_publisher(Float64MultiArray, 'state/data', 10)
        self.timer = self.create_timer(self.params.publish_period * 0.001, self.timer_callback)

        self.get_logger().info(f'Using command_mode: {self.params.command_mode}.')

        if self.params.command_period % EGM_PERIOD != 0 and self.params.command_mode in [Mode.POSE.value, Mode.JOINT.value]:
            self.get_logger().warning(f'Command period of {self.params.command_period} ms is not a positive multiple of {EGM_PERIOD}, forcing to {EGM_PERIOD} ms.')
            self.params.command_period = EGM_PERIOD
        elif self.params.command_period % EGM_PATH_CORR_PERIOD != 0 and self.params.command_mode == Mode.CORR.value:
            self.get_logger().warning(f'Command period of {self.params.command_period} ms is not a positive multiple of {EGM_PATH_CORR_PERIOD}, forcing to {EGM_PATH_CORR_PERIOD} ms.')
            self.params.command_period = EGM_PATH_CORR_PERIOD

        self.divisor = self.params.command_period / EGM_PERIOD
        self.counter = 0
        self.using_dh_joint_cmd = False

        if self.params.command_mode == Mode.POSE.value:
            self.subscription_pose_cmd = self.create_subscription(Pose, 'command/pose', self.pose_listener_callback, 10)
            self.subscription_traj_movel = self.create_subscription(Pose, 'trajectory/movel', self.trajectory_movel_listener_callback, 10)

            self.get_logger().info(f'Using max_lin_velocity (trajectories): {self.params.max_lin_velocity} mm/s.')
            self.get_logger().info(f'Using max_lin_acceleration (trajectories): {self.params.max_lin_acceleration} mm/s^2.')

            if self._parse_kinematic_parameters():
                self.subscription_joint_cmd = self.create_subscription(Float32MultiArray, 'command/joint', self.joint_listener_callback, 10)
                self.subscription_traj_joint = self.create_subscription(Float32MultiArray, 'trajectory/joint', self.trajectory_joint_listener_callback, 10)
                self.subscription_traj_movej = self.create_subscription(Pose, 'trajectory/movej', self.trajectory_movej_listener_callback, 10)

                self.get_logger().info(f'Using max_joint_velocity (trajectories): {self.params.max_joint_velocity} deg/s.')
                self.get_logger().info(f'Using max_joint_acceleration (trajectories): {self.params.max_joint_acceleration} deg/s^2.')

                self.using_dh_joint_cmd = True
                self.get_logger().info('DH parameters provided, joint commands are enabled in pose command mode.')
            else:
                self.get_logger().info('No valid DH parameters provided, joint commands are disabled in pose command mode.')
        elif self.params.command_mode == Mode.JOINT.value:
            self.subscription_joint_cmd = self.create_subscription(Float32MultiArray, 'command/joint', self.joint_listener_callback, 10)
            self.subscription_traj_joint = self.create_subscription(Float32MultiArray, 'trajectory/joint', self.trajectory_joint_listener_callback, 10)

            self.get_logger().info(f'Using max_joint_velocity (trajectories): {self.params.max_joint_velocity} deg/s.')
            self.get_logger().info(f'Using max_joint_acceleration (trajectories): {self.params.max_joint_acceleration} deg/s^2.')

            if self._parse_kinematic_parameters():
                self.subscription_traj_movej = self.create_subscription(Pose, 'trajectory/movej', self.trajectory_movej_listener_callback, 10)
        elif self.params.command_mode == Mode.CORR.value:
            self.subscription_corr_cmd = self.create_subscription(Point, 'command/path_corr', self.corr_listener_callback, 10)
        else:
            self.get_logger().error(f'Invalid command mode "{self.params.command_mode}". This should never happen due to parameter validation.')

        if self.params.command_mode != Mode.CORR.value:
            self.subscription_do = self.create_subscription(Bool, 'command/do', self.do_listener_callback, 10)
            self.subscription_data = self.create_subscription(Float64MultiArray, 'command/data', self.data_listener_callback, 10)

        self.running = True
        self.initialized = False

        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

        self.get_logger().info('EGM Driver is ready and running.')

    def _parse_kinematic_parameters(self):
        self.chain = kdl.Chain()

        self.min_limits = kdl.JntArray(len(self.params.dh_parameters.links))
        self.max_limits = kdl.JntArray(len(self.params.dh_parameters.links))

        for i, link in enumerate(self.params.dh_parameters.links):
            theta = self.params.dh_parameters.get_entry(link).theta
            D = self.params.dh_parameters.get_entry(link).D
            A = self.params.dh_parameters.get_entry(link).A
            alpha = self.params.dh_parameters.get_entry(link).alpha

            if all(param == 0.0 for param in [theta, D, A, alpha]):
                break # stop parsing if all parameters are zero, indicating no more valid links

            min_limit = self.params.dh_parameters.get_entry(link).min_limit
            max_limit = self.params.dh_parameters.get_entry(link).max_limit

            self.min_limits[i] = math.radians(min_limit)
            self.max_limits[i] = math.radians(max_limit)

            self.get_logger().info(f'Parsed DH parameters for {link}: theta={theta}°, D={D} mm, A={A} mm, alpha={alpha}°. Limits: min={min_limit}°, max={max_limit}°.')

            frame = kdl.Frame.DH(A, math.radians(alpha), D, math.radians(theta))
            segment = kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame)
            self.chain.addSegment(segment)

        if self.chain.getNrOfSegments() == 0:
            return False

        self.min_limits.resize(self.chain.getNrOfJoints())
        self.max_limits.resize(self.chain.getNrOfJoints())

        self.get_logger().info(f'Total DH parameters parsed successfully. Robot model has {self.chain.getNrOfJoints()} joints.')

        tcp_x = self.params.tcp_frame.x
        tcp_y = self.params.tcp_frame.y
        tcp_z = self.params.tcp_frame.z

        tcp_roll = self.params.tcp_frame.roll
        tcp_pitch = self.params.tcp_frame.pitch
        tcp_yaw = self.params.tcp_frame.yaw

        if any(param != 0.0 for param in [tcp_x, tcp_y, tcp_z, tcp_roll, tcp_pitch, tcp_yaw]):
            self.get_logger().info(f'TCP frame provided: x={tcp_x} mm, y={tcp_y} mm, z={tcp_z} mm, roll={tcp_roll}°, pitch={tcp_pitch}°, yaw={tcp_yaw}°.')
            tcp_rotation = kdl.Rotation.RPY(math.radians(tcp_roll), math.radians(tcp_pitch), math.radians(tcp_yaw))
            tcp_translation = kdl.Vector(tcp_x, tcp_y, tcp_z)
            tcp_frame = kdl.Frame(tcp_rotation, tcp_translation)
            segment = kdl.Segment(kdl.Joint(kdl.Joint.Fixed), tcp_frame)
            self.chain.addSegment(segment)

        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver_vel = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver_pos = kdl.ChainIkSolverPos_NR_JL(self.chain, self.min_limits, self.max_limits, self.fk_solver_pos, self.ik_solver_vel)

        return True

    def _solve_fk(self, joint_angles_rad):
        q = kdl.JntArray(self.chain.getNrOfJoints())
        H = kdl.Frame()

        for i in range(len(joint_angles_rad)):
            q[i] = joint_angles_rad[i]

        self.fk_solver_pos.JntToCart(q, H)
        (kx, ky, kz, kw) = H.M.GetQuaternion()

        pos = [H.p.x(), H.p.y(), H.p.z()]
        ori = [kw, kx, ky, kz]

        return pos, ori

    def pose_listener_callback(self, msg):
        self.target_pos = [msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0]
        self.target_orient = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.processing_trajectory = None

    def trajectory_movel_listener_callback(self, msg):
        if self.current_pos is None or self.current_orient is None:
            self.get_logger().warning('Received linear trajectory command before robot state is available. Ignoring command.')
            return

        start_pos = kdl.Vector(self.current_pos[0], self.current_pos[1], self.current_pos[2])
        start_orient = kdl.Rotation.Quaternion(self.current_orient[1], self.current_orient[2], self.current_orient[3], self.current_orient[0])

        end_pos = kdl.Vector(msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0)
        end_orient = kdl.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        H_base_start = kdl.Frame(start_orient, start_pos)
        H_base_end = kdl.Frame(end_orient, end_pos)

        path = m3.PathLine(H_base_start, H_base_end)

        if self.params.max_lin_acceleration > 0:
            profile = m3.VelocityProfileTrapezoidal(self.params.max_lin_velocity, self.params.max_lin_acceleration)
            type_str = 'trapezoidal'
        else:
            profile = m3.VelocityProfileRectangular(self.params.max_lin_velocity)
            type_str = 'rectangular'

        profile.set_profile(0, path.path_length())
        self.lin_trajectory = m3.TrajectorySegment(path, profile, profile.duration())

        self.trajectory_start_time = self.get_clock().now()
        self.processing_trajectory = 'lin'

        self.get_logger().info(f'Executing {type_str} linear trajectory. Duration: {self.lin_trajectory.duration():.2f} s, path length: {path.path_length():.2f} mm.')

    def trajectory_movej_listener_callback(self, msg):
        if self.current_joint is None:
            self.get_logger().warning('Received joint command before robot state is available. Ignoring command.')
            return

        q = kdl.JntArray(self.chain.getNrOfJoints())
        qd = kdl.JntArray(self.chain.getNrOfJoints())

        for i in range(len(self.current_joint)):
            q[i] = math.radians(self.current_joint[i])

        xd = kdl.Frame(kdl.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                       kdl.Vector(msg.position.x * 1000.0, msg.position.y * 1000.0, msg.position.z * 1000.0))

        if self.ik_solver_pos.CartToJnt(q, xd, qd) >= 0:
            qd_deg_str = ', '.join([f'{math.degrees(qd[i]):.2f}' for i in range(qd.rows())])
            self.get_logger().info(f'IK solution found: [{qd_deg_str}] [deg]')
            msg = Float32MultiArray()
            msg.data = [qd[i] for i in range(qd.rows())]
            self.trajectory_joint_listener_callback(msg)
        else:
            self.get_logger().warning('Inverse kinematics failed for the given target pose. Ignoring command.')

    def joint_listener_callback(self, msg):
        if self.current_joint is None:
            self.get_logger().warning('Received joint command before robot state is available. Ignoring command.')
            return

        if len(msg.data) != len(self.current_joint):
            self.get_logger().warning(f'Received joint command with incorrect number of joints. Expected {len(self.current_joint)}, got {len(msg.data)}.')
            return

        if not self.using_dh_joint_cmd:
            self.target_joint = list(map(math.degrees, msg.data))
        elif len(msg.data) != self.chain.getNrOfJoints():
            self.get_logger().warning(f'Received joint command with incorrect number of joints for DH model. Expected {self.chain.getNrOfJoints()}, got {len(msg.data)}.')
        else:
            self.target_pos, self.target_orient = self._solve_fk(msg.data)
            self.processing_trajectory = None

    def trajectory_joint_listener_callback(self, msg):
        if self.current_joint is None:
            self.get_logger().warning('Received joint trajectory before robot state is available. Ignoring command.')
            return

        if len(msg.data) != len(self.current_joint):
            self.get_logger().warning(f'Received joint trajectory with incorrect number of joints. Expected {len(self.current_joint)}, got {len(msg.data)}.')
            return

        if len(msg.data) != self.chain.getNrOfJoints():
            self.get_logger().warning(f'Received joint trajectory with incorrect number of joints for DH model. Expected {self.chain.getNrOfJoints()}, got {len(msg.data)}.')
            return

        targets = list(map(math.degrees, msg.data))
        diffs = [abs(a - b) for a, b in zip(targets, self.current_joint)]
        max_distance = max(diffs)

        if self.params.max_joint_acceleration > 0:
            self.joint_profiles = [m3.VelocityProfileTrapezoidal(self.params.max_joint_velocity, self.params.max_joint_acceleration) for i in range(len(msg.data))]
            type_str = 'trapezoidal'
        else:
            self.joint_profiles = [m3.VelocityProfileRectangular(self.params.max_joint_velocity) for i in range(len(msg.data))]
            type_str = 'rectangular'

        index = diffs.index(max_distance)
        self.joint_profiles[index].set_profile(self.current_joint[index], targets[index])
        duration = self.joint_profiles[index].duration()

        for i in range(len(msg.data)):
            self.joint_profiles[i].set_profile_duration(self.current_joint[i], targets[i], duration)

        self.trajectory_start_time = self.get_clock().now()
        self.processing_trajectory = 'joint'

        self.get_logger().info(f'Executing {type_str} joint trajectory. Duration: {duration:.2f} s, max joint distance: {max_distance:.2f} deg.')

    def corr_listener_callback(self, msg):
        self.target_corr = [msg.x * 1000.0, msg.y * 1000.0, msg.z * 1000.0]

    def do_listener_callback(self, msg):
        self.send_do = msg.data

    def data_listener_callback(self, msg):
        self.data_out = msg.data[:DATA_LENGTH]

    def timer_callback(self):
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

        if self.current_joint is not None:
            joint_msg = JointState()
            joint_msg.position = list(map(math.radians, self.current_joint))
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

        if self.data_in is not None and len(self.data_in) == DATA_LENGTH:
            data_msg = Float64MultiArray()
            data_msg.data = self.data_in
            self.publisher_data.publish(data_msg)

    def filter(self, current, target):
        return current + (target - current) * self.params.smooth_factor

    def send_command(self, egm):
        if self.initialized and self.counter % self.divisor == 0:
            if self.params.command_mode == Mode.JOINT.value:
                if self.processing_trajectory == 'joint' and self.joint_profiles is not None and self.trajectory_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
                    self.current_send_joint = [profile.position(elapsed_time) for profile in self.joint_profiles]
                else:
                    axes = len(self.current_send_joint) # type: ignore
                    self.current_send_joint = [self.filter(self.current_send_joint[i], self.target_joint[i]) for i in range(axes)] # type: ignore

                egm.send_to_robot(self.current_send_joint, rapid_to_robot=self.data_out, digital_signal_to_robot=self.send_do)
            elif self.params.command_mode == Mode.POSE.value:
                if self.processing_trajectory == 'lin' and self.lin_trajectory is not None and self.trajectory_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
                    H_base_tcp = self.lin_trajectory.position(elapsed_time)
                    q = H_base_tcp.M.GetQuaternion()
                    self.current_send_pos = [H_base_tcp.p.x(), H_base_tcp.p.y(), H_base_tcp.p.z()]
                    self.current_send_orient = [q[3], q[0], q[1], q[2]]
                elif self.processing_trajectory == 'joint' and self.joint_profiles is not None and self.trajectory_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
                    joint_angles = [profile.position(elapsed_time) for profile in self.joint_profiles]
                    self.current_send_pos, self.current_send_orient = self._solve_fk(list(map(math.radians, joint_angles)))
                else:
                    self.current_send_pos = [self.filter(self.current_send_pos[i], self.target_pos[i]) for i in range(3)] # type: ignore
                    self.current_send_orient = [self.filter(self.current_send_orient[i], self.target_orient[i]) for i in range(4)] # type: ignore

                egm.send_to_robot_cart(self.current_send_pos, self.current_send_orient, rapid_to_robot=self.data_out, digital_signal_to_robot=self.send_do)
            elif self.params.command_mode == Mode.CORR.value:
                self.current_send_corr = [self.filter(self.current_send_corr[i], self.target_corr[i]) for i in range(3)] # type: ignore
                egm.send_to_robot_path_corr(self.current_send_corr)

    def run_egm_loop(self):
        with EGM(port=self.params.egm_port) as egm:
            self.get_logger().info('Waiting response from robot...')

            while self.running:
                success, state = egm.receive_from_robot()

                if not success or state is None or state.cartesian is None:
                    self.get_logger().warning('Failed to receive robot state. Retrying...')
                    continue

                self.current_joint = state.joint_angles.tolist() # clone list to avoid reference issues
                self.current_pos = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
                self.current_orient = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]
                self.data_in = state.rapid_from_robot.tolist()

                if not self.initialized:
                    self.target_joint = list(self.current_joint)
                    self.target_pos = list(self.current_pos)
                    self.target_orient = list(self.current_orient)

                    self.current_send_joint = list(self.current_joint)
                    self.current_send_pos = list(self.current_pos)
                    self.current_send_orient = list(self.current_orient)

                    self.get_logger().info('Robot state received. Entering control loop.')
                    self.initialized = True

                self.counter += 1
                self.send_command(egm)

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
