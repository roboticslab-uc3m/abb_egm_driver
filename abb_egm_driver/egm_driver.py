import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rl_cartesian_control_msgs.srv import Act, Inv
from rl_cartesian_control_msgs.action import JointTrajectory, PoseTrajectory
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
        self.trajectory_duration = None
        self.trajectory_progress = 1.0

        self.has_kinematics = False
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.trajectory_done_event = threading.Event()
        self.FeedBackType = None

        self.get_logger().info(f'Using EGM port: {self.params.egm_port}.')
        self.get_logger().info(f'Using smooth_factor: {self.params.smooth_factor}.')
        self.get_logger().info(f'Using publish_period: {self.params.publish_period} ms.')

        self.publisher_joint = self.create_publisher(JointState, 'state/joint', 10)
        self.publisher_pose = self.create_publisher(Pose, 'state/pose', 10)

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
            self.publisher_data = self.create_publisher(Float64MultiArray, 'state/data', 10)

            self.subscription_pose_cmd = self.create_subscription(Pose, 'command/pose', self.pose_listener_callback, 10)
            self.subscription_data = self.create_subscription(Float64MultiArray, 'command/data', self.data_listener_callback, 10)

            self.act_service = self.create_service(Act, 'actuate_tool', self.act_service_callback)
            self.stop_service = self.create_service(Trigger, 'stop_control', self.stop_service_callback)

            self.action_pose_traj = ActionServer(self, PoseTrajectory, 'trajectory/pose',
                                                 goal_callback=self.trajectory_goal_callback_pose,
                                                 handle_accepted_callback=self.trajectory_handle_accepted_callback,
                                                 cancel_callback=self.trajectory_cancel_callback,
                                                 execute_callback=self.trajectory_execute_callback_pose,
                                                 callback_group=ReentrantCallbackGroup())

            self.get_logger().info(f'Using max_lin_velocity (trajectories): {self.params.max_lin_velocity} mm/s.')
            self.get_logger().info(f'Using max_lin_acceleration (trajectories): {self.params.max_lin_acceleration} mm/s^2.')

            if self.parse_kinematic_parameters():
                self.subscription_joint_cmd = self.create_subscription(Float32MultiArray, 'command/joint', self.joint_listener_callback, 10)
                self.ik_service = self.create_service(Inv, 'inv', self.inv_service_callback)

                self.action_joint_traj = ActionServer(self, JointTrajectory, 'trajectory/joint',
                                                      goal_callback=self.trajectory_goal_callback_joint,
                                                      handle_accepted_callback=self.trajectory_handle_accepted_callback,
                                                      cancel_callback=self.trajectory_cancel_callback,
                                                      execute_callback=self.trajectory_execute_callback_joint,
                                                      callback_group=ReentrantCallbackGroup())

                self.get_logger().info(f'Using max_joint_velocity (trajectories): {self.params.max_joint_velocity} deg/s.')
                self.get_logger().info(f'Using max_joint_acceleration (trajectories): {self.params.max_joint_acceleration} deg/s^2.')

                self.using_dh_joint_cmd = True
                self.has_kinematics = True
                self.get_logger().info('DH parameters provided, joint commands are enabled in pose command mode.')
            else:
                self.get_logger().info('No valid DH parameters provided, joint commands are disabled in pose command mode.')
        elif self.params.command_mode == Mode.JOINT.value:
            self.subscription_joint_cmd = self.create_subscription(Float32MultiArray, 'command/joint', self.joint_listener_callback, 10)

            self.stop_service = self.create_service(Stop, 'stop_control', self.stop_service_callback)

            self.action_joint_traj = ActionServer(self, JointTrajectory, 'trajectory/joint',
                                                  goal_callback=self.trajectory_goal_callback_joint,
                                                  handle_accepted_callback=self.trajectory_handle_accepted_callback,
                                                  cancel_callback=self.trajectory_cancel_callback,
                                                  execute_callback=self.trajectory_execute_callback_joint,
                                                  callback_group=ReentrantCallbackGroup())

            self.get_logger().info(f'Using max_joint_velocity (trajectories): {self.params.max_joint_velocity} deg/s.')
            self.get_logger().info(f'Using max_joint_acceleration (trajectories): {self.params.max_joint_acceleration} deg/s^2.')

            if self.parse_kinematic_parameters():
                self.action_pose_traj = ActionServer(self, PoseTrajectory, 'trajectory/pose',
                                                     goal_callback=self.trajectory_goal_callback_pose,
                                                     handle_accepted_callback=self.trajectory_handle_accepted_callback,
                                                     cancel_callback=self.trajectory_cancel_callback,
                                                     execute_callback=self.trajectory_execute_callback_pose,
                                                     callback_group=ReentrantCallbackGroup())

                self.has_kinematics = True
                self.get_logger().info('DH parameters provided, pose commands are enabled in joint command mode.')
            else:
                self.get_logger().info('No valid DH parameters provided, pose commands are disabled in joint command mode.')
        elif self.params.command_mode == Mode.CORR.value:
            self.subscription_corr_cmd = self.create_subscription(Point, 'command/path_corr', self.corr_listener_callback, 10)
        else:
            self.get_logger().error(f'Invalid command mode "{self.params.command_mode}". This should never happen due to parameter validation.')

        self.running = True
        self.initialized = False

        self.egm_thread = threading.Thread(target=self.run_egm_loop)
        self.egm_thread.start()

        self.get_logger().info('EGM Driver is ready and running.')

    def parse_kinematic_parameters(self):
        self.chain = kdl.Chain()

        wobj_x = self.params.wobj_frame.x
        wobj_y = self.params.wobj_frame.y
        wobj_z = self.params.wobj_frame.z

        wobj_qw = self.params.wobj_frame.qw
        wobj_qx = self.params.wobj_frame.qx
        wobj_qy = self.params.wobj_frame.qy
        wobj_qz = self.params.wobj_frame.qz

        if any(param != 0.0 for param in [wobj_x, wobj_y, wobj_z, wobj_qx, wobj_qy, wobj_qz]):
            self.get_logger().info(f'Work object frame provided: x={wobj_x} mm, y={wobj_y} mm, z={wobj_z} mm, qw={wobj_qw}, qx={wobj_qx}, qy={wobj_qy}, qz={wobj_qz}.')
            wobj_rotation = kdl.Rotation.Quaternion(wobj_qx, wobj_qy, wobj_qz, wobj_qw)
            wobj_translation = kdl.Vector(wobj_x, wobj_y, wobj_z)
            wobj_frame = kdl.Frame(wobj_rotation, wobj_translation)
            segment = kdl.Segment(kdl.Joint(kdl.Joint.Fixed), wobj_frame.Inverse())
            self.chain.addSegment(segment)

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

        tool_x = self.params.tool_frame.x
        tool_y = self.params.tool_frame.y
        tool_z = self.params.tool_frame.z

        tool_qw = self.params.tool_frame.qw
        tool_qx = self.params.tool_frame.qx
        tool_qy = self.params.tool_frame.qy
        tool_qz = self.params.tool_frame.qz

        if any(param != 0.0 for param in [tool_x, tool_y, tool_z, tool_qx, tool_qy, tool_qz]):
            self.get_logger().info(f'Tool frame provided: x={tool_x} mm, y={tool_y} mm, z={tool_z} mm, qw={tool_qw}, qx={tool_qx}, qy={tool_qy}, qz={tool_qz}.')
            tool_rotation = kdl.Rotation.Quaternion(tool_qx, tool_qy, tool_qz, tool_qw)
            tool_translation = kdl.Vector(tool_x, tool_y, tool_z)
            tool_frame = kdl.Frame(tool_rotation, tool_translation)
            segment = kdl.Segment(kdl.Joint(kdl.Joint.Fixed), tool_frame)
            self.chain.addSegment(segment)

        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver_vel = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver_pos = kdl.ChainIkSolverPos_NR_JL(self.chain, self.min_limits, self.max_limits, self.fk_solver_pos, self.ik_solver_vel)

        return True

    def solve_fk(self, joint_angles_rad):
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
            self.target_pos, self.target_orient = self.solve_fk(msg.data)
            self.processing_trajectory = None

    def corr_listener_callback(self, msg):
        self.target_corr = [msg.x * 1000.0, msg.y * 1000.0, msg.z * 1000.0]

    def data_listener_callback(self, msg):
        self.data_out = msg.data[:DATA_LENGTH]

    def act_service_callback(self, request, response):
        self.get_logger().info(f'Received actuate tool request: {request.cmd}')

        if request.cmd == Act.Request.CLOSE:
            self.send_do = True
        elif request.cmd == Act.Request.OPEN:
            self.send_do = False
        else:
            self.get_logger().warning('Received actuate tool request with unsupported command. Ignoring command.')
            response.success = False
            return response

        response.success = True
        return response

    def stop_service_callback(self, request, response):
        self.get_logger().info('Received stop request. Stopping any ongoing trajectory.')

        if self.goal_handle is not None and self.goal_handle.is_active:
            self.goal_handle.abort()
            self.processing_trajectory = None
            self.trajectory_done_event.set()
            self.get_logger().info('Ongoing trajectory aborted successfully.')
            response.success = True
        else:
            self.get_logger().info('No active trajectory to stop.')
            response.success = False

        return response

    def inv_service_callback(self, request, response):
        self.get_logger().info('Received inverse kinematics request')

        q = kdl.JntArray(self.chain.getNrOfJoints())
        qd = kdl.JntArray(self.chain.getNrOfJoints())

        for i in range(len(self.current_joint)):
            q[i] = math.radians(self.current_joint[i])

        xd = kdl.Frame(kdl.Rotation.Quaternion(request.x.orientation.x, request.x.orientation.y, request.x.orientation.z, request.x.orientation.w),
                       kdl.Vector(request.x.position.x * 1000.0, request.x.position.y * 1000.0, request.x.position.z * 1000.0))

        if self.ik_solver_pos.CartToJnt(q, xd, qd) >= 0:
            response.success = True
            position = [qd[i] for i in range(qd.rows())]
            response.q = position
        else:
            response.success = False
            self.get_logger().warning('Inverse kinematics failed for the given target pose.')

        return response

    def trajectory_goal_callback_joint(self, goal_request):
        self.get_logger().info('Received joint trajectory goal request')

        if self.current_joint is None:
            self.get_logger().warning('Received joint trajectory before robot state is available. Ignoring command.')
            return GoalResponse.REJECT

        if len(goal_request.data) != len(self.current_joint):
            self.get_logger().warning(f'Received joint trajectory with incorrect number of joints. Expected {len(self.current_joint)}, got {len(goal_request.data)}.')
            return GoalResponse.REJECT

        if len(goal_request.data) != self.chain.getNrOfJoints():
            self.get_logger().warning(f'Received joint trajectory with incorrect number of joints for DH model. Expected {self.chain.getNrOfJoints()}, got {len(goal_request.data)}.')
            return GoalResponse.REJECT

        self.FeedBackType = JointTrajectory.Feedback
        return GoalResponse.ACCEPT

    def trajectory_goal_callback_pose(self, goal_request):
        self.get_logger().info('Received pose trajectory goal request')

        if goal_request.type == PoseTrajectory.Goal.JOINT:
            if not self.has_kinematics:
                self.get_logger().warning('Received MoveJ trajectory but no valid DH parameters are provided. Ignoring command.')
                return GoalResponse.REJECT

            if self.current_joint is None:
                self.get_logger().warning('Received MoveJ trajectory before robot state is available. Ignoring command.')
                return GoalResponse.REJECT
        elif goal_request.type == PoseTrajectory.Goal.LINEAR:
            if self.current_pos is None or self.current_orient is None:
                self.get_logger().warning('Received MoveL trajectory before robot state is available. Ignoring command.')
                return GoalResponse.REJECT
        else:
            self.get_logger().warning('Received trajectory goal with unknown type. Ignoring command.')
            return GoalResponse.REJECT

        self.FeedBackType = PoseTrajectory.Feedback
        return GoalResponse.ACCEPT

    def trajectory_handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self.goal_handle.abort()
                self.trajectory_done_event.set()

            self.goal_handle = goal_handle

        goal_handle.execute()

    def trajectory_cancel_callback(self, goal):
        self.get_logger().info('Received trajectory cancel request')
        return CancelResponse.ACCEPT

    def trajectory_execute_callback_joint(self, goal_handle):
        self.get_logger().info('Executing joint trajectory')
        self.prepare_trajectory_joint(goal_handle.request.position)

        self.trajectory_done_event.wait()
        self.trajectory_done_event.clear()

        if goal_handle.is_active:
            self.goal_handle.succeed()

        result = JointTrajectory.Result()
        result.success = True
        result.duration = self.trajectory_duration
        result.progress = self.trajectory_progress

        return result

    def trajectory_execute_callback_pose(self, goal_handle):
        if goal_handle.request.type == PoseTrajectory.Goal.JOINT:
            self.get_logger().info('Executing MoveJ trajectory')
            self.prepare_trajectory_movej(goal_handle.request.pose)
        elif goal_handle.request.type == PoseTrajectory.Goal.LINEAR:
            self.get_logger().info('Executing MoveL trajectory')
            self.prepare_trajectory_movel(goal_handle.request.pose)
        else:
            self.get_logger().warning('Received trajectory goal with unknown type. Ignoring command.')
            goal_handle.abort()

        if not goal_handle.is_active:
            self.get_logger().info('Trajectory goal was aborted before execution started.')
            result = PoseTrajectory.Result()
            result.success = False
            return result

        self.trajectory_done_event.wait()
        self.trajectory_done_event.clear()

        if goal_handle.is_active:
            self.goal_handle.succeed()

        result = PoseTrajectory.Result()
        result.success = True
        result.duration = self.trajectory_duration
        result.progress = self.trajectory_progress

        return result

    def prepare_trajectory_joint(self, q):
        targets = list(map(math.degrees, q))
        diffs = [abs(a - b) for a, b in zip(targets, self.current_joint)]
        max_distance = max(diffs)

        if self.params.max_joint_acceleration > 0:
            self.joint_profiles = [m3.VelocityProfileTrapezoidal(self.params.max_joint_velocity, self.params.max_joint_acceleration) for i in range(len(q))]
            type_str = 'trapezoidal'
        else:
            self.joint_profiles = [m3.VelocityProfileRectangular(self.params.max_joint_velocity) for i in range(len(q))]
            type_str = 'rectangular'

        index = diffs.index(max_distance)
        self.joint_profiles[index].set_profile(self.current_joint[index], targets[index])
        self.trajectory_duration = self.joint_profiles[index].duration()

        for i in range(len(q)):
            self.joint_profiles[i].set_profile_duration(self.current_joint[i], targets[i], self.trajectory_duration)

        self.trajectory_start_time = self.get_clock().now()
        self.processing_trajectory = 'joint'

        self.get_logger().info(f'Executing {type_str} joint trajectory. Duration: {self.trajectory_duration:.2f} s, max joint distance: {max_distance:.2f} deg.')

    def prepare_trajectory_movej(self, pose):
        q = kdl.JntArray(self.chain.getNrOfJoints())
        qd = kdl.JntArray(self.chain.getNrOfJoints())

        for i in range(len(self.current_joint)):
            q[i] = math.radians(self.current_joint[i])

        xd = kdl.Frame(kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                       kdl.Vector(pose.position.x * 1000.0, pose.position.y * 1000.0, pose.position.z * 1000.0))

        if self.ik_solver_pos.CartToJnt(q, xd, qd) >= 0:
            qd_deg_str = ', '.join([f'{math.degrees(qd[i]):.2f}' for i in range(qd.rows())])
            self.get_logger().info(f'IK solution found: [{qd_deg_str}] [deg]')
            position = [qd[i] for i in range(qd.rows())]
            self.prepare_trajectory_joint(position)
        else:
            self.get_logger().warning('Inverse kinematics failed for the given target pose. Aborting command.')

            if self.goal_handle is not None and self.goal_handle.is_active:
                self.goal_handle.abort()

    def prepare_trajectory_movel(self, pose):
        start_pos = kdl.Vector(self.current_pos[0], self.current_pos[1], self.current_pos[2])
        start_orient = kdl.Rotation.Quaternion(self.current_orient[1], self.current_orient[2], self.current_orient[3], self.current_orient[0])

        end_pos = kdl.Vector(pose.position.x * 1000.0, pose.position.y * 1000.0, pose.position.z * 1000.0)
        end_orient = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

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
        self.trajectory_duration = self.lin_trajectory.duration()

        self.get_logger().info(f'Executing {type_str} linear trajectory. Duration: {self.trajectory_duration:.2f} s, path length: {path.path_length():.2f} mm.')

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

        if self.goal_handle is not None and self.goal_handle.is_active:
            if self.goal_handle.is_cancel_requested:
                self.get_logger().info('Trajectory goal cancel requested')
                self.goal_handle.canceled()
                self.processing_trajectory = None
                self.trajectory_done_event.set()
            else:
                feedback_msg = self.FeedBackType()
                feedback_msg.progress = self.trajectory_progress
                self.goal_handle.publish_feedback(feedback_msg)

    def filter(self, current, target):
        return current + (target - current) * self.params.smooth_factor

    def send_command(self, egm):
        if self.initialized and self.counter % self.divisor == 0:
            if self.params.command_mode == Mode.JOINT.value:
                if self.processing_trajectory == 'joint' and self.joint_profiles is not None and self.trajectory_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
                    self.current_send_joint = [profile.position(elapsed_time) for profile in self.joint_profiles]
                    self.target_joint = list(self.current_send_joint)
                    self.trajectory_progress = min(elapsed_time / self.trajectory_duration, 1.0)

                    if elapsed_time >= self.trajectory_duration:
                        self.get_logger().info('Joint trajectory completed.')
                        self.processing_trajectory = None

                        if self.goal_handle is not None and self.goal_handle.is_active:
                            self.trajectory_done_event.set()
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
                    self.target_pos = list(self.current_send_pos)
                    self.target_orient = list(self.current_send_orient)
                    self.trajectory_progress = min(elapsed_time / self.trajectory_duration, 1.0)

                    if elapsed_time >= self.trajectory_duration:
                        self.get_logger().info('MoveL trajectory completed.')
                        self.processing_trajectory = None

                        if self.goal_handle is not None and self.goal_handle.is_active:
                            self.trajectory_done_event.set()
                elif self.processing_trajectory == 'joint' and self.joint_profiles is not None and self.trajectory_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
                    joint_angles = [profile.position(elapsed_time) for profile in self.joint_profiles]

                    self.current_send_pos, self.current_send_orient = self.solve_fk(list(map(math.radians, joint_angles)))
                    self.target_pos = list(self.current_send_pos)
                    self.target_orient = list(self.current_send_orient)
                    self.trajectory_progress = min(elapsed_time / self.trajectory_duration, 1.0)

                    if elapsed_time >= self.trajectory_duration:
                        self.get_logger().info('MoveJ trajectory completed.')
                        self.processing_trajectory = None

                        if self.goal_handle is not None and self.goal_handle.is_active:
                            self.trajectory_done_event.set()
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
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.egm_thread.join()
        node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
