# Modules
import numpy as np
import torch
import threading

# ROS2
from rclpy import (
    init,
    ok,
    shutdown,
)
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import tf2_ros

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from lifecycle_msgs.msg import State as StateMsg
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# CNN model
from cnn_model import CNNModel
from cnn_model import load_cnn_params
from cnn_model import set_seed
from cnn_model import normalize_scan
from cnn_model import normalize_sub_goal
from cnn_model import normalize_final_goal
from cnn_model import unnormalize_velocities

# CNN Goal Checker
from goal_checker import GoalChecker

# Costmap
from costmap2d import Costmap2D

# TB3 parameters
from tb3_utils import TB3Params


# do not modify
seed = 1337
set_seed(seed)


# Exceptions
class InvalidPath(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(message)


class NoValidControl(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(message)


# Helper functions
def validate_twist(msg: Twist) -> bool:
    """Validate the given Twist message."""
    if np.isinf(msg.linear.x) or np.isnan(msg.linear.x):
        return False
    if np.isinf(msg.linear.y) or np.isnan(msg.linear.y):
        return False
    if np.isinf(msg.linear.z) or np.isnan(msg.linear.z):
        return False
    if np.isinf(msg.angular.x) or np.isnan(msg.angular.x):
        return False
    if np.isinf(msg.angular.y) or np.isnan(msg.angular.y):
        return False
    if np.isinf(msg.angular.z) or np.isnan(msg.angular.z):
        return False
    return True


def get_position_from_pose(pose: Pose) -> np.array:
    """Extract position as numpy array from Pose."""
    return np.array([pose.position.x, pose.position.y])


def get_yaw(quaternion: Quaternion) -> float:
    """
    Extract the yaw (rotation around the Z-axis) from a quaternion.

    Inputs:
        quaternion (geometry_msgs.msg.Quaternion): The input quaternion.
    """
    return np.arctan2(quaternion.z, quaternion.w) * 2.0


# -----------------------------------------------------------------------------
#
# the main program starts here
#
# -----------------------------------------------------------------------------
class CNNControllerNode(LifecycleNode):
    # Constructor
    def __init__(self) -> None:
        # Initialize the ROS2 node
        LifecycleNode.__init__(self, 'cnn_controller_node')

        self.costmap_node = Costmap2D(costmap_name='local')

        # Declare parameters
        self.declare_parameter(
            'cnn_param_file',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to the CNN parameter file'
            )
        )
        self.declare_parameter(
            'cnn_weights_file',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to the CNN model weights file'
            )
        )
        self.declare_parameter(
            'device',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Device to run the CNN model on (cpu, gpu)'
            )
        )
        self.declare_parameter(
            'controller_frequency',
            5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Frequency [Hz] at which to run the controller'
            )
        )
        self.declare_parameter(
            'robot_frame_id',
            'base_footprint',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Robot frame id'
            )
        )
        self.declare_parameter(
            'map_frame_id',
            'map',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Map frame id'
            )
        )
        self.declare_parameter(
            'lookahead',
            5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Lookahead distance [m] for local goal selection'
            )
        )
        self.declare_parameter(
            'goal_margin',
            3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Goal margin [m] to consider goal reached'
            )
        )
        self.declare_parameter(
            'tb3_model',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='TurtleBot3 model type'
            )
        )

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Configure the lifecycle node."""
        self.get_logger().info('Configuring...')
        super().on_configure(state)
        self.costmap_node.on_configure(state)

        # Initialize parameters
        self.cnn_params = load_cnn_params(self.get_parameter('cnn_param_file').value)
        self.tb3_params = TB3Params(self.get_parameter('tb3_model').value)

        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.lookahead = self.get_parameter('lookahead').value
        self.goal_margin = self.get_parameter('goal_margin').value
        self.controller_frequency = self.get_parameter('controller_frequency').value

        # Set the pytorch device to use
        device = self.get_parameter_or('device', 'NONE')
        if device == 'NONE':
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        elif device in ['cpu', 'gpu']:
            self.device = torch.device(device)
        else:
            self.get_logger().error('Device {device} not valid')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(f'Using PyTorch device: {self.device}')

        # Load CNN model
        if not self.has_parameter('cnn_weights_file'):
            self.get_logger().error('Model weights file parameter not set')
            return TransitionCallbackReturn.FAILURE
        weights_file = self.get_parameter('cnn_weights_file').value
        self.model = CNNModel(
            in_channels=1,
            num_hiddens=self.cnn_params['num_channels']
        )
        # moves the model to the device
        self.model.to(self.device)
        # load the weights
        checkpoint = torch.load(weights_file, map_location=self.device, weights_only=True)
        self.model.load_state_dict(checkpoint['model'])
        # set the model to evaluate
        self.model.eval()
        self.get_logger().info('Finished loading CNN model')

        # Data from subscribers
        self.has_scan = False

        # Speed limit
        self.speed_limit = None

        # Lock to keep data thread safe
        self.lock = threading.Lock()

        # Initialize goal checker
        self.goal_checker = GoalChecker(
            xy_goal_tolerance=self.goal_margin
        )

        # Visualization markers
        # Initialize goal marker message
        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = self.map_frame_id
        self.goal_marker.ns = 'goal'
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.pose.orientation.w = 1.0
        self.goal_marker.scale.x = 0.1
        self.goal_marker.scale.y = 0.1
        self.goal_marker.scale.z = 0.1
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.a = 0.5  # set transparency

        # Lookahead marker
        self.lookahead_marker = Marker()
        self.lookahead_marker.header.frame_id = self.robot_frame_id
        self.lookahead_marker.ns = 'lookahead'
        self.lookahead_marker.type = Marker.LINE_STRIP
        self.lookahead_marker.action = Marker.ADD
        self.lookahead_marker.pose.orientation.w = 1.0
        self.lookahead_marker.scale.x = 0.03
        self.lookahead_marker.color.r = 1.0
        self.lookahead_marker.color.b = 1.0
        self.lookahead_marker.color.a = 0.5  # set transparency
        for theta in np.linspace(0, 2*np.pi, 20, endpoint=True):
            self.lookahead_marker.points.append(Point(
                x=self.lookahead*np.cos(theta),
                y=self.lookahead*np.sin(theta),
                z=0.0
            ))

        # Goal marker
        self.margin_marker = Marker()
        self.margin_marker.header.frame_id = self.robot_frame_id
        self.margin_marker.ns = 'goal_margin'
        self.margin_marker.type = Marker.SPHERE
        self.margin_marker.action = Marker.ADD
        self.margin_marker.pose.orientation.w = 1.0
        self.margin_marker.scale.x = 2.0*self.goal_margin
        self.margin_marker.scale.y = 2.0*self.goal_margin
        self.margin_marker.scale.z = 0.01
        self.margin_marker.color.r = 1.0
        self.margin_marker.color.a = 0.25  # set transparency

        # Initialize ROS objects
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        self.speed_limit_sub = self.create_subscription(
            SpeedLimit,
            'speed_limit',
            self.speed_limit_callback,
            10
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.goal_pub = self.create_lifecycle_publisher(
            MarkerArray,
            'controller_marker',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.velocity_pub = self.create_lifecycle_publisher(
            Twist,
            'cmd_vel_nav',
            10
        )
        self.path_pub = self.create_lifecycle_publisher(
            Path,
            'received_global_plan',
            1
        )

        # Action Servers
        self.action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.compute_control,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('Successfully configured CNN Controller Node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Activate the lifecycle node."""
        self.get_logger().info(f'{self.get_name()}: Activating...')
        super().on_activate(state)
        self.costmap_node.on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Deactivate the lifecycle node."""
        self.get_logger().info(f'{self.get_name()}: Deactivating...')
        super().on_deactivate(state)
        self.costmap_node.on_deactivate(state)
        self.publish_zero_velocity()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Cleanup the lifecycle node."""
        self.get_logger().info(f'{self.get_name()}: Cleaning up...')
        super().on_cleanup(state)
        self.costmap_node.on_cleanup(state)
        self.action_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """Shutdown the lifecycle node."""
        self.get_logger().info(f'{self.get_name()}: Shutting down...')
        super().on_shutdown(state)
        self.costmap_node.on_shutdown(state)
        return TransitionCallbackReturn.SUCCESS

    def compute_control(self, goal_handle: ServerGoalHandle) -> FollowPath.Result:
        """
        Handle action server updates and spin server until goal is reached.

        Provides global path to controller received from action client. Twist
        velocities for the robot are calculated and published using controller at
        the specified rate till the goal is reached.
        @throw nav2_core::PlannerException
        """
        if not self._state_machine.current_state[0] is StateMsg.PRIMARY_STATE_ACTIVE:
            self.get_logger().error('CNN Controller is not active. Cannot compute control.')
            goal_handle.abort()
            return FollowPath.Result()
        self.get_logger().info('Computing control...')

        # Publish the received path
        self.path_pub.publish(goal_handle.request.path)

        try:
            loop_rate = self.create_rate(self.controller_frequency)
            while ok():
                if not self.ready():
                    self.get_logger().warning(
                        'Waiting for data to compute control...',
                        throttle_duration_sec=5.0
                    )
                    continue

                start_time = self.get_clock().now()

                # Get scan data
                scan = None
                with self.lock:
                    scan = np.copy(self.scan_data)

                # Pose
                pose = self.get_robot_pose()
                current_position = get_position_from_pose(pose.pose)

                # Goal
                final_pose = goal_handle.request.path.poses[-1]
                final_goal = get_position_from_pose(final_pose.pose)
                if self.cnn_params['goal_input'] == 'sub_goal':
                    goal = self.find_subgoal(goal_handle.request.path, current_position)
                elif self.cnn_params['goal_input'] == 'final_goal':
                    goal = final_goal
                else:
                    self.get_logger().error(
                        f"Goal input type {self.cnn_params['goal_input']} not recognized"
                    )
                    goal_handle.abort()
                    return FollowPath.Result()
                self.publish_markers(goal)

                # Put goal into robot frame
                theta = get_yaw(pose.pose.orientation)
                R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
                goal_local = R @ (goal - current_position)
                goal_local = goal_local.flatten()

                # Compute velocity
                vx = self.compute_and_publish_velocity(scan, goal_local)

                # Publish feedback
                feedback = FollowPath.Feedback()
                feedback.distance_to_goal = np.linalg.norm(final_goal - current_position)
                feedback.speed = vx
                goal_handle.publish_feedback(feedback)

                # Check if goal reached
                if self.goal_checker.is_goal_reached(pose.pose, final_pose.pose):
                    self.get_logger().info('Goal reached!')
                    goal_handle.succeed()
                    self.on_goal_exit()
                    return FollowPath.Result()

                # Ensure loop runs at desired rate
                try:
                    loop_rate.sleep()
                except Exception:
                    elapsed = self.get_clock().now() - start_time
                    sleep_time = Duration(seconds=(1.0 / self.controller_frequency)) - elapsed
                    if sleep_time < Duration(seconds=0.0):
                        self.get_logger().warning(f'Control loop missed its desired rate of {self.controller_frequency:.4f} Hz. Current loop rate is {(1.0 / elapsed.nanoseconds * 1e9):.4f} Hz')  # noqa: E501

        except InvalidPath as e:
            self.get_logger().error(f'Invalid path: {e}')
            goal_handle.abort()
            return FollowPath.Result()

        except Exception as e:
            self.get_logger().error(f'Exception in compute_control: {e}')
            goal_handle.abort()
            return FollowPath.Result()

    def cancel_callback(self, goal_handle) -> bool:
        """Handle request to cancel the current goal."""
        self.get_logger().info('Goal cancelled!')
        self.on_goal_exit(force_stop=True)
        goal_handle.canceled()
        return CancelResponse.ACCEPT

    def find_subgoal(self, path: Path, current_position: np.array) -> np.array:
        """
        Find the subgoal on the path at lookahead distance from current position.

        Inputs:
          x = numpy array with 2 elements (x and y position of robot)
          pt, dist, seg = outputs of find_closest_point
        Outputs:
          goal = numpy array with 2 elements (x and y position of goal)
        """
        def findClosestPoint(path, x, seg=-1):
            """Find closest point on path to position x."""
            # initialize values
            dist_min = np.inf
            pt_min = np.array([np.nan, np.nan])
            seg_min = -1

            if seg == -1:
                # find closest point on entire path
                for i in range(len(path.poses)-1):
                    pt, dist, _ = findClosestPoint(path, x, i)

                    if dist < dist_min:
                        dist_min = dist
                        pt_min = pt
                        seg_min = i
            else:
                # find closest point on segment seg
                # calculate unit vector in direction of segment
                p0 = p1 = None
                p0 = get_position_from_pose(path.poses[seg].pose)
                p1 = get_position_from_pose(path.poses[seg+1].pose)
                length_seg = np.linalg.norm(p1 - p0)
                p = (p1 - p0) / length_seg

                # find distance from start of segment to x along the direction of the segment
                dist_projected = np.dot(x - p0, p)

                # find closest point
                if dist_projected < 0.0:
                    pt_min = p0
                elif dist_projected > length_seg:
                    pt_min = p1
                else:
                    pt_min = p0 + dist_projected * p
                dist_min = np.linalg.norm(x - pt_min)
                seg_min = seg

            return (pt_min, dist_min, seg_min)

        (pt, dist, seg) = findClosestPoint(path, current_position)
        if np.isnan(pt).any():
            raise Exception('No valid point found')

        goal = None
        if dist > self.lookahead:
            goal = pt
        else:
            seg_end = get_position_from_pose(path.poses[seg+1].pose)
            d = np.linalg.norm(current_position - seg_end)
            while d < self.lookahead and seg < len(path.poses)-2:
                seg = seg + 1
                seg_end = get_position_from_pose(path.poses[seg+1].pose)
                d = np.linalg.norm(current_position - seg_end)

            if d < self.lookahead:
                goal = get_position_from_pose(path.poses[-1].pose)
            else:
                seg_start = get_position_from_pose(path.poses[seg].pose)
                v = (seg_end - seg_start) / np.linalg.norm(seg_end - seg_start)
                proj = seg_start + np.dot(current_position - seg_start, v) * v
                length = np.sqrt(self.lookahead**2 - np.linalg.norm(current_position-proj)**2)
                goal = proj + length * v
        return goal

    def compute_and_publish_velocity(self, scan: np.array, goal: np.array) -> float:
        """Calculate velocity and publish to "cmd_vel" topic."""
        try:
            # Normalize input data:
            scan = normalize_scan(scan, self.cnn_params['normalization_method'])
            if self.cnn_params['goal_input'] == 'sub_goal':
                goal = normalize_sub_goal(goal, self.cnn_params['normalization_method'])
            else:
                goal = normalize_final_goal(goal, self.cnn_params['normalization_method'])

            # Switch to torch tensor
            batch_scan = torch.tensor(scan, dtype=torch.float32).to(self.device)
            batch_goal = torch.tensor(goal, dtype=torch.float32).to(self.device)

            # Run CNN model
            with torch.no_grad():
                vel_cmd = self.model(batch_scan, batch_goal)

            # Unnormalize
            vx, wz = unnormalize_velocities(vel_cmd.data.cpu().numpy().flatten())
        except Exception as e:
            raise NoValidControl(e)

        # Apply velocity limits
        v_max = self.tb3_params.v_max
        w_max = self.tb3_params.w_max
        with self.lock:
            if self.speed_limit is not None:
                if self.speed_limit.percentage:
                    v_max = min(v_max, v_max * (self.speed_limit.speed_limit / 100.0))
                else:
                    v_max = min(v_max, self.speed_limit.speed_limit)
        if np.abs(vx) > v_max:
            wz = wz * (v_max / np.abs(vx))
            vx = np.sign(vx) * v_max
        if np.abs(wz) > w_max:
            vx = vx * (w_max / np.abs(wz))
            wz = np.sign(wz) * w_max
        # Ensure types are float
        vx = float(vx)
        wz = float(wz)

        # Create Twist message
        velocity = Twist()
        velocity.linear.x = vx
        velocity.angular.z = wz

        self.publish_velocity(velocity)

        return vx

    def publish_velocity(self, velocity: Twist) -> None:
        """Call velocity publisher to publish the velocity on "cmd_vel" topic."""
        if not validate_twist(velocity):
            self.get_logger().error('Velocity message contains NaNs or Infs! Ignoring as invalid!')
            return

        if self.velocity_pub.get_subscription_count() > 0:
            self.velocity_pub.publish(velocity)

    def publish_zero_velocity(self) -> None:
        """Call velocity publisher to publish zero velocity."""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0

        self.publish_velocity(zero_twist)

    def on_goal_exit(self, force_stop: bool = False) -> None:
        """Stop the robot on goal exit."""
        if force_stop:
            self.publish_zero_velocity()

    def get_robot_pose(self) -> PoseStamped:
        """Obtain current pose of the robot in map frame."""
        # Lookup transformation
        transformation = self.tf_buffer.lookup_transform(
            self.map_frame_id,
            self.robot_frame_id,
            Time(),
            timeout=Duration(seconds=2.0)
        )

        # Convert Transform to PoseStamped
        pose = PoseStamped()
        pose.header = transformation.header
        pose.pose.position.x = transformation.transform.translation.x
        pose.pose.position.y = transformation.transform.translation.y
        pose.pose.position.z = transformation.transform.translation.z
        pose.pose.orientation = transformation.transform.rotation

        return pose

    def speed_limit_callback(self, msg: SpeedLimit) -> None:
        """Process incoing speed limit data."""
        # TODO implement speed limit handling
        with self.lock:
            self.speed_limit = msg

    def ready(self) -> bool:
        """Check if ready to start going."""
        with self.lock:
            return self.has_scan

    def odom_callback(self, msg: Odometry) -> None:
        """Process incoming odometry data."""
        with self.lock:
            # get the odometry data:
            self.odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        """Process incoming lidar data."""
        with self.lock:
            # get the laser scan data:
            self.scan_data = np.array(msg.ranges, dtype=np.float32)
            self.scan_data[np.isnan(self.scan_data)] = msg.range_max
            self.scan_data[np.isinf(self.scan_data)] = msg.range_max
            self.has_scan = True

    def publish_markers(self, goal: np.array) -> None:
        """Publish markers showing the controller goal."""
        assert len(goal) == 2, 'Goal must be a 2D point'

        # Get most recent pose for time stamp
        pose = self.get_robot_pose()

        # Update goal marker
        self.goal_marker.header.stamp = pose.header.stamp
        self.goal_marker.pose.position.x = goal[0]
        self.goal_marker.pose.position.y = goal[1]

        # Update circle marker
        self.lookahead_marker.header.stamp = self.goal_marker.header.stamp
        self.margin_marker.header.stamp = self.goal_marker.header.stamp

        # Create and publish marker array
        ma = MarkerArray()
        ma.markers.append(self.goal_marker)
        ma.markers.append(self.lookahead_marker)
        ma.markers.append(self.margin_marker)
        self.goal_pub.publish(ma)


def main(args=None):
    # Start up ROS2
    init(args=args)

    # Create the node
    cnn_node = CNNControllerNode()

    # Create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(cnn_node)

    try:
        # Spin the executor to process callbacks from all added nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown and destroy nodes
        executor.shutdown()
        cnn_node.destroy_node()
        shutdown()


if __name__ == '__main__':
    main()
