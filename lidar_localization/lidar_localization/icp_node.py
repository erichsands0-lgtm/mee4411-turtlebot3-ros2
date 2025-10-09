import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import tf2_ros

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from threading import Lock

from lidar_localization.icp2d import ICP2D
from occupancy_grid.occupancy_grid import OccupancyGridMap

# Indexing values
X = 0
Y = 1
THETA = 2


class ICPLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('icp_localization_node')

        # Declare parameters
        self.declare_parameter(
            'initial_pose_x',
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'initial_pose_y',
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'initial_pose_a',
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'odom_frame',
            'odom',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'time_travel',
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            'use_map_topic',
            False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL))

        # Get initial pose (of odom with respect to map frame)
        initial_pose_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        initial_pose_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        initial_pose_a = self.get_parameter('initial_pose_a').get_parameter_value().double_value

        # Store current pose (of odom with respect to map) in (x, y, theta) format
        self.pose = [initial_pose_x, initial_pose_y, initial_pose_a]

        # Initialize transformation (of odom with respect to map)
        self.tf_map_odom = TransformStamped()
        self.tf_map_odom.child_frame_id = \
            self.get_parameter('odom_frame').get_parameter_value().string_value

        # Initialize ICP object
        self.icp = ICP2D()
        self.tolerance = None  # tolerance for fitting
        self.lock = Lock()  # to ensure ICP object can be locked

        # Get map
        if self.get_parameter('use_map_topic').get_parameter_value().bool_value:
            latching_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                'map',
                self.map_callback,
                qos_profile=latching_qos)
            self.map = None
        else:
            # Get map from map server
            map_client = self.create_client(GetMap, 'map_server/map')
            map_client.wait_for_service()
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))
            future = map_client.call_async(GetMap.Request())
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error('Map service call failed')
                return
            with self.lock:
                # Initialize the ICP map points
                self.ogm = OccupancyGridMap.from_msg(future.result().map)
                self.initialize_icp(future.result().map)

        # TF information
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Time delay to account for in transforms
        self.tf_time_travel = \
            Duration(self.get_parameter('time_travel').get_parameter_value().double_value)

        # Publish initial transformation
        self.get_clock().sleep(0.1)  # pause to let the tf broadcaster start up
        self.publish_map_odom_tf(self.get_clock().now())

        # Scan subscriber
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def initialize_icp(self, map: OccupancyGrid) -> None:
        """
        Initialize the ICP algorithm with the given map.

        Inputs:
            map: the occupancy grid map (nav_msgs/OccupancyGrid)
        Outputs:
            None

        This function should:
            1. Set the map tolerance to be the map cell size
            2. Set the transformation frame to be the map frame
            3. Extract the locations of all objects in the map
            4. Use those points to initialize the ICP using the set_map_points method
        """
        # Set map tolerance to be the map cell size
        self.tolerance = map.info.resolution

        # Set transformation frame to be the map frame
        self.tf_map_odom.header.frame_id = map.header.frame_id

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract the locations of all objects in the map
        pass

        # TODO use those points to inialize the ICP using the set_map_points method
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    def map_callback(self, msg: OccupancyGrid) -> None:
        """
        Save the map and use it to initialize the ICP map points.

        Inputs:
            msg: the occupancy grid map (nav_msgs/OccupancyGrid)
        Outputs:
            None
        """
        with self.lock:
            self.ogm = OccupancyGridMap(msg)
            self.initialize_icp(msg)

    def publish_map_odom_tf(self, time: Time) -> None:
        """
        Update the current transform from the map to odom frames stored in self.tf_map_odom.

        Inputs:
            time: the time to stamp the transform with (rclpy.time.Time)
        Outputs:
            None

        This function should:
            1. Fill in the header stamp with the given time
            2. Fill in the translation and rotation based on the current pose
               (stored in self.pose)
            3. Publish the transform using self.tf_broadcaster
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in the current time
        pass

        # TODO Fill in the current transform (using the x, y, theta in self.pose)
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.tf_map_odom)

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Process the lidar scan and use ICP to perform localization.

        Inputs:
            msg: the lidar scan (sensor_msgs/LaserScan)
        Outputs:
            None

        This function should:
            1. Convert the lidar points to (x, y) coordinates in the *odom* frame
            2. Use ICP to find the pose of the odom frame with respect to the map frame
            3. Update self.pose with the new pose
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Convert lidar all points to (x,y)
        # NOTE You should only keep points that are between the minimum and maximum range
        pass

        # TODO Lookup transformation from odom to lidar coordiante frame
        pass

        # TODO Use the transformation to transform the lidar points into the odom coordinate frame
        pass

        # TODO Use ICP to find pose of odom with respect to the map
        with self.lock:
            # NOTE need to do this inside of the lock to ensure correct operation
            pass

        # TODO Update self.pose using the transformation found by ICP
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Publish transform
        self.publish_map_odom_tf(msg.header.stamp)


def main(args=None):
    # Start up ROS2
    rclpy.init(args=args)

    # Create the node
    icp_node = ICPLocalizationNode()

    # Let the node run until it is killed
    rclpy.spin(icp_node)

    # Clean up the node and stop ROS2
    icp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
