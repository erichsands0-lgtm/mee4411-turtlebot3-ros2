from rclpy import (
    init,
    shutdown,
    spin,
    spin_until_future_complete,
)
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from visualization_msgs.msg import MarkerArray

import cv2 as cv
import numpy as np
from threading import Lock

from global_planning import PRM
from tb3_utils import TB3Params


class PRMNode(Node, TB3Params):
    def __init__(self) -> None:
        Node.__init__(self, 'prm_node')

        # Declare parameters
        self.declare_parameter(
            'tb3_model',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='TurtleBot3 model type (burger, waffle, waffle_pi)'))
        self.declare_parameter(
            'robot_frame',
            'base_footprint',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Robot frame ID'))
        self.declare_parameter(
            'occ_threshold',
            50,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Occupancy threshold for binarizing the map'))
        self.declare_parameter(
            'show_prm',
            True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether to publish the PRM graph for visualization'))
        self.declare_parameter(
            'connection_radius',
            1.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Connection radius for the PRM'))
        self.declare_parameter(
            'step_size',
            0.01,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Step size for collision checking in the PRM'))
        self.declare_parameter(
            'num_points',
            1000,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of points to sample in the PRM'))

        # Initialize TB3 Parameters
        robot_model = self.get_parameter('tb3_model').get_parameter_value().string_value
        try:
            TB3Params.__init__(self, robot_model)
        except Exception as e:
            self.get_logger().error(str(e))
            raise

        # Robot parameters
        self.robot_frame_id = self.get_parameter('robot_frame').get_parameter_value().string_value

        # Map
        self.occ_threshold = \
            self.get_parameter('occ_threshold').get_parameter_value().integer_value
        self.map_frame_id = None

        # PRM
        self.show_prm = self.get_parameter('show_prm').get_parameter_value().bool_value
        self.prm_params['connection_radius'] = \
            self.get_parameter('connection_radius').get_parameter_value().double_value
        self.prm_params['step_size'] = \
            self.get_parameter('step_size').get_parameter_value().double_value
        self.prm_params['num_points'] = \
            self.get_parameter('num_points').get_parameter_value().integer_value

        # Publishers and subscribers
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.path_pub = self.create_publisher(Path, 'path', latching_qos)
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', latching_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '~graph', latching_qos) \
            if self.show_prm else None

        self.goal_sub = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Get map
        use_map_topic = self.get_parameter('use_map_topic').get_parameter_value().bool_value
        self.lock = Lock()
        if use_map_topic:
            self.map_sub = \
                self.create_subscription(OccupancyGrid, 'map', self.map_callback, latching_qos)
        else:
            # Get map from map server
            map_client = self.create_client(GetMap, 'map_server/map')
            map_client.wait_for_service()
            future = map_client.call_async(GetMap.Request())
            spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error('Map service call failed')
                return
            with self.lock:
                self.have_map = True
                self.prepare_prm(future.result().map)

    def prepare_prm(self, map: OccupancyGrid) -> None:
        """
        Convert the input map to a costmap to use for planning.

        Inputs:
            map: OccupancyGrid map from ROS
        Outputs:
            None

        Key steps:
            1) Binarize the map
                a) Keep all unknown values (-1) in place
                b) Make all values < self.occ_threshold to 0
                c) Make all values >= self.occ_threshold to 100
            2) Inflate obstacles by the robot size
            3) Use the inflated map to create a PRM
        """
        # Save frame ID
        self.map_frame_id = map.header.frame_id

        # Convert map to numpy array
        img = np.array(map.data, dtype=np.int8)
        ind_unknown = img == -1  # save indices of unknown cells

        # Shift map values up by 1 to ensure non-negative
        img += 1  # put unknown at 0
        thresh = self.occ_threshold + 1
        max_val = np.uint8(100) + 1
        min_val = np.uint8(0) + 1

        # Binarize occupancy grid
        img = img.astype(np.uint8)  # convert to uint8 to use dilate
        img[np.logical_and(img >= thresh, img <= max_val)] = max_val
        img[np.logical_and(img >= min_val, img < thresh)] = min_val

        # Convert to 2D image
        img = img.reshape((map.info.height, map.info.width))

        # Get robot kernel (i.e., shape)
        r = np.int8(np.ceil(self.robot_radius / map.info.resolution))  # robot radius in map cells
        robot_img = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*r+1, 2*r+1), (r, r))

        # Inflate obstacles using dilate function
        img = cv.dilate(img, robot_img)

        # Update map data
        map.data = img.flatten().astype(np.int8) - 1  # shift values back down
        map.data[np.logical_and(ind_unknown, map.data == 0)] = -1  # put unknown back to -1

        # Publish costmap
        self.costmap_pub.publish(map)

        # Make PRM
        self.prm = PRM(map, **self.prm_params, publisher=self.marker_pub, clock=self.get_clock())

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Use the incoming map to create a PRM."""
        with self.lock:
            self.prepare_prm(msg)

    def goal_callback(self, goal: PoseStamped) -> None:
        """
        Use the PRM to plan a path from the current pose of the robot to the goal pose.

        Inputs:
            goal: Goal pose as a PoseStamped message
        Outputs:
            None
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Look up current pose of the robot in the map frame
        pass

        # TODO Plan a path using prm.query to plan a path
        #   from the current robot position to the goal position
        # NOTE the query needs to be called inside of `with self.lock:`
        #   to ensure that the PRM does not get modified while being used
        #   with self.lock:
        #       self.prm.query(<INPUTS>)
        pass

        # TODO Publish the path using the ROS publisher
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266


def main(args=None):
    # Start up ROS2
    init(args=args)

    # Create the node
    prm_node = PRMNode()

    # Let the node run until it is killed
    spin(prm_node)

    # Clean up the node and stop ROS2
    prm_node.destroy_node()
    shutdown()


if __name__ == '__main__':
    main()

# Replace planner server in nav2 with this node
# https://docs.nav2.org/configuration/packages/configuring-planner-server.html
# https://docs.nav2.org/plugins/index.html#planners


# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/turtlebot3_world.yaml
