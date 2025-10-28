from rclpy import (
    init,
    ok,
    shutdown,
    spin,
    spin_once,
    spin_until_future_complete,
)
from rclpy.action import ActionServer
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import tf2_ros

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from visualization_msgs.msg import MarkerArray

import cv2 as cv
import numpy as np
from threading import Lock

from global_planning import PRM
from tb3_utils import TB3Params


class PRMNode(LifecycleNode, TB3Params):
    def __init__(self) -> None:
        LifecycleNode.__init__(self, 'prm_node')
        
        self.declare_parameter(
            'tb3_model',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='TurtleBot3 model type (burger, waffle, waffle_pi)'))

        # Initialize TB3 Parameters
        robot_model = self.get_parameter('tb3_model').get_parameter_value().string_value
        try:
            TB3Params.__init__(self, robot_model)
        except Exception as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node."""
        super().on_configure(state)
        self.get_logger().info('Configuring PRM Node...')

        # Declare parameters
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
            'show_every_n',
            20,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Publish the PRM graph every n added points'))
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
        self.declare_parameter(
            'use_nav2_costmap',
            False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether to use the Nav2 costmap (true/false)'))
        self.declare_parameter(
            'use_map_topic',
            False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Whether to use the map topic (true/false)'))

        # Robot parameters
        self.robot_frame_id = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.get_logger().info(f'Robot frame ID: {self.robot_frame_id}')

        # Map
        self.occ_threshold = \
            self.get_parameter('occ_threshold').get_parameter_value().integer_value
        self.get_logger().info(f'Occupancy threshold: {self.occ_threshold}')

        # PRM
        self.show_prm = self.get_parameter('show_prm').get_parameter_value().bool_value
        self.get_logger().info(f'Show PRM: {self.show_prm}')
        self.prm_params = {}
        self.prm_params['publish_every_n'] = \
            self.get_parameter('show_every_n').get_parameter_value().integer_value
        self.prm_params['connection_radius'] = \
            self.get_parameter('connection_radius').get_parameter_value().double_value
        self.prm_params['step_size'] = \
            self.get_parameter('step_size').get_parameter_value().double_value
        self.prm_params['num_points'] = \
            self.get_parameter('num_points').get_parameter_value().integer_value
        self.get_logger().info(f'PRM parameters: {self.prm_params}')

        # Publishers and subscribers
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        use_nav2_costmap = self.get_parameter('use_nav2_costmap').get_parameter_value().bool_value

        self.costmap_pub = self.create_publisher(Costmap, 'costmap', latching_qos) \
            if not use_nav2_costmap else None
        self.marker_pub = self.create_publisher(MarkerArray, self.get_fully_qualified_name() + '/graph', latching_qos) \
            if self.show_prm else None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        return TransitionCallbackReturn.SUCCESS
    
    def ensure_client_active(self, client: Client) -> bool:
        """Ensure that the given lifecycle client is active."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} not available, waiting...', once=True)
        while ok():
            get_state_req = GetState.Request()
            future = client.call_async(get_state_req)
            spin_until_future_complete(self, future)
            if future.result() is not None:
                current_state = future.result().current_state
                if current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
                    return True
                else:
                    self.get_logger().info(
                        f'{client.srv_name} is not ready yet (current state: {current_state.label}), waiting...',
                        once=True)
            else:
                self.get_logger().error(f'{client.srv_name} service call failed')
                return False
            self.get_clock().sleep_for(Duration(seconds=1.0))
        return False

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Use the incoming map to create a PRM."""
        with self.lock:
            costmap = self.prepare_costmap(msg)
            self.prepare_prm(costmap)

    def prepare_costmap(self, map: OccupancyGrid) -> Costmap:
        """
        Convert the input map to a costmap to use for planning.

        Inputs:
            map: OccupancyGrid map from ROS
        Outputs:
            costmap: Costmap created from the input map

        Key steps:
            1) Binarize the map
                a) Keep all unknown values (-1) in place
                b) Make all values < self.occ_threshold to 0
                c) Make all values >= self.occ_threshold to 100
            2) Inflate obstacles by the robot size
            3) Use the inflated map to create a PRM
        """
        costmap = Costmap()
        costmap.header = map.header
        costmap.metadata.map_load_time = map.info.map_load_time
        costmap.metadata.resolution = map.info.resolution
        costmap.metadata.size_x = map.info.width
        costmap.metadata.size_y = map.info.height
        costmap.metadata.origin = map.info.origin

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
        costmap.data = img.flatten().astype(np.int8) - 1  # shift values back down
        costmap.data[np.logical_and(ind_unknown, costmap.data == 0)] = -1  # put unknown back to -1

        # Publish costmap
        if self.costmap_pub is not None:
            self.costmap_pub.publish(costmap)

        return

    def prepare_prm(self, costmap: Costmap) -> None:
        """
        Prepare the PRM using the given costmap.

        Inputs:
            costmap: Costmap to use for PRM construction
        Outputs:
            None
        """
        # Make PRM
        self.prm = PRM(costmap, **self.prm_params, publisher=self.marker_pub, clock=self.get_clock())
    
    def on_activate(self, state) -> TransitionCallbackReturn:
        """Activate the node."""
        super().on_activate(state)
        self.get_logger().info('Activating PRM Node...')

        use_map_topic = self.get_parameter('use_map_topic').get_parameter_value().bool_value
        use_nav2_costmap = self.get_parameter('use_nav2_costmap').get_parameter_value().bool_value

        # Get map
        self.lock = Lock()
        self.have_map = False
        self.map_sub = None
        if use_nav2_costmap:
            # Make sure Nav2 costmap server is active
            if not self.ensure_client_active(self.create_client(GetState, 'global_costmap/get_state')):
                return TransitionCallbackReturn.FAILURE
            # Get costmap from Nav2 costmap server
            costmap_client = self.create_client(GetCostmap, 'global_costmap/get_costmap')
            while not costmap_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Get global costmaps service not available, waiting...', once=True)
            future = costmap_client.call_async(GetCostmap.Request())
            spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error('Costmap service call failed')
                return
            with self.lock:
                self.have_map = True
                self.prepare_prm(future.result().map)
        elif use_map_topic:
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                'map',
                self.map_callback,
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        else:
            # Make sure map client is active
            if not self.ensure_client_active(self.create_client(GetState, 'map_server/get_state')):
                return TransitionCallbackReturn.FAILURE
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
                costmap = self.prepare_costmap(future.result().map)
                self.prepare_prm(costmap)
        self.get_logger().info('got map for PRM Node...')

        ready = self.have_map
        while not ready:
            self.get_logger().info('Waiting for map to be received...', once=True)
            spin_once(self, timeout_sec=1.0)
            with self.lock:
                ready = self.have_map
        self.get_logger().info('PRM Node ready!')

        # Create action server
        self.action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose_prm',
            self.path_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Deactivate the node."""
        super().on_deactivate(state)
        self.get_logger().info('Deactivating PRM Node...')
        self.action_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    def path_callback(self, request: ComputePathToPose) -> ComputePathToPose:
        """
        Use the PRM to plan a path from the current pose of the robot to the goal pose.

        Inputs:
            request: Action request (nav2_msgs/Action/ComputePathToPose/Request)
        Outputs:
            response: Action response (nav2_msgs/Action/ComputePathToPose/Response)
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Read the information in the action request, paying attention to initial pose
        # NOTE The use_start field coming in should always be False
        pass

        # TODO Check that the requested data is valid, if not return the proper error code
        #   (see the action response definition for the list of error codes)

        # NOTE No need to publish feedback since the action definition does not include it

        # TODO Plan a path using prm.query to plan a path
        #   from the current robot position to the goal position
        # NOTE the query needs to be called inside of `with self.lock:`
        #   to ensure that the PRM does not get modified while being used
        #   with self.lock:
        #       self.prm.query(<INPUTS>)
        # NOTE You can optionally choose to add points to the PRM if no path is found
        # NOTE You need to make sure to connect the start and goal points to the PRM graph
        #   before calling query
        pass

        # TODO Create the action response, fill it in, and return it
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
    
    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Cleanup the node."""
        super().on_cleanup(state)
        self.get_logger().info('Cleaning up PRM Node...')
        # Reset variables
        self.have_map = False
        self.prm = None

        # Destroy publishers and subscribers
        if self.marker_pub is not None:
            self.marker_pub.destroy()
        if self.costmap_pub is not None:
            self.costmap_pub.destroy()
        if self.map_sub is not None:
            self.map_sub.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down PRM Node...')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    # Start up ROS2
    init(args=args)

    # Create the node
    prm_node = PRMNode()

    # Let the node run until it is killed
    spin(prm_node)

    # Clean up the node and stop ROS2
    shutdown()


if __name__ == '__main__':
    main()
