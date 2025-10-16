from rclpy.clock import Clock
from rclpy.publisher import Publisher

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

import networkx as nx
import numpy as np
from scipy.spatial import KDTree
from tqdm import tqdm

from occupancy_grid import OccupancyGridMap


Z = 0.0  # z coordinate for the graph display
ALPHA = 0.25  # alpha value for graph transparency


class PRM(OccupancyGridMap):
    def __init__(
            self,
            map: OccupancyGrid,
            num_points: int,
            connection_radius: float,
            step_size: float,
            *,
            publisher: Publisher = None,
            clock: Clock = None) -> None:
        """
        Initialize the probabilistic roadmap in the given occupancy grid.

        Inputs:
            map: An OccupancyGrid object representing the environment
            num_points: Number of points to sample in the PRM
            connection_radius: Radius in which to check for connections between nodes
            step_size: Step size to use when checking for collisions between nodes
            publisher: ROS publisher for visualization
            clock: ROS clock for timestamps
        Outputs:
            None
        """
        super(PRM, self).from_msg(map)

        # Check inputs
        if publisher is not None and clock is None:
            raise Exception('Clock is required when publishing')

        # Parameters
        self.connection_radius = connection_radius  # radius in which to check for connections
        self.step_size = step_size  # size of step to take for collision checking

        # Data structures
        self.graph = None  # networkx graph
        self.kdtree = None  # KDTree for quickly finding nearest node

        # Build the PRM
        self.build_prm(num_points, publisher=publisher, clock=clock)

    def build_prm(self, num_points: int, *,
                  publisher: Publisher = None, clock: Clock = None) -> None:
        """
        Build a PRM graph consisting of num_points.

        Inputs:
            num_points: Number of points to sample in the PRM
            publisher: ROS publisher for visualization
            clock: ROS clock for timestamps
        Outputs:
            None
        """
        # Initialize empty graph and add points
        self.graph = nx.Graph()  # intialize empty graph
        self.add_to_prm(num_points, publisher=publisher, clock=clock)

    def add_to_prm(self, num_points: int, *,
                   publisher: Publisher = None, clock: Clock = None) -> None:
        """
        Add num_points to the PRM graph.

        Inputs:
            num_points: Number of points to sample in the PRM
            publisher: ROS publisher for visualization
            clock: ROS clock for timestamps
        Outputs:
            None

        All points should be in the free space of the map.
        Points should be connected if they are within self.connection_radius and
            the edge between them is valid (i.e., not in collision).
        """
        # Wrap in tqdm for progress bar
        for i in tqdm(range(num_points)):
            ##### YOUR CODE STARTS HERE ##### # noqa: E266
            # TODO Generate valid node
            pass

            # TODO add the point to the graph node list
            #   Include an attribute called 'location' holding the 2D position
            #   'location' can be formatted as a list or as a numpy array
            #   see documentation here:
            #   https://networkx.org/documentation/stable/tutorial.html#adding-attributes-to-graphs-nodes-and-edges
            pass

            # Connect to other nodes in the graph
            # TODO find other nodes within the connection radius
            pass

            # TODO check to see if the path from the new point to any node is obstacle free
            pass

            # TODO if it is a clear path, add it to the graph edge list
            pass
            ##### YOUR CODE ENDS HERE   ##### # noqa: E266
            # Display graph as it is being built
            if publisher is not None and i % 20 == 19:
                publisher.publish(self.to_msg(clock=clock))

        # Show final graph
        if publisher is not None:
            publisher.publish(self.to_msg(clock=clock))

        # Initialize KD tree for quickly finding nearest node
        pts = np.array([p for _, p in self.graph.nodes.data('location')])
        assert pts.shape[1] == 2
        self.kdtree = KDTree(pts)

    def sample_free_point(self) -> np.array:
        """
        Draw a random points from within the free space of the map.

        Inputs:
            None
        Outputs:
            2D point within the map as a numpy.array
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Draw a random point within the boundary
        pt = np.random.rand(2)

        # TODO Check if point is valid (i.e., not in collision based on the map)
        #      If it is try again, if not then return the point
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return pt

    def valid_edge(self, p0: np.array, p1: np.array) -> bool:
        """
        Check to see if an edge connecting p0 to p1 is in collision with the map.

        Inputs:
            p0: 2D point as a numpy.array
            p1: 2D point as a numpy.array
        Outputs:
            True if the edge is valid (i.e., not in collision), False otherwise
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO create a series of points starting at p0 and ending at p1 in steps of self.step_size
        pass

        # TODO Check to make sure none of the points collide with the map
        pass

        return False
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    def find_nearest_node(self, pt: np.array) -> int:
        """
        Find the nearest node in the graph (with a valid edge) to the input point pt.

        Inputs:
            pt: 2D point as a numpy.array
        Outputs:
            Return the index of the node (or None if no valid node found)
        """
        # Get the 10 closest points from the KDTree
        _, ind = self.kdtree.query(pt, k=10)
        # Check each one to see if there is a valid edge
        for i in ind:
            if self.valid_edge(pt, self.graph.nodes[i]['location']):
                return i
        return None

    def query(self, start: np.array, goal: np.array) -> Path:
        """
        Query the PRM to get a path from a start point to a goal point.

        Inputs:
            start: 2D point as a numpy.array
            goal: 2D point as a numpy.array
        Outputs:
            Return a nav_msgs/msg/Path object from start to goal
        """
        # Make sure the PRM is initialized
        if self.graph is None:
            raise Exception('PRM not initialized')

        # Find nearest nodes to start and goal
        ind_start = self.find_nearest_node(start)
        if ind_start is None:
            raise Exception('Start point invalid')
        ind_goal = self.find_nearest_node(goal)
        if ind_goal is None:
            raise Exception('Goal point invalid')

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # Plan path using A*
        # TODO use networkx library to call A* to find a path from ind_start to ind_goal
        pass

        # Generate returned path
        # TODO convert the path returned by networkx to a nav_msgs/Path
        # NOTE make sure to include the start and goal points
        path = Path()
        pass

        return path
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    def to_msg(self, clock: Clock = None) -> None:
        """Convert the PRM graph to a visualization_msgs/msg/MarkerArray."""
        if clock is None:
            raise Exception('Clock is required for to_msg')

        # Create marker array
        ma = MarkerArray()

        # Create marker to show the nodes in the graph
        points_marker = Marker()
        points_marker.header.frame_id = self.frame_id
        points_marker.header.stamp = clock.now().to_msg()
        points_marker.ns = 'points'
        points_marker.type = Marker.SPHERE_LIST
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = 0.1
        points_marker.scale.y = 0.1
        points_marker.scale.z = 0.1
        points_marker.color.r = 1.0
        points_marker.color.a = ALPHA

        for _, loc in self.graph.nodes.data('location'):
            points_marker.points.append(Point(x=loc[0], y=loc[1], z=Z))

        ma.markers.append(points_marker)

        # Create marker to show the edges in the graph
        edges_marker = Marker()
        edges_marker.header.frame_id = self.frame_id
        edges_marker.header.stamp = points_marker.header.stamp
        edges_marker.ns = 'edges'
        edges_marker.type = Marker.LINE_LIST
        edges_marker.pose.orientation.w = 1.0
        edges_marker.scale.x = 0.05
        edges_marker.color.b = 1.0
        edges_marker.color.a = ALPHA

        for e in self.graph.edges:
            p0 = self.graph.nodes[e[0]]['location']
            p1 = self.graph.nodes[e[1]]['location']
            edges_marker.points.append(Point(x=p0[0], y=p0[1], z=Z))
            edges_marker.points.append(Point(x=p1[0], y=p1[1], z=Z))

        ma.markers.append(edges_marker)

        return ma
