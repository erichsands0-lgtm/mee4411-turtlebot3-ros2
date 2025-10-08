#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # noqa: F401

from .map_conversions import MapConversions  # noqa: F401
from .occupancy_grid_map import OccupancyGridMap  # noqa: F401


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Set up the ROS node
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos)

        # TODO Set up the ROS publisher for the occupancy grid map
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('boundary', [0.0, 0.0, 10.0, 10.0])
        self.declare_parameter('blocks', [])
        self.declare_parameter('start_unknown', True)

        frame_id = self.get_parameter('frame_id').value
        resolution = float(self.get_parameter('resolution').value)
        boundary = [float(v) for v in self.get_parameter('boundary').value]
        blocks = self.get_parameter('blocks').value
        start_unknown = bool(self.get_parameter('start_unknown').value)

        # TODO Read in the map information from the ROS parameter server
        ogm = OccupancyGridMap(boundary, resolution, frame_id)
        if start_unknown:
            ogm.data[:] = -1 #Unknown everywhere at start

        # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
        for b in blocks:
            b = [float(b[0]), float(b[1]), float(b[2]), float(b[3])]
            ogm.add_block(np.array(b, dtype=float))

        # TODO Create and publish a nav_msgs/OccupancyGrid msg
        msg = ogm.to_msg(self.get_clock().now())
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published OccupancyGrid once: frame={frame_id}, res={resolution}, "
            f"size={ogm.array_shape}, boundary={tuple(boundary)} (Qos: TRANSIENT_LOCAL)"
        )
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266


def main(args=None):
    # Start up ROS2
    rclpy.init(args=args)

    # Create the node
    og_node = OccupancyGridNode()

    # Let the node run until it is killed
    rclpy.spin(og_node)

    # Clean up the node and stop ROS2
    og_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
