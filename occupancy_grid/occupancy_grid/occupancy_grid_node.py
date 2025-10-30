import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # noqa: F401
from .map_conversions import MapConversions  # noqa: F401
from .occupancy_grid_map import OccupancyGridMap  # noqa: F401
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import yaml
import os

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        
        # read parameters
        resolution = self.declare_parameter('resolution').value
        frame_id = self.declare_parameter('frame_id').value

         # Load YAML map

        boundary = self.declare_parameter('boundary').value
        blocks = self.declare_parameter('blocks').value
        # logger that prof used self.get_logger().info([resolution,frame_id,boundary,blocks])

        # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
        self.ogm = OccupancyGridMap(boundary, resolution, frame_id)
        for i in range(0, len(blocks), 4):
            self.ogm.add_block(blocks[i:i+4])

         # TODO Set up the ROS publisher for the occupancy grid map
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

      # TODO Create and publish a nav_msgs/OccupancyGrid msg
        msg = self.ogm.to_msg(self.get_clock().now())
        self.publisher_.publish(msg)
        self.get_logger().info("Occupancy grid published successfully.")


def main(args=None):
    rclpy.init(args=args)
    og_node = OccupancyGridNode()
    rclpy.spin(og_node)
    og_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
