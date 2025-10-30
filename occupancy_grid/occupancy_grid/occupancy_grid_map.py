from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from .map_conversions import MapConversions

class OccupancyGridMap(MapConversions):
    """
    Occupancy grid map with data array and conversion utilities.
    """

    def __init__(self, boundary, resolution, frame_id) -> None:
        super().__init__(boundary, resolution)
        self.frame_id = frame_id
        # 0=free, 100=occupied
        self.data = np.zeros(self.array_shape, dtype=np.int8)

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        """Create an object from an OccupancyGrid msg."""
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract boundary, resolution, and frame_id from input message
        resolution = msg.info.resolution
        xmin = msg.info.origin.position.x
        ymin = msg.info.origin.position.y
        xmax = xmin + msg.info.width * resolution
        ymax = ymin + msg.info.height * resolution
        boundary = [xmin, ymin, xmax, ymax]
        frame_id = msg.header.frame_id
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        ogm = cls(boundary, resolution, frame_id)
        # Fill data
        ogm.data = np.array(msg.data, dtype=np.int8).reshape(ogm.array_shape)
        return ogm

    def add_block(self, block: np.array) -> None:
        """
        Add a block to the map stored in self.data.

        Inputs:
            block   np.array in the format (xmin, ymin, xmax, ymax)
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in all the cells that overlap with the block
        row_min, col_min = self.xy2sub(np.array([block[0]]), np.array([block[1]]))
        row_max, col_max = self.xy2sub(np.array([block[2]]), np.array([block[3]]))
        if row_min[0] < 0 or col_min[0] < 0 or row_max[0] < 0 or col_max[0] < 0:
            return
        self.data[row_min[0]:row_max[0]+1, col_min[0]:col_max[0]+1] = 100
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    def to_msg(self, time: Time) -> OccupancyGrid:
        """
        Convert the OccupancyGridMap object into an OccupancyGrid ROS message.

        Inputs:
            time    current ROS time
        Outputs:
            msg     OccupancyGrid ROS message
        """
        msg = OccupancyGrid()
        msg.header.stamp = time.to_msg()
        msg.header.frame_id = self.frame_id

        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.array_shape[1]
        msg.info.height = self.array_shape[0]
        msg.info.origin.position.x = self.boundary[0]
        msg.info.origin.position.y = self.boundary[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # no rotation

        msg.data = self.data.flatten().tolist()
        return msg

    def is_occupied(self, x: np.array, y: np.array) -> np.array:
        """
        Check occupancy for given (x, y) points.
        """
        rows, cols = self.xy2sub(x, y)
        mask = (rows < 0) and (rows >= self.array_shape[0]) and (cols < 0) and (cols >= self.array_shape[1])
        occ = np.zeros_like(x, dtype=bool)
        occ[mask] = self.data[rows[mask], cols[mask]] >= 50
        return occ

    def where_occupied(self, format='xy') -> np.array:
        """
        Find the locations of all cells that are occupied.

        Optional Inputs:
            format      requested format of the returned data: 'xy', 'rc', 'ind' (default 'xy')
            threshold   minimum value to consider a cell occupied (default 50)
        Outputs:
            locations   np.array with the locations of occupied cells in the requested format
        """
        rows, cols = np.where(self.data > 50)
        if format == 'rc':
            return np.vstack((rows, cols)).T
        elif format == 'ind':
            return self.sub2ind(rows, cols)
        elif format == 'xy':
            x, y = self.sub2xy(rows, cols)
            return np.vstack((x, y)).T
        else:
            raise ValueError("format must be 'xy', 'rc', or 'ind'")
