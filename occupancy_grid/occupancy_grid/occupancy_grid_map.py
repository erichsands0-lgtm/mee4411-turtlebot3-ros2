from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid

import numpy as np

from .map_conversions import MapConversions


class OccupancyGridMap(MapConversions):
    def __init__(self, boundary, resolution, frame_id) -> None:
        super(OccupancyGridMap, self).__init__(boundary, resolution)
        # Set coordinate frame ID
        self.frame_id = frame_id
        # Initialize empty data array (2D array holding values)
        #   In the range [0, 100], representing the probability of occupancy
        #   If a cell is unknown, set to -1
        self.data = np.zeros(self.array_shape)

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        """Create an object from an OccupancyGrid msg."""
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract boundary, resolution, and frame_id from input message
        resolution =float(msg.info.resolution)
        xmin = float(msg.info.origin.position.x)
        ymin = float(msg.info.origin.position.y)
        width = int(msg.info.width)
        height = int(msg.info.height)
        xmax = xmin + width * resolution
        ymax = ymin + height * resolution
        boundary = [xmin, ymin, xmax, ymax]
        frame_id = msg.header.frame_id
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Initialize object
        ogm = cls(boundary, resolution, frame_id)

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Update data array in ogm, based on conventions in the __init__ method
        arr = np.array(msg.data, dtype=np.int16)
        arr = arr.clip(arr, -1, 100).astype(np.int8)
        ogm.data = arr.reshape(ogm.array_shape)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return ogm

    def add_block(self, block: np.array) -> None:
        """
        Add a block to the map stored in self.data.

        Inputs:
            block   np.array in the format (xmin, ymin, xmax, ymax)
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in all the cells that overlap with the block
        xmin_b, ymin_b, xmax_b, ymax_b = map(float, block)
        xmin, ymin, xmax, ymax = self.boundary
        s = self.resolution
        nrows, ncols = self.array_shape

        eps = 1e-9
        r0 = int(np.floor((ymin_b - ymin) / s + eps))
        r1 = int(np.ceil ((ymax_b - ymin) / s - eps)) - 1
        c0 = int(np.floor((xmin_b - xmin) / s + eps))
        c1 = int(np.ceil ((xmax_b - xmin) / s - eps)) - 1

        r0 = max(0, r0); r1 = min(nrows - 1, r1)
        c0 = max(0, c0); c1 = min(ncols - 1, c1)

        if r0 <= r1 and c0 <= c1:
            self.data[r0:r1+1, c0:c1+1] = 100
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
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in all the fields of the msg using the data from the class
        msg.header.stamp = time.to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.array_shape[1])
        msg.info.height = int(self.array_shape[0])

        xmin, ymin, xmax, ymax = self.boundary
        msg.info.origin.position.x = float(xmin)
        msg.info.origin.position.y = float(ymin)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        clipped = np.clip(self.data, -1, 100).astype(np.int8)
        msg.data = clipped.reshape(-1).tolist()
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return msg

    def is_occupied(self, x: np.array, y: np.array, *, threshold=50) -> bool:
        """
        Check whether the given cells are occupied.

        Inputs:
            x           numpy array of x values
            y           numpy array of y values
        Optional Inputs:
            threshold   minimum value to consider a cell occupied (default 50)
        Outputs:
            occupied    np.array of bool values
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Check for occupancy in the map based on the input type
        rows, cols = self.xy2rc(x, y)
        nrows, ncols = self.array_shape
        valid = (rows >= 0) & (cols >= 0) & (rows < nrows) & (cols < ncols)

        occupied = np.zeros_like(x, dtype=bool)
        occupied[valid] = self.data[rows[valid], cols[valid]] >= 50
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return occupied

    def where_occupied(self, *, format='xy', threshold=50) -> np.array:
        """
        Find the locations of all cells that are occupied.

        Optional Inputs:
            format      requested format of the returned data: 'xy', 'rc', 'ind' (default 'xy')
            threshold   minimum value to consider a cell occupied (default 50)
        Outputs:
            locations   np.array with the locations of occupied cells in the requested format
        """
        # Check that requested format is valid
        if format not in ('xy', 'rc', 'ind'):
            raise Exception(f'Requested format {format} invalid, must be xy, rc, or ind')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Check for occupancy in the map based on the input type
        nrows, ncols = self.array_shape
        mask = self.data >= 50

        r, c = np.where(mask)

        if format == 'rc':
            locations = np.vstack((r, c)).T
        elif format == 'ind':
            locations = (r * ncols + c).astype(int)
        else: 
            x, y = self.sub2xy(r, c)
            locations = np.vstack((x, y)).T
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return locations
