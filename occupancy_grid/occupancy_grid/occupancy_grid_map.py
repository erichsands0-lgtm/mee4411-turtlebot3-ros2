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
        boundary = [0., 0., 1., 1.]
        resolution = 1.
        frame_id = 'frame'
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Initialize object
        ogm = cls(boundary, resolution, frame_id)

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Update data array in ogm, based on conventions in the __init__ method
        pass
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
        pass
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
        pass
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
        occupied = np.zeros_like(x, dtype='bool')
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
        locations = np.zeros(2)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return locations
