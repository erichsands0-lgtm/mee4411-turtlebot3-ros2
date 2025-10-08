from nav_msgs.msg import OccupancyGrid

import numpy as np
from typing import Tuple


class MapConversions:
    def __init__(self, boundary, resolution: float) -> None:
        """
        Create an object from parameters.

        inputs:
            boundary    edges of the environment in the order (xmin, ymin, xmax, ymax) [m]
            resolution  size of the cells in the occupancy grid [m]
        """
        # Boundary of the envrionment in the format (xmin, ymin, xmax, ymax)
        self.boundary = boundary
        # Size of the cells in the occupancy grid
        self.resolution = resolution
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Create the array shape in the format (# rows, # columns)
        xmin, ymin, xmax, ymax, = boundary
        Nr = int((ymax - ymin) / resolution)
        Nc = int((xmax - xmin) / resolution)
        self.array_shape = (Nr, Nc)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        """Create an object from an OccupancyGrid ROS msg."""
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract the boundary and cell resolution from the occupancy grid message
        xmin = msg.info.origin.position.x
        ymin = msg.info.origin.position.y
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        
        xmax = xmin + width * resolution
        ymax = ymin + height * resolution
        boundary = [xmin, ymin, xmax, ymax]
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return cls(boundary, resolution)

    def sub2ind(self, rows: np.array, cols: np.array) -> np.array:
        """
        sub2ind coverts subscript (row, column) pairs into linear indices in row-major order.

        inputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        outputs:
            inds    numpy array of integer indices
        Note: (row, column) pairs that are not valid should have a
              corresponding output index of -1
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Convert data in (row, col) format to ind format
        Nr, Nc=self.array_shape
        valid = (rows >= 0) & (cols >= 00) & (rows < Nr) & (cols < Nc)
        inds = np.where(valid, rows * Nc + cols, -1)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return inds

    def ind2sub(self, inds: np.array) -> Tuple[np.array, np.array]:
        """
        ind2sub converts linear indices in a row-major array to subscript (row, column) pairs.

        inputs:
            inds    numpy array of integer indices
        outputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        Note: any indices that are not valid should have row and column
              subscripts outputs of -1
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Convert data in ind format to (row, col) format
        Nr, Nc = self.array_shape
        valid = (inds >= 0) & (inds < Nr * Nc)
        rows = np.where(valid, np.floor_divide(inds, Nc), -1)
        cols = np.where(valid, np.mod(inds, Nc), -1)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return rows, cols

    def xy2sub(self, x: np.array, y: np.array) -> Tuple[np.array, np.array]:
        """
        xy2sub converts (x,y) coordinate pairs into (row, column) subscript pairs.

        inputs:
            x       numpy array of x values
            y       numpy array of y values
        outputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        Note: any (x,y) pairs that are not valid should have subscript
              outputs of -1
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Convert data in (x, y) format to (row, col) format
        xmin, ymin, xmax, ymax = self.boundary
        s = self.resolution

        r = np.floor((y - ymin) / s).astype(int)
        c = np.floor((x - xmin) / s).astype(int)

        Nr, Nc = self.array_shape
        valid = (x >= xmin) & (x < xmax) & (y >= ymin) & (y < ymax)
        r[~valid] = -1
        c[~valid] = -1
        rows, cols = r, c
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return rows, cols

    def sub2xy(self, rows: np.array, cols: np.array) -> Tuple[np.array, np.array]:
        """
        sub2xy converts (row, column) subscript pairs into (x,y) coordinate pairs.

        inputs:
            rows        numpy array of row indices
            cols        numpy array of column indices
        outputs:
            x       numpy array of x coordinates of center of each cell
            y       numpy array of y coordinates of center of each cell
        Note: any (row, col) pairs that are not valid should have outputs
              of numpy NaN
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Convert data in (row, col) format to (x, y) format
        xmin, ymin, xmax, ymax = self.boundary
        s = self.resolution
        Nr, Nc = self.array_shape

        valid = (rows >= 0) & (rows < Nr) & (cols >= 0) & (cols < Nc)
        x = np.where(valid, xmin + s * (cols + 0.5), np.nan)
        y = np.where(valid, ymin + s * (rows + 0.5), np.nan)
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return x, y

    def xy2ind(self, x: np.array, y: np.array) -> np.array:
        """
        xy2ind converts (x,y) coordinate pairs into linear indices in row-major order.

        inputs:
            x           numpy array of x values
            y           numpy array of y values
        outputs:
            numpy array of row indices
            numpy array of column indices
        """
        rows, cols = self.xy2sub(x, y)
        ind = self.sub2ind(rows, cols)
        return ind

    def ind2xy(self, inds: np.array) -> Tuple[np.array, np.array]:
        """
        ind2xy converts linear indices in row-major order into (x,y) coordinate pairs.

        inputs:
            inds        numpy array of indices
        outputs:
            numpy array of x coordinates
            numpy array of y coordinates
        """
        rows, cols = self.ind2sub(inds)
        x, y = self.sub2xy(rows, cols)
        return (x, y)
