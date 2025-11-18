import numpy as np
from typing import Tuple

class MapConversions:
    """
    Class for coordinate conversions between world (x, y), grid indices (row, col), and linear indices.
    """

    def __init__(self, boundary, resolution: float) -> None:
        """
        Initialize with boundary [xmin, ymin, xmax, ymax] and cell resolution.

        boundary: [xmin, ymin, xmax, ymax]
        resolution: size of each grid cell in meters
        """
        self.boundary = boundary
        self.resolution = resolution
        # Compute grid size (#rows, #cols)
        self.array_shape = (int(np.ceil((boundary[3] - boundary[1]) / resolution)),
                            int(np.ceil((boundary[2] - boundary[0]) / resolution)))

    @classmethod
    def from_msg(cls, msg):
        """
        Extract MapConversions from an OccupancyGrid message.
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract the boundary and cell resolution from the occupancy grid message

        resolution = msg.info.resolution
        xmin = msg.info.origin.position.x
        ymin = msg.info.origin.position.y
        xmax = xmin + msg.info.width * resolution
        ymax = ymin + msg.info.height * resolution
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

        inds = -np.ones_like(rows)
        mask = (rows >= 0) & (cols >= 0) & (rows < self.array_shape[0]) & (cols < self.array_shape[1])
        inds[mask] = rows[mask] * self.array_shape[1] + cols[mask]
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

        rows = -np.ones_like(inds)
        cols = -np.ones_like(inds)
        mask = (inds >= 0) & (inds < np.prod(self.array_shape))
        rows[mask] = inds[mask] // self.array_shape[1]
        cols[mask] = inds[mask] % self.array_shape[1]
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

        col = np.array(np.floor((x - self.boundary[0]) / self.resolution))
        row = np.array(np.floor((y - self.boundary[1]) / self.resolution))
        ## Look for top/right edge cases
        # Top Edge
        edge1=self.boundary[3]
        # Right edge
        edge2=self.boundary[2]

        mask1 = (y == edge1)  
        row[mask1] = row[mask1] -1

        mask2 = (x == edge2)
        col[mask2] = col[mask2] -1
        # invalid coordinates out the bottom
        mask = ( y < self.boundary[1]) |  ( x < self.boundary[0]) | ( y > self.boundary[3]) |  ( x > self.boundary[2]) | (np.isnan(y)) |(np.isnan(x))
        row[mask] = -1
        col[mask] = -1
        col = col.astype('int')
        row = row.astype('int')
         ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return row, col

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

        x = self.boundary[0] + (cols + 0.5) * self.resolution
        y = self.boundary[1] + (rows + 0.5) * self.resolution
        mask = (rows < 0) | (rows >= self.array_shape[0]) | (cols < 0) | (cols >= self.array_shape[1])
        x[mask] = np.nan
        y[mask] = np.nan
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return x, y

    def xy2ind(self, x: np.array, y: np.array) -> np.array:
        """
        Convert (x, y) coordinates to linear indices.
        """
        rows, cols = self.xy2sub(x, y)
        return self.sub2ind(rows, cols)

    def ind2xy(self, inds: np.array) -> Tuple[np.array, np.array]:
        """
        Convert linear indices to (x, y) coordinates.
        """
        rows, cols = self.ind2sub(inds)
        return self.sub2xy(rows, cols)
