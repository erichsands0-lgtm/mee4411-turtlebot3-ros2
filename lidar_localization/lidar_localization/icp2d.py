import numpy as np
from typing import Optional, Tuple, Union
from sklearn.neighbors import NearestNeighbors


class ICP2D:
    """2D Iterative Closest Point (ICP) algorithm."""

    def __init__(self) -> None:
        pass

    def set_map_points(self, map_pts: np.ndarray) -> None:
        """
        Initialize a set of points to match against.

        Inputs:
            map_pts: 2xN numpy.ndarray of 2D points

        Outputs:
            None

        This function should:
            1. Store the map points in self.map_pts
            2. Create a NearestNeighbors object and fit it to the map points
               (store the object in self.neighbors)
        """
        # Ensure points of the correct dimension
        assert map_pts.shape[0] == 2, 'Points must be formatted as 2xN numpy arrays'

        # Store the map points and create the nearest neighbors object
        self.map_pts = map_pts
        self.neighbors = NearestNeighbors(n_neighbors=1).fit(map_pts.T)

    def best_fit_transform(self, pts: np.array) -> np.ndarray:
        """
        Calculate the least-squares best-fit transform that maps pts on to map_pts.

        Inputs:
            pts: 2xN numpy.ndarray of points

        Outputs:
            T: 3x3 homogeneous transformation matrix that maps pts on to map_pts
            error: average distance between points in the match

        Note: The points pts and map_pts must be of the same size. This function
              finds the best-fit transform that maps pts on to map_pts.
        """
        # Ensure points of the correct dimension
        assert pts.shape[0] == 2

        # Find the nearest (Euclidean) neighbor in the map for each point in pts
        distances, indices = self.neighbors.kneighbors(pts.T, return_distance=True)
        distances = distances.ravel()  # make 1D arrays
        indices = indices.ravel()

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract corresponding points from self.map_pts and store in the variable map
        map = pts

        # Check to make sure you have the same number of points
        assert pts.shape == map.shape

        # TODO Translate point sets (pts, map) to their centroids
        pass

        # TODO Use the SVD to find the rotation matrix using the np.linalg.svd function
        pass

        # TODO Make sure det(R) > 0, if not, multiply the last column of Vt by -1 and recalculate R
        pass

        # TODO Find the translation vector using the formula to calculate the translation vector
        pass

        # TODO Fill in the homogeneous transformation
        T = np.identity(3)

        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        error = np.mean(distances)
        return T, error

    def icp(self,
            pts: np.ndarray,
            init_pose: Optional[Union[np.array, None]] = None,
            max_iterations: Optional[int] = 20,
            tolerance: Optional[float] = 0.05) -> Tuple[np.ndarray, np.array, int]:
        """
        Find the best-fit transform that maps points A on to points B using ICP.

        Inputs:
            pts: 2xN numpy.ndarray of source points
            init_pose: 3x3 homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria

        Outputs:
            T: final homogeneous transformation that maps pts on to map_pts
            mean_error: mean Euclidean distance (error) of the nearest neighbor fit
            i: number of iterations to converge
        """
        # Get number of dimensions and ensure that it is correct
        m = pts.shape[0]
        assert m == 2, 'Points must be formatted as 2xN numpy arrays'

        # Initialize the transformation
        T = np.eye(3)  # initialize identity transformation

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Make points homogeneous, copy them to maintain the originals
        # TODO See if there an initial pose estimate, if so then fill it in
        pass

        # TODO Apply the initial pose estimate to the points (pts)
        pass

        # TODO Apply the initial pose estimate
        pass

        # Run ICP
        prev_error = 1e6  # initialize to a large number
        for i in range(max_iterations):
            # TODO Compute the TF between the current source and nearest destination points
            pass
            mean_error = 0  # TODO replace this with the output of best_fit_transform

            # TODO Update the current estimated transofmration and point locations using T_delta
            pass
            ##### YOUR CODE ENDS HERE   ##### # noqa: E266

            # Check error
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # Return
        return T, mean_error, i
