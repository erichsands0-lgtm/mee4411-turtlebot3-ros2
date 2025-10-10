from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from geometry_msgs.msg import Transform

import numpy as np
from typing import Optional, Tuple, Union

from .transform2d_utils import transform2homogeneous, transform2xyt


def lookup_transform(tf_buffer: tf2_ros.Buffer,
                     base_frame: str,
                     child_frame: str,
                     *,
                     stamp: Time = Time(),
                     time_travel: Optional[Duration] = Duration(seconds=0),
                     format: Optional[str] = 'transform'
                     ) -> Union[None, np.ndarray, Transform, Tuple]:
    """
    Look up transformation on the TF2 buffer from base_frame to child_frame.

    Inputs:
        logger: ROS2 logger object to write output
        tf_buffer: TF2 buffer object
        base_frame: base frame of transformation
        child_frame: child frame of transformation
        stamp: time stamp for TF lookup (rclpy.time.Time() for the latest available)
        time_travel: amount to move backward in time
        format: format of the returned transformation 'transform', 'homogeneous', or 'xyt'
    Outputs:
        Transformation in the given format
    """
    # Formation validation
    if format not in {'transform', 'homogeneous', 'xyt'}:
        raise ValueError(f'Format {format} not valid, must be: transform, homogeneous, or xyt')

    # Lookup transform
    try:
        trans = tf_buffer.lookup_transform(
            base_frame,
            child_frame,
            stamp - time_travel,
            timeout=Duration(seconds=2.0)
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        raise

    # Return value
    if format == 'transform':
        return trans
    elif format == 'homogeneous':
        return transform2homogeneous(trans.transform)
    elif format == 'xyt':
        return transform2xyt(trans.transform)
