from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class GoalChecker:
    def __init__(self, xy_goal_tolerance: float = 0.25) -> None:
        self.xy_goal_tolerance = xy_goal_tolerance
        self.xy_goal_tolerance_sq = xy_goal_tolerance**2

    def reset(self) -> None:
        self.check_xy = True

    def is_goal_reached(
            self,
            query_pose: Pose,
            goal_pose: Pose,
            velocity: Twist = None
    ) -> bool:
        """
        Check whether the goal should be considered reached.

        query_pose The pose to check
        goal_pose The pose to check against
        velocity The robot's current velocity

        return True if goal is reached
        """
        dx = query_pose.position.x - goal_pose.position.x
        dy = query_pose.position.y - goal_pose.position.y
        return dx * dx + dy * dy < self.xy_goal_tolerance_sq
