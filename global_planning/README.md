
```bash
pip3 install networkx tqdm
pip3 install --upgrade opencv-python

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```



- Use `robot_navigator` as the primary node
- Switch up the clients to use different our nodes
    - E.g., `get_costmap_global_srv` will come from a node that subscribes to map and sends out the costmap
- Add planner node
- Add local control node
- Look into smoothing?



- bt_navigator subscribes to goal_pose
- also has both server and client for /navigate_to_pose

- behavior_server is the thing publishing /cmd_vel

- planner_server is doing the actual planning
- using the /compute_path_to_pose action server (nav2_msgs/action/ComputePathToPose)
- client is the /bt_navigator_navigate_to_pose_rclcpp_node
- need to replace this server with my own
    - remap the current action
    - replace with mine
    