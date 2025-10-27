
```bash
pip3 install networkx tqdm

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```



- Use `robot_navigator` as the primary node
- Switch up the clients to use different our nodes
    - E.g., `get_costmap_global_srv` will come from a node that subscribes to map and sends out the costmap
- Add planner node
- Add local control node
- Look into smoothing?
