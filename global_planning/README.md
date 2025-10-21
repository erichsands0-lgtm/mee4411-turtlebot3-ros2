
```bash
pip3 install networkx tqdm
```



- Use `robot_navigator` as the primary node
- Switch up the clients to use different our nodes
    - E.g., `get_costmap_global_srv` will come from a node that subscribes to map and sends out the costmap
- Add planner node
- Add local control node
- Look into smoothing?
