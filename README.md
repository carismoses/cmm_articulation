# cmm_articulation

# To run
1. you have to have the tum_vision/articulation package
2. make sure `roscore` is running in a terminal
2. `roslaunch cmm_articulation articulation_busybox.py`
3. this node subscribes to /bb_poses (geometry_msgs/PoseStamped). If you don't have something publishing to this you can test by running this:
```
rostopic pub -r 10 /bb_poses geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 1.0
    y: 2.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
