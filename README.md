# Particle Filter Localization

## Particle Filter Exercise

This package implements the particle filter localisation using sensor and motion update from the Pioneer P3-DX robot. The methods in `src/pf_localisation/pf.py` have  be completed  to run the node. 

### Running the node:

The localisation node can be tested in stage simulation (without the need for robot).

        roscore
        rosrun map_server map_server <catkin_ws>/map.yaml
        rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world
        roslaunch socspioneer keyboard_teleop.launch  # ---- run only if you want to move robot using keyboard 
        rosrun pf_localisation node.py    # ----- requires laser_trace, and completed pf.py methods.

### Published Topics:

Running the node successfully will publish the following topics:

* `/map` 
* `/amcl_pose` 
* `/particle_cloud`

All of these can be visualised in RViz by adding the appropriate Views.

