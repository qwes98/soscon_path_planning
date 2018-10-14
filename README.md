## Build

1. Put this package in your `catkin_ws/src`
2. `cd catkin_ws`
3. `catkin_make`
4. `rospack profile`
5. `source catkin_ws/devel/setup.bash`

## Launch coverage path planing service

1. `roslaunch coverage_path_planning covereage_path_planning_service.launch`

## Test coverage path planing service 

1. `roslaunch coverage_path_planning covereage_path_planning_client.launch`

   If succeed some path poses will be printed in the terminal.

## How to Trouble Shooting?

need some info to trouble shooting
- map file

```
    example: like "map.yaml" and "map.pgm"
```

- param list when make service call

```
    example: 
            srv.request.erosion_radius = 0.01;     //  unit: meter
            srv.request.robot_radius = 0.04;        //  unit: meter
            srv.request.occupancy_threshold = 95;  //  range:0~100
```

- screen shot for bad planner result in rviz


