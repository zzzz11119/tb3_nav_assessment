# Week 02 SLAM Comparison

## Objective

Compare SLAM Toolbox and Cartographer using the same TurtleBot3 sensor input in a small, repeatable Gazebo environment.

The formal assessment map remains the SLAM Toolbox map stored as `maps/assessment_map.yaml` and `maps/assessment_map.pgm`.

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Burger
- Comparison world: `worlds/simple_room.world`
- Map resolution: 0.05 m/cell
- Playback rate: 1.0x
- Simulation time enabled

## Fixed Input

A neutral rosbag was recorded while Gazebo and the TurtleBot3 simulation were running without either SLAM algorithm.

- Duration: 484.60 seconds
- Size: 22.1 MiB
- `/scan`: 1,832 messages
- `/odom`: 10,786 messages
- `/tf`: 17,904 messages
- `/tf_static`: 1 message
- `/clock`: 3,667 messages
- `/joint_states`: 10,786 messages

Both algorithms processed this same bag from a clean ROS graph. No `/map` topic was present in the recorded data.

## Results

| Metric | SLAM Toolbox | Cartographer |
|---|---:|---:|
| Resolution | 0.05 m/cell | 0.05 m/cell |
| Grid width | 121 cells | 132 cells |
| Grid height | 121 cells | 133 cells |
| Map coverage | 6.05 × 6.05 m | 6.60 × 6.65 m |
| Origin X | -3.01 m | -3.26 m |
| Origin Y | -3.00 m | -3.34 m |
| Outer walls | Closed and clean | Closed and recognizable |
| Obstacles | Clear outlines | Clear outlines with residual gray artifacts |
| Free-space consistency | Stable | Visible smearing along parts of the route |

Generated artifacts:

- `maps/simple_room_slam_toolbox.yaml`
- `maps/simple_room_slam_toolbox.pgm`
- `maps/simple_room_cartographer.yaml`
- `maps/simple_room_cartographer.pgm`

For Cartographer, trajectory 0 was explicitly finalized using the `/finish_trajectory` service before saving the map.

## Conclusion

Both algorithms completed mapping successfully from the same recorded input. SLAM Toolbox produced the cleaner and more compact occupancy grid, with fewer residual artifacts and bounds closer to the 6 m × 6 m comparison room.

SLAM Toolbox is therefore retained as the primary mapping solution for the assessment. Cartographer is retained as the required lightweight comparison and demonstrates that the project can reproduce mapping with a second SLAM implementation.
