# mrs_uav_o3de_api

MRS UAV System HW API plugin for the [O3DE multirotor simulator](https://github.com/klaxalk/o3de_drone_sim).

## Interface

The simulator uses standard messages only; the plugin translates the MRS HW API commands.

| MRS HW API | Simulator topic | Type | Mapping |
|---|---|---|---|
| attitude rate | `/$UAV_NAME/cmd/attitude_rate` | `geometry_msgs/TwistStamped` | `twist.angular` = body rate, `twist.linear.z` = throttle |
| attitude | `/$UAV_NAME/cmd/attitude` | `geometry_msgs/PoseStamped` | `pose.orientation` = attitude, `pose.position.z` = throttle |

Inputs: `/$UAV_NAME/odom` (`nav_msgs/Odometry`, twist in the body frame) and `/$UAV_NAME/imu` (`sensor_msgs/Imu`).
Everything else (GNSS, altitude, heading, ...) is derived from the odometry.

The simulator cuts the throttle 0.5 s after the last command, so the plugin disarms by not forwarding commands.

## Limitations

- O3DE has no rangefinder yet. `distance_sensor` is computed from the odometry over a flat floor at `rangefinder/ground_z` (`config/hw_api.yaml`), so it ignores obstacles under the UAV. A real sensor could replace it by publishing `sensor_msgs/Range`.
- `config/mrs_uav_system/x500.yaml` must match the simulator: `motor_params/a` is `1 / sqrt(k_f * omega_max^2)` and `b` is 0.
- `rangefinder/ground_z` is the negative of the spawn height, because the odometry starts at the spawn point.

## Fixed simulation step

O3DE exchanges data with ROS once per rendered frame, including `/clock`. In real time, each frame covers a varying amount of sim time, and at low frame rates a slow frame looks like a data dropout to the MRS UAV System.

The tmux session therefore runs the simulator with `--t_simulationTickDeltaOverride=17000`, in microseconds. Every frame then advances sim time by exactly 17 ms, so sim time depends on the frame rate instead of the wall clock: a slow frame only slows the simulation down. All nodes must use sim time, so keep `USE_SIM_TIME=true`.

The realtime factor is 0.017 s times the frame rate, about 0.5 at 30 FPS. A smaller step gives higher control and camera rates in sim time, at a lower realtime factor. The camera rate in sim time is limited by the step, 59 Hz at 17 ms.

## Build

Build against the MRS packages from `/opt/ros/jazzy`. The `colcon` wrapper from the MRS shell additions refuses to run if `ROS_WORKSPACE` points to another workspace.

```sh
cd ~/ws_o3de
unset ROS_WORKSPACE
source /opt/ros/jazzy/setup.zsh
colcon build
```

The launch file must use `component_container_events_cbg`: with `component_container_mt`, the subscribers stop receiving data after a moment.

## Run

Build the simulator first (`~/git/o3de_drone_sim`, or set `$O3DE_PROJECT`), then:

```sh
source ~/ws_o3de/install/setup.zsh
cd ~/ws_o3de/src/mrs_uav_o3de_api/tmux/one_drone
./start.sh
```
