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

## Low frame rates

O3DE exchanges data with ROS once per rendered frame, including `/clock`, while physics keeps real time. With rendering on the integrated GPU, that is about 25 Hz. The configuration accounts for it:

- `input_mode` uses attitude only. The attitude loop then runs inside O3DE at 500 Hz. Attitude rate mode crashed during a goto at 25 Hz, though it flew fine at about 900 Hz with `--rhi=null`.
- `tmux/one_drone/config/custom_config.yaml` raises `odometry_max_missing_time` to 0.5 s, because one slow frame can exceed the default 0.1 s.
- The session waits 30 s before takeoff. The frame rate can dip to about 3 Hz in the first 30 s, which triggered a failsafe landing with the default timeout.

The frame rate also dips during flight, and a dip can cost a few tens of centimeters of height.

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
