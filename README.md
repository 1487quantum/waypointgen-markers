# waypointgen

[![ROS CI](https://img.shields.io/github/workflow/status/1487quantum/waypointgen/ROS%20CI?label=CI&logo=ros&style=for-the-badge)](https://github.com/1487quantum/waypointgen/actions/workflows/ros_ci.yml)



A waypoint generator (using InteractiveMarkers in Rviz) and a waypoint server for waypoint playback. The waypoints are saved as a **YAML** file.

## Compilation

Git clone the ROS Package into the relevant workspace.

```bash
cd ~/catkin_ws/src
git clone https://github.com/1487quantum/waypointgen.git
```

Compile the package.

```bash
cd ~/catkin_ws
catkin_make
```

or 

```bash
cd ~/catkin_ws
catkin build waypointgen
```

## Setpoint Markers
![Rviz](assets/b.jpg)

> Utilizing interactive markers for a more visual approach in generating waypoints.

Launch the Waypoint Generator via *roslaunch*, which would launch the program and rviz:
```bash
$ roslaunch waypointgen setpoint_marker.launch  
```
Next, add the InteractiveMarker in Rviz. (Topic name: */setpoint_marker/update*)

![InteractiveMarkers](assets/a.jpg)

> Alternatively, load `wp_markers.rviz` into Rviz via `rviz -d wp_markers.rviz`.

There are 3 options available in the context menu (when the marker is _right-clicked_):
- **Get Location**: Get current location of marker selected.
- **Waypoint**
  - **Add**: Add new waypoint.
  - **Remove**: Remove selected waypoint.
- **Generate Waypoint List**: Generate waypoint list to be used in waypoint playback.


## Waypoint Server
The waypoint server would load the YAML which is specified in the *roslaunch* file, which would then publish the various navigation goals.

> **Note:** The setpoint_server node subscribes to */move_base/TebLocalPlanner/GlobalPath*, so change it accordingly if TebLocalPlanner Plugin is not used!

### Playback
Launch the playback server via *roslaunch*:
```bash
$ roslaunch waypointgen setpoint_server.launch  
```
The path of the waypoint list is specified via the `list_path` param, and is set through the `list_path` arguement.
```xml
  <arg name="list_path" default="$(find waypointgen)/wp_list/a.yaml" />
```
**Starting the server playback**

A ROS Service call would be used to start the playback of the points, call the `/trigger_play` service.

```bash
rosservice call /trigger_play <Delay in seconds>
```
For example, triggering the playback after 3s.

```bash
rosservice call /trigger_play 3
```

## Changelog

### v0.1.0

- Initial commit.

### v0.1.1

- Convert functions to classes.
- Refactored code.
- Replace ros topic subscription to ros service to trigger playback.
- Add ROS CI.

### v0.1.2

- Add benchmark metrics.
- Split into seperate ROS nodes.
- Refactored code.
- Removed custom msg.
- Update launch files.
