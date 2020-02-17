# waypointgen
A waypoint generator (using InteractiveMarkers in Rviz) and a waypoint server for waypoint playback. The waypoints are saved as a **YAML** file.

## Usage
Ensure that the waypointgen action msg is cloned into the workspace before cloning the main waypointgen package:
```
$ git clone https://github.com/1487quantum/wpg_msg.git
```

Git clone the ROS Package into the relevant workspace & compile it:
```
$ git clone https://github.com/1487quantum/waypointgen.git
$ catkin_make
```

### Setpoint Markers
![Rviz](assets/b.jpg)

Launch the Waypoint Generator via *roslaunch*, which would launch the program and rviz:
```
$ roslaunch waypointgen setpoint_marker.launch  
```
Next, add the InteractiveMarker in Rviz. (Topic name: */setpoint_marker/update*)

![InteractiveMarkers](assets/a.jpg)

> Alternatively, load _wp_markers.rviz_ into Rviz!

There are 3 options to choose from the context menu (when the marker is _right-clicked_), which are
- **Get Location**: Get current location of marker selected.
- **Waypoint**
  - **Add**: Add new waypoint.
  - **Remove**: Remove selected waypoint.
- **Generate Waypoint List**: Generate waypoint list to be used in waypoint playback.


### Waypoint Server
The waypoint server would load the YAML which is specified in the *roslaunch* file, which would then publish the various navigation goals.
> **Note:** The setpoint_server node subscribes to */move_base/TebLocalPlanner/GlobalPath*, so change it accordingly if TebLocalPlanner Plugin is not used!
#### Playback
Launch the playback server via *roslaunch*:
```
$ roslaunch waypointgen setpoint_server.launch  
```
The path of the waypoint list is specified via the *pathway* param in the launch file.
```
<param name="pathway" type="str" value="a.yaml"/>
```
The server would send out the waypoint goals from the list specified in the launch file 10s after initialisation.
