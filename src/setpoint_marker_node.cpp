#include "waypointgen/setpoint_marker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "setpoint_marker");

  ros::NodeHandle n;
  waypointgen_marker wpm(n, false);

  // start the ROS main loop
  ros::spin();
  wpm.reset_server();
}