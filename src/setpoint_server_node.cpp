#include "waypointgen/setpoint_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "setpoint_server");
  ros::NodeHandle n("~");
  waypointgen_server wpg(n);
  if (wpg.init() < 0)
    return 0;

  // Start Multithreading Process(Async thread):
  // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning

  ros::AsyncSpinner spinner(std::thread::hardware_concurrency());
  ros::Rate r(10); // Run at 10Hz

  spinner.start();

  ros::waitForShutdown();
  return 0;
}