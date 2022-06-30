#ifndef SETPOINT_MARKER_H
#define SETPOINT_MARKER_H

#include <boost/bind.hpp>
#include <map>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "waypointgen/waypointgen_utils.h"

using p_map = std::map<std::string, geometry_msgs::PoseWithCovariance>;

class waypointgen_marker {
public:
  waypointgen_marker(const ros::NodeHandle &nh);
  waypointgen_marker(const ros::NodeHandle &nh, const bool &debug);

  void updateWypt(p_map &tmp, const std::string &wp_name,
                  const geometry_msgs::PoseWithCovariance &pt);
  void addWaypointMarker(const unsigned int &mrk_id);

  // Sub/Pub
  void updateWaypointPos(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void setpointListCallback(
      const visualization_msgs::InteractiveMarkerInitConstPtr msg);

  // Menu
  void mnu_getLocation(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mnu_addNewWaypoint(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mnu_createList(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mnu_removeWaypoint(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void addContextMnu(const bool &init);

  // Markers
  void setHeader(visualization_msgs::InteractiveMarker &imk_h,
                 const int &index_h);
  void setPos(visualization_msgs::InteractiveMarker &imk_h, const int &sx,
              const int &sy);
  visualization_msgs::Marker makeArrow(unsigned char mrk_id);

  // Controls
  void addIntCtrl(visualization_msgs::InteractiveMarkerControl &imc,
                  const unsigned char &mk_id);
  void addMovementControl(visualization_msgs::InteractiveMarkerControl &im_c,
                          const bool &mw, const bool &mx, const bool &my,
                          const bool &mz, const std::string &mName,
                          const bool &mvAxis);
  void addMotionControl(visualization_msgs::InteractiveMarker &i_mk,
                        visualization_msgs::InteractiveMarkerControl &imc_am);

  void updateWPList(p_map &tmp);

  void reset_server();

  p_map wpl;

  // Subscriber
  ros::Subscriber sub_setpoint_list;

  // Menu
  interactive_markers::MenuHandler menu_handler;
  waypointgen_utils wpg_utils;

private:
  bool menu_init_;

  // Marker
  int marker_id_;
  int marker_count_; // Keep track of number of waypoints created

  float lastMarker[2] = {0.0, 0.0}; // Get pos of last marker

  bool DEBUG;

  ros::NodeHandle nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

#endif