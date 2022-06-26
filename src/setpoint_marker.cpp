#include "waypointgen/setpoint_marker.h"

waypointgen_marker::waypointgen_marker(const ros::NodeHandle &nh) {
  waypointgen_marker(nh, false);
}

waypointgen_marker::waypointgen_marker(const ros::NodeHandle &nh,
                                       const bool &debug)
    : menu_init_(false), marker_id_(0), marker_count_(0), DEBUG(debug) {

  this->nh_ = nh;

  server_.reset(new interactive_markers::InteractiveMarkerServer(
      "setpoint_marker", "", false));

  // Add waypoint at start
  addWaypointMarker(this->marker_id_);

  // Subscribe to the waypoints published
  sub_setpoint_list =
      this->nh_.subscribe("setpoint_marker/update_full", 1,
                          &waypointgen_marker::setpointListCallback, this);
}

void waypointgen_marker::updateWaypointPos(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
    // Update marker position when user releases marker
    if (DEBUG) {
      ROS_INFO(
          "[MOUSE_UP] %s: \nframe: %s\nPos: %f, %f, %f\nOrient: %f, %f, %f, %f",
          feedback->marker_name.c_str(), feedback->header.frame_id.c_str(),
          feedback->pose.position.x, feedback->pose.position.y,
          feedback->pose.position.z, feedback->pose.orientation.x,
          feedback->pose.orientation.y, feedback->pose.orientation.z,
          feedback->pose.orientation.w);
    }
    // Update lastMarker position
    lastMarker[0] = feedback->pose.position.x;
    lastMarker[1] = feedback->pose.position.y;

    geometry_msgs::Point p_point;
    p_point = feedback->pose.position;

    tf::Quaternion p_quat(
        feedback->pose.orientation.x, feedback->pose.orientation.y,
        feedback->pose.orientation.z,
        feedback->pose.orientation.w); // change geometry to tf quaternion

    auto pwc = wpg_utils.addPoseCov(
        p_point,
        p_quat); // Update pose in map, geometry_msgs::PoseWithCovariance

    updateWypt(wpl, feedback->marker_name, pwc);
    server_->applyChanges();
  }
}

// Subscriber callback
void waypointgen_marker::setpointListCallback(
    const visualization_msgs::InteractiveMarkerInitConstPtr msg) {
  geometry_msgs::PoseWithCovariance pt;
  updateWPList(wpl);
  for (auto mk : msg->markers) {
    geometry_msgs::Point p_point;
    p_point = mk.pose.position;
    p_point.z = 0.0;

    tf::Quaternion qtmp(
        mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z,
        mk.pose.orientation
            .w); // Create Quaternion for rotation, tf::Quaternion
    pt = wpg_utils.addPoseCov(p_point, qtmp); // Create pose
    updateWypt(wpl, mk.name, pt);             // Add to map
    if (DEBUG)
      wpg_utils.printDebugPose("LIST SUB CALLBACK", mk.name, pt);
  }
}

void waypointgen_marker::updateWPList(p_map &tmp) {
  if (tmp.empty())
    ROS_INFO("Map is empty! Updating...");

  for (const auto &it : tmp)
    std::cout << it.first << " :: " << it.second << std::endl;
}

// Add waypoint to map
void waypointgen_marker::updateWypt(
    p_map &tmp, const std::string &wp_name,
    const geometry_msgs::PoseWithCovariance &pt) {
  if (tmp.insert(std::make_pair(wp_name, pt)).second == false) {
    if (DEBUG)
      ROS_WARN("Overwriting %s", wp_name.c_str());
    tmp[wp_name] = pt;
  } else {
    tmp.insert(std::make_pair(wp_name, pt));
    if (DEBUG)
      wpg_utils.printDebugPose("INSERT WAYPONT", wp_name, pt);
  }
}

// Show selected waypoint Location
void waypointgen_marker::mnu_getLocation(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  geometry_msgs::PoseWithCovariance pwc_gl;
  pwc_gl.pose = feedback->pose;
  wpg_utils.printDebugPose("CURRENT LOC", feedback->marker_name,
                           pwc_gl); // To display on console
}

// Add waypoint
void waypointgen_marker::mnu_addNewWaypoint(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (marker_id_ < 256) {
    addWaypointMarker(++marker_id_);
    ROS_INFO("Waypoint Added!");
  } else {
    ROS_INFO("Setpoint limit reached!");
  }
}

// Remove waypoint_
void waypointgen_marker::mnu_removeWaypoint(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std_msgs::String dispMsg;               // To display on console
  auto markerName{feedback->marker_name}; // Marker Name, std::string
  auto delimiter{"_"}; // Delimit to the end to get id, std::string

  auto markerID{
      stoi(markerName.substr(markerName.find(delimiter) + 1,
                             -1))}; // Get last part of string, since name is
                                    // waypoint_[ID]; Convert str->int

  if (marker_count_ > 1) {
    wpl.erase(markerName); // Remove from map

    // Remove from server
    server_->erase(feedback->marker_name);
    server_->applyChanges();

    --marker_count_; // Decrement counter

    dispMsg.data = feedback->marker_name + " removed!";
  } else {
    dispMsg.data = "Cannot remove all points!";
  }

  ROS_INFO("%s", dispMsg.data.c_str());
}

void waypointgen_marker::mnu_createList(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO("%s %i", "[Generating list] Total Waypoints ->", marker_count_);

  // Save as YAML file
  YAML::Node pts;
  pts["count"] = marker_count_; // Write total number of waypoints

  auto idx{0};

  // Iterate through map
  for (const auto &it : wpl) {
    std::string wp_index = "WP" + std::to_string(idx);
    // Add Position to YAML list
    // pts[wp_index].push_back(it->first);

    // Save quaternion to YAML
    pts[wp_index].push_back(it.second.pose.position.x);
    pts[wp_index].push_back(it.second.pose.position.y);
    pts[wp_index].push_back(it.second.pose.orientation.x);
    pts[wp_index].push_back(it.second.pose.orientation.y);
    pts[wp_index].push_back(it.second.pose.orientation.z);
    pts[wp_index].push_back(it.second.pose.orientation.w);
    idx++;
  }

  // Create list
  std::fstream exportlist;
  std::string loc = ros::package::getPath("waypointgen") +
                    "/wp_list/"; // Get package path, Save into list directory

  // Check if list directory exist
  const char *path = loc.c_str();
  std::filesystem::path dir(path);
  if (std::filesystem::create_directory(dir))
    ROS_INFO("'list' directory not found, creating one -> %s", loc.c_str());

  auto lstName = wpg_utils.getCurrentTime() + "_wplist.yaml"; // File Name
  loc += lstName; // Append to main path
  if (DEBUG)
    ROS_INFO("Writing waypoint list to %s", loc.c_str());

  exportlist.open(loc, std::fstream::out);
  exportlist << pts;
  exportlist.close();
  ROS_INFO_STREAM("Saved to -> " << loc);
}

/*
Create waypoint
*/

// Create Arrow
visualization_msgs::Marker waypointgen_marker::makeArrow(unsigned char mrk_id) {
  using vmarker = visualization_msgs::Marker;
  vmarker sp_marker;
  sp_marker.type = vmarker::ARROW;
  sp_marker.scale.x = 0.5;
  sp_marker.scale.y = 0.1;
  sp_marker.scale.z = 0.1;
  sp_marker.color.r = 0.2;
  sp_marker.color.g = 1.0;
  sp_marker.color.b = 0.2;
  sp_marker.color.a = 1.0;
  return sp_marker;
}

/*
====================
MARKER CONTROLS
====================
*/

// Set InteractiveMarker Header
void waypointgen_marker::setHeader(visualization_msgs::InteractiveMarker &imk_h,
                                   const int &index_h) {
  // Header
  wpg_utils.update_header(imk_h.header,
                          "map"); // Set frame relative to map frame
  // Name
  imk_h.name = "waypoint_" + std::to_string(index_h); // Add marker ID
  imk_h.description = "Waypoint Marker " + std::to_string(index_h);
}

// Set InteractiveMarker position
void waypointgen_marker::setPos(visualization_msgs::InteractiveMarker &imk_h,
                                const int &sx, const int &sy) {
  imk_h.pose.position.x = sx;
  imk_h.pose.position.y = sy;

  // Initialise default angle at 0
  tf::Quaternion qtn;
  qtn.setRPY(0, 0, 0);
  qtn = qtn.normalize();

  geometry_msgs::Quaternion quat_msg;
  quaternionTFToMsg(qtn, quat_msg);

  imk_h.pose.orientation = quat_msg;
}

// Add controls
void waypointgen_marker::addMotionControl(
    visualization_msgs::InteractiveMarker &i_mk,
    visualization_msgs::InteractiveMarkerControl &imc_am) {
  // Position & Orientation
  // Add the control to the interactive marker
  addMovementControl(imc_am, 1, 1, 0, 0, "move_x",
                     true); // Translate about X-axis
  i_mk.controls.push_back(imc_am);

  addMovementControl(imc_am, 1, 0, 0, 1, "move_y",
                     true); // Translate about Y-axis
  i_mk.controls.push_back(imc_am);

  addMovementControl(imc_am, 1, 0, 1, 0, "rotate_z",
                     false); // Rotate about Z-axis
  i_mk.controls.push_back(imc_am);
}

// Add motion controls (To modify waypoint position and orientation)
// mvAxis: 0->Rotate, 1:Translate;
void waypointgen_marker::addMovementControl(
    visualization_msgs::InteractiveMarkerControl &im_c, const bool &mw,
    const bool &mx, const bool &my, const bool &mz, const std::string &mName,
    const bool &mvAxis) {
  im_c.orientation.x = mx;
  im_c.orientation.y = my;
  im_c.orientation.z = mz;
  im_c.orientation.w = mw;

  im_c.name = mName;
  // Set either translational or rotational
  im_c.interaction_mode =
      mvAxis ? visualization_msgs::InteractiveMarkerControl::MOVE_AXIS
             : visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
}

// Add menu
void waypointgen_marker::addContextMnu(const bool &init) {
  if (!init) {
    // Create menu at start if it does not exist
    // Entries
    interactive_markers::MenuHandler::EntryHandle entry_showLoc;
    interactive_markers::MenuHandler::EntryHandle entry_add;
    interactive_markers::MenuHandler::EntryHandle mnu_entry_add;
    interactive_markers::MenuHandler::EntryHandle mnu_entry_waypoint;
    interactive_markers::MenuHandler::EntryHandle entry_generateYAML;

    // Add to menu
    entry_showLoc = menu_handler.insert(
        "Get Location",
        boost::bind(&waypointgen_marker::mnu_getLocation, this, _1));

    // Waypoint submenu
    mnu_entry_waypoint = menu_handler.insert("Waypoint");
    entry_add = menu_handler.insert(
        mnu_entry_waypoint, "Add",
        boost::bind(&waypointgen_marker::mnu_addNewWaypoint, this, _1));
    entry_add = menu_handler.insert(
        mnu_entry_waypoint, "Remove",
        boost::bind(&waypointgen_marker::mnu_removeWaypoint, this, _1));

    // Generate list of waypoints
    entry_generateYAML = menu_handler.insert(
        "Generate Waypoint List",
        boost::bind(&waypointgen_marker::mnu_createList, this, _1));

    this->menu_init_ =
        true; // Make it true so that the menu only initializes once
  }
}

void waypointgen_marker::addIntCtrl(
    visualization_msgs::InteractiveMarkerControl &imc,
    const unsigned char &mk_id) {
  imc.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  imc.always_visible = true;

  // Add control waypoint function
  // Create Arrow (Visual)
  imc.markers.push_back(makeArrow(mk_id));
}

void waypointgen_marker::addWaypointMarker(const unsigned int &mrk_id) {
  visualization_msgs::InteractiveMarker mrk;
  visualization_msgs::InteractiveMarkerControl imc_m;

  // Set Header
  setHeader(mrk, mrk_id);
  setPos(mrk, lastMarker[0] + 1,
         lastMarker[1] + 1); // Set next marker close to previous one

  // Update
  for (auto &i : lastMarker)
    i += 1;

  addMotionControl(mrk, imc_m);    // Add motion CONTROLS
  addContextMnu(this->menu_init_); // Add MENU
  addIntCtrl(imc_m, mrk_id);       // Add the control to the interactive marker
  mrk.controls.push_back(imc_m);

  // Insert into server
  server_->insert(mrk);
  server_->setCallback(mrk.name,
                       boost::bind(&waypointgen_marker::updateWaypointPos, this,
                                   _1)); // Attach callback when user
                                         // updates the marker position
  menu_handler.apply(
      *server_, mrk.name); // Apply to int_marker.name, which is setpoint_marker

  ++marker_count_;         // Increment marker count tracker
  server_->applyChanges(); // Update map_server
}

void waypointgen_marker::reset_server() { server_.reset(); }
