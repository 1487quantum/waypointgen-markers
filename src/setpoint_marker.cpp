#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <yaml-cpp/yaml.h>

#define DEBUG 1

typedef std::map<std::string, geometry_msgs::PoseWithCovariance> p_map;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

// Marker
unsigned char marker_id = 0;    // Take a max value of 255 waypoints
unsigned char marker_count = 0; // Keep track of number of waypoints created

p_map wpl;

float lastMarker[2] = {0.0, 0.0}; // Get pos of last marker

// Subscriber
ros::Subscriber sub_setpoint_list;

// Menu
interactive_markers::MenuHandler menu_handler;
auto menuInit{false};

void updateWypt(p_map &tmp, const std::string &wp_name,
                const geometry_msgs::PoseWithCovariance &pt);
void addWaypointMarker(const unsigned int &mrk_id);
void printDebugPose(const std::string &dmsg, const std::string &wp_name,
                    const geometry_msgs::PoseWithCovariance &pw);
// Sub/Pub
void updateWaypointPos(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

void set_pose_position(geometry_msgs::Point &from_point,
                       const geometry_msgs::Point &to_point);

void setpointListCallback(
    const visualization_msgs::InteractiveMarkerInitConstPtr msg);
// Menu
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
// Export
geometry_msgs::PoseWithCovariance addPoseCov(const geometry_msgs::Point &pt,
                                             const tf::Quaternion &q_rotate);

std::string getCurrentTime();
void updateWPList(p_map &tmp);

// Get current time
std::string getCurrentTime() {
  std::time_t now = time(nullptr);
  std::tm ltm = *std::localtime(&now); // tm -> timestruct
  std::stringstream buf;
  buf << std::put_time(&ltm, "%d%m%Y_%H%M%S");
  return buf.str(); // ddMMYYYY_hhmmss
}

// Param: position,orientation
geometry_msgs::PoseWithCovariance addPoseCov(const geometry_msgs::Point &pt,
                                             const tf::Quaternion &q_rotate) {
  geometry_msgs::Quaternion quat_msg;
  quaternionTFToMsg(q_rotate, quat_msg);

  geometry_msgs::PoseWithCovariance cov_pose;
  cov_pose.pose.position = pt;
  cov_pose.pose.orientation = quat_msg;
  return cov_pose;
}

void updateWaypointPos(
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
    set_pose_position(p_point, feedback->pose.position);

    tf::Quaternion p_quat(
        feedback->pose.orientation.x, feedback->pose.orientation.y,
        feedback->pose.orientation.z,
        feedback->pose.orientation.w); // change geometry to tf quaternion

    auto pwc = addPoseCov(
        p_point,
        p_quat); // Update pose in map, geometry_msgs::PoseWithCovariance

    updateWypt(wpl, feedback->marker_name, pwc);
    server->applyChanges();
  }
}

void set_pose_position(geometry_msgs::Point &from_point,
                       const geometry_msgs::Point &to_point) {
  from_point.x = to_point.x;
  from_point.y = to_point.y;
  from_point.z = to_point.z;
}

void printDebugPose(const std::string &dmsg, const std::string &wp_name,
                    const geometry_msgs::PoseWithCovariance &pw) {
  tf::Quaternion q(pw.pose.orientation.x, pw.pose.orientation.y,
                   pw.pose.orientation.z, pw.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[%s] %s(x,y,ang) -> %f, %f, %f", dmsg.c_str(), wp_name.c_str(),
           pw.pose.position.x, pw.pose.position.y, yaw * 180 / 3.1415927);
}

// Subscriber callback
void setpointListCallback(
    const visualization_msgs::InteractiveMarkerInitConstPtr msg) {
  geometry_msgs::PoseWithCovariance pt;
  updateWPList(wpl);
  for (auto mk : msg->markers) {
    geometry_msgs::Point p_point;
    p_point.x = mk.pose.position.x;
    p_point.y = mk.pose.position.y;
    p_point.z = 0.0;

    tf::Quaternion qtmp(
        mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z,
        mk.pose.orientation
            .w); // Create Quaternion for rotation, tf::Quaternion
    pt = addPoseCov(p_point, qtmp); // Create pose
    updateWypt(wpl, mk.name, pt);   // Add to map
    if (DEBUG)
      printDebugPose("LIST SUB CALLBACK", mk.name, pt);
  }
}

void updateWPList(p_map &tmp) {
  if (tmp.empty())
    ROS_INFO("Map is empty! Updating...");

  for (const auto &it : tmp)
    std::cout << it.first << " :: " << it.second << std::endl;
}

// Add waypoint to map
void updateWypt(p_map &tmp, const std::string &wp_name,
                const geometry_msgs::PoseWithCovariance &pt) {
  if (tmp.insert(std::make_pair(wp_name, pt)).second == false) {
    ROS_WARN("Overwriting %s", wp_name.c_str());
    tmp[wp_name] = pt;
  } else {
    tmp.insert(std::make_pair(wp_name, pt));
    if (DEBUG)
      printDebugPose("INSERT WAYPONT", wp_name, pt);
  }
}

// Show selected waypoint Location
void mnu_getLocation(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  geometry_msgs::PoseWithCovariance pwc_gl;
  set_pose_position(pwc_gl.pose.position, feedback->pose.position);

  pwc_gl.pose.orientation.x = feedback->pose.orientation.x;
  pwc_gl.pose.orientation.y = feedback->pose.orientation.y;
  pwc_gl.pose.orientation.z = feedback->pose.orientation.z;
  pwc_gl.pose.orientation.w = feedback->pose.orientation.w;
  // To display on console
  printDebugPose("CURRENT LOC", feedback->marker_name, pwc_gl);
}

// Add waypoint
void mnu_addNewWaypoint(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (marker_id < 256) {
    addWaypointMarker(++marker_id);
    ROS_INFO("Waypoint Added!");
  } else {
    ROS_INFO("Setpoint limit reached!");
  }
}

// Remove waypoint_
void mnu_removeWaypoint(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std_msgs::String dispMsg;               // To display on console
  auto markerName{feedback->marker_name}; // Marker Name, std::string
  auto delimiter{"_"}; // Delimit to the end to get id, std::string

  auto markerID{
      stoi(markerName.substr(markerName.find(delimiter) + 1,
                             -1))}; // Get last part of string, since name is
                                    // waypoint_[ID]; Convert str->int

  if (marker_count > 1) {
    wpl.erase(markerName); // Remove from map

    // Remove from server
    server->erase(feedback->marker_name);
    server->applyChanges();

    // Decrement counter
    marker_count -= 1;

    dispMsg.data = feedback->marker_name;
    dispMsg.data += " removed!";
  } else {
    dispMsg.data = "Cannot remove all points!";
  }

  ROS_INFO("%s", dispMsg.data.c_str());
}

void mnu_createList(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO("%s %i", "[Generating list] Total Waypoints ->", marker_count);

  // Save as YAML file
  YAML::Node pts;
  pts["count"] =
      static_cast<int>(marker_count); // Write total number of waypoints

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

  auto lstName = getCurrentTime() + "_wplist.yaml"; // File Name
  loc += lstName;                                   // Append to main path
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
visualization_msgs::Marker makeArrow(unsigned char mrk_id) {
  typedef visualization_msgs::Marker vmarker;
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
void setHeader(visualization_msgs::InteractiveMarker &imk_h,
               const int &index_h) {
  // Header
  imk_h.header.frame_id = "map"; // Set frame relative to map frame
  imk_h.header.stamp = ros::Time::now();
  // Name
  imk_h.name = "waypoint_";
  imk_h.name += std::to_string(index_h); // Add marker ID
  imk_h.description = "Waypoint Marker ";
  imk_h.description += std::to_string(index_h);
}

// Set InteractiveMarker position
void setPos(visualization_msgs::InteractiveMarker &imk_h, const int &sx,
            const int &sy) {
  imk_h.pose.position.x = sx;
  imk_h.pose.position.y = sy;
}

// Add controls
void addMotionControl(visualization_msgs::InteractiveMarker &i_mk,
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
void addMovementControl(visualization_msgs::InteractiveMarkerControl &im_c,
                        const bool &mw, const bool &mx, const bool &my,
                        const bool &mz, const std::string &mName,
                        const bool &mvAxis) {
  im_c.orientation.w = mw;
  im_c.orientation.x = mx;
  im_c.orientation.y = my;
  im_c.orientation.z = mz;
  im_c.name = mName;
  // Set either translational or rotational
  im_c.interaction_mode =
      mvAxis ? visualization_msgs::InteractiveMarkerControl::MOVE_AXIS
             : visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
}

// Add menu
void addContextMnu(const bool &init) {
  if (!init) {
    // Create menu at start if it does not exist
    // Entries
    interactive_markers::MenuHandler::EntryHandle entry_showLoc;
    interactive_markers::MenuHandler::EntryHandle entry_add;
    interactive_markers::MenuHandler::EntryHandle mnu_entry_add;
    interactive_markers::MenuHandler::EntryHandle mnu_entry_waypoint;
    interactive_markers::MenuHandler::EntryHandle entry_generateYAML;

    // Add to menu
    entry_showLoc = menu_handler.insert("Get Location", &mnu_getLocation);

    // Waypoint submenu
    mnu_entry_waypoint = menu_handler.insert("Waypoint");
    entry_add =
        menu_handler.insert(mnu_entry_waypoint, "Add", &mnu_addNewWaypoint);
    entry_add =
        menu_handler.insert(mnu_entry_waypoint, "Remove", &mnu_removeWaypoint);

    // Generate list of waypoints
    entry_generateYAML =
        menu_handler.insert("Generate Waypoint List", &mnu_createList);

    menuInit = true; // Make it true so that the menu only initializes once
  }
}

void addIntCtrl(visualization_msgs::InteractiveMarkerControl &imc,
                const unsigned char &mk_id) {
  imc.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  imc.always_visible = true;

  // Add control waypoint function
  // Create Arrow (Visual)
  imc.markers.push_back(makeArrow(mk_id));
}

void addWaypointMarker(const unsigned int &mrk_id) {
  visualization_msgs::InteractiveMarker mrk;
  visualization_msgs::InteractiveMarkerControl imc_m;

  // Set Header
  setHeader(mrk, mrk_id);
  // mrk = setPos(mrk,mrk_id%5 - 2,mrk_id%6 - 2);  //Vary spawn location

  setPos(mrk, lastMarker[0] + 1,
         lastMarker[1] + 1); // Set next marker close to previous one

  // Update
  for (auto &i : lastMarker)
    i += 1;

  addMotionControl(mrk, imc_m); // Add motion CONTROLS
  addContextMnu(menuInit);      // Add MENU
  addIntCtrl(imc_m, mrk_id);    // Add the control to the interactive marker
  mrk.controls.push_back(imc_m);

  // Insert into server
  server->insert(mrk);
  server->setCallback(mrk.name,
                      &updateWaypointPos); // Attach callback when user updates
                                           // the marker position

  menu_handler.apply(
      *server, mrk.name); // Apply to int_marker.name, which is setpoint_marker

  // Increment marker count tracker
  marker_count += 1;

  // Update map_server
  server->applyChanges();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "setpoint_marker");
  server.reset(new interactive_markers::InteractiveMarkerServer(
      "setpoint_marker", "", false));

  // Add waypoint at start
  addWaypointMarker(marker_id);

  // Subscriber
  ros::NodeHandle n;
  // Subscribe to the waypoints published
  sub_setpoint_list =
      n.subscribe("setpoint_marker/update_full", 1, setpointListCallback);

  // start the ROS main loop
  ros::spin();
  server.reset();
}
