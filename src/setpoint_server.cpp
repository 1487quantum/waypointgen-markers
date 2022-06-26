#include "waypointgen/setpoint_server.h"

waypointgen_server::waypointgen_server(ros::NodeHandle nh)
    : wp_count(0), distToGoal(0.0), ecDist(0.0), gpDist(0.0), s_state_delay(0),
      current_state(ServerState::IDLE), gpDistMax(-1.0),
      global_path_topic("/move_base/TebLocalPlannerROS/global_plan") {
  this->nh_ = nh;
}

// Start sub and pub
int waypointgen_server::init() {
  ROS_INFO("Init waypoint_server");

  // Service
  trigger_server = nh_.advertiseService("/trigger_play",
                                        &waypointgen_server::start_p2p, this);

  // Subscribers

  // Check if global_plan_topic param specified
  setGlobalPathTopic(nh_, "global_plan_topic", global_path_topic);

  gPlanSub = nh_.subscribe(global_path_topic, 10,
                           &waypointgen_server::gPlanCallback, this);

  // Publisher
  pointPubGoal = nh_.advertise<geometry_msgs::PoseStamped>(
      "/current_waypoint_goal", 10,
      true); // Turn on latch so that last published msg would be saved
  distToGoalPub = nh_.advertise<std_msgs::Float32>(
      "/dist_to_goal", 10,
      true); // Turn on latch so that last published msg would be saved
  timerGoal = nh_.createTimer(ros::Duration(0.2),
                              &waypointgen_server::timerGoalCallback, this);

  // Get waypoint list to use in the wp_list directory
  // Check if param specified
  auto param_name{"waypoint_listpath"};
  if (!get_nh()->hasParam(param_name)) {
    ROS_ERROR("No param named '%s' found, please set the path "
              "before running this node!\nExiting...",
              param_name);
    return -1;
  }

  std::string l_path;
  get_nh()->getParam(param_name, l_path);

  // Check if valid path
  if (!std::filesystem::exists(l_path)) {
    ROS_ERROR("File at '%s' not found!\nExiting...", l_path.c_str());
    return 0;
  }
  ROS_INFO("Setting waypoint list path -> %s", l_path.c_str());

  set_waypointcount(loadWaypointList(l_path)); // Load the waypoint list
  return 1;
}

void waypointgen_server::setGlobalPathTopic(const ros::NodeHandle &n,
                                            const std::string &param_name,
                                            std::string &target_topic) {
  if (!n.hasParam(param_name)) {
    ROS_ERROR("No param named '%s' found, using "
              "/move_base/TebLocalPlannerROS/global_plan as global path "
              "planner instead...",
              param_name.c_str());
  } else {
    n.getParam(param_name, target_topic);
    ROS_INFO("Setting global path topic -> %s", target_topic.c_str());
  }
}

// Callbacks
bool waypointgen_server::start_p2p(waypointgen::Trigger::Request &req,
                                   waypointgen::Trigger::Response &res) {
  if (*get_state() == waypointgen_server::ServerState::IDLE) {
    // Start playback
    set_state(waypointgen_server::ServerState::PLAY);
    set_s_state_delay(req.play_delay);
    res.play_triggered = true;
    ROS_INFO("Setting [%d] delay before playback", req.play_delay);
    begin_playback();
  }
  return true;
}

// move_base_action CB
void waypointgen_server::goalDoneCB(
    const actionlib::SimpleClientGoalState &state,
    const move_base_msgs::MoveBaseResultConstPtr &msg) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

// Get current location
void waypointgen_server::goalFeedbackCB(
    const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {
  // Calculate displacement from goal
  ecDist = getPathDist(get_currentWaypoint()->position,
                       feedback->base_position.pose.position);
  if (ecDist > ecDistMax)
    ecDistMax = ecDist;
  // ROS_INFO("Euclidean distance: %f", ecDist);

  // ROS_INFO("[X]:%f [Y]:%f [W]:
  // %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w);
}

// Get global path length
void waypointgen_server::gPlanCallback(const nav_msgs::Path &msg) {
  gpDist = getPathDist(msg.poses); // Calculate path to target waypoint
  if (gpDist > gpDistMax)
    gpDistMax = gpDist;

  // ROS_INFO("Global Path len: %f", gpDist);
}

// Publish average of Global path length & Euclidean distance from target if
// both are non zero
void waypointgen_server::timerGoalCallback(const ros::TimerEvent &event) {
  std_msgs::Float32 tgoal;
  if (ecDist == 0)
    tgoal.data = gpDist;
  else if (gpDist == 0)
    tgoal.data = ecDist;
  else
    tgoal.data = (ecDist + gpDist) / 2;

  distToGoalPub.publish(tgoal);
  // ROS_INFO("Avg distance: %f",tgoal.data);
}

// Calculate path length (point start: 1, point end: 2)
float waypointgen_server::getPathDist(const geometry_msgs::Point &pointStart,
                                      const geometry_msgs::Point &pointEnd) {
  auto path_mag{sqrt(pow((pointEnd.x - pointStart.x), 2) +
                     pow((pointEnd.y - pointStart.y), 2))};
  // ROS_INFO("Path len: %f",distToGoal);
  return path_mag;
}

// Calculate path length
float waypointgen_server::getPathDist(
    const std::vector<geometry_msgs::PoseStamped> &pathVector) {
  float dst = 0;
  // Get num of path points
  for (int i = 0; i < pathVector.size() - 1; i++) {
    auto x1 = pathVector[i].pose.position.x;
    auto x2 = pathVector[i + 1].pose.position.x;
    auto y1 = pathVector[i].pose.position.y;
    auto y2 = pathVector[i + 1].pose.position.y;
    dst += sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    // ROS_INFO("%f %f %f %f",x1,x2,y1,y2);
    // ROS_INFO("Path len: %f",distToGoal);
  }
  return dst;
}

/*
YAML Parsing
*/

// Load and parse the waypoint list, return number of waypoints
int waypointgen_server::loadWaypointList(const std::string &list_path) {
  // Open yaml file, parse it as string
  std::ifstream ifs(list_path);
  std::string yml_content((std::istreambuf_iterator<char>(ifs)),
                          (std::istreambuf_iterator<char>()));

  YAML::Node node = YAML::Load(yml_content); // Load YAML

  // ROS_INFO_STREAM(node.Type()<<","<<node.size()<<","<<node.IsSequence());

  wp_count = 0; // Reset counter
  if (node["count"]) {
    wp_count = node["count"].as<int>();
    ROS_INFO("Waypoint count-> %i", wp_count);
  }

  // ROS_INFO_STREAM("== Saving pose to list ==");

  // Temporary pose array list to be published
  geometry_msgs::PoseArray tmpPoseArray;

  // Iterate through all the points
  for (int j = 0; j < wp_count; j++) {
    std::string wpID = "WP" + std::to_string(j); // Waypoint ID
    geometry_msgs::Pose tempPose;                // Temp Pose to store in vector

    if (node[wpID]) {
      // Get quaternion
      for (std::size_t i = 0; i < node[wpID].size(); i++) {
        //   ROS_INFO_STREAM(i<<": "<<node[wpID][i].as<float>() );
        switch (i) {
        case 0:
          tempPose.position.x = node[wpID][i].as<float>();
          break;
        case 1:
          tempPose.position.y = node[wpID][i].as<float>();
          break;
        case 2:
          tempPose.orientation.x = node[wpID][i].as<float>();
          break;
        case 3:
          tempPose.orientation.y = node[wpID][i].as<float>();
          break;
        case 4:
          tempPose.orientation.z = node[wpID][i].as<float>();
          break;
        case 5:
          tempPose.orientation.w = node[wpID][i].as<float>();
          break;
        }
      }
      // Insert pose into list
      wpList.push_back(tempPose);
      tmpPoseArray.poses.push_back(tempPose); // Add to publisher
      ROS_INFO("WP%i: %f %f %f %f", j, tempPose.position.x, tempPose.position.y,
               tempPose.orientation.z, tempPose.orientation.w);
    }
  }
  ROS_INFO("Loaded waypoint(s)! Waiting for trigger service...");

  // pointPub.publish(tmpPoseArray);

  // ROS_INFO_STREAM("== Check list val ==");
  // // Check list values
  // for (const auto &i : wpList)
  //   ROS_INFO_STREAM('\n' << i);
  return wp_count;
}

// Timer
void waypointgen_server::reset_timer() { path_timer = clock_type::now(); }

double waypointgen_server::elapsed_time() {
  return std::chrono::duration_cast<d_type>(clock_type::now() - path_timer)
      .count();
}

// Point to point navigation
bool waypointgen_server::p2p(const int &currentWP,
                             const geometry_msgs::Pose &qpt) {
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // ROS_INFO("Moving out soon...");

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Reset distance;
  ecDistMax = 0;
  gpDistMax = 0;

  // Create goal waypoint
  move_base_msgs::MoveBaseGoal goal_movebase;
  wpg_utils.add_timestamp(goal_movebase.target_pose, "map", qpt);

  // Goal Pose, add timestamp
  geometry_msgs::PoseStamped goal_pose;
  wpg_utils.add_timestamp(goal_pose, "map", qpt);

  // Convert Quaternion to Euler Angle
  tf::Quaternion q(qpt.orientation.x, qpt.orientation.y, qpt.orientation.z,
                   qpt.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw *= 180 / wpg_utils.PI; // Convert to degrees

  // Current waypoint, total waypoint, x pos, y pos, angular (yaw)
  ROS_INFO(">> Sending next goal [%i/%i]: (%.2f,%.2f, %.2f)", currentWP + 1,
           wp_count, qpt.position.x, qpt.position.y, yaw);

  ac.sendGoal(goal_movebase,
              boost::bind(&waypointgen_server::goalDoneCB, this, _1, _2),
              MoveBaseClient::SimpleActiveCallback(),
              boost::bind(&waypointgen_server::goalFeedbackCB, this, _1));

  reset_timer(); // Restart timer

  pointPubGoal.publish(goal_pose); // Publish current waypoint goal
  ac.waitForResult();

  // Benchmark
  auto tt{elapsed_time()};
  auto egp{gpDistMax / ecDistMax};
  auto isSuccessful{false};

  // todo: Add final dist and ang from goal post when action server returns val,
  std::string results_msg{"<< "};
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    results_msg += "Reached!";
    isSuccessful = true;
  } else if (ac.getState() == actionlib::SimpleClientGoalState::LOST) {
    results_msg += "Lost, skipping to next goal...";
  } else {
    results_msg += "The base failed to move forward for some reason";
  }

  ROS_INFO("%s", results_msg.c_str());
  ROS_INFO("== Results ==");
  ROS_INFO("Reached goal?\t\t\t\t%s ", isSuccessful ? "Yes" : "No");
  ROS_INFO("Time taken:\t\t\t\t%.4f s", tt);
  ROS_INFO("Global Path Distance:\t\t\t%.4f m", gpDistMax);
  ROS_INFO("Euclidean Path Distance:\t\t\t%.4f m", ecDistMax); // To way point
  ROS_INFO("Global Path/Euclidean Ratio:\t\t%.4f ", egp);      // Normalised
  ROS_INFO("Average speed:\t\t\t\t%.4f ms-1",
           0.5 * (gpDistMax + ecDistMax) / tt);
  ROS_INFO("==========================");

  return isSuccessful;
}

void waypointgen_server::begin_playback() {
  // Wait for s_state_delay seconds before starting
  for (auto k = get_s_state_delay(); k > 0; --k) {
    ROS_INFO("Commencing navigation in %3is", k);
    ros::Duration(1).sleep(); // Wait 1 sec
  }

  // Start Navigation
  for (int i = 0; i < get_wpList()->size(); ++i) {
    set_currentWaypoint(get_wpList()->at(i));
    wpt_benchmark_success.push_back(p2p(i, *get_currentWaypoint()));
  }

  auto q = [](auto bool_list) {
    auto j{0};
    for (const auto &i : bool_list)
      j += i;
    return j;
  };

  ROS_INFO("Success ratio: %i/%i", q(wpt_benchmark_success),
           static_cast<int>(wpt_benchmark_success.size())); // Print results

  set_state(waypointgen_server::ServerState::IDLE);
  ROS_INFO("Completed playback!");
}