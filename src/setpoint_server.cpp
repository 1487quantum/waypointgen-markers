#include "waypointgen/setpoint_server.h"

waypointgen_server::waypointgen_server(ros::NodeHandle nh)
    : wp_count(0), numOfWaypoints(0), distToGoal(0.0), ecDist(0.0), gpDist(0.0),
      s_state_delay(0), current_state(ServerState::IDLE),
      global_path_topic("/move_base/TebLocalPlannerROS/global_plan") {
  this->nh_ = nh;
}

// Start sub and pub
int waypointgen_server::init() {
  ROS_INFO("Init waypoint_server");

  // Service
  trigger_server = nh_.advertiseService("/trigger_play",
                                        &waypointgen_server::start_p2p, this);

  // Subscriber

  // Check if global_plan_topic param specified
  auto gp_name{"global_plan_topic"};
  if (!get_nh()->hasParam(gp_name)) {
    ROS_ERROR("No param named '%s' found, using "
              "/move_base/TebLocalPlannerROS/global_plan as global path "
              "planner instead...",
              gp_name);
  } else {
    get_nh()->getParam(gp_name, global_path_topic);
    ROS_INFO("Setting global path topic -> %s", global_path_topic.c_str());
  }

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
  ROS_INFO("Setting waypoint list path -> %s", l_path.c_str());

  // Check if valid path
  if (!std::filesystem::exists(l_path)) {
    ROS_ERROR("File at '%s' not found!\nExiting...", l_path.c_str());
    return 0;
  }

  set_waypointcount(loadWaypointList(l_path)); // Load the waypoint list

  return 1;
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

  std::vector<geometry_msgs::PoseStamped> dFromGoal;

  // Insert waypoint goalPose
  geometry_msgs::PoseStamped poseTemp = convertToPoseStamped(
      feedback->base_position.header.frame_id, currentWaypoint);
  dFromGoal.push_back(poseTemp);

  // Insert current position
  poseTemp = convertToPoseStamped(feedback->base_position.header.frame_id,
                                  feedback->base_position.pose);
  dFromGoal.push_back(poseTemp);

  // Calculate displacement from goal
  ecDist = getPathDist(dFromGoal);
#ifdef DEBUG
// ROS_INFO("Euclidean distance: %f",ecDist);
#endif

  // ROS_INFO("[X]:%f [Y]:%f [W]:
  // %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w);
}

// Get global path length
void waypointgen_server::gPlanCallback(const nav_msgs::Path &msg) {
  std::vector<geometry_msgs::PoseStamped> path_poses = msg.poses;
  gpDist = getPathDist(path_poses); // Calculate path to target waypoint
#ifdef DEBUG
// ROS_INFO("Global Path len: %f", gpDist);
#endif
}

// Publish average of Global path length & Euclidean distance from target if
// both are non zero
void waypointgen_server::timerGoalCallback(const ros::TimerEvent &event) {
  std_msgs::Float32 tgoal;
  if (ecDist == 0) {
    tgoal.data = gpDist;
  } else if (gpDist == 0) {
    tgoal.data = ecDist;
  } else {
    tgoal.data = (ecDist + gpDist) / 2;
  }
  distToGoalPub.publish(tgoal);
  // ROS_INFO("Avg distance: %f",tgoal.data);
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

  // Load YAML
  YAML::Node node = YAML::Load(yml_content);

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

// Adds timestamps to poses, converts pose to poseStamped: poseTarget ->
// poseStamped
geometry_msgs::PoseStamped waypointgen_server::convertToPoseStamped(
    const std::string &poseFrameID, const geometry_msgs::Pose &poseTarget) {
  geometry_msgs::PoseStamped poseStamped;

  // Create waypoint header
  poseStamped.header.frame_id = poseFrameID; // reference to map
  poseStamped.header.stamp = ros::Time::now();

  poseStamped.pose.position = poseTarget.position; // Set position

  tf::Quaternion q;
  q.setRPY(0, 0, PI); // rotate by pi (offset)

  poseStamped.pose.orientation =
      poseTarget.orientation * q; // Set rotation (Quaternion)

  return poseStamped;
}

// Point to point navigation
void waypointgen_server::p2p(const int &currentWP,
                             const geometry_msgs::Pose &qpt) {
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ROS_INFO("Moving out soon...");

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Create goal waypoint
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map"; // reference to map
  goal.target_pose.header.stamp = ros::Time::now();
  // set pose
  goal.target_pose.pose.position = qpt.position;
  goal.target_pose.pose.orientation = qpt.orientation;

  // Goal Pose
  geometry_msgs::PoseStamped goalPose;
  goalPose = convertToPoseStamped("map", qpt); // Adds timestamp to goal pose

  // Convert Quaternion to Euler Angle
  tf::Quaternion q(qpt.orientation.x, qpt.orientation.y, qpt.orientation.z,
                   qpt.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw *= 180 / PI; // Convert to degrees

  // Current waypoint, total waypoint, x pos, y pos, angular (yaw)
  ROS_INFO("Sending next goal [%i/%i]: (%.2f,%.2f, %.2f)", currentWP + 1,
           wp_count, qpt.position.x, qpt.position.y, yaw);

  // ac.sendGoal(goal,boost::bind(&waypointgen::goalDoneCB,
  // this,_1,_2),MoveBaseClient::SimpleActiveCallback(),
  // MoveBaseClient::SimpleFeedbackCallback());
  ac.sendGoal(goal, boost::bind(&waypointgen_server::goalDoneCB, this, _1, _2),
              MoveBaseClient::SimpleActiveCallback(),
              boost::bind(&waypointgen_server::goalFeedbackCB, this, _1));

  // Publish current waypoint goal
  pointPubGoal.publish(goalPose);

  ac.waitForResult();

  // wayptCounter++;

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Done!");
  } else if (ac.getState() == actionlib::SimpleClientGoalState::LOST) {
    ROS_INFO("Skipping to next goal");
    exit(0);
  } else {
    ROS_INFO("The base failed to move forward for some reason");
  }
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
    p2p(i, *get_currentWaypoint());
  }

  ROS_INFO("Completed playback!");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "setpoint_server");
  ros::NodeHandle n("~");
  waypointgen_server wpg(n);
  if (wpg.init() < 0)
    return 0;

  // Start Multithreading Process(Async thread):
  // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
  // std::thread::hardware_concurrency -> Returns the number of concurrent
  // threads supported by the implementation
  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  ros::Rate r(10); // Run at 10Hz

  spinner.start();

  ros::waitForShutdown();
  return 0;
}
