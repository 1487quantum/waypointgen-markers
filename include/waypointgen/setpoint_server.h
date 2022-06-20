#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <nav_msgs/Path.h>

#include "waypointgen/wpg_stat.h"

#define DEBUG 1
#define PI 3.1415926535897932385

class waypointgen_server {
public:
  ros::NodeHandle nh_;
  std::vector<geometry_msgs::Pose> wpList; // Waypoint listed

  geometry_msgs::Pose currentWaypoint; // current waypoint

  int numOfWaypoints;
  float distToGoal;

  // Status of server
  /*
  Server would have 5 states:
  PLAY-> Run the server
  STOP-> Stop the server
  PAUSE -> Pause server
  //Below 2 are not callable by topic
  IDLE->Wait for wpg_server_status topic
  DONE->Server complete waypoint list
  */

  std::string s_state = "WAIT";
  int s_state_delay = 0; // Delay before starting the server play, in seconds

  // Constructor & Destructor
  waypointgen_server(ros::NodeHandle nh_);
  ~waypointgen_server(void) {}

  void init();

  // Callbacks
  void gPlanCallback(const nav_msgs::Path &msg);
  void wpgStatCallback(const waypointgen::wpg_stat &msg);
  void timerGoalCallback(const ros::TimerEvent &event);

  // Move base action callback
  void goalDoneCB(const actionlib::SimpleClientGoalState &state,
                  const move_base_msgs::MoveBaseResultConstPtr &msg);
  void goalFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  // Waypoints
  int loadWaypointList(std::string list_path);
  float getPathDist(std::vector<geometry_msgs::PoseStamped> pathVector);
  geometry_msgs::PoseStamped
  convertToPoseStamped(std::string poseFrameID, geometry_msgs::Pose poseTarget);

  // Point 2 Point
  void p2p(int currentWP, geometry_msgs::Pose qpt);

private:
  //  typedef boost::shared_ptr<const geometry_msgs::PoseStamped> PoseConstPtr;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      MoveBaseClient;

  // Subscriber
  ros::Subscriber gPlanSub; // Subscribe to global path plan by TebLocalPlanner
  ros::Subscriber wpgStatSub; // Subscribe to /wpg_server_status topic of type
                              // wpg_stat message

  // Publisher
  ros::Publisher pointPubGoal;  // publish waypoint_goal
  ros::Publisher distToGoalPub; // publish distance to goal

  // Total number of waypoints
  int wp_count;

  // Distance to goal
  ros::Timer timerGoal; // Refresh and get distance to goal
  float ecDist;         // Euclidean distance from goal
  float gpDist;         // Global Path distance from goal
};
