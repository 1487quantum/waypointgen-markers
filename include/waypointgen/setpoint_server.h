#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include "waypointgen/waypointgen_utils.h"
#include "waypointgen/Trigger.h"

constexpr bool DEBUG = 1;

using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using clock_type = std::chrono::steady_clock;
using d_type = std::chrono::duration<double, std::ratio<1>>;

class waypointgen_server {
public:
  // Constructor & Destructor
  waypointgen_server(ros::NodeHandle nh_);
  ~waypointgen_server(void) {}

  int init();
  inline ros::NodeHandle *get_nh() { return &this->nh_; };

  // Status of server
  /*
  Server would have 4 states:
  PLAY-> Run the server
  STOP-> Stop the server
  PAUSE -> Pause server
  IDLE->Wait for wpg_server_status topic
  */
  enum class ServerState { PLAY, STOP, PAUSE, IDLE };

  void setGlobalPathTopic(const ros::NodeHandle &n,
                          const std::string &param_name,
                          std::string &target_topic);

  // Callbacks
  bool start_p2p(waypointgen::Trigger::Request &req,
                 waypointgen::Trigger::Response &res);
  void gPlanCallback(const nav_msgs::Path &msg);
  void timerGoalCallback(const ros::TimerEvent &event);

  // Move base action callback
  void goalDoneCB(const actionlib::SimpleClientGoalState &state,
                  const move_base_msgs::MoveBaseResultConstPtr &msg);
  void goalFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  // Waypoints
  int loadWaypointList(const std::string &list_path);
  float getPathDist(const std::vector<geometry_msgs::PoseStamped> &pathVector);
  float getPathDist(const geometry_msgs::Point &pointStart,
                    const geometry_msgs::Point &pointEnd);

  inline void set_waypointcount(const int &num_wp) { this->wp_count = num_wp; };

  inline int get_s_state_delay() { return s_state_delay; }
  inline void set_s_state_delay(const int &sdelay) {
    this->s_state_delay = sdelay;
  }

  inline ServerState *get_state() { return &this->current_state; };
  inline void set_state(const ServerState &ss) { this->current_state = ss; };

  inline std::vector<geometry_msgs::Pose> *get_wpList() {
    return &this->wpList;
  }

  inline void set_currentWaypoint(const geometry_msgs::Pose &cwp) {
    this->currentWaypoint = cwp;
  }

  inline geometry_msgs::Pose *get_currentWaypoint() {
    return &this->currentWaypoint;
  }

  // Timer
  void reset_timer();
  double elapsed_time();

  // Play back
  bool p2p(const int &currentWP,
           const geometry_msgs::Pose &qpt); // Point 2 Point
  void begin_playback();

  waypointgen_utils wpg_utils;

private:
  ros::NodeHandle nh_;

  // Service
  ros::ServiceServer trigger_server;

  // Subscriber
  ros::Subscriber gPlanSub; // Subscribe to global path plan by TebLocalPlanner
  ros::Subscriber wpgStatSub; // Subscribe to /wpg_server_status topic
                              // of type wpg_stat message

  // Publisher
  ros::Publisher pointPubGoal;  // publish waypoint_goal
  ros::Publisher distToGoalPub; // publish distance to goal

  std::string global_path_topic;

  ServerState current_state;
  int s_state_delay; // Delay before starting the server play, in seconds

  std::vector<geometry_msgs::Pose> wpList; // Waypoint listed
  geometry_msgs::Pose currentWaypoint;     // current waypoint

  int wp_count; // Total number of waypoints
  float distToGoal;

  // Distance to goal
  ros::Timer timerGoal; // Refresh and get distance to goal
  float ecDist;         // Euclidean distance from goal
  float gpDist;         // Global Path distance from goal
  float ecDistMax;      // Euclidean distance from goal, Max
  float gpDistMax;      // Global Path distance from goal, Max

  std::chrono::time_point<clock_type> path_timer{
      clock_type::now()};                  // Timer for benchmarking
  std::vector<bool> wpt_benchmark_success; // Benchmark vals
};
