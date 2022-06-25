#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <string>
#include <tf/tf.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <thread>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "waypointgen/Trigger.h"
#include "waypointgen/wpg_stat.h"

#define DEBUG 1
#define PI 3.1415926535897932385

class waypointgen_server {
public:
  // Constructor & Destructor
  waypointgen_server(ros::NodeHandle nh_);
  ~waypointgen_server(void) {}

  int init();

  // Status of server
  /*
  Server would have 4 states:
  PLAY-> Run the server
  STOP-> Stop the server
  PAUSE -> Pause server
  IDLE->Wait for wpg_server_status topic
  */
  enum class ServerState { PLAY, STOP, PAUSE, IDLE };

  // Callbacks
  bool start_p2p(waypointgen::Trigger::Request &req,
                 waypointgen::Trigger::Response &res);
  void gPlanCallback(const nav_msgs::Path &msg);
  // void wpgStatCallback(const waypointgen::wpg_stat &msg);
  void timerGoalCallback(const ros::TimerEvent &event);

  // Move base action callback
  void goalDoneCB(const actionlib::SimpleClientGoalState &state,
                  const move_base_msgs::MoveBaseResultConstPtr &msg);
  void goalFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  // Waypoints
  int loadWaypointList(const std::string &list_path);
  float getPathDist(const std::vector<geometry_msgs::PoseStamped> &pathVector);
  geometry_msgs::PoseStamped
  convertToPoseStamped(const std::string &poseFrameID,
                       const geometry_msgs::Pose &poseTarget);

  inline void set_waypointcount(const int &num_wp) {
    this->numOfWaypoints = num_wp;
  };

  inline int get_s_state_delay() { return s_state_delay; }
  inline void set_s_state_delay(const int &sdelay) {
    this->s_state_delay = sdelay;
  }

  inline ServerState *get_state() { return &this->current_state; };
  inline void set_state(const ServerState &ss) { this->current_state = ss; };

  inline ros::NodeHandle *get_nh() { return &this->nh_; };

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

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      MoveBaseClient;

  // Service
  ros::ServiceServer trigger_server;

  // Subscriber
  ros::Subscriber gPlanSub; // Subscribe to global path plan by TebLocalPlanner
  ros::Subscriber wpgStatSub; // Subscribe to /wpg_server_status topic
                              // of type wpg_stat message

  // Publisher
  ros::Publisher pointPubGoal;  // publish waypoint_goal
  ros::Publisher distToGoalPub; // publish distance to goal

  ros::NodeHandle nh_;

  std::string global_path_topic;

  int numOfWaypoints;
  float distToGoal;

  // Total number of waypoints
  int wp_count;

  // Distance to goal
  ros::Timer timerGoal; // Refresh and get distance to goal
  float ecDist;         // Euclidean distance from goal
  float gpDist;         // Global Path distance from goal
  float ecDistMax;      // Euclidean distance from goal, Max
  float gpDistMax;      // Global Path distance from goal, Max

  ServerState current_state;
  int s_state_delay; // Delay before starting the server play, in
                     // seconds

  std::vector<geometry_msgs::Pose> wpList; // Waypoint listed
  geometry_msgs::Pose currentWaypoint;     // current waypoint

  // Timer for benchmarking
  using clock_type = std::chrono::steady_clock;
  using d_type = std::chrono::duration<double, std::ratio<1>>;

  std::chrono::time_point<clock_type> path_timer{clock_type::now()};

  //Benchmark vals
  std::vector<bool> wpt_benchmark_success;
};
