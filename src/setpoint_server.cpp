#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseResult.h>

#include "waypointgen/wpg_stat.h"

#define DEBUG 1
#define PI 3.1415926535897932385

class waypointgen_server{

private:
  //  typedef boost::shared_ptr<const geometry_msgs::PoseStamped> PoseConstPtr;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  //Subscriber
  ros::Subscriber gPlanSub;         // Subscribe to global path plan by TebLocalPlanner
  ros::Subscriber wpgStatSub;       // Subscribe to /wpg_server_status topic of type wpg_stat message

  //Publisher
  ros::Publisher pointPubGoal;          // publish waypoint_goal
  ros::Publisher distToGoalPub;     // publish distance to goal

  //Total number of waypoints
  int wp_count=0;

  //Distance to goal
  ros::Timer timerGoal;             //Refresh and get distance to goal
  float ecDist = 0;                 //Euclidean distance from goal
  float gpDist = 0;                 //Global Path distance from goal


public:
  ros::NodeHandle nh_;
  std::vector<geometry_msgs::Pose> wpList;      //Waypoint listed

  geometry_msgs::Pose currentWaypoint;          //current waypoint

  int numOfWaypoints = 0;
  float distToGoal =0;

  //Constructor
  waypointgen_server(std::string name, ros::NodeHandle nh_) {
    this->nh_=nh_;
  }

  //Destructor
  ~waypointgen_server(void){
  }

  void init();

  //Callbacks
  void gPlanCallback(const nav_msgs::Path &msg);
  void wpgStatCallback(const waypointgen::wpg_stat &msg);
  void timerGoalCallback(const ros::TimerEvent& event);

  //Move base action callback
  void goalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& msg);
  void goalFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  //Waypoints
  int loadWaypointList(  std::string list_path);
  float getPathDist(std::vector<geometry_msgs::PoseStamped> pathVector);
  geometry_msgs::PoseStamped convertToPoseStamped(std::string poseFrameID, geometry_msgs::Pose poseTarget);

  //Point 2 Point
  void p2p(int currentWP,geometry_msgs::Pose qpt);
};

//Start sub and pub
void waypointgen_server::init(){
  ROS_INFO("Init pub & sub");

//Subscriber
  gPlanSub =  nh_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, &waypointgen_server::gPlanCallback, this);
wpgStatSub = nh_.subscribe("/wpg_server_status", 10, &waypointgen_server::wpgStatCallback, this);
  //Publisher
  pointPubGoal= nh_.advertise<geometry_msgs::PoseStamped>("/current_waypoint_goal", 10, true); //Turn on latch so that last published msg would be saved
  distToGoalPub= nh_.advertise<std_msgs::Float32>("/dist_to_goal", 10, true); //Turn on latch so that last published msg would be saved
  timerGoal= nh_.createTimer(ros::Duration(0.2), &waypointgen_server::timerGoalCallback,this);
}


//Callbacks
//move_base_action CB
void waypointgen_server::goalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& msg){
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

//Get current location
void waypointgen_server::goalFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
  float dispFromGoal; //Displacement from goal

  std::vector<geometry_msgs::PoseStamped> dFromGoal;

  //Insert waypoint goalPose
  geometry_msgs::PoseStamped poseTemp = convertToPoseStamped(feedback->base_position.header.frame_id, currentWaypoint);
  dFromGoal.push_back(poseTemp);

  //Insert current position
  poseTemp = convertToPoseStamped(feedback->base_position.header.frame_id, feedback->base_position.pose);
  dFromGoal.push_back(poseTemp);

  //Calculate displacement from goal
  ecDist = getPathDist(dFromGoal);
  #ifdef DEBUG
  //ROS_INFO("Euclidean distance: %f",ecDist);
  #endif

  //ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w);
}

//Get global path length
void waypointgen_server::gPlanCallback(const nav_msgs::Path &msg) {
  std::vector<geometry_msgs::PoseStamped> path_poses = msg.poses;
  gpDist = getPathDist(path_poses);                                   // Calculate path to target waypoint
  #ifdef DEBUG
  //ROS_INFO("Global Path len: %f", gpDist);
  #endif
}

//Get wpg_server_status callback
void waypointgen_server::wpgStatCallback(const waypointgen::wpg_stat &msg) {
  ROS_INFO_STREAM(msg.status);
}

//Publish average of Global path length & Euclidean distance from target if both are non zero
void waypointgen_server::timerGoalCallback(const ros::TimerEvent& event)
{
  std_msgs::Float32 tgoal;
  if(ecDist==0){
    tgoal.data = gpDist;
  }else if(gpDist==0){
    tgoal.data = ecDist;
  }else{
    tgoal.data = (ecDist+gpDist)/2;
  }
  distToGoalPub.publish(tgoal);
  //ROS_INFO("Avg distance: %f",tgoal.data);
}

//Calculate path length
float waypointgen_server::getPathDist(std::vector<geometry_msgs::PoseStamped> pathVector){
  float dst = 0;
  //Get num of path points
  int sz = pathVector.size();
  for(int i=0;i<sz-1;i++){
    float x1 = pathVector[i].pose.position.x;
    float x2 = pathVector[i+1].pose.position.x;
    float y1 = pathVector[i].pose.position.y;
    float y2 = pathVector[i+1].pose.position.y;
    //ROS_INFO("%f %f %f %f",x1,x2,y1,y2);
    dst += sqrt(pow((x2-x1),2)+pow((y2-y1),2));
    //ROS_INFO("Path len: %f",distToGoal);
  }
  return dst;
}

/*
YAML Parsing
*/

//Load and parse the waypoint list, return number of waypoints
int waypointgen_server::loadWaypointList(std::string list_path){
  //Open yaml file, parse it as string
  std::ifstream ifs(list_path);
  std::string yml_content( (std::istreambuf_iterator<char>(ifs) ),(std::istreambuf_iterator<char>()) );

  //Load YAML
  YAML::Node node = YAML::Load(yml_content);

  #ifdef DEBUG
  //ROS_INFO_STREAM(node.Type()<<","<<node.size()<<","<<node.IsSequence());
  #endif

  wp_count=0; //Reset counter
  if (node["count"]) {
    wp_count=node["count"].as<int>();
    ROS_INFO("Waypoint count-> %i",wp_count);
  }

  #ifdef DEBUG
  ROS_INFO_STREAM("== Saving pose to list ==");
  #endif

  //Temporary pose array list to be published
  geometry_msgs::PoseArray tmpPoseArray;

  //Iterate through all the points
  for(int j=0;j<wp_count;j++){
    std::string wpID="WP"+std::to_string(j);        //Waypoint ID
    geometry_msgs::Pose tempPose;                   //Temp Pose to store in vector

    if(node[wpID]){
      //Get quaternion
      for (std::size_t i=0;i<node[wpID].size();i++) {
        #ifdef DEBUG
        //   ROS_INFO_STREAM(i<<": "<<node[wpID][i].as<float>() );
        #endif

        switch(i){
          case 0:
          tempPose.position.x=node[wpID][i].as<float>();
          break;
          case 1:
          tempPose.position.y=node[wpID][i].as<float>();
          break;
          case 2:
          tempPose.orientation.x=node[wpID][i].as<float>();
          break;
          case 3:
          tempPose.orientation.y=node[wpID][i].as<float>();
          break;
          case 4:
          tempPose.orientation.z=node[wpID][i].as<float>();
          break;
          case 5:
          tempPose.orientation.w=node[wpID][i].as<float>();
          break;
        }
      }
      //Insert pose into list
      wpList.push_back(tempPose);
      tmpPoseArray.poses.push_back(tempPose); //Add to publisher
      ROS_INFO("WP%i: %f %f %f %f",j,tempPose.position.x,tempPose.position.y,tempPose.orientation.z,tempPose.orientation.w);
    }
  }

  //pointPub.publish(tmpPoseArray);

  #ifdef DEBUG
  ROS_INFO_STREAM("== Check list val ==");
  //Check list values
  std::vector <geometry_msgs::Pose> :: iterator it;
  for(it = wpList.begin(); it != wpList.end(); ++it){
    //ROS_INFO_STREAM( '\n' << *it);
  }
  #endif
}

//Adds timestamps to poses, converts pose to poseStamped: poseTarget -> poseStamped
geometry_msgs::PoseStamped waypointgen_server::convertToPoseStamped(std::string poseFrameID, geometry_msgs::Pose poseTarget){
  geometry_msgs::PoseStamped poseStamped;

  //Create waypoint header
  poseStamped.header.frame_id = poseFrameID; // reference to map
  poseStamped.header.stamp = ros::Time::now();

  // set x,y coordinates
  poseStamped.pose.position.x = poseTarget.position.x;
  poseStamped.pose.position.y = poseTarget.position.y;
  //Set rotation (Quaternion)
  poseStamped.pose.orientation.x = poseTarget.orientation.x;
  poseStamped.pose.orientation.y = poseTarget.orientation.y;
  poseStamped.pose.orientation.z = poseTarget.orientation.z;
  poseStamped.pose.orientation.w = poseTarget.orientation.w;

  return poseStamped;
}

//Point to point navigation
void waypointgen_server::p2p(int currentWP, geometry_msgs::Pose qpt){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ROS_INFO("Moving out soon...");

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped goalPose;

  //Create goal waypoint
  goal.target_pose.header.frame_id = "map"; // reference to map
  goal.target_pose.header.stamp = ros::Time::now();

  // set x,y coordinates
  goal.target_pose.pose.position.x = qpt.position.x;
  goal.target_pose.pose.position.y = qpt.position.y;
  //Set rotation
  goal.target_pose.pose.orientation.x = qpt.orientation.x;
  goal.target_pose.pose.orientation.y = qpt.orientation.y;
  goal.target_pose.pose.orientation.z = qpt.orientation.z;
  goal.target_pose.pose.orientation.w = qpt.orientation.w;

  //Goal pose, pose that would be published

  //Adds timestamp to goal pose
  goalPose = convertToPoseStamped("map",qpt);

  //Convert Quaternion to Euler Angle
  tf::Quaternion q(qpt.orientation.x, qpt.orientation.y, qpt.orientation.z, qpt.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw*=180/PI;   //Convert to degrees

  //Current waypoint, total waypoint, x pos, y pos, angular (yaw)
  ROS_INFO("Sending next goal [%i/%i]: (%.2f,%.2f, %.2f)",currentWP+1, wp_count, qpt.position.x,qpt.position.y,yaw);

  //ac.sendGoal(goal,boost::bind(&waypointgen::goalDoneCB, this,_1,_2),MoveBaseClient::SimpleActiveCallback(), MoveBaseClient::SimpleFeedbackCallback());
  ac.sendGoal(goal,boost::bind(&waypointgen_server::goalDoneCB, this,_1,_2),MoveBaseClient::SimpleActiveCallback(), boost::bind(&waypointgen_server::goalFeedbackCB, this,_1));

  //Publish current waypoint goal
  pointPubGoal.publish(goalPose);

  ac.waitForResult();

  //wayptCounter++;

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Done!");
  }else if (ac.getState() == actionlib::SimpleClientGoalState::LOST) {
    ROS_INFO("Skipping to next goal");
    exit(0);
  }else{
    ROS_INFO("The base failed to move forward for some reason");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "setpoint_server");
  ros::NodeHandle n("~");
  waypointgen_server wpg("WPG",n);
  wpg.init();

  //Start Multithreading Process(Async thread): http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
  //std::thread::hardware_concurrency -> Returns the number of concurrent threads supported by the implementation, would be 4
  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  ros::Rate r(10);  //Run at 10Hz

  spinner.start();

  //Get waypoint list to use in the wp_list directory (Inlude yaml extension at back)
  std::string l_path;
  if (wpg.nh_.getParam("/setpoint_server/pathway", l_path))
  {
    l_path = ros::package::getPath("waypointgen") + "/wp_list/"+l_path;
    if(DEBUG){
      ROS_INFO("Setting waypoint list path -> %s", l_path.c_str());
    }
  }
  else
  {
    //Set default to waypoint directory
    ROS_ERROR("Unable to find waypoint list path. Did you set the 'list_path' param? ");
  }

  //Load the waypoint list
  wpg.numOfWaypoints = wpg.loadWaypointList(l_path);

  //Wait for 10s before starting
  int dr = 10;
  for(int k=dr;k>0;k--){
    ROS_INFO("Commencing navigation in %is",k);
    ros::Duration(1).sleep();
  }

  //Start Navigation
  for(int i=0;i<wpg.wpList.size();i++){
    wpg.currentWaypoint = wpg.wpList.at(i);
    wpg.p2p(i, wpg.wpList.at(i));

  }

  ROS_INFO("Completed route!");


  ros::waitForShutdown();
  //ros::spin();
  return 0;
}
