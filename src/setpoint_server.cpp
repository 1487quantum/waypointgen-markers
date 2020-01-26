#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <wpg_msg/wpServerAction.h>

#define DEBUG 1

class waypointgen{
protected:
  //actionlib
  actionlib::SimpleActionServer<wpg_msg::wpServerAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  // create messages that are used to published feedback/result
  wpg_msg::wpServerActionFeedback feedback_;
  wpg_msg::wpServerActionResult result_;

private:
  //  typedef boost::shared_ptr<const geometry_msgs::PoseStamped> PoseConstPtr;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  std::string an;

  //Total number of waypoints
  int wp_count=0;

public:
  ros::NodeHandle nh_;
  std::vector<geometry_msgs::Pose> wpList;  //Waypoint listed

  geometry_msgs::PoseStamped currentLoc; //current pos

  //Publisher
  ros::Publisher pointPub; // publish pose_goal

  float distToGoal =1000;

  //Constructor
  waypointgen(std::string name, ros::NodeHandle nh_) :
  as_(nh_, name, boost::bind(&waypointgen::posCurrentCallback, this, _1), false),
  an(name){
    this->nh_=nh_;
    as_.start();
  };

  //Destructor
  ~waypointgen(void){
  }

  void init();
  void sendWaypointCallback(const wpg_msg::wpServerGoalConstPtr &msg);
  void posCurrentCallback(const wpg_msg::wpServerGoalConstPtr &msg);

  void loadWaypointList(  std::string list_path);

  void publishPoint(ros::Publisher pb,geometry_msgs::PoseStamped msg);
  void p2p(ros::Publisher pb,geometry_msgs::Pose qpt);
};

//Start sub and pub
void waypointgen::init(){
  ROS_INFO("Init pub & sub");
  ros::Subscriber posCurrentSub; // Subscribe to robot_pose
  posCurrentSub=  nh_.subscribe("/robot_pose", 10, &waypointgen::posCurrentCallback, this);
  pointPub= nh_.advertise<geometry_msgs::PoseArray>("/target_pts", 10, true); //Turn on latch so that last published msg would be saved
}

//Callbacks
void waypointgen::sendWaypointCallback(const wpg_msg::wpServerGoalConstPtr &msg){

}

//Get current location
void waypointgen::posCurrentCallback(const wpg_msg::wpServerGoalConstPtr &msg) {
  // helper variables
  ros::Rate r(1);
  bool success = true;

  //  currentLoc = *msg;
  ROS_INFO("%s: Pose call back", an.c_str());
  //  ROS_INFO("%s: Test->%i", an.c_str(), msg->wp_goal);

  if(success)
  {
    ROS_INFO("%s: Succeeded", an.c_str());
    // set the action state to succeeded
  }
}

//Load and parse the waypoint list
void waypointgen::loadWaypointList(std::string list_path){
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
          tempPose.orientation.z=node[wpID][i].as<float>();
          break;
          case 3:
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
    ROS_INFO_STREAM( '\n' << *it);
  }
  #endif
}

//Pub current loc
void waypointgen::publishPoint(ros::Publisher pb, geometry_msgs::PoseStamped msg){
  pb.publish(msg);
}

//Point to point navigation
void waypointgen::p2p(ros::Publisher pb, geometry_msgs::Pose qpt){
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
  goal.target_pose.pose.orientation.w = qpt.orientation.z;
  goal.target_pose.pose.orientation.z = qpt.orientation.w;

  //Pose
  //Header
  goalPose.header.frame_id = "map"; // reference to map
  goalPose.header.stamp = ros::Time::now();
  // set x,y coordinates
  goalPose.pose.position.x =  qpt.position.x;
  goalPose.pose.position.y =  qpt.position.y;
  // Set goal marker
  goalPose.pose.orientation.x = 0.0;
  goalPose.pose.orientation.y = 0.0;
  goalPose.pose.orientation.z = qpt.orientation.z;
  goalPose.pose.orientation.w = qpt.orientation.w;

  ROS_INFO("Sending next goal");
  ac.sendGoal(goal);
  publishPoint(pb, goalPose);
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

  // double xd=goalPose.pose.position.x-currentLoc.pose.position.x;
  // double yd=goalPose.pose.position.y-currentLoc.pose.position.y;
  // double odz=abs(goalPose.pose.orientation.z-currentLoc.pose.orientation.z);
  // distToGoal=sqrt(xd*xd+yd*yd);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "setpoint_server");
  ros::NodeHandle n("~");
  waypointgen wpg("WPG",n);
  wpg.init();

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
  wpg.loadWaypointList(l_path);

  //Wait for 10s before starting
  int dr = 10;
  for(int k=dr;k>0;k--){
    ROS_INFO("Commencing navigation in %is",k);
    ros::Duration(1).sleep();
  }

  //Start Navigation
  for(int i=0;i<wpg.wpList.size();i++){
    wpg.p2p(wpg.pointPub, wpg.wpList.at(i));
  }

  ros::spin();
  return 0;
}
