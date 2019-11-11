#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

#define DEBUG 1

typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseConstPtr;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::vector<geometry_msgs::Pose> wpList;  //Waypoint listed

geometry_msgs::PoseStamped currentLoc; //current pos

//Subscriber
  ros::Subscriber posCurrentSub; // Subscribe to robot_pose

//Publisher
ros::Publisher pointPub; // publish pose_goal

float distToGoal =1000;

//Callbacks
//Get current location
void posCurrentCallback(const PoseConstPtr& msg) {
  currentLoc = *msg;
}

//Pub current loc
void publishPoint(geometry_msgs::PoseStamped msg){
  pointPub.publish(msg);
}

//Point to point navigation
void p2p(geometry_msgs::Pose qpt){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

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
  publishPoint(goalPose);
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

  ros::NodeHandle n;

//Subscriber
posCurrentSub= n.subscribe("/robot_pose", 10, &posCurrentCallback);

//Create Publisher
pointPub= n.advertise<geometry_msgs::PoseStamped>("/target_pts", 10);

  //Get waypoint list to use in the wp_list directory (Inlude yaml extension at back)
  std::string l_path;
  if (n.getParam("/setpoint_server/pathway", l_path))
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

  //Open yaml file, parse it as string
  std::ifstream ifs(l_path);
  std::string yml_content( (std::istreambuf_iterator<char>(ifs) ),
  (std::istreambuf_iterator<char>()    ) );

  //Load YAML
  YAML::Node node = YAML::Load(yml_content);
  //ROS_INFO_STREAM(node.Type()<<","<<node.size()<<","<<node.IsSequence());

  //Get total number of waypoints, save to counter
  int wp_count=0;
  if (node["count"]) {
    wp_count=node["count"].as<int>();
    ROS_INFO("Waypoint count-> %i",wp_count);
  }

  if(DEBUG){
    ROS_INFO_STREAM("== Saving pose to list ==");
  }
  //Iterate through all the points
  for(int j=0;j<wp_count;j++){
    //Waypoint ID
    std::string wpID="WP"+std::to_string(j);
    //Temp Pose
    geometry_msgs::Pose tempPose;

    if(node[wpID]){
      //Get quaternion
      for (std::size_t i=0;i<node[wpID].size();i++) {
        // if(DEBUG){
        //   ROS_INFO_STREAM(i<<": "<<node[wpID][i].as<float>() );
        // }
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
      ROS_INFO("WP%i: %f %f %f %f",j,tempPose.position.x,tempPose.position.y,tempPose.orientation.z,tempPose.orientation.w);
    }
  }

if(DEBUG){
  ROS_INFO_STREAM("== Check list val ==");
  //Check list values
  std::vector <geometry_msgs::Pose> :: iterator it;
    for(it = wpList.begin(); it != wpList.end(); ++it)
        ROS_INFO_STREAM( '\n' << *it);
}

//Wait for 10s before starting
int dr = 10;
for(int k=dr;k>0;k--){
  ROS_INFO("Commencing navigation in %is",k);
  ros::Duration(1).sleep();
}

//Start Navigation
for(int i=0;i<wpList.size();i++){
p2p(wpList.at(i));
}


  ros::spin();
  return 0;
}
