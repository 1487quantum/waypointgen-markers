#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/tf.h>

#include <map>
#include <stdlib.h>
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <fstream>

#define DEBUG 1

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;

//Marker
unsigned char marker_id = 0; //Take a max value of 255 waypoints
unsigned char marker_count = 0; //Keep track of number of waypoints created

typedef std::map<std::string,geometry_msgs::PoseWithCovariance> p_map;
p_map wpl;

float lastMarker[2]={0.0,0.0};//Get pos of last marker

//Subscriber
ros::Subscriber sub_setpoint_list;

//Menu
MenuHandler menu_handler;
bool menuInit = false;

//Fx declaration
void updateWypt(p_map tmp, string wp_name, geometry_msgs::PoseWithCovariance pt);
void addWaypointMarker(unsigned int mrk_id);
void printDebugPose(std::string dmsg, std::string wp_name, geometry_msgs::PoseWithCovariance pw);
//Sub/Pub
void updateWaypointPos( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void setpointListCallback(const visualization_msgs::InteractiveMarkerInitConstPtr msg);
//Menu
void mnu_addNewWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void mnu_createList(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void mnu_removeWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void addContextMnu(bool init);
//Markers
InteractiveMarker setHeader(InteractiveMarker imk_h, int index_h);
InteractiveMarker setPos(InteractiveMarker imk_h, int sx, int sy);
Marker makeArrow(unsigned char mrk_id);
//Controls
InteractiveMarkerControl addIntCtrl(InteractiveMarkerControl imc, unsigned char mk_id);
InteractiveMarkerControl addMovementControl(InteractiveMarkerControl im_c,bool mw,bool mx,bool my,bool mz,string mName, bool mvAxis);
InteractiveMarker addMotionControl(InteractiveMarker i_mk, InteractiveMarkerControl imc_am);
//Export
geometry_msgs::PoseWithCovariance addPoseCov(double pX,double pY,double pZ,double oX,double oY,double oZ,double oW);
geometry_msgs::PoseWithCovariance createPose(double cx,double cy,tf::Quaternion q_rotate);
string addZero(int a);
string getCurrentTime();
void updateWPList(p_map tmp);


//Add zero infront if less than 10
string addZero(int a){
  string res;
  if(a<10){
    res = "0";
  }
  res+=to_string(a);
  return res;
}

//Get current time
string getCurrentTime(){
  time_t now = time(0);
  tm *ltm = localtime(&now);  //tm -> timestruct
  //dMMYYYY_hhmmss
  string timenow  = addZero(ltm->tm_mday) +  std::to_string(1 + ltm->tm_mon) + std::to_string(1900 + ltm->tm_year)+"_";
  timenow += addZero(ltm->tm_hour)+addZero(ltm->tm_min)+addZero(ltm->tm_sec);
  return timenow;
}

//Param: position,orientation
geometry_msgs::PoseWithCovariance addPoseCov(double pX,double pY,double pZ,double oX,double oY,double oZ,double oW){
  geometry_msgs::PoseWithCovariance cov_pose;
  cov_pose.pose.position.x = pX;
  cov_pose.pose.position.y = pY;
  cov_pose.pose.position.z = pZ;
  cov_pose.pose.orientation.x = oX;
  cov_pose.pose.orientation.y = oY;
  cov_pose.pose.orientation.z = oZ;
  cov_pose.pose.orientation.w = oW;
return cov_pose;
}

geometry_msgs::PoseWithCovariance createPose(double cx,double cy,tf::Quaternion q_rotate){
  geometry_msgs::PoseWithCovariance wpoint;
  wpoint = addPoseCov(cx,cy,0,q_rotate[0],q_rotate[1],q_rotate[2],q_rotate[3]);
  return wpoint;
}

void updateWaypointPos( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  if( feedback->event_type==visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
    //Update marker position when user releases marker
    if(DEBUG){
      ROS_INFO("[MOUSE_UP] %s: \nframe: %s\nPos: %f, %f, %f\nOrient: %f, %f, %f, %f",
      feedback->marker_name.c_str(),feedback->header.frame_id.c_str(),
      feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
      feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
    }
    //Update lastMarker position
    lastMarker[0]=feedback->pose.position.x;
    lastMarker[1]=feedback->pose.position.y;
    //Update pose in map
    geometry_msgs::PoseWithCovariance pwc;
    pwc = addPoseCov(feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z,feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
    updateWypt(wpl,feedback->marker_name,pwc);

    server->applyChanges();
  }
}

void printDebugPose(std::string dmsg, std::string wp_name, geometry_msgs::PoseWithCovariance pw){
  tf::Quaternion q(pw.pose.orientation.x, pw.pose.orientation.y, pw.pose.orientation.z, pw.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[%s] %s(x,y,ang) -> %f, %f, %f", dmsg.c_str(), wp_name.c_str(),pw.pose.position.x,pw.pose.position.y,yaw*180/3.1415927);
}

//Subscriber callback
void setpointListCallback(const visualization_msgs::InteractiveMarkerInitConstPtr msg){
  geometry_msgs::PoseWithCovariance pt;
  updateWPList(wpl);
  for(auto mk:msg->markers){
    //Create Quaternion for rotation
    tf::Quaternion qtmp(mk.pose.orientation.x,mk.pose.orientation.y,mk.pose.orientation.z,mk.pose.orientation.w);
    //Create pose
    pt = createPose(mk.pose.position.x,mk.pose.position.y,qtmp);

    //Add to map
    updateWypt(wpl,mk.name,pt);

    if(DEBUG){
      printDebugPose("LIST SUB CALLBACK", mk.name, pt);
    }
  }
}

void updateWPList(p_map tmp){
  if(tmp.empty()){
    ROS_INFO("Map is empty! Updating...");
  }
  p_map::iterator it = tmp.begin();
  while(it != tmp.end())
  {
    std::cout<<it->first<<" :: "<<it->second<<std::endl;
    it++;
  }
}

//Add waypoint to map
void updateWypt(p_map tmp, string wp_name, geometry_msgs::PoseWithCovariance pt){
  bool id_exist = (wpl.insert(std::make_pair(wp_name,pt)).second == false);
  if(id_exist){
    ROS_WARN("Overwriting %s", wp_name.c_str());
    wpl[wp_name]=pt;
  }else{
    if(DEBUG){
      printDebugPose("INSERT WAYPONT", wp_name, pt);
    }
    wpl.insert(std::make_pair(wp_name,pt));
  }
}

//Show selected waypoint Location
void mnu_getLocation(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::PoseWithCovariance pwc_gl;
  pwc_gl.pose.position.x=feedback->pose.position.x;
  pwc_gl.pose.position.y=feedback->pose.position.y;
  pwc_gl.pose.position.z=feedback->pose.position.z;

  pwc_gl.pose.orientation.x=feedback->pose.orientation.x;
  pwc_gl.pose.orientation.y=feedback->pose.orientation.y;
  pwc_gl.pose.orientation.z=feedback->pose.orientation.z;
  pwc_gl.pose.orientation.w=feedback->pose.orientation.w;
  //To display on console
  printDebugPose("CURRENT LOC", feedback->marker_name, pwc_gl);
}

//Add waypoint
void mnu_addNewWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  if(marker_id<256){
    addWaypointMarker(++marker_id);
    ROS_INFO("Waypoint Added!");
  }else{
    ROS_INFO("Setpoint limit reached!");
  }
}

//Remove waypoint_
void mnu_removeWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  std_msgs::String dispMsg; //To display on console
  string markerName = feedback->marker_name; //Marker Name

  //Delimit to the end to get id
  string delimiter = "_";
  int markerID = stoi(markerName.substr(markerName.find(delimiter)+1, -1)); // Get last part of string, since name is waypoint_[ID]; Convert str->int

  if(marker_count>1){
    //Remove from map
    wpl.erase(markerName);

    //Remove from server
    server->erase(feedback->marker_name);
    server->applyChanges();

    //Decrement counter
    marker_count -=1;

    dispMsg.data=feedback->marker_name;
    dispMsg.data+=" removed!";
  }else{
    dispMsg.data="Cannot remove all points!";
  }

  ROS_INFO("%s",dispMsg.data.c_str());
}

void mnu_createList(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO("%s %i","[Generating list] Total Waypoints ->",marker_count);

  //Save as YAML file
  YAML::Node pts;
  pts["count"] = (int) marker_count;  // Write total number of waypoints

  int idx=0;

  //Iterate through map
  for(  p_map::iterator it=wpl.begin(); it != wpl.end();it++)
  {
    std::string wp_index = "WP"+std::to_string(idx);
    //Add Position to YAML list
    //pts[wp_index].push_back(it->first);

    //Save quaternion to YAML
    pts[wp_index].push_back(it->second.pose.position.x);
    pts[wp_index].push_back(it->second.pose.position.y);
    pts[wp_index].push_back(it->second.pose.orientation.x);
    pts[wp_index].push_back(it->second.pose.orientation.y);
    pts[wp_index].push_back(it->second.pose.orientation.z);
    pts[wp_index].push_back(it->second.pose.orientation.w);
    idx++;
  }

  //Create list
  fstream exportlist;
  string loc = ros::package::getPath("waypointgen") + "/wp_list/";  //Get package path, Save into list directory

  //Check if list directory exist
  const char* path = loc.c_str();
  boost::filesystem::path dir(path);
  if(boost::filesystem::create_directory(dir))
  {
    ROS_INFO("'list' directory not found, creating one -> %s", loc.c_str());
  }

  //File Name
  string lstName=getCurrentTime()+"_wplist.yaml";
  //Append to main path
  loc+=lstName;
  if(DEBUG){
    ROS_INFO("Writing waypoint list to %s", loc.c_str());
  }

  exportlist.open(loc,fstream::out);
  exportlist << pts;
  exportlist.close();
  ROS_INFO_STREAM("Saved to -> " << loc);
}

/*
Create waypoint
*/

//Create Arrow
Marker makeArrow(unsigned char mrk_id){
  // create a green arrow
  Marker sp_marker;
  sp_marker.type = Marker::ARROW;
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

//Set InteractiveMarker Header
InteractiveMarker setHeader(InteractiveMarker imk_h, int index_h){
  //Header
  imk_h.header.frame_id = "map";    //Set frame relative to map frame
  imk_h.header.stamp=ros::Time::now();
  //Name
  imk_h.name = "waypoint_";
  imk_h.name += to_string(index_h);   //Add marker ID
  imk_h.description = "Waypoint Marker ";
  imk_h.description += to_string(index_h);
  return imk_h;
}

//Set InteractiveMarker position
InteractiveMarker setPos(InteractiveMarker imk_h, int sx, int sy){
  imk_h.pose.position.x= sx;
  imk_h.pose.position.y= sy;
  return imk_h;
}

//Add controls
InteractiveMarker addMotionControl(InteractiveMarker i_mk, InteractiveMarkerControl imc_am){
  //Position & Orientation
  // add the control to the interactive marker
  i_mk.controls.push_back(addMovementControl(imc_am,1,1,0,0,"move_x", true));  //Translate about X-axis
  i_mk.controls.push_back(addMovementControl(imc_am,1,0,0,1,"move_y",true));  //Translate about Y-axis
  i_mk.controls.push_back(addMovementControl(imc_am,1,0,1,0,"rotate_z",false));   //Rotate about Z-axis
  return i_mk;
}

//Add motion controls (To modify waypoint position and orientation)
//mvAxis: 0->Rotate, 1:Translate;
InteractiveMarkerControl addMovementControl(InteractiveMarkerControl im_c,bool mw,bool mx,bool my,bool mz,string mName, bool mvAxis){
  im_c.orientation.w = mw;
  im_c.orientation.x = mx;
  im_c.orientation.y = my;
  im_c.orientation.z = mz;
  im_c.name = mName;
  //Set either translational or rotational
  im_c.interaction_mode = mvAxis?InteractiveMarkerControl::MOVE_AXIS:InteractiveMarkerControl::ROTATE_AXIS;
  return im_c;
}

//Add menu
void addContextMnu(bool init){
  if(!init){
    //Create menu at start if it does not exist
    //Entries
    MenuHandler::EntryHandle entry_showLoc;
    MenuHandler::EntryHandle entry_add;
    MenuHandler::EntryHandle mnu_entry_add;
    MenuHandler::EntryHandle mnu_entry_waypoint;
    MenuHandler::EntryHandle entry_generateYAML;

    //Add to menu
    entry_showLoc = menu_handler.insert( "Get Location" , &mnu_getLocation );

    //Waypoint submenu
    mnu_entry_waypoint = menu_handler.insert( "Waypoint"  );
    entry_add = menu_handler.insert( mnu_entry_waypoint, "Add"  , &mnu_addNewWaypoint );
    entry_add = menu_handler.insert( mnu_entry_waypoint, "Remove" , &mnu_removeWaypoint);

    //Generate list of waypoints
    entry_generateYAML =  menu_handler.insert( "Generate Waypoint List" , &mnu_createList );

    menuInit=true;  //Make it true so that the menu only initializes once
  }
}

InteractiveMarkerControl addIntCtrl(InteractiveMarkerControl imc, unsigned char mk_id){
  imc.interaction_mode = InteractiveMarkerControl::BUTTON;
  imc.always_visible = true;

  //Add control waypoint function
  //Create Arrow (Visual)
  imc.markers.push_back(makeArrow(mk_id));
  return imc;
}

void addWaypointMarker(unsigned int mrk_id){
  InteractiveMarker mrk;
  InteractiveMarkerControl imc_m;

  //Set Header
  mrk = setHeader(mrk,mrk_id);
  //mrk = setPos(mrk,mrk_id%5 - 2,mrk_id%6 - 2);  //Vary spawn location

  //Set next marker close to previous one
  mrk = setPos(mrk, lastMarker[0]+1,lastMarker[1]+1);
  //Update
  lastMarker[0]+=1;
  lastMarker[1]+=1;

  //Add motion CONTROLS
  mrk = addMotionControl(mrk,imc_m);
  //Add MENU
  addContextMnu(menuInit);
  // Add the control to the interactive marker
  imc_m = addIntCtrl(imc_m, mrk_id);
  mrk.controls.push_back( imc_m);

  //Insert into server
  server->insert(mrk);
  server->setCallback(mrk.name, &updateWaypointPos); //Attach callback when user updates the marker position

  menu_handler.apply( *server, mrk.name); //Apply to int_marker.name, which is setpoint_marker

  //Increment marker count tracker
  marker_count+=1;

  //Update map_server
  server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "setpoint_marker");
  server.reset( new InteractiveMarkerServer("setpoint_marker","",false) );

  //Add waypoint at start
  addWaypointMarker(marker_id);

  //Subscriber
  ros::NodeHandle n;
  //Subscribe to the waypoints published
  sub_setpoint_list = n.subscribe("setpoint_marker/update_full", 1, setpointListCallback);

  // start the ROS main loop
  ros::spin();
  server.reset();
}
