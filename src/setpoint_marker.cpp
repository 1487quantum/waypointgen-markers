#include <std_msgs/String.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;

//Marker
float marker_pos = 0;
unsigned char marker_id = 0; //Take a max value of 255 waypoints
std::list<InteractiveMarker> marker_list;

//Menu
MenuHandler menu_handler;
bool menuInit = false;

//Subscriber
ros::Subscriber sub_setpoint_list;

//Fx declaration
void mnu_addNewWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void addWaypoint(unsigned int mrk_id);

//Subscriber callback
void setpointListCallback(const visualization_msgs::InteractiveMarkerInitConstPtr msg){
  std_msgs::String a;
  for(auto mk:msg->markers){
    a.data=mk.name;
    a.data+=": ";
    a.data+=std::to_string(mk.pose.position.x);
    a.data+=",";
    a.data+=std::to_string(mk.pose.position.y);
    a.data+=",";
    a.data+=std::to_string(mk.pose.orientation.z);

    ROS_INFO_STREAM(a.data.c_str());
  }

}

//Menu callback actions
void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
  << feedback->pose.position.x << ", " << feedback->pose.position.y
  << ", " << feedback->pose.orientation.w );
}

void mnu_getLocation(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  std_msgs::String dispMsg; //To display on console
  dispMsg.data=feedback->marker_name;
  dispMsg.data+=": ";
  dispMsg.data+=std::to_string(feedback->pose.position.x);
  dispMsg.data+=",";
  dispMsg.data+=std::to_string(feedback->pose.position.y);
  dispMsg.data+=",";
  dispMsg.data+=std::to_string(feedback->pose.orientation.w);

  //Show Location
  ROS_INFO("%s",dispMsg.data.c_str());
}

//Add waypoint
void mnu_addNewWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("Waypoint Added!");
  if(marker_list.size()<256){
    addWaypoint(++marker_id);
  }else{
    ROS_INFO("Setpoint limit reached!");
  }
}

//Remove waypoint_
void mnu_removeWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  std_msgs::String dispMsg; //To display on console
  std::string markerName = feedback->marker_name; //Marker Name

  //Delimit to the end to get id
  std::string delimiter = "_";
  int markerID = stoi(markerName.substr(markerName.find(delimiter)+1, -1)); // Get last part of string, since name is waypoint_[ID]; Convert str->int

  if(marker_list.size()>1){
    //Remove from server
    server->erase(feedback->marker_name);
    server->applyChanges();

    //Remove from list
    //marker_list.remove( feedback->marker_name);

    dispMsg.data=feedback->marker_name;
    dispMsg.data+=" removed!";
  }else{
    dispMsg.data="Cannot remove all points!";
  }

  ROS_INFO("%s",dispMsg.data.c_str());
}

void mnu_createList(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  std_msgs::String dispMsg;

  dispMsg.data = std::to_string(marker_list.size());
  ROS_INFO("marker_list len -> %s\n===========",dispMsg.data.c_str());
  for (std::list<InteractiveMarker>::iterator it = marker_list.begin(); it != marker_list.end(); ++it)
  {
    dispMsg.data= it->name;

    //ROS_INFO("%s",dispMsg.data.c_str());
  }
  server->applyChanges(); //Update the interactive marker ist before saving the data

  //Call sub once to get update values
  ros::NodeHandle n;
  sub_setpoint_list= n.subscribe("setpoint_marker/update_full", 10, setpointListCallback);
}

void deepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std_msgs::String dispMsg; //To display on console
  std::string markerName = feedback->marker_name;

  dispMsg.data ="Got ID: ";
  dispMsg.data+= markerName;

  //Delimit to the end to get id
  std::string delimiter = "_";
  std::string markerID = markerName.substr(markerName.find(delimiter)+1, -1); // Get last part of string, since name is waypoint_[ID]
  dispMsg.data+= "\n";
  dispMsg.data+=markerID;
  dispMsg.data+=",";
  dispMsg.data+=std::to_string(feedback->pose.position.x);

  //Get Position
  ROS_INFO("%s\n",dispMsg.data.c_str());
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
  //marker_list.push_back(sp_marker);
  return sp_marker;
}

/*
====================
MARKER CONTROLS
====================
*/
//Add motion controls (To modify waypoint position and orientation)
//mvAxis: 0->Rotate, 1:Translate;
InteractiveMarkerControl addMovementControl(InteractiveMarkerControl im_c,bool mw,bool mx,bool my,bool mz,std::string mName, bool mvAxis){
  im_c.orientation.w = mw;
  im_c.orientation.x = mx;
  im_c.orientation.y = my;
  im_c.orientation.z = mz;
  im_c.name = mName;
  im_c.interaction_mode = mvAxis?InteractiveMarkerControl::MOVE_AXIS:InteractiveMarkerControl::ROTATE_AXIS;
  return im_c;
}

//Add controls
void addControls(InteractiveMarker i_mk, unsigned char mrk_id){
  InteractiveMarkerControl imc;

  //Position & Orientation
  // add the control to the interactive marker
  i_mk.controls.push_back(addMovementControl(imc,1,1,0,0,"move_x", true));  //Translate about X-axis
  i_mk.controls.push_back(addMovementControl(imc,1,0,0,1,"move_y",true));  //Translate about Y-axis
  i_mk.controls.push_back(addMovementControl(imc,1,0,1,0,"rotate_z",false));   //Rotate about Z-axis

  //MENU
  if(!menuInit){
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

    menuInit=true;
  }

  imc.interaction_mode = InteractiveMarkerControl::BUTTON;
  imc.always_visible = true;

  //Add functions to control
  //Create Arrow (Visual)
  imc.markers.push_back( makeArrow(mrk_id));
  // add the control to the interactive marker
  i_mk.controls.push_back( imc );

  server->insert(i_mk);
  menu_handler.apply( *server, i_mk.name); //Apply to int_marker.name, which is setpoint_marker

}

void addWaypoint(unsigned int mrk_id){
  InteractiveMarker mrk;
  //Header
  mrk.header.frame_id = "base_link";
  mrk.header.stamp=ros::Time::now();
  //Name
  mrk.name = "waypoint_";
  mrk.name += std::to_string(mrk_id);   //Add marker ID
  mrk.description = "Waypoint Marker ";
  mrk.description += std::to_string(mrk_id);
  //locations (Vary spawn location)
  mrk.pose.position.x=marker_id%5 - 2;
  mrk.pose.position.y=marker_id%6 - 2;

  marker_list.push_back(mrk); //Add to list

  //Add CONTROLS
  addControls(mrk,mrk_id);

  //Update map_server
  server->applyChanges();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "setpoint_marker");
  server.reset( new InteractiveMarkerServer("setpoint_marker","",false) );

  //Add waypoint at start
  addWaypoint(marker_id);

  // start the ROS main loop
  ros::spin();
  server.reset();
}