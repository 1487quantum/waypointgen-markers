#include "waypointgen/waypointgen_utils.h"

waypointgen_utils::waypointgen_utils(){};

void waypointgen_utils::update_header(std_msgs::Header &hd,
                                   const std::string &frameID) {
  hd.frame_id = frameID; // reference frame
  hd.stamp = ros::Time::now();
}

// Param: position,orientation
geometry_msgs::PoseWithCovariance
waypointgen_utils::addPoseCov(const geometry_msgs::Point &pt,
                              const tf::Quaternion &q_rotate) {
  geometry_msgs::Quaternion quat_msg;
  quaternionTFToMsg(q_rotate, quat_msg);

  geometry_msgs::PoseWithCovariance cov_pose;
  cov_pose.pose.position = pt;
  cov_pose.pose.orientation = quat_msg;
  return cov_pose;
}

// Get current time
std::string waypointgen_utils::getCurrentTime() {
  std::time_t now = time(nullptr);
  std::tm ltm = *std::localtime(&now); // tm -> timestruct
  std::stringstream buf;
  buf << std::put_time(&ltm, "%d%m%Y_%H%M%S");
  return buf.str(); // ddMMYYYY_hhmmss
}

void waypointgen_utils::printDebugPose(
    const std::string &dmsg, const std::string &wp_name,
    const geometry_msgs::PoseWithCovariance &pw) {
  tf::Quaternion q(pw.pose.orientation.x, pw.pose.orientation.y,
                   pw.pose.orientation.z, pw.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[%s] %s(x,y,ang) -> %f, %f, %f", dmsg.c_str(), wp_name.c_str(),
           pw.pose.position.x, pw.pose.position.y, yaw * 180 / PI);
}