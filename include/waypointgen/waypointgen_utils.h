
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

class waypointgen_utils {

public:
  waypointgen_utils();

  geometry_msgs::PoseWithCovariance addPoseCov(const geometry_msgs::Point &pt,
                                               const tf::Quaternion &q_rotate);
  void set_pose_position(geometry_msgs::Point &from_point,
                         const geometry_msgs::Point &to_point);

  std::string getCurrentTime();

  void printDebugPose(const std::string &dmsg, const std::string &wp_name,
                      const geometry_msgs::PoseWithCovariance &pw);

};
