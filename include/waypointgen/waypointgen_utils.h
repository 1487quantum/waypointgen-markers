#ifndef WAYPOINTGEN_UTILS_H
#define WAYPOINTGEN_UTILS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

class waypointgen_utils {

public:
  waypointgen_utils();
  void update_header(std_msgs::Header &hd, const std::string &frameID);
  template <typename T>
  inline void add_timestamp(T &targetMsg, const std::string frameID,
                            const geometry_msgs::Pose &initPose) {
    update_header(targetMsg.header, frameID);
    targetMsg.pose = initPose; // set pose
  };

  geometry_msgs::PoseWithCovariance addPoseCov(const geometry_msgs::Point &pt,
                                               const tf::Quaternion &q_rotate);

  std::string getCurrentTime();

  void printDebugPose(const std::string &dmsg, const std::string &wp_name,
                      const geometry_msgs::PoseWithCovariance &pw);

  static constexpr double PI = 3.1415926535897932385;
};

#endif