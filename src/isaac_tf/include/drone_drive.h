#ifndef __DRONE_DRIVE_H
#define __DRONE_DRIVE_H

#include "drone_msgs/PositionCommand.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

namespace Drone {

class DroneDrive {
public:
  DroneDrive():o_c_listener(tf_buffer){};
  void init(ros::NodeHandle &);

private:
  // drone drive
  ros::Subscriber posititon_cmd_sub;
  ros::Publisher joint_pub;
  ros::Timer test_timer;

  drone_msgs::PositionCommand posititon_cmd;

  void positionCommandCb(const drone_msgs::PositionCommandConstPtr &);
  void testTimerCb(const ros::TimerEvent &);

  // camera pose tf
  ros::Subscriber odom_sub;
  ros::Publisher camera_pose_pub;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener o_c_listener;

  void odomCb(const nav_msgs::OdometryConstPtr &);
};

} // namespace Drone

#endif
