#ifndef __DRONE_DRIVE_H
#define __DRONE_DRIVE_H

#include "drone_msgs/ExecStatus.h"
#include "drone_msgs/PositionCommand.h"
#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace Drone {

struct {
  double x, y, z, yaw;
  bool planner_ctrl;
} typedef PosCmd;

class DroneDrive {
public:
  DroneDrive() : o_c_listener(tf_buffer){};
  void init(ros::NodeHandle &);

private:
  // drone drive
  ros::Subscriber posititon_cmd_sub;
  ros::Publisher joint_pub, nav_pub;
  ros::Timer cmd_pub_timer;
  PosCmd planner_pos_cmd = {0, 0, 0, 0, false};
  void positionCommandCb(const drone_msgs::PositionCommandConstPtr &);
  void cmdPubTimerCb(const ros::TimerEvent &);
  bool inPosition(PosCmd);
  void pubWaypoint(PosCmd);

  // camera pose tf
  ros::Subscriber odom_sub;
  ros::Publisher camera_pose_pub;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener o_c_listener;
  nav_msgs::Odometry odom;
  void odomCb(const nav_msgs::OdometryConstPtr &);

  // exec status
  enum {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    NONE,
  };
  uint8_t exec_status = NONE;
  ros::Subscriber exec_status_sub;
  void execStatusCb(const drone_msgs::ExecStatusConstPtr &);
};

} // namespace Drone

#endif
