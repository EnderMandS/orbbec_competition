#ifndef __DRONE_DRIVE_H
#define __DRONE_DRIVE_H

#include "drone_msgs/ExecStatus.h"
#include "drone_msgs/PositionCommand.h"
#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace Drone {

struct PosCmd {
  double x = 0, y = 0, z = 0, yaw = 0;
  bool planner_ctrl = false, picture = false;
} typedef PosCmd;

class SmoothTraj {
private:
#define TRAJ_TIME (1.0)
  PosCmd start_pos, end_pos;
  double start_time;

public:
  void genNew(PosCmd, PosCmd);
  std::vector<double> getPosNow(void);
  bool trajComplete(void);
};

class DroneDrive {
public:
  DroneDrive() : o_c_listener(tf_buffer){};
  void init(ros::NodeHandle &);

private:
  // drone drive
  ros::Subscriber posititon_cmd_sub;
  ros::Publisher joint_pub, nav_pub;
  ros::Timer cmd_pub_timer;
  ros::ServiceClient set_yaw_client, take_picture_client, start_record_client,
      stop_record_client;
  PosCmd planner_pos_cmd;
  SmoothTraj smooth_pos;
  void positionCommandCb(const drone_msgs::PositionCommandConstPtr &);
  void cmdPubTimerCb(const ros::TimerEvent &);
  bool inPosition(PosCmd);
  bool inYaw(PosCmd);
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
