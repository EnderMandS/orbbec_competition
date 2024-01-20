#include "drone_drive.h"
#include <cmath>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_drive");
  ros::NodeHandle nh;

  Drone::DroneDrive drone_drive;
  drone_drive.init(nh);
  ROS_INFO("DroneDrive start.");
  ros::spin();
  return 0;
}

using namespace Drone;

void DroneDrive::init(ros::NodeHandle &nh) {
  posititon_cmd_sub =
      nh.subscribe("/position_cmd", 5, &DroneDrive::positionCommandCb, this);
  joint_pub = nh.advertise<sensor_msgs::JointState>("/drone_cmd", 10);
  test_timer =
      nh.createTimer(ros::Duration(0.1), &DroneDrive::testTimerCb, this);
}

void DroneDrive::positionCommandCb(
    const isaac_tf::PositionCommandConstPtr &msg) {
  posititon_cmd = *msg;

  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"x_joint","y_joint","z_joint","R_body"};
  joint_cmd.position = {msg->position.x, msg->position.y, msg->position.z, msg->yaw};
  joint_pub.publish(joint_cmd);
}

void DroneDrive::testTimerCb(const ros::TimerEvent &e) {
  static auto t_start = e.current_real.toSec();
  auto t = e.current_real.toSec()-t_start;
  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"x_joint","y_joint","z_joint","R_body"};
  joint_cmd.position = {t * 0.05, t * 0.05, t * 0.05, sin(t)*M_PI};
  joint_pub.publish(joint_cmd);
}
