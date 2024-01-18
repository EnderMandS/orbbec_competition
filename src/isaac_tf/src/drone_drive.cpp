#include "drone_drive.h"
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_drive");
  ros::NodeHandle nh;

  Drone::DroneDrive drone_drive;
  drone_drive.init(nh);

  ros::spin();
  return 0;
}

using namespace Drone;

void DroneDrive::init(ros::NodeHandle &nh) {
  posititon_cmd_sub =
      nh.subscribe("/position_cmd", 1, &DroneDrive::positionCommandCb, this);
  joint_pub = nh.advertise<sensor_msgs::JointState>("/drone_cmd", 10);
  ROS_INFO("DroneDrive init.");
}

void DroneDrive::positionCommandCb(
    const isaac_tf::PositionCommandConstPtr &msg) {
  posititon_cmd = *msg;

  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"base_link"};
  joint_cmd.position = {msg->position.x,msg->position.y,msg->position.z};
  joint_pub.publish(joint_cmd);
}
