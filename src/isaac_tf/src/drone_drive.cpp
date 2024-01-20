#include "drone_drive.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
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
  odom_sub = nh.subscribe("/odom", 1, &DroneDrive::odomCb, this);
  camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 5);
  // test_timer =
  //     nh.createTimer(ros::Duration(0.1), &DroneDrive::testTimerCb, this);
}

void DroneDrive::positionCommandCb(
    const drone_msgs::PositionCommandConstPtr &msg) {
  posititon_cmd = *msg;

  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};
  joint_cmd.position = {msg->position.x, msg->position.y, msg->position.z,
                        msg->yaw};
  joint_pub.publish(joint_cmd);
}

void DroneDrive::testTimerCb(const ros::TimerEvent &e) {
  static auto t_start = e.current_real.toSec();
  auto t = e.current_real.toSec() - t_start;
  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};
  joint_cmd.position = {t * 0.05, t * 0.05, t * 0.05, sin(t) * M_PI};
  joint_pub.publish(joint_cmd);
}

void DroneDrive::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  geometry_msgs::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer.lookupTransform("odom", "camera", ros::Time(0));
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }

  geometry_msgs::PoseStamped camera_pose;
  camera_pose.pose.orientation = tf_stamped.transform.rotation;
  camera_pose.pose.position.x = tf_stamped.transform.translation.x;
  camera_pose.pose.position.y = tf_stamped.transform.translation.y;
  camera_pose.pose.position.z = tf_stamped.transform.translation.z;
  camera_pose.header.frame_id = tf_stamped.child_frame_id;
  camera_pose.header.stamp = ros::Time::now();
  camera_pose_pub.publish(camera_pose);
}
