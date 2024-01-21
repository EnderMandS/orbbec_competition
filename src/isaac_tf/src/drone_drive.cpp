#include "drone_drive.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <cmath>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

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
      nh.subscribe("planning/pos_cmd", 5, &DroneDrive::positionCommandCb, this);
  joint_pub = nh.advertise<sensor_msgs::JointState>("/drone_cmd", 10);
  odom_sub = nh.subscribe("/odom", 1, &DroneDrive::odomCb, this);
  camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 5);
  exec_status_sub =
      nh.subscribe("/planning/exec_status", 1, &DroneDrive::execStatusCb, this);
  cmd_pub_timer =
      nh.createTimer(ros::Duration(0.01), &DroneDrive::cmdPubTimerCb, this);
  nav_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
}

bool pos_cmd_update = false;
void DroneDrive::positionCommandCb(
    const drone_msgs::PositionCommandConstPtr &msg) {
  // yaw convert -2PI~2PI
  static int loop_cnt = 0;
  static double last_yaw = msg->yaw;
  double out_yaw = 0;
  if (abs(msg->yaw - last_yaw) > 1.5 * M_PI) {
    if (msg->yaw < 0) {
      ++loop_cnt;
    } else {
      --loop_cnt;
    }
  }
  last_yaw = msg->yaw;
  if (loop_cnt > 0) {
    out_yaw = msg->yaw + 2 * M_PI;
  } else if (loop_cnt < 0) {
    out_yaw = msg->yaw - 2 * M_PI;
  } else {
    out_yaw = msg->yaw;
  }
  if (out_yaw > 2 * M_PI) {
    --loop_cnt;
    out_yaw = msg->yaw;
  } else if (out_yaw < -2 * M_PI) {
    ++loop_cnt;
    out_yaw = msg->yaw;
  }
  // ROS_INFO("msg_yaw:%5f, out_yaw:%5f, loop:%d", msg->yaw, out_yaw, loop_cnt);
  planner_pos_cmd.x = msg->position.x;
  planner_pos_cmd.y = msg->position.y;
  planner_pos_cmd.z = msg->position.z;
  planner_pos_cmd.yaw = out_yaw;
  pos_cmd_update = true;
}

void DroneDrive::execStatusCb(const drone_msgs::ExecStatusConstPtr &msg) {
  exec_status = msg->exec_flag;
}

const PosCmd waypoint_list[] = {
    {0, 0, 0, 0, false},
    {0, 0, 0.6, 0, false},

    // {7.18814, -4.13339, 2.9, 0, true},
    // {7.18814, -4.13339, 2.9, 0, false},

    {7.18814, 0.48393, 2.9, 0, true},
    {7.18814, 0.48393, 2.9, 0, false},

    {7.18814, 5.48069, 2.9, 0, true},
    {7.18814, 5.48069, 2.9, 0, false},

    {28, 3.50703, 2.9, -M_PI / 2, true},
    {28, 3.50703, 2.9, -M_PI / 2, false},

    {28, -1.97831, 0.94074, -M_PI, true},
    {28, -1.97831, 0.94074, -M_PI, false},

    {0, 0, 1, 0, true},
    {0, 0, 0.5, 0, false},

    {0, 0, 0, 0, false},
};
bool DroneDrive::inPosition(PosCmd p2) {
#define DIS_ABS (0.25)
#define YAW_ABS (5.0 / 180.0 * M_PI)
  geometry_msgs::Pose p1 = odom.pose.pose;
  double dis =
      sqrt(pow(p1.position.x - p2.x, 2) + pow(p1.position.y - p2.y, 2) +
           pow(p1.position.z - p2.z, 2));
  if (dis < DIS_ABS) {
    tf::Quaternion q(p1.orientation.x, p1.orientation.y, p1.orientation.z,
                     p1.orientation.w);
    double r, p, y;
    tf::Matrix3x3(q).getRPY(r, p, y);
    if (abs(y - p2.yaw) < YAW_ABS) {
      return true;
    } else {
      ROS_INFO("yaw error: %f",abs(y - p2.yaw));
    }
  } else {
    ROS_INFO("distance error: %f",dis);
  }
  return false;
}
void DroneDrive::pubWaypoint(PosCmd p) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.position.z = p.z;
  tf2::Quaternion q;
  q.setRPY(0, 0, p.yaw);
  q.normalize();
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();

  nav_msgs::Path path;
  path.poses.push_back(pose);
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  nav_pub.publish(path);
}

void DroneDrive::cmdPubTimerCb(const ros::TimerEvent &e) {
  static int waypoint_now = 0;
  if (exec_status == NONE) {
    return;
  }

  if (inPosition(waypoint_list[waypoint_now]) == true) {
    ++waypoint_now;
    ROS_INFO("waypoint x:%.1f, y:%.1f, z:%.1f, yaw:%.1f, plan_ctrl=%d",
             waypoint_list[waypoint_now].x, waypoint_list[waypoint_now].y,
             waypoint_list[waypoint_now].z, waypoint_list[waypoint_now].yaw,
             waypoint_list[waypoint_now].planner_ctrl);
    if (waypoint_list[waypoint_now].planner_ctrl == true) {
      pubWaypoint(waypoint_list[waypoint_now]);
      pos_cmd_update = false;
      return; // return wait for pos_cmd_update;
    }
  }

  if (waypoint_list[waypoint_now].planner_ctrl && !pos_cmd_update) {
    return;
  }
  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};
  if (waypoint_list[waypoint_now].planner_ctrl) {
    joint_cmd.position = {planner_pos_cmd.x, planner_pos_cmd.y,
                          planner_pos_cmd.z, planner_pos_cmd.yaw};
  } else {
    joint_cmd.position = {
        waypoint_list[waypoint_now].x, waypoint_list[waypoint_now].y,
        waypoint_list[waypoint_now].z, waypoint_list[waypoint_now].yaw};
  }
  joint_pub.publish(joint_cmd);
}

void DroneDrive::odomCb(const nav_msgs::OdometryConstPtr &msg) {
  odom = *msg;

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
