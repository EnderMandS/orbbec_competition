#include "drone_drive.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <cmath>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
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
  set_yaw_client = nh.serviceClient<std_srvs::Empty>("/set_yaw");
  take_picture_client = nh.serviceClient<std_srvs::Empty>("/take_picture");
}

// yaw convert to [-2PI, 2PI]
double yawConvert(double yaw_in) {
  static int loop_cnt = 0;
  static double last_yaw = yaw_in;
  double out_yaw = 0;
  if (abs(yaw_in - last_yaw) > 1.5 * M_PI) {
    if (yaw_in < 0) {
      ++loop_cnt;
    } else {
      --loop_cnt;
    }
  }
  last_yaw = yaw_in;
  if (loop_cnt > 0) {
    out_yaw = yaw_in + 2 * M_PI;
  } else if (loop_cnt < 0) {
    out_yaw = yaw_in - 2 * M_PI;
  } else {
    out_yaw = yaw_in;
  }
  if (out_yaw > 2 * M_PI) {
    --loop_cnt;
    out_yaw = yaw_in;
  } else if (out_yaw < -2 * M_PI) {
    ++loop_cnt;
    out_yaw = yaw_in;
  }
  return out_yaw;
}

bool pos_cmd_update = false;
void DroneDrive::positionCommandCb(
    const drone_msgs::PositionCommandConstPtr &msg) {
  planner_pos_cmd.x = msg->position.x;
  planner_pos_cmd.y = msg->position.y;
  planner_pos_cmd.z = msg->position.z;
  planner_pos_cmd.yaw = msg->yaw;
  pos_cmd_update = true;
}

void DroneDrive::execStatusCb(const drone_msgs::ExecStatusConstPtr &msg) {
  exec_status = msg->exec_flag;
}

const PosCmd waypoint_list[] = {
    {0, 0, 0, 0, false},
    {0, 0, 0.6, 0, false},

    {7.18814, -4.13339, 2.9, 0, true},
    {7.18814, -4.13339, 2.9, 0, false, true},

    {7.18814, 0.48393, 2.9, 0, true},
    {7.18814, 0.48393, 2.9, 0, false, true},

    {7.18814, 5.48069, 2.9, 0, true},
    {7.18814, 5.48069, 2.9, 0, false, true},

    {28, 3.50703, 2.9, -M_PI / 2, true},

    {28, -1.97831, 0.94074, -M_PI, true},

    {0, 0, 1, 0, true},
    {0, 0, 0.5, 0, false},

    {0, 0, 0, 0, false},
};

bool DroneDrive::inPosition(PosCmd p2) {
#define DIS_ABS (0.2)

  geometry_msgs::Pose p1 = odom.pose.pose;
  double dis =
      sqrt(pow(p1.position.x - p2.x, 2) + pow(p1.position.y - p2.y, 2) +
           pow(p1.position.z - p2.z, 2));

  if (dis < DIS_ABS) {
    return true;
  }

  // ROS_INFO("distance error: %f", dis);
  // ROS_INFO("waypoint x:%.1f, y:%.1f, z:%.1f, plan_ctrl=%d", p2.x, p2.y, p2.z,
  //          p2.planner_ctrl);
  // ROS_INFO("odom x:%.1f, y:%.1f, z:%.1f", p1.position.x, p1.position.y,
  //          p1.position.z);

  return false;
}

bool DroneDrive::inYaw(PosCmd p2) {
#define YAW_ABS (5.0 / 180.0 * M_PI)

  geometry_msgs::Pose p1 = odom.pose.pose;
  tf::Quaternion q(p1.orientation.x, p1.orientation.y, p1.orientation.z,
                   p1.orientation.w);
  double r, p, y;
  tf::Matrix3x3(q).getRPY(r, p, y);

  if (abs(y - p2.yaw) < YAW_ABS) {
    return true;
  }

  // ROS_INFO("yaw error: %.1f, now:%.1f, expect:%.1f", abs(y - p2.yaw), y,
  //          p2.yaw);
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
  static bool finish = false;
  if (exec_status == NONE || finish) {
    return;
  }

  // traj complete?
  if (inPosition(waypoint_list[waypoint_now]) == true) { // arrived?
    bool next = false, take_picture = false;
    if (waypoint_list[waypoint_now].planner_ctrl ==
        false) {                       // planner or manual
      if (smooth_pos.trajComplete()) { // manual complete
        next = true;
      }
    } else if (exec_status == WAIT_TARGET) { // planner complete
      next = true;
    }
    if (next) { // current traj complete, execute next
      if (waypoint_list[waypoint_now].picture) { // take picture?
        std_srvs::Empty srv;
        if (!take_picture_client.call(srv)) {
          ROS_WARN("Take picture fail at x:%.1f, y:%.1f, z:%.1f. Go on.",
                   waypoint_list[waypoint_now].x, waypoint_list[waypoint_now].y,
                   waypoint_list[waypoint_now].z);
        }
      }

      ++waypoint_now;
      if (waypoint_now >= sizeof(waypoint_list) / sizeof(PosCmd)) {
        ROS_INFO("All waypoint published. Complete.");
        finish = true;
        cmd_pub_timer.stop();
        return;
      }
      ROS_INFO("new waypoint x:%.1f, y:%.1f, z:%.1f, yaw:%.1f, plan_ctrl=%d",
               waypoint_list[waypoint_now].x, waypoint_list[waypoint_now].y,
               waypoint_list[waypoint_now].z, waypoint_list[waypoint_now].yaw,
               waypoint_list[waypoint_now].planner_ctrl);
      if (waypoint_list[waypoint_now].planner_ctrl == true) { // next planner
        pos_cmd_update = false; // clean flag of planner control command receive
        if (waypoint_list[waypoint_now - 1].planner_ctrl == false) {
          std_srvs::Empty srv;
          if (!set_yaw_client.call(srv)) {
            ROS_WARN("Set yaw fail!");
          }
          ROS_INFO("Reseted yaw.");
        }
        pubWaypoint(waypoint_list[waypoint_now]);
        return; // return wait for pos_cmd_update;
      } else {  // next manual
        smooth_pos.genNew(waypoint_list[waypoint_now - 1],
                          waypoint_list[waypoint_now]);
      }
    }
  }

  // make sure receive planner contorl command
  if (waypoint_list[waypoint_now].planner_ctrl == true) {
    if (!pos_cmd_update) {
      return;
    } else {
      pos_cmd_update = false;
    }
  }

  // publish joint control command
  sensor_msgs::JointState joint_cmd;
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};
  if (waypoint_list[waypoint_now].planner_ctrl) { // planner
    joint_cmd.position = {planner_pos_cmd.x, planner_pos_cmd.y,
                          planner_pos_cmd.z, yawConvert(planner_pos_cmd.yaw)};
  } else { // manual
    joint_cmd.position = smooth_pos.getPosNow();
  }
  joint_cmd.header.stamp = ros::Time::now();
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

void SmoothTraj::genNew(PosCmd start, PosCmd end) {
  start_time = ros::Time::now().toSec();
  start_pos = start;
  end_pos = end;
}
std::vector<double> SmoothTraj::getPosNow(void) {
  if (trajComplete()) {
    return {end_pos.x, end_pos.y, end_pos.z, yawConvert(end_pos.yaw)};
  }
  double scale = (ros::Time::now().toSec() - start_time) / TRAJ_TIME;
  double x, y, z, yaw;
  x = scale * (end_pos.x - start_pos.x) + start_pos.x;
  y = scale * (end_pos.y - start_pos.y) + start_pos.y;
  z = scale * (end_pos.z - start_pos.z) + start_pos.z;
  yaw = scale * (end_pos.yaw - start_pos.yaw) + start_pos.yaw;
  yaw = yawConvert(yaw);
  return {x, y, z, yaw};
}
bool SmoothTraj::trajComplete(void) {
  if (ros::Time::now().toSec() >= start_time + TRAJ_TIME) {
    return true;
  }
  return false;
}
