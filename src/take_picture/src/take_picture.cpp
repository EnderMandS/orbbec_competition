#include "nav_msgs/Odometry.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "tf/LinearMath/Matrix3x3.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <string>
#include <sys/stat.h>
#include <tf/tf.h>

std::string image_save_path = "/home/m/code/ros_ws/image/";
std::string video_save_path = "/home/m/code/ros_ws/video/";
sensor_msgs::ImageConstPtr camera_msg = nullptr;
nav_msgs::OdometryConstPtr odom = nullptr;

cv::VideoWriter output_video;
bool recording = false;

bool takePictureCb(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
bool startRecord(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
bool stopRecord(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
void imageCb(const sensor_msgs::ImageConstPtr &);
void odomCb(const nav_msgs::OdometryConstPtr &);
bool pathExist(std::string);

int main(int argc, char **argv) {
  ros::init(argc, argv, "take_picture_server");
  ros::NodeHandle nh;

  // get param from launch
  nh.getParam("image_save_path", image_save_path);
  nh.getParam("video_save_path", video_save_path);

  // path exist?
  if (!pathExist(image_save_path)) {
    ROS_ERROR("Path %s not exist. Please create directory first.",
              image_save_path.c_str());
    return -1;
  }
  if (!pathExist(video_save_path)) {
    ROS_ERROR("Path %s not exist. Please create directory first.",
              video_save_path.c_str());
    return -1;
  }

  // ros relate
  ros::Subscriber image_sub = nh.subscribe("/camera_rgb", 1, imageCb);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCb);
  ros::ServiceServer picture_server =
      nh.advertiseService("/take_picture", takePictureCb);
  ros::ServiceServer start_record_server =
      nh.advertiseService("/start_record", startRecord);
  ros::ServiceServer stop_record_server =
      nh.advertiseService("/stop_record", stopRecord);

  if (image_sub.getNumPublishers() < 1) {
    ROS_INFO("Wait for topic /camera_rgb");
    ros::Duration(2.0).sleep();
  }

  ROS_INFO("Take picture server start. Image will save to: %s",
           image_save_path.c_str());
  ROS_INFO("Video will save to: %s", video_save_path.c_str());
  ros::spin();
  return 0;
}

bool pathExist(std::string path) {
  struct stat info;
  if (stat(path.c_str(), &info) != 0) {
    return false;
  } else if (info.st_mode & S_IFDIR) {
    return true;
  }
  return false;
}

bool takePictureCb(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  if (camera_msg == nullptr) {
    ROS_ERROR("No camera message.");
    return false;
  }

  // conver sensor_msgs to opencv
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(camera_msg, camera_msg->encoding);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  cv::Mat bgr_image;
  cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

  // generate filename from time
  auto now_c =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
  std::string filename = image_save_path + ss.str() + ".jpg";

  cv::imwrite(filename, bgr_image);
  ROS_INFO("Image saved to %s", filename.c_str());

  return true;
}
bool startRecord(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  if (!recording) {
    auto now_c =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    std::string filename = video_save_path + ss.str() + ".mp4";
    int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
    output_video.open(filename, fourcc, 30.0, cv::Size(1280, 720), true);
    if (!output_video.isOpened()) {
      ROS_ERROR("Unable to open the video for write.");
      return false;
    }
    ROS_INFO("Start recording video.");
    recording = true;
    return true;
  } else {
    ROS_WARN("Video already in recording.");
    return false;
  }
}
bool stopRecord(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  if (recording) {
    ROS_INFO("Stop recording video.");
    ROS_INFO("Video save to %s", video_save_path.c_str());
    recording = false;
    output_video.release();
    return true;
  }
  ROS_WARN("Video already not at recording.");
  return false;
}

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
  camera_msg = msg;

  // record video
  if (recording) {
    // conver sensor_msgs to opencv
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat bgr_image;
    cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
    output_video.write(bgr_image);
  }

  //   static int cnt = 0;
  //   if (odom == nullptr) {
  //     ROS_WARN("No odom message.");
  //     return;
  //   }
  //   static nav_msgs::Odometry odom_last = *odom;

  //   // motion enough?
  //   bool save = false;
  //   if (cnt == 0) {
  //     save = true;
  //   } else {
  //     float distance_2 =
  //         pow(odom->pose.pose.position.x - odom_last.pose.pose.position.x, 2)
  //         + pow(odom->pose.pose.position.y - odom_last.pose.pose.position.y,
  //         2) + pow(odom->pose.pose.position.z -
  //         odom_last.pose.pose.position.z, 2);
  //     tf::Quaternion q(
  //         odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
  //         odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  //     tf::Quaternion q_last(
  //         odom_last.pose.pose.orientation.x,
  //         odom_last.pose.pose.orientation.y,
  //         odom_last.pose.pose.orientation.z,
  //         odom_last.pose.pose.orientation.w);
  //     double r, r_l, p, p_l, y, y_l;
  //     tf::Matrix3x3(q).getRPY(r, p, y);
  //     tf::Matrix3x3(q_last).getRPY(r_l, p_l, y_l);
  //     float dr = abs(r - r_l), dp = abs(p - p_l), dy = abs(y - y_l);

  // #define DISTANCE (0.2)
  // #define ANGLE ((float)(5.f / 180.f * M_PI))
  //     if (distance_2 > pow(DISTANCE, 2) || dr > ANGLE || dp > ANGLE ||
  //         dy > ANGLE) {
  //       save = true;
  //     }
  //   }
  //   if (!save) {
  //     return;
  //   }
  //   odom_last = *odom;

  //   // conver sensor_msgs to opencv
  //   cv_bridge::CvImagePtr cv_ptr;
  //   try {
  //     cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  //   } catch (cv_bridge::Exception &e) {
  //     ROS_ERROR("cv_bridge exception: %s", e.what());
  //     return;
  //   }
  //   cv::Mat bgr_image;
  //   cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

  //   std::ostringstream seq_name;
  //   seq_name << std::setw(4) << std::setfill('0') << cnt;
  //   std::string filename = image_nerf_path + seq_name.str() + ".jpg";
  //   cv::imwrite(filename, bgr_image);
  //   ROS_INFO("Image saved to %s", filename.c_str());
  //   ++cnt;
}
void odomCb(const nav_msgs::OdometryConstPtr &msg) { odom = msg; }
