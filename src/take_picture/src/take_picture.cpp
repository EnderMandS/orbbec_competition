#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <std_srvs/Empty.h>

std::string image_save_path = "/home/m/code/ros_ws/image/";
sensor_msgs::ImageConstPtr camera_msg = nullptr;

bool takePictureCb(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
void imageCb(const sensor_msgs::ImageConstPtr &);

int main(int argc, char **argv) {
  ros::init(argc, argv, "take_picture_server");
  ros::NodeHandle nh;

  nh.getParam("image_save_path", image_save_path);

  ros::Subscriber image_sub = nh.subscribe("/camera_rgb", 1, imageCb);
  ros::ServiceServer server =
      nh.advertiseService("/take_picture", takePictureCb);

  if (image_sub.getNumPublishers() < 1) {
    ROS_INFO("Wait for topic /camera_rgb");
    ros::Duration(2.0).sleep();
  }

  ROS_INFO("Take picture server start. Image will save to: %s",
           image_save_path.c_str());
  ros::spin();
  return 0;
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
  std::string filename = image_save_path + ss.str() + ".png";

  cv::imwrite(filename, bgr_image);
  ROS_INFO("Image saved to %s", filename.c_str());

  return true;
}

void imageCb(const sensor_msgs::ImageConstPtr &msg) { camera_msg = msg; }
