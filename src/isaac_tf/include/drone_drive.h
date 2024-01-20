#ifndef __DRONE_DRIVE_H
#define __DRONE_DRIVE_H

#include "drone_msgs/PositionCommand.h"
#include <ros/ros.h>

namespace Drone {

class DroneDrive {
public:
  DroneDrive(){};
  void init(ros::NodeHandle &);

private:
  ros::Subscriber posititon_cmd_sub;
  ros::Publisher joint_pub;
  ros::Timer test_timer;

  drone_msgs::PositionCommand posititon_cmd;

  void positionCommandCb(const drone_msgs::PositionCommandConstPtr &);
  void testTimerCb(const ros::TimerEvent&);
};

} // namespace Drone

#endif
