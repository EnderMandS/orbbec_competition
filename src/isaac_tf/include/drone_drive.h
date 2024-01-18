#ifndef __DRONE_DRIVE_H
#define __DRONE_DRIVE_H

#include "isaac_tf/PositionCommand.h"
#include <ros/ros.h>

namespace Drone {

class DroneDrive {
public:
  DroneDrive(){};
  void init(ros::NodeHandle &);

private:
  ros::Subscriber posititon_cmd_sub;
  ros::Publisher joint_pub;

  isaac_tf::PositionCommand posititon_cmd;

  void positionCommandCb(const isaac_tf::PositionCommandConstPtr &);
};

} // namespace Drone

#endif
