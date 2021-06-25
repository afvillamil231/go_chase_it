#pragma once

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"

namespace ball_chaser
{
class DriveBot
{
public:
  DriveBot();
  ~DriveBot() = default;

private:
  bool commandRobot(DriveToTarget::Request& req, DriveToTarget::Response& res);

  ros::NodeHandle n_;
  ros::Publisher motor_command_publisher_;
  ros::ServiceServer command_robot_service_;
};

}  // namespace ball_chaser