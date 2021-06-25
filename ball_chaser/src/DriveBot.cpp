#include "ball_chaser/DriveBot.hpp"
#include "geometry_msgs/Twist.h"

using namespace ball_chaser;

DriveBot::DriveBot() : n_()
{
  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic
  // with a publishing queue size of 10
  motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a safe_move service with a handle_safe_move_request callback function
  command_robot_service_ = n_.advertiseService("/ball_chaser/command_robot", &DriveBot::commandRobot, this);
}

bool DriveBot::commandRobot(DriveToTarget::Request& req, DriveToTarget::Response& res)
{
  float linear_x = (float)req.linear_x;
  float angular_z = (float)req.angular_z;
  ROS_INFO("Speed request received - linear_x:%1.2f, angular_z:%1.2f", linear_x, angular_z);
  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  // Set wheel velocities, forward [0.5, 0.0]
  motor_command.linear.x = linear_x;
  motor_command.angular.z = angular_z;
  // Publish angles to drive the robot
  motor_command_publisher_.publish(motor_command);

  // Return a response message
  res.msg_feedback =
      "Speed sent - linear_x: " + std::to_string(linear_x) + " , angular_z: " + std::to_string(angular_z);
  ROS_INFO_STREAM(res.msg_feedback);
  return true;
}
