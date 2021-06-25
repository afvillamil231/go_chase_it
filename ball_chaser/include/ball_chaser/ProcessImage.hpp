#pragma once

#include <sensor_msgs/Image.h>

#include <vector>

#include "ros/ros.h"

namespace ball_chaser
{
class ProcessImage
{
public:
  ProcessImage();
  ~ProcessImage() = default;

private:
  void processImageCallback(const sensor_msgs::Image img);
  void sendVelocities(const float& linear_x, const float& angular_z);
  std::vector<size_t> getLongestWhiteRow(const sensor_msgs::Image& img);
  std::vector<size_t> getWhitePixels(const size_t& current_row, const sensor_msgs::Image& img);

  ros::NodeHandle n_;
  ros::Subscriber camera_subscriber_;
  ros::ServiceClient command_robot_client_;

  const size_t CENTER_REGION_{ 500 };
  const uint8_t WHITE_PIXEL_{ 255 };
  const float ROW_PECENTAGE_{ 0.5 };
  const float SPEED_{ 0.5 };
  const float K_{ 0.0005 };
};

}  // namespace ball_chaser