#include "ball_chaser/DriveToTarget.h"
#include "ball_chaser/ProcessImage.hpp"
#include "geometry_msgs/Twist.h"

using namespace ball_chaser;

ProcessImage::ProcessImage() : n_()
{
  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  camera_subscriber_ = n_.subscribe("camera/rgb/image_raw", 10, &ProcessImage::processImageCallback, this);

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  command_robot_client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
}

void ProcessImage::processImageCallback(const sensor_msgs::Image img)
{
  // Get the longest row of white pixels
  std::vector<size_t> longest_row{ getLongestWhiteRow(img) };
  // If couldn't find any white pixel or the robot is close to the ball then
  // stop the robot
  if (longest_row.empty() || longest_row.size() > ROW_PECENTAGE_ * img.step)
  {
    sendVelocities(0.0, 0.0);
    return;
  }
  // Get the center pixel of the row
  size_t center_pixel = longest_row[longest_row.size() / 2 - 1] % img.step;
  size_t center_col{ img.step / 2 };
  int error = center_col - center_pixel;
  float angular = -K_ * error;
  sendVelocities(SPEED_, angular);
}

std::vector<size_t> ProcessImage::getLongestWhiteRow(const sensor_msgs::Image& img)
{
  size_t current_row{ 0 };
  std::vector<size_t> previous_row{};
  // Loop through each pixel in the image and get the longest white row
  while (current_row < img.height)
  {
    std::vector<size_t> longest_row{ getWhitePixels(current_row, img) };
    // If the current row has less white pixels than the previous one, then the previous one is the center of the ball
    if (longest_row.size() < previous_row.size())
    {
      break;
    }
    previous_row = std::move(longest_row);
    longest_row.clear();
    current_row++;
  }
  return previous_row;
}

std::vector<size_t> ProcessImage::getWhitePixels(const size_t& current_row, const sensor_msgs::Image& img)
{
  std::vector<size_t> white_pixels{};
  // Iterate over the current row to get its number of white pixels
  for (size_t i = 0; i < img.step; i++)
  {
    size_t pos = i + current_row * img.step;
    if (img.data[pos] == WHITE_PIXEL_)
    {
      white_pixels.push_back(pos);
    }
    else if (pos > 0 && img.data[pos - 1] == WHITE_PIXEL_)
    {
      // if it founds a non white pixel after a white pixel, the ball row is over
      break;
    }
  }
  return white_pixels;
}

void ProcessImage::sendVelocities(const float& linear_x, const float& angular_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = linear_x;
  srv.request.angular_z = angular_z;
  // Call the safe_move service and pass the requested joint angles
  if (!command_robot_client_.call(srv))
    ROS_ERROR("Failed to call service command_robot");
}