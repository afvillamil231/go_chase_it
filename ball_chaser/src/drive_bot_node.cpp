
#include "ball_chaser/DriveBot.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_bot");
  ball_chaser::DriveBot drive_bot;
  ros::spin();
  return 0;
}