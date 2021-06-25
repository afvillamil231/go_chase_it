
#include "ball_chaser/ProcessImage.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_image");
  ball_chaser::ProcessImage process_image;
  ros::spin();
  return 0;
}