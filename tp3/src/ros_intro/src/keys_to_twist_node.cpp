#include <ros/ros.h>
#include "keys_to_twist.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keys_to_twist");
  ros::NodeHandle n;
  robmovil::KeysToTwist keys_to_twist(n);
  ros::spin();
  return 0;
}
