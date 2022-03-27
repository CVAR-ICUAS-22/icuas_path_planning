#include "laserscan_to_map.hpp"
#include "ros/ros.h"
// #include "ros/init.h"

// #include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_to_map_node");

  LaserscanToMap node;

  ros::Rate rate(100);
  while (ros::ok())
  {
    // updating all the ros msgs
    ros::spinOnce();
    // running the node
    node.run();
    rate.sleep();
  }
  ros::shutdown();

  return 0;
}