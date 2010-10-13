/*
 * environment_node.cpp
 *
 *  Created on: Oct 11, 2010
 *      Author: dhewlett
 */

#include <ros/ros.h>
#include <wubble_mdp/environment.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment");
  ros::NodeHandle nh;

  Environment env(nh);

  ros::spin();

  return 0;
}
