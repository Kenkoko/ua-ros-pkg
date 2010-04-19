#ifndef SIMULATION_MANAGER_H
#define SIMULATION_MANAGER_H

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <boost/thread.hpp>

#include <std_srvs/Empty.h>

#include <gazebo/Simulator.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/Global.hh>
#include <gazebo/World.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>

#include "ros/ros.h"

#include "simulator_experiments/CreateWorld.h"

class SimulationManager {
public:
  SimulationManager();
  void spin();

  bool createWorldHandler(simulator_experiments::CreateWorld::Request &req,
                          simulator_experiments::CreateWorld::Response &resp);

  bool createWorld(const char *world_file_name, int server_id, bool start_paused);

  bool startWorldHandler(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &resp);

  bool pauseWorldHandler(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &resp);

  bool resetWorldHandler(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &resp);

private:
  

  // Fields
  int next_server_id_;

  // The ROS node
  ros::NodeHandle *node_;

  // ROS Services
  ros::ServiceServer create_world_srv_, delete_world_srv_;
  ros::ServiceServer start_world_srv_, pause_world_srv_, reset_world_srv_;
  
  //ros::ServiceServer pause_service_, start_service_, print_service_;

  // Other fields, representing the default values from main.cc
  static const bool optServerForce = true;
  static const bool optGuiEnabled = true;
  static const bool optRenderEngineEnabled = true;
  static const double optTimeout = -1;
  static const unsigned int optMsgLevel = 1;
  static const int optTimeControl = 1;
  //const bool optPhysicsEnabled  = true;
  static const bool optPaused = false;
};

#endif
