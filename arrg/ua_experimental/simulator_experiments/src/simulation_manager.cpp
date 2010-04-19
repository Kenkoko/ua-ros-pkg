#include "simulator_experiments/simulation_manager.h"

SimulationManager::SimulationManager() 
{
  // This will be the ID of the generated worlds
  next_server_id_ = 0;

  // Initial Gazebo Params
  //gazebo::Simulator::Instance()->SetGuiEnabled( optGuiEnabled );
  //gazebo::Simulator::Instance()->SetRenderEngineEnabled( optRenderEngineEnabled );

  std::cout << "ENTERING CREATE NODE" << std::endl;

  // Create the ROS node
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo_manager");
  
  node_ = new ros::NodeHandle();
  
  // Before we start Gazebo, let's make a ROS node and advertise some services
  //create_world_srv_ = node_->advertiseService("create_world", 
  //                                           &SimulationManager::createWorldHandler,
  //                                            this);
  start_world_srv_ = node_->advertiseService("start_world", 
                                             &SimulationManager::startWorldHandler,
                                             this);
  pause_world_srv_ = node_->advertiseService("pause_world", 
                                             &SimulationManager::pauseWorldHandler,
                                             this);
  reset_world_srv_ = node_->advertiseService("reset_world", 
                                             &SimulationManager::resetWorldHandler,
                                             this);
  ROS_INFO("Ready to create worlds.");
}

// Is this the best way to do this?
void SimulationManager::spin()
{
  ros::spin();
  std::cout << "END ROS SPIN" << std::endl;

  // Not sure about this
  gazebo::Simulator::Instance()->SetUserQuit();
}

bool SimulationManager::createWorldHandler(simulator_experiments::CreateWorld::Request &req,
                                           simulator_experiments::CreateWorld::Response &resp)
{
  //bool result = createWorld(req.world_file.c_str(), ++next_server_id_, req.start_paused);
  bool result = createWorld(req.world_file.c_str(), 0, req.start_paused);
  return result;
}

// This doesn't work
bool SimulationManager::createWorld(const char *world_file_name, int server_id, bool start_paused)
{
  //Load the simulator
  try
  {
    gazebo::Simulator::Instance()->Close();
    //gazebo::Simulator::Instance()->Load(world_file_name, server_id);
    //gazebo::Simulator::Instance()->SetTimeout(optTimeout);
    //gazebo::Simulator::Instance()->SetPhysicsEnabled(true);
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Error Loading Gazebo" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return false;
  }

  // Initialize the simulator
  try
  {
    //gazebo::Simulator::Instance()->Init();
    //gazebo::Simulator::Instance()->SetPaused(start_paused);
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Initialization failed" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return false;
  }

  return true;
}

bool SimulationManager::startWorldHandler(std_srvs::Empty::Request &req,
                                          std_srvs::Empty::Response &resp)
{
  ROS_INFO("Starting simulation...");
  gazebo::Simulator::Instance()->SetPaused(false);
  ROS_INFO("Simulation started.");

  return true;
}

bool SimulationManager::pauseWorldHandler(std_srvs::Empty::Request &req,
                                          std_srvs::Empty::Response &resp)
{
  std::cout << "HERE" << std::endl;
  ROS_INFO("Pausing simulation...");
  gazebo::Simulator::Instance()->SetPaused(true);
  ROS_INFO("Simulation paused.");

  return true;
}

// TODO: Why does this not reset all the way back to the world file?
// It does not recover deleted objects 
bool SimulationManager::resetWorldHandler(std_srvs::Empty::Request &req,
                                          std_srvs::Empty::Response &resp)
{
  std::cout << "HERE" << std::endl;
  ROS_INFO("Resetting simulation...");
  gazebo::World::Instance()->Reset();
  ROS_INFO("Simulation reset.");

  return true;
}













////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int /*dummy*/ )
{
  //TODO: use a boost::signal
  gazebo::Simulator::Instance()->SetUserQuit();
  return;
}

void startSimulationManager() 
{
  SimulationManager sm;
  sm.spin();
}

/////////////////////////////
// OTHER MAIN FUNCTION
int main(int argc, char **argv)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  boost::thread starter_thread(boost::bind(&startSimulationManager));

  const char* worldFileName = "/home/dhewlett/ros/ua-ros-pkg/arrg/ua_experimental/simulator_experiments/worlds/empty.world";

  gazebo::Simulator::Instance()->SetGuiEnabled(true);
  gazebo::Simulator::Instance()->SetRenderEngineEnabled(true);

  //Load the simulator
  try
  {
    gazebo::Simulator::Instance()->Load(worldFileName, 0);
    gazebo::Simulator::Instance()->SetTimeout(-1);
    gazebo::Simulator::Instance()->SetPhysicsEnabled(true);
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Error Loading Gazebo" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Initialize the simulator
  try
  {
    gazebo::Simulator::Instance()->Init();
    gazebo::Simulator::Instance()->SetPaused(false);
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Initialization failed" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Main loop of the simulator
  try
  {
    gazebo::Simulator::Instance()->MainLoop();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Main simulation loop failed" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Finalization and clean up
  try
  {
    gazebo::Simulator::Instance()->Fini();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Finalization failed" << std::endl;
    std::cerr << e << std::endl;
    return -1;
  }

  printf("Done.\n");
  return 0;
}



































































/*bool printWorldState(simulator_experiments::Pause::Request  &req,
                     simulator_experiments::Pause::Response &resp) 
{
  /// \bridf: list of all models in the world
  std::vector<gazebo::Model*> models;
  std::vector<gazebo::Model*>::iterator miter;

  /// \bridf: list of all bodies in the model
  const std::map<std::string,gazebo::Body*> *bodies;

  std::map<std::string,gazebo::Body*> all_bodies;
  all_bodies.clear();

  models = gazebo::World::Instance()->GetModels();

  for (miter = models.begin(); miter != models.end(); miter++) {
    std::vector<std::string> test;
    (*miter)->GetModelInterfaceNames(test);
    std::vector<std::string>::const_iterator iname_iter;
    for (iname_iter = test.begin(); iname_iter != test.end(); ++iname_iter) {
      std::cout << (*iname_iter) << std::endl;
    }
    
    bodies = (*miter)->GetBodies();
    
    // Iterate through all bodies
    std::map<std::string, gazebo::Body*>::const_iterator biter;
    for (biter=bodies->begin(); biter!=bodies->end(); biter++) {
        all_bodies.insert(make_pair(biter->first,biter->second));
    }
  }

  return true;
  }*/
