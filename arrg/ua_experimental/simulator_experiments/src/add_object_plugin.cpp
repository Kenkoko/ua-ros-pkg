#include <algorithm>
#include <assert.h>

#include <simulator_experiments/add_object_plugin.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("add_object_plugin", AddObjectPlugin);

AddObjectPlugin::AddObjectPlugin(Entity *parent)
    : Controller(parent)
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_add_object_node", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    
    this->rosnode_ = new ros::NodeHandle();

    ros::AdvertiseServiceOptions delete_aso = ros::AdvertiseServiceOptions::create<gazebo_plugins::SpawnModel>(
        "add_model", 
        boost::bind( &AddObjectPlugin::addModel, this, _1, _2 ), 
        ros::VoidPtr(), 
        &this->callback_queue_);
    this->addModelService_ = this->rosnode_->advertiseService(delete_aso);

}

AddObjectPlugin::~AddObjectPlugin()
{
}

// utilites for checking incoming string
bool AddObjectPlugin::isGazeboModelXML(std::string robot_model)
{
  ROS_INFO("%s\n", robot_model.c_str());
  TiXmlDocument doc_in;
  doc_in.Parse(robot_model.c_str());
  if (doc_in.FirstChild("model:physical"))
    return true;
  else
    return false;
}

// Service for spawning models in Gazebo
bool AddObjectPlugin::addModel(gazebo_plugins::SpawnModel::Request &req,
                               gazebo_plugins::SpawnModel::Response &res)
{

  ROS_INFO("Entering addModel");

  // Test if the model exists
  if (!gazebo::World::Instance()->GetModelByName(req.model.model_name)) {
    if (req.model.xml_type != req.model.GAZEBO_XML) {
      res.success = false;
      res.status_message = std::string("This service only accepts GAZEBO_XML.");
      return true;
    }
  } else {
    res.success = false;
    res.status_message = std::string("Cannot create a model that already exists.");
    return true;
  }

  // get namespace for the corresponding model plugins
  std::string robot_namespace = req.model.robot_namespace;

  // get model initial pose
  geometry_msgs::Pose initial_pose = req.model.initial_pose;

  std::string robot_model = req.model.robot_model; // incoming robot model string
 
  if (req.model.xml_type == req.model.GAZEBO_XML)
  {
    // incoming robot model string is a string containing a Gazebo Model XML
    // perform simple XML check
    if (!this->isGazeboModelXML(robot_model))
    {
      ROS_ERROR("input Gazebo Model XML must begin with <model:physical>\n");
      return false;
    }
  }

  ROS_INFO("Inserting Entity (Model name is [%s])", req.model.model_name.c_str());
  gazebo::World::Instance()->InsertEntity(robot_model);
  
  // Wait for model to be created
  while (!gazebo::World::Instance()->GetModelByName(req.model.model_name))
  {
    ROS_INFO("Waiting for model creation (%s)", req.model.model_name.c_str());
    usleep(1000);
  }

  res.success = true;
  res.status_message = std::string("Model successfully created.");

  return true;
}

void AddObjectPlugin::CallbackQueueThread()
{
  static const double timeout = 0.1;

  while (this->rosnode_->ok())
  {
    this->callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}


// Load the controller
void AddObjectPlugin::LoadChild(XMLConfigNode *node)
{
}

// Initialize the controller
void AddObjectPlugin::InitChild()
{
    this->callback_queue_thread_ = new boost::thread( boost::bind( &AddObjectPlugin::CallbackQueueThread, this ) );
}

// Update the controller
void AddObjectPlugin::UpdateChild()
{
}

// Finalize the controller
void AddObjectPlugin::FiniChild()
{
    this->rosnode_->shutdown();
    this->callback_queue_thread_->join();
}
