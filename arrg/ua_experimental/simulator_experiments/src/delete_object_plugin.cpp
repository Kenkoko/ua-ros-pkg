#include <algorithm>
#include <assert.h>

#include <simulator_experiments/delete_object_plugin.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Simulator.hh>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gazebo;
using namespace boost::assign;

GZ_REGISTER_DYNAMIC_CONTROLLER("delete_object_plugin", DeleteObjectPlugin)
;

DeleteObjectPlugin::DeleteObjectPlugin(Entity *parent) :
  Controller(parent)
{
  // TODO: Read the whitelist from a param, unless we want to solve this a different way
  white_list_ += "clock", "point_white", "gplane"; // Cool boost assignment

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo_delete_object_node", ros::init_options::NoSigintHandler
      | ros::init_options::AnonymousName);

  this->rosnode_ = new ros::NodeHandle();

  ros::AdvertiseServiceOptions delete_aso =
      ros::AdvertiseServiceOptions::create<gazebo_plugins::DeleteModel>("delete_model",
                                                                        boost::bind(&DeleteObjectPlugin::deleteModel,
                                                                                    this, _1, _2), ros::VoidPtr(),
                                                                        &this->callback_queue_);
  this->deleteModelService_ = this->rosnode_->advertiseService(delete_aso);

  ros::AdvertiseServiceOptions delete_all_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>("delete_all_models",
                                                            boost::bind(&DeleteObjectPlugin::deleteAllModels, this, _1,
                                                                        _2), ros::VoidPtr(), &this->callback_queue_);
  this->deleteAllModelsService_ = this->rosnode_->advertiseService(delete_all_aso);
}

DeleteObjectPlugin::~DeleteObjectPlugin()
{
  delete rosnode_;
}

bool DeleteObjectPlugin::deleteModel(gazebo_plugins::DeleteModel::Request &req,
                                     gazebo_plugins::DeleteModel::Response &res)
{
  ROS_INFO("Entering deleteModel");

  // If the model doesn't exist, don't delete it.
  if (!gazebo::World::Instance()->GetModelByName(req.model_name))
  {
    res.success = false;
    res.status_message = string("Cannot delete non-existent model");
    return true;
  }
  else
  {
    // Acquire the deletion lock from Gazebo...
    boost::recursive_mutex::scoped_lock delete_lock(*Simulator::Instance()->GetMDMutex());
    gazebo::World::Instance()->DeleteEntity(req.model_name.c_str());
    // ... and release it here
  }

  // Moved outside of the "else" so that the lock is surrendered, preventing deadlock
  while (gazebo::World::Instance()->GetModelByName(req.model_name))
  {
    ROS_INFO("Waiting for model deletion (%s)", req.model_name.c_str());
    usleep(1000);
  }

  res.success = true;
  res.status_message = string("Successfully deleted model");

  ROS_INFO("Exiting deleteModel");

  return true;
}

bool DeleteObjectPlugin::deleteAllModels(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Entering deleteAllModels");

  vector<string> deleted_models;
  { // Dummy block so that scoped lock is released before waiting
    boost::recursive_mutex::scoped_lock delete_lock(*Simulator::Instance()->GetMDMutex());

    vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();

    for (vector<gazebo::Model*>::iterator miter = models.begin(); miter != models.end(); miter++)
    {
      string name = string((*miter)->GetName());

      vector<string>::iterator result = find(this->white_list_.begin(), this->white_list_.end(), name);
      if (result == this->white_list_.end())
      {
        ROS_INFO("model %s is not on whitelist, deleting it.", name.c_str());
        gazebo::World::Instance()->DeleteEntity(name.c_str());
        deleted_models.push_back(name);
      } else {
        ROS_INFO("model %s is on whitelist, not deleting it.", name.c_str());
      }
    }
  }

  bool allGone = false;
  while (!allGone)
  {
    allGone = true;
    for (vector<string>::iterator it = deleted_models.begin(); it != deleted_models.end(); ++it)
    {
      if (gazebo::World::Instance()->GetModelByName(*it))
      {
        allGone = false;
        break;
      }
    }

    ROS_INFO("Waiting for all models to be deleted.");
    usleep(1000);
  }

  ROS_INFO("Exiting deleteAllModels");

  return true;
}

void DeleteObjectPlugin::CallbackQueueThread()
{
  static const double timeout = 0.1;

  while (this->rosnode_->ok())
  {
    this->callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void DeleteObjectPlugin::LoadChild(XMLConfigNode *node)
{
}

void DeleteObjectPlugin::InitChild()
{
  this->callback_queue_thread_ = new boost::thread(boost::bind(&DeleteObjectPlugin::CallbackQueueThread, this));
}

void DeleteObjectPlugin::UpdateChild()
{
}

void DeleteObjectPlugin::FiniChild()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_->join();
}
