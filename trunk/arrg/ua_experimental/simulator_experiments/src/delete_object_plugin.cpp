#include <algorithm>
#include <assert.h>

#include <simulator_experiments/delete_object_plugin.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("delete_object_plugin", DeleteObjectPlugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
DeleteObjectPlugin::DeleteObjectPlugin(Entity *parent)
    : Controller(parent)
{
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_delete_object_node", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    
    this->rosnode_ = new ros::NodeHandle();

    ros::AdvertiseServiceOptions delete_aso = ros::AdvertiseServiceOptions::create<gazebo_plugins::DeleteModel>(
        "delete_model", 
        boost::bind( &DeleteObjectPlugin::deleteModel, this, _1, _2 ), 
        ros::VoidPtr(), 
        &this->callback_queue_);
    this->deleteModelService_ = this->rosnode_->advertiseService(delete_aso);

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DeleteObjectPlugin::~DeleteObjectPlugin()
{
    // TODO: Probably should have a destructor!
}

bool DeleteObjectPlugin::deleteModel(gazebo_plugins::DeleteModel::Request &req,
                                    gazebo_plugins::DeleteModel::Response &res) 
{
    ROS_INFO("Entering deleteModel");

    // Test if the model exists
    if (!gazebo::World::Instance()->GetModelByName(req.model_name)) {
        res.success = false;
        res.status_message = std::string("Cannot delete non-existent model");
        
    } else {
        // Delete the model from the world
        gazebo::World::Instance()->DeleteEntity(req.model_name.c_str());

        while (gazebo::World::Instance()->GetModelByName(req.model_name))
        {
            ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
            usleep(1000);
        }

        // set result
        res.success = true;
        res.status_message = std::string("successfully deleted model");
    }

    return 1;
}

void DeleteObjectPlugin::CallbackQueueThread()
{
  static const double timeout = 0.1;

  while (this->rosnode_->ok())
  {
    this->callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}



////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DeleteObjectPlugin::LoadChild(XMLConfigNode *node)
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void DeleteObjectPlugin::InitChild()
{
    this->callback_queue_thread_ = new boost::thread( boost::bind( &DeleteObjectPlugin::CallbackQueueThread, this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void DeleteObjectPlugin::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void DeleteObjectPlugin::FiniChild()
{
    this->rosnode_->shutdown();
    this->callback_queue_thread_->join();
}



