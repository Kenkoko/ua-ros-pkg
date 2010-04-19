#ifndef GAZEBO_ROS_TEMPLATE2_HH
#define GAZEBO_ROS_TEMPLATE2_HH

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Model.hh>

#include <tinyxml/tinyxml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo_plugins/GazeboModel.h>
#include <gazebo_plugins/SpawnModel.h>

namespace gazebo
{

class AddObjectPlugin : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: AddObjectPlugin(Entity *parent);

  /// \brief Destructor
  public: virtual ~AddObjectPlugin();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  /// \brief ros service
  private: ros::ServiceServer addModelService_;

  private: bool isGazeboModelXML(std::string robot_model);

  /// \brief ros service call to delete model in Gazebo
  private: bool addModel(gazebo_plugins::SpawnModel::Request &req,
                         gazebo_plugins::SpawnModel::Response &res);

  /// \brief use custom callback queue
  private: ros::CallbackQueue callback_queue_;
  private: void CallbackQueueThread();
  private: boost::thread* callback_queue_thread_;

};

}
#endif

