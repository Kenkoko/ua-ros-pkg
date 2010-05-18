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

public:
  AddObjectPlugin(Entity *parent);
  virtual ~AddObjectPlugin();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  ros::NodeHandle* rosnode_;
  ros::ServiceServer addModelService_;

  bool isGazeboModelXML(std::string robot_model);

  bool addModel(gazebo_plugins::SpawnModel::Request &req, gazebo_plugins::SpawnModel::Response &res);

  ros::CallbackQueue callback_queue_;
  void CallbackQueueThread();
  boost::thread* callback_queue_thread_;
};

}
#endif

