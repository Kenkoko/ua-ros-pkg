#ifndef GAZEBO_ROS_TEMPLATE2_HH
#define GAZEBO_ROS_TEMPLATE2_HH

#include <vector>
#include <string>

//#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Model.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo_plugins/GazeboModel.h>
#include <gazebo_plugins/SpawnModel.h>
#include <gazebo_plugins/DeleteModel.h>

#include <std_srvs/Empty.h>

namespace gazebo
{

class DeleteObjectPlugin : public Controller
{

public:
  DeleteObjectPlugin(Entity *parent);
  virtual ~DeleteObjectPlugin();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  ros::NodeHandle* rosnode_;
  ros::ServiceServer deleteModelService_;
  ros::ServiceServer deleteAllModelsService_;

  bool deleteModel(gazebo_plugins::DeleteModel::Request &req, gazebo_plugins::DeleteModel::Response &res);
  bool deleteAllModels(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  ros::CallbackQueue callback_queue_;
  void CallbackQueueThread();
  boost::thread* callback_queue_thread_;

  std::vector<std::string> white_list_;
};

}
#endif

