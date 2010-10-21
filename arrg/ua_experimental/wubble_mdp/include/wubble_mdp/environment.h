/*
 * environment.h
 *
 *  Created on: Oct 9, 2010
 *      Author: dhewlett
 */

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <simulator_state/WorldState.h>
#include <oomdp_msgs/PerformAction.h>
#include <oomdp_msgs/SimulateAction.h>
#include <oomdp_msgs/InitializeEnvironment.h>
#include <oomdp_msgs/DescribeMDP.h>
#include <oomdp_msgs/ComputeRelations.h>
#include <oomdp_msgs/Relation.h>
#include <oomdp_msgs/MDPState.h>

#include <wubble_mdp/entity.h>
#include <wubble_mdp/robot.h>

class Environment
{

public:
  Environment(ros::NodeHandle nh);
  virtual ~Environment();

  static std::vector<Entity*> makeEntityList(std::vector<oomdp_msgs::MDPObjectState> states);
  static oomdp_msgs::MDPState makeState(std::vector<Entity*> entities);
  static std::vector<oomdp_msgs::Relation> computeRelationsFromStates(std::vector<oomdp_msgs::MDPObjectState> states);
  static std::vector<oomdp_msgs::Relation> computeRelationsFromEntities(std::vector<Entity*> entities);

  std::vector<Entity*> getEntityList();
  Robot* findRobot(std::vector<Entity*> entities);
private:
  bool describeMDP(oomdp_msgs::DescribeMDP::Request& req, oomdp_msgs::DescribeMDP::Response& res);
  bool initialize(oomdp_msgs::InitializeEnvironment::Request& req, oomdp_msgs::InitializeEnvironment::Response& res);
  bool simulateAction(oomdp_msgs::SimulateAction::Request& req, oomdp_msgs::SimulateAction::Response& res);
  bool performAction(oomdp_msgs::PerformAction::Request& req, oomdp_msgs::PerformAction::Response& res);
  bool computeRelations(oomdp_msgs::ComputeRelations::Request& req, oomdp_msgs::ComputeRelations::Response& res);

  oomdp_msgs::MDPState updateState();

  bool clearSimulation();

  ros::NodeHandle nh_;
  ros::ServiceServer describe_;
  ros::ServiceServer initialize_;
  ros::ServiceServer simulate_;
  ros::ServiceServer perform_;
  ros::ServiceServer compute_;
  ros::ServiceClient get_state_client_;
  ros::ServiceClient pause_physics_client_;
  ros::ServiceClient unpause_physics_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client_;

  std::vector<Entity*> entity_list_;
  std::map<std::string, Entity*> entities_;
  std::map<std::string, Entity*> last_entities_;
};

#endif /* ENVIRONMENT_H_ */
