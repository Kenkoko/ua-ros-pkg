/*
 * environment.cpp
 *
 *  Created on: Oct 9, 2010
 *      Author: dhewlett
 */

#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <ros/ros.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>
#include <actionlib/client/terminal_state.h>

#include <std_srvs/Empty.h>
#include <simulator_state/GetState.h>
#include <simulator_state/ObjectInfo.h>
#include <oomdp_msgs/MDPClassDescription.h>

#include <wubble_mdp/environment.h>
#include <wubble_mdp/robot.h>
#include <wubble_mdp/location.h>
#include <wubble_mdp/object.h>
#include <wubble_mdp/relations.h>

using namespace boost::assign;
using std::vector;
using std::map;
using std::string;
using std::cout;
using std::cerr;
using std::endl;
using simulator_state::ObjectInfo;
using oomdp_msgs::Relation;
using oomdp_msgs::MDPObjectState;
using oomdp_msgs::MDPState;

vector<Entity*> Environment::makeEntityList(vector<MDPObjectState> states)
{
  vector<Entity*> entities;
  vector<MDPObjectState>::const_iterator obj_it;
  for (obj_it = states.begin(); obj_it != states.end(); ++obj_it)
  {
    entities.push_back(wubble_mdp::makeEntity(*obj_it));
  }
  return entities;
}

MDPState Environment::makeState(vector<Entity*> entities)
{
  MDPState state;
  for (vector<Entity*>::iterator ent_it = entities.begin(); ent_it != entities.end(); ++ent_it)
  {
    state.object_states.push_back((*ent_it)->makeObjectState());
  }
  state.relations = computeRelationsFromEntities(entities);
  return state;
}

vector<Relation> Environment::computeRelationsFromStates(vector<oomdp_msgs::MDPObjectState> states)
{
  vector<Entity*> entities = Environment::makeEntityList(states);
  return computeRelationsFromEntities(entities);
}

vector<Relation> Environment::computeRelationsFromEntities(vector<Entity*> entities)
{
  vector<Relation> result;

  // First compute the predicates (unary relations)
  for (vector<Entity*>::iterator ent_it = entities.begin(); ent_it != entities.end(); ++ent_it)
  {
    vector<Relation> predicates = (*ent_it)->computePredicates();
    result.insert(result.end(), predicates.begin(), predicates.end());
  }

  if (entities.size() > 1)
  {
    for (uint i = 0; i < entities.size(); i++)
    {
      for (uint j = i + 1; j < entities.size(); j++)
      {
        // Now, compute the general binary relations (like DistanceDecreased)
        // For now, both versions of symmetric rels will be included
        vector<Relation> rels = wubble_mdp::computeBinaryRelations(entities[i], entities[j]);
        result.insert(result.end(), rels.begin(), rels.end());
        vector<Relation> rev_rels = wubble_mdp::computeBinaryRelations(entities[j], entities[i]);
        result.insert(result.end(), rev_rels.begin(), rev_rels.end());

        // These are for special relations that only some classes compute (like InFrontOf)
        vector<Relation> first_specials = entities[i]->computeBinaryRelations(entities[j]);
        result.insert(result.end(), first_specials.begin(), first_specials.end());

        vector<Relation> second_specials = entities[j]->computeBinaryRelations(entities[i]);
        result.insert(result.end(), second_specials.begin(), second_specials.end());
      }
    }
  }

  return result;
}

// END STATIC METHODS

Environment::Environment(ros::NodeHandle nh)
{
  nh_ = nh;

  describe_ = nh_.advertiseService("environment/describe_mdp", &Environment::describeMDP, this);
  initialize_ = nh_.advertiseService("environment/initialize", &Environment::initialize, this);
  perform_ = nh_.advertiseService("environment/perform_action", &Environment::performAction, this);
  simulate_ = nh_.advertiseService("environment/simulate_action", &Environment::simulateAction, this);
  compute_ = nh_.advertiseService("environment/compute_relations", &Environment::computeRelations, this);

  // TODO: What's the problem here? new?
  ros::ServiceClient get_state_client_ = nh_.serviceClient<simulator_state::GetState> ("/gazebo/get_state");
  ros::ServiceClient pause_physics_client_ = nh_.serviceClient<std_srvs::Empty> ("/gazebo/pause_physics");
  ros::ServiceClient unpause_physics_client_ = nh_.serviceClient<std_srvs::Empty> ("/gazebo/unpause_physics");

  move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base");

  ROS_INFO("Services advertised, Environment is ready.");
}

Environment::~Environment()
{
  delete move_base_client_;
}

bool Environment::describeMDP(oomdp_msgs::DescribeMDP::Request& req, oomdp_msgs::DescribeMDP::Response& res)
{
  oomdp_msgs::MDPClassDescription robot_class;
  robot_class.name = "Robot";
  robot_class.attributes += "x", "y", "orientation", "last_x", "last_y", "last_orientation";

  oomdp_msgs::MDPClassDescription object_class;
  object_class.name = "Object";
  object_class.attributes += "x", "y", "orientation", "last_x", "last_y", "last_orientation";

  oomdp_msgs::MDPClassDescription location_class;
  location_class.name = "Location";
  location_class.attributes += "x", "y";

  res.description.classes.push_back(robot_class);
  res.description.classes.push_back(object_class);
  res.description.classes.push_back(location_class);

  return true;
}

bool Environment::initialize(oomdp_msgs::InitializeEnvironment::Request& req,
                             oomdp_msgs::InitializeEnvironment::Response& res)
{
  entities_.clear(); // TODO: Does this deallocate the old entities, or do I have to manually delete them?

  // TODO: Need to actually set up the simulator to match the state! For now, just reads whatever is there

//  vector<Entity*> entity_list = Environment::makeEntityList(req.object_states);
//  for (vector<Entity*>::const_iterator ent_it = entity_list.begin(); ent_it != entity_list.end(); ++ent_it)
//  {
//    entities_[(*ent_it)->name_] = *ent_it;
//  }
//  res.start_state.object_states = req.object_states;
//  res.start_state.relations = Environment::computeRelationsFromEntities(entity_list);

  res.start_state = updateState();

  return true;
}

bool Environment::performAction(oomdp_msgs::PerformAction::Request& req, oomdp_msgs::PerformAction::Response& res)
{
  std_srvs::Empty empty;
  ros::service::call("/gazebo/unpause_physics", empty);

  if (entities_.empty())
  {
    updateState(); // TODO: For now this is OK. Should really call initialize first
  }

  Robot* robot = findRobot(getEntityList());

  if (robot == NULL)
  {
    ROS_ERROR("THERE IS NO ROBOT!");
    return false;
  }

  move_base_client_->waitForServer();
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.pose = robot->computeTargetPose(req.action);
  move_base_client_->sendGoal(goal); // TODO: Why is he stopping 0.2 away from the goal? Is it something in the nav params?

  bool finished_before_timeout = move_base_client_->waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = move_base_client_->getState();
    ROS_INFO("Action finished, client state: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }

  ros::service::call("/gazebo/pause_physics", empty);

  res.new_state = updateState();

  return true;
}

bool Environment::simulateAction(oomdp_msgs::SimulateAction::Request& req, oomdp_msgs::SimulateAction::Response& res)
{
  // NB: For now, this method does not use the simulator, so it can only naively simulate the robot's movement

  vector<Entity*> entities = Environment::makeEntityList(req.state.object_states);
  // Find the robot:
  Robot* robot = findRobot(entities);

  if (robot == NULL)
  {
    ROS_ERROR("CANNOT SIMULATE STATE WITH NO ROBOT");
    ROS_ERROR_STREAM(req.state);
    res.new_state = req.state; // The state is unchanged
  }
  else
  {
    robot->simulateAction(req.action);
  }

  res.new_state = Environment::makeState(entities);

  return true;
}

Robot* Environment::findRobot(vector<Entity*> entities)
{
  Robot* robot = NULL;
  for (vector<Entity*>::iterator ent_it = entities.begin(); ent_it != entities.end(); ++ent_it)
  {
    if ((*ent_it)->getClassString() == "Robot")
    {
      robot = dynamic_cast<Robot*> (*ent_it);
    }
  }
  return robot;
}

vector<Entity*> Environment::getEntityList()
{
  vector<Entity*> entity_list;
  for (map<string, Entity*>::iterator ent_it = entities_.begin(); ent_it != entities_.end(); ++ent_it)
  {
    entity_list.push_back(ent_it->second);
  }
  return entity_list;
}

bool Environment::computeRelations(oomdp_msgs::ComputeRelations::Request& req,
                                   oomdp_msgs::ComputeRelations::Response& res)
{
  res.state.object_states = req.object_states; // Just pass these on unchanged
  res.state.relations = computeRelationsFromStates(req.object_states);

  return true;
}

oomdp_msgs::MDPState Environment::updateState()
{
  //  ROS_INFO_STREAM("EXISTS? " << get_state_client_.exists());
  //
  //  simulator_state::GetStateRequest get_state_req;
  //  simulator_state::GetStateResponse get_state_res;
  ////  get_state_client_.waitForExistence();
  //  bool result = get_state_client_.call(get_state_req, get_state_res);
  //
  //  ROS_INFO_STREAM("SUCCESS? " << result);
  //  ROS_INFO_STREAM(get_state_res.state);
  // TODO: WHY DOESN'T THIS WORK??? WTF???

  simulator_state::GetState get_state;
  ros::service::call("/gazebo/get_state", get_state);
  //  ROS_INFO_STREAM("BARE SERVICE CALL: " << );
  simulator_state::WorldState world_state = get_state.response.state;

  vector<ObjectInfo>::const_iterator obj_it;
  for (obj_it = world_state.object_info.begin(); obj_it != world_state.object_info.end(); ++obj_it)
  {
    ObjectInfo info = (*obj_it);
    string name = obj_it->name;

    ROS_INFO_STREAM(name << endl);
    ROS_INFO_STREAM(info);

    bool exists = (entities_.find(name) != entities_.end());

    if (exists)
    {
      ROS_INFO("EXISTS");
      entities_[name]->update(info);
    }
    else
    {
      Entity *entity = NULL;
      if (name.find("robot") == 0)
      {
        ROS_INFO("ROBOT");
        entity = new Robot(info);
      }
      else if (name.find("goal") == 0)
      {
        ROS_INFO("LOCATION");
        entity = new Location(info);
      }
      else
      {
        ROS_INFO("OBJECT");
        entity = new Object(info);
      }
      entities_[name] = entity;
    }
  }

  // TODO: Handle any relations computed gazebo-side, etc.
  return Environment::makeState(getEntityList());
}
