/*
 * robot.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <wubble_mdp/entity.h>
//#include <boost/shared_ptr.hpp>

class Robot;
class Item;

typedef boost::shared_ptr<Robot> RobotPtr;

class Robot : public Entity
{
private:
  btVector3 computeNewPose(std::string action);

public:
  Robot(simulator_state::ObjectInfo obj_info);
  Robot(oomdp_msgs::MDPObjectState state);
  void init(std::vector<EntityPtr> entities);
  virtual ~Robot();

  virtual oomdp_msgs::MDPObjectState makeObjectState();
  virtual void update(simulator_state::ObjectInfo new_info);

  virtual bool addToWorld();
  bool removeFromWorld();

  virtual std::string getClassString();
  virtual btVector3 getPosition();
  virtual btVector3 getLastPosition();
  virtual geometry_msgs::Pose getPose();

  virtual std::vector<oomdp_msgs::Relation> computePredicates();
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(EntityPtr other);

  void simulateAction(std::string action, std::vector<EntityPtr> entities);
  void drop();
  void pickUp(boost::shared_ptr<Item> item);
  void pickUp(std::string item_name, std::vector<EntityPtr> entities);

  double computeRelativeAngle(btVector3 other_pos);
  geometry_msgs::Pose computeTargetPose(std::string action);

  double orientation_;
  double last_x_, last_y_;
  double last_orientation_;
  boost::shared_ptr<Item> carried_item_;

  static const double delta_ = 0.5; // TODO: Put this somewhere more sensible, like a "Constants" class
};

#endif /* ROBOT_H_ */
