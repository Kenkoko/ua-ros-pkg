/*
 * robot.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <wubble_mdp/entity.h>
#include <wubble_mdp/item.h>

class Robot : public Entity
{
private:
  btVector3 computeNewPose(std::string action);

public:
  Robot(simulator_state::ObjectInfo obj_info);
  Robot(oomdp_msgs::MDPObjectState state);
  void init(std::vector<Entity*> entities);
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
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(Entity* other);

  void simulateAction(std::string action, std::vector<Entity*> entities);
  void drop();
  void pickUp(Item* item);
  void pickUp(std::string item_name, std::vector<Entity*> entities);

  double computeRelativeAngle(btVector3 other_pos);
  geometry_msgs::Pose computeTargetPose(std::string action);

  double orientation_;
  double last_x_, last_y_;
  double last_orientation_;
  Item* carried_item_;

  static const double delta_ = 0.5; // TODO: Put this somewhere more sensible, like a "Constants" class
};

#endif /* ROBOT_H_ */
