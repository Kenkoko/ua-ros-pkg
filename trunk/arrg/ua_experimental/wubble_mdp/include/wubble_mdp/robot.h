/*
 * robot.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <geometry_msgs/Pose.h>
#include <simulator_state/ObjectInfo.h>
#include <wubble_mdp/entity.h>
#include <oomdp_msgs/MDPObjectState.h>

class Robot : public Entity
{
private:
  btVector3 computeNewPose(std::string action);

public:
  Robot(simulator_state::ObjectInfo obj_info);
  Robot(oomdp_msgs::MDPObjectState state);
  virtual ~Robot();

  virtual oomdp_msgs::MDPObjectState makeObjectState();
  virtual void update(simulator_state::ObjectInfo new_info);

  virtual bool addToWorld();

  virtual std::string getClassString();
  virtual btVector3 getPosition();
  virtual btVector3 getLastPosition();

  virtual std::vector<oomdp_msgs::Relation> computePredicates();
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(Entity* other);

  void simulateAction(std::string action);
  double computeRelativeAngle(btVector3 other_pos);
  geometry_msgs::Pose computeTargetPose(std::string action);

  double orientation_;
  double last_x_, last_y_;
  double last_orientation_;
  static const double delta_ = 0.5; // TODO: Is this bad style?
};

#endif /* ROBOT_H_ */
