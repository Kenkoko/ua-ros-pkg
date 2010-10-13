/*
 * location.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef LOCATION_H_
#define LOCATION_H_

#include <wubble_mdp/entity.h>
#include <oomdp_msgs/MDPObjectState.h>
#include <oomdp_msgs/Relation.h>

class Location : public Entity
{
public:
  Location(simulator_state::ObjectInfo obj_info);
  Location(oomdp_msgs::MDPObjectState state);
  virtual ~Location();

  virtual oomdp_msgs::MDPObjectState makeObjectState();
  virtual void update(simulator_state::ObjectInfo new_info);

  virtual std::string getClassString();
  virtual btVector3 getPosition();
  virtual btVector3 getLastPosition();

  virtual std::vector<oomdp_msgs::Relation> computePredicates();
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(Entity* other);

};

#endif /* LOCATION_H_ */
