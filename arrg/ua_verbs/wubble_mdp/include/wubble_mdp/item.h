/*
 * item.h
 *
 *  Created on: Dec 2, 2010
 *      Author: dhewlett
 */

#ifndef ITEM_H_
#define ITEM_H_

#include <wubble_mdp/entity.h>

class Item;
class Robot;

typedef boost::shared_ptr<Item> ItemPtr;

class Item : public Entity
{

public:
  Item(simulator_state::ObjectInfo obj_info);
  Item(oomdp_msgs::MDPObjectState state);
  virtual ~Item();

  virtual oomdp_msgs::MDPObjectState makeObjectState();
  virtual void update(simulator_state::ObjectInfo new_info);

  virtual std::string getClassString();
  virtual btVector3 getPosition();
  virtual btVector3 getLastPosition();
  virtual geometry_msgs::Pose getPose();

  virtual std::vector<oomdp_msgs::Relation> computePredicates();
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(EntityPtr other);

  virtual bool addToWorld();

  bool shouldInitPickUp(Robot* robot);
  bool handlePickUp(Robot* robot);
  bool handlePutDown(Robot* robot);
  void updatePosition(Robot* robot, bool move_in_world);

private:
  double last_x_, last_y_;
  bool is_carried_;
  std::string carrier_name_;

};

#endif /* ITEM_H_ */
