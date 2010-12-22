/*
 * state_utils.h
 *
 *  Created on: Oct 12, 2010
 *      Author: dhewlett
 */

#ifndef STATE_UTILS_H_
#define STATE_UTILS_H_

#include <gazebo/Vector3.hh>
#include <gazebo/Pose3d.hh>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

namespace simulator_state
{

btTransform convertTransform(gazebo::Pose3d pose);
btConvexShape* convertAABB(gazebo::Vector3 min, gazebo::Vector3 max);
void printVector(btVector3 v);
void printQuat(btQuaternion q);

}

#endif /* STATE_UTILS_H_ */
