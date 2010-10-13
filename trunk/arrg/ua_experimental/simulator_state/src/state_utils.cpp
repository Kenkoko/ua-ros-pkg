/*
 * state_utils.cpp
 *
 *  Created on: Oct 12, 2010
 *      Author: dhewlett
 */

#include <simulator_state/state_utils.h>

using namespace std;
using namespace gazebo;

namespace simulator_state {

btTransform convertTransform(Pose3d pose)
{
  btVector3 pos(pose.pos.x, pose.pos.y, pose.pos.z);

  //  cout << "POS: " << pose.pos << endl;

  // This is right
  //  btQuaternion rot(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.u);
  btQuaternion rot(0, 0, 0, 1); // This is for AABBs

  //  cout << "QUATERNIONS!" << endl;
  //  cout << pose.rot.x << " " << pose.rot.y << " " << pose.rot.z << " " << pose.rot.u << endl;
  //  cout << pose.rot.GetAsEuler() << endl;
  //  cout << rot.getX() << " " << rot.getY() << " " << rot.getZ() << " " << rot.getW() << endl;
  //  btQuaternion test;
  //  test.setRPY(pose.rot.GetAsEuler().x, pose.rot.GetAsEuler().y, pose.rot.GetAsEuler().z);
  //  cout << test.getX() << " " << test.getY() << " " << test.getZ() << " " << test.getW() << endl;

  return btTransform(rot, pos);
}

btConvexShape* convertAABB(Vector3 min, Vector3 max)
{
  Vector3 size = max - min;
  size /= 2;
  btVector3 half_extents(size.x, size.y, size.z);
  btConvexShape* box = new btBoxShape(half_extents);
  return box;
}

void printVector(btVector3 v)
{
  cout << v.x() << " " << v.y() << " " << v.z() << endl;
}

void printQuat(btQuaternion q)
{
  cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
}

}
