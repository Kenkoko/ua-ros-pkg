/*
 * relations.cpp
 *
 *  Created on: Oct 11, 2010
 *      Author: dhewlett
 */

#include <iostream>
#include <iomanip>
#include <math.h>

#include <oomdp_msgs/Relation.h>

#include <wubble_mdp/entity.h>
#include <wubble_mdp/relations.h>

using namespace std;
using oomdp_msgs::Relation;

namespace wubble_mdp
{

vector<Relation> computeBinaryRelations(EntityPtr first, EntityPtr second)
{
  vector<Relation> result;

  vector<string> names;
  names.push_back(first->name_);
  names.push_back(second->name_);
  int curr_dist = chessDistance(first->getPosition(), second->getPosition());
  result.push_back(makeRelation("Contact", names, (curr_dist == 0)));

  int last_dist = chessDistance(first->getLastPosition(), second->getLastPosition());
//  std::cout << setprecision(4) << std::fixed << curr_dist << " " << last_dist << std::endl;
  result.push_back(makeRelation("DistanceConstant", names, (curr_dist == last_dist)));
  result.push_back(makeRelation("DistanceDecreased", names, (curr_dist < last_dist)));
  result.push_back(makeRelation("DistanceIncreased", names, (curr_dist > last_dist)));

  return result;
}

}
