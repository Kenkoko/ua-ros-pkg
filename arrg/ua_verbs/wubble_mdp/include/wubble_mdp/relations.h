/*
 * relations.h
 *
 *  Created on: Oct 11, 2010
 *      Author: dhewlett
 */

#ifndef RELATIONS_H_
#define RELATIONS_H_

#include <wubble_mdp/entity.h>

namespace wubble_mdp
{
std::vector<oomdp_msgs::Relation> computeBinaryRelations(EntityPtr first, EntityPtr second);
}

#endif /* RELATIONS_H_ */
