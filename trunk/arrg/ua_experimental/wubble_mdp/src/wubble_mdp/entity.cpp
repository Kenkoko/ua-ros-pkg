/*
 * entity.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <algorithm>
#include <stdarg.h>
#include <math.h>
#include <boost/math/constants/constants.hpp>

#include <wubble_mdp/entity.h>
#include <wubble_mdp/robot.h>
#include <wubble_mdp/object.h>
#include <wubble_mdp/location.h>

namespace wubble_mdp
{
std::string makeOrientationString(double yaw)
{
  const double pi = boost::math::constants::pi<double>();
  if (yaw >= 0)
  {
    if (yaw < (pi / 8))
    {
      return "E";
    }
    else if (yaw < (3 * pi / 8))
    {
      return "NE";
    }
    else if (yaw < (5 * pi / 8))
    {
      return "N";
    }
    else if (yaw < (7 * pi / 8))
    {
      return "NW";
    }
    else
    {
      return "W";
    }
  }
  else
  {
    if (yaw > -(pi / 8))
    {
      return "E";
    }
    else if (yaw > -(3 * pi / 8))
    {
      return "SE";
    }
    else if (yaw > -(5 * pi / 8))
    {
      return "S";
    }
    else if (yaw > -(7 * pi / 8))
    {
      return "SW";
    }
    else
    {
      return "W";
    }
  }
}

double extractYaw(simulator_state::ObjectInfo info)
{
  btQuaternion q(info.pose.orientation.x, info.pose.orientation.y, info.pose.orientation.z, info.pose.orientation.w);
  double roll, pitch, yaw;
  btMatrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

std::string extractOrientationString(simulator_state::ObjectInfo info)
{
  double yaw = extractYaw(info);
  return makeOrientationString(yaw);
}

double convertOrientationString(std::string orientation)
{
  const double pi = boost::math::constants::pi<double>();
  std::map<std::string, double> omap;
  omap["E"] = 0.0;
  omap["NE"] = pi / 4;
  omap["N"] = pi / 2;
  omap["NW"] = 3 * pi / 4;
  omap["W"] = pi;
  omap["SW"] = -3 * pi / 4;
  omap["S"] = -pi / 2;
  omap["SE"] = -pi / 4;

  if (omap.find(orientation) != omap.end())
  {
    return omap[orientation];
  }
  else
  {
    std::cerr << "INVALID ORIENTATION STRING " << orientation << std::endl;
    return 0;
  }
}

std::string doubleToString(double d)
{
  double rounded = roundToDelta(d, 0.1);
  std::ostringstream s;
  s.precision(1);
  //    cout.setf(ios::fixed,ios::floatfield);
  s << std::fixed << rounded;
  return s.str();
}

std::map<std::string, std::string> extractAttributeValues(oomdp_msgs::MDPObjectState state)
{
  std::map<std::string, std::string> result;
  if (state.attributes.size() != state.values.size())
  {
    // Freak Out!
    std::cerr << "DIFFERENT NUMBER OF ATTRIBUTES AND VALUES\n";
  }
  else
  {
    for (uint i = 0; i < state.attributes.size(); i++)
    {
      result[state.attributes[i]] = state.values[i];
    }
  }
  return result;
}

Entity* makeEntity(oomdp_msgs::MDPObjectState state)
{
  if (state.class_name == "Robot")
  {
    return new Robot(state);
  }
  else if (state.class_name == "Object")
  {
    return new Object(state);
  }
  else if (state.class_name == "Location")
  {
    return new Location(state);
  }
  else
  {
    std::cerr << "INVALID CLASS NAME" << state << std::endl;
    return NULL;
  }
}

oomdp_msgs::Relation makeRelation(std::string relation, std::vector<std::string> names, bool value)
{
  oomdp_msgs::Relation result;
  result.relation = relation;
  result.obj_names = names;
  result.value = value;

  return result;
}

double deltaAngle(double theta_1, double theta_2)
{
  return atan2(sin(theta_1 - theta_2), cos(theta_1 - theta_2));
}

// TODO: Need to pull out step size as a constant somewhere
int chessDistance(btVector3 first_pos, btVector3 second_pos)
{
  double x_diff = fabs(first_pos.x() - second_pos.x());
  double y_diff = fabs(first_pos.y() - second_pos.y());
  double max = std::max(x_diff, y_diff); // Remember that the regular max returns an integer!
  double in_steps = max / 0.5;
  int result = round(in_steps);

  //  std::cout << x_diff << " " << y_diff << " " << max << " " << in_steps << " " << result << std::endl;

  return result;
}

double roundToDelta(double number, double delta)
{
  int multiplier = round(number / delta);
  //  std::cout << "ROUNDING: " << number << " " << delta << " " << multiplier << std::endl;
  return delta * multiplier;
}

// Rounds to the nearest pi/8 in range (-pi,pi]
double roundYaw(double yaw)
{
  const double pi = boost::math::constants::pi<double>();
  int multiplier = round(yaw / (pi / 8));
  if (multiplier == -8)
  {
    multiplier = 8;
  }
  return (pi / 8) * multiplier;
}

} // End namespace
