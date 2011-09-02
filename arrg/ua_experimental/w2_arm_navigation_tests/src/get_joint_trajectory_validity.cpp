#include <ros/ros.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/GetJointTrajectoryValidity.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_trajectory_validity_test");
  ros::NodeHandle rh;

  ros::service::waitForService("environment_server_left_arm/get_trajectory_validity");
  arm_navigation_msgs::GetJointTrajectoryValidity::Request req;
  arm_navigation_msgs::GetJointTrajectoryValidity::Response res;
  ros::ServiceClient check_trajectory_validity_client_ = rh.serviceClient<arm_navigation_msgs::GetJointTrajectoryValidity>("environment_server_left_arm/get_trajectory_validity");

  ros::service::waitForService("environment_server_left_arm/get_robot_state");
  ros::ServiceClient get_state_client_ = rh.serviceClient<arm_navigation_msgs::GetRobotState>("environment_server_left_arm/get_robot_state");
  arm_navigation_msgs::GetRobotState::Request request;
  arm_navigation_msgs::GetRobotState::Response response;
  if(get_state_client_.call(request,response))
  {
    req.robot_state = response.robot_state;
  }
  else
  {
    ROS_ERROR("Service call to get robot state failed on %s",get_state_client_.getService().c_str());
  }

  req.trajectory.joint_names.push_back("shoulder_yaw_joint");
  req.trajectory.points.resize(100);
  for(unsigned int i=0; i < 100; i++)
  {    
    req.trajectory.points[i].positions.push_back(-(3*M_PI*i/4.0)/100.0);
  }
  req.check_collisions = true;
  if(check_trajectory_validity_client_.call(req,res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested trajectory is not in collision");
    else
      ROS_INFO("Requested trajectory is in collision. Error code: %d",res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to check trajectory validity failed %s",check_trajectory_validity_client_.getService().c_str());
    return false;
  }
  ros::shutdown();
}

