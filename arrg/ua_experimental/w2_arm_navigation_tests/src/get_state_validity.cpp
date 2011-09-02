#include <ros/ros.h>
#include <arm_navigation_msgs/GetStateValidity.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::service::waitForService("environment_server_left_arm/get_state_validity");
  ros::ServiceClient check_state_validity_client_ = rh.serviceClient<arm_navigation_msgs::GetStateValidity>("environment_server_left_arm/get_state_validity");

  arm_navigation_msgs::GetStateValidity::Request req;
  arm_navigation_msgs::GetStateValidity::Response res;

  req.robot_state.joint_state.name.push_back("shoulder_pitch_joint");
  req.robot_state.joint_state.name.push_back("shoulder_yaw_joint");
  req.robot_state.joint_state.name.push_back("shoulder_roll_joint");
  req.robot_state.joint_state.name.push_back("elbow_pitch_joint");
  req.robot_state.joint_state.name.push_back("wrist_roll_joint");
  req.robot_state.joint_state.name.push_back("wrist_pitch_joint");
  req.robot_state.joint_state.name.push_back("wrist_yaw_joint");
  req.robot_state.joint_state.position.resize(7,0.0);

  //these set whatever non-zero joint angle values are desired
  req.robot_state.joint_state.position[0] = 1.0;
  
  req.robot_state.joint_state.header.stamp = ros::Time::now();
  req.check_collisions = true;
  if(check_state_validity_client_.call(req,res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested state is not in collision");
    else
      ROS_INFO("Requested state is in collision. Error code: %d",res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to check state validity failed %s",check_state_validity_client_.getService().c_str());
    return false;
  }
  ros::shutdown();
}
