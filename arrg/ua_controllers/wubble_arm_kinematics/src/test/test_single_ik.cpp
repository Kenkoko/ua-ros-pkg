#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_ik");
  ros::NodeHandle rh;

  ros::service::waitForService("arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("arm_kinematics/get_ik");
  ROS_INFO("Connected to IK services");

  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("arm_kinematics/get_ik_solver_info");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("arm_kinematics/get_ik");
  ros::ServiceClient fk_client = rh.serviceClient<kinematics_msgs::GetPositionFK>("arm_kinematics/get_fk");

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(-1);
  }

  ROS_INFO("Starting now");

  // define the service messages
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;

  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "L7_wrist_yaw_link";

  gpik_req.ik_request.pose_stamped.header.frame_id = "base_footprint";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.40;
  gpik_req.ik_request.pose_stamped.pose.position.y = 0.10;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0.20;

  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1;

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }

  gpik_req.ik_request.ik_seed_state.joint_state.position[0] = 0;
  gpik_req.ik_request.ik_seed_state.joint_state.position[1] = 0;
  gpik_req.ik_request.ik_seed_state.joint_state.position[2] = 0;
  gpik_req.ik_request.ik_seed_state.joint_state.position[3] = 0;
  gpik_req.ik_request.ik_seed_state.joint_state.position[4] = 0;
  gpik_req.ik_request.ik_seed_state.joint_state.position[5] = 0; 
  gpik_req.ik_request.ik_seed_state.joint_state.position[6] = 0;

  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
    else
      ROS_ERROR("Inverse kinematics failed");
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");

  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  fk_request.header.frame_id = "base_footprint";
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = "L7_wrist_yaw_link";

  fk_request.robot_state.joint_state.position = gpik_res.solution.joint_state.position;
  fk_request.robot_state.joint_state.name = gpik_res.solution.joint_state.name;

  if(fk_client.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
      {
        ROS_INFO_STREAM("Link: " << fk_response.fk_link_names[i].c_str());
        ROS_INFO_STREAM("Position: " << 
          fk_response.pose_stamped[i].pose.position.x << "," <<  
          fk_response.pose_stamped[i].pose.position.y << "," << 
          fk_response.pose_stamped[i].pose.position.z);
        ROS_INFO("Orientation: %f %f %f %f",
          fk_response.pose_stamped[i].pose.orientation.x,
          fk_response.pose_stamped[i].pose.orientation.y,
          fk_response.pose_stamped[i].pose.orientation.z,
          fk_response.pose_stamped[i].pose.orientation.w);
      } 
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
  }

  ros::shutdown();
}
