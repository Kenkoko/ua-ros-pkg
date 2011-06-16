#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_planning_msgs/GetMotionPlan.h>

#include <motion_planning_msgs/DisplayTrajectory.h>
#include <planning_environment/monitors/joint_state_monitor.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main(int argc, char **argv){
  ros::init (argc, argv, "ompl_joint_goal_test");
  boost::thread spin_thread(&spinThread);

  ros::NodeHandle nh;

  motion_planning_msgs::GetMotionPlan::Request request;
  motion_planning_msgs::GetMotionPlan::Response response;

  std::vector<std::string> names(7);
  names[0] = "shoulder_pitch_joint";
  names[1] = "shoulder_pan_joint";
  names[2] = "upperarm_roll_joint";
  names[3] = "elbow_flex_joint";
  names[4] = "forearm_roll_joint";
  names[5] = "wrist_pitch_joint";
  names[6] = "wrist_roll_joint";

  request.motion_plan_request.group_name = "left_arm";
  request.motion_plan_request.num_planning_attempts = 1;
  request.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  request.motion_plan_request.planner_id= std::string("");
  //  request.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  request.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < request.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    request.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    request.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    request.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    request.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }

  request.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.650;
  request.motion_plan_request.goal_constraints.joint_constraints[1].position = -1.465;
  request.motion_plan_request.goal_constraints.joint_constraints[2].position =  3.430;
  request.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.970;
  request.motion_plan_request.goal_constraints.joint_constraints[4].position = -1.427;
  request.motion_plan_request.goal_constraints.joint_constraints[5].position =  0.337;
  request.motion_plan_request.goal_constraints.joint_constraints[6].position =  0.046;

  ros::ServiceClient service_client = nh.serviceClient<motion_planning_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");
  service_client.call(request,response);
  if(response.error_code.val != response.error_code.SUCCESS)
  {
    ROS_ERROR("Motion planning failed");
  }
  else
  {
    ROS_INFO("Motion planning succeeded");
  }


  planning_environment::JointStateMonitor joint_state_monitor;
  ros::Publisher display_trajectory_publisher = nh.advertise<motion_planning_msgs::DisplayTrajectory>("/joint_path_display", 1);
  while(display_trajectory_publisher.getNumSubscribers() < 1 && nh.ok())
  {
    ROS_INFO("Waiting for subscriber");
    ros::Duration(0.1).sleep();
  }
  motion_planning_msgs::DisplayTrajectory display_trajectory;

  display_trajectory.model_id = "pr2";
  display_trajectory.trajectory.joint_trajectory.header.frame_id = "base_footprint";
  display_trajectory.trajectory.joint_trajectory.header.stamp = ros::Time::now();
  display_trajectory.robot_state.joint_state =  joint_state_monitor.getJointStateRealJoints();
  display_trajectory.trajectory = response.trajectory;
  ROS_INFO("Publishing path for display");
  display_trajectory_publisher.publish(display_trajectory);
  joint_state_monitor.stop();
  ros::shutdown();
  spin_thread.join();
  return(0);
}

