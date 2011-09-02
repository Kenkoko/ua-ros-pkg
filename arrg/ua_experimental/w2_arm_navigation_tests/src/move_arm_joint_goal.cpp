#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal goalB;
  std::vector<std::string> names(7);
  names[0] = "shoulder_pitch_joint";
  names[1] = "shoulder_pan_joint";
  names[2] = "upperarm_roll_joint";
  names[3] = "elbow_flex_joint";
  names[4] = "forearm_roll_joint";
  names[5] = "wrist_pitch_joint";
  names[6] = "wrist_roll_joint";

  goalB.motion_plan_request.group_name = "left_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(50.0);

  goalB.motion_plan_request.planner_id= std::string("");
  goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }

  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.9715;
  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -1.7406;
  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position =  0.0213;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.1807;
  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = -1.8408;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position =  1.0840;
  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position =  0.1483;

//  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = 0;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = 0;

//  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position =  0.02157211345145359;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -1.1142890004516011;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position =  0.12352207000000073;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position =  0.9809237263952848;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position =  2.6988996511458869;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -1.7704154220413237;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position =  0.21388434755330812;

//  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.715475;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position =  0.036352;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position =  1.040500;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position =  0.133051;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position =  0.447000;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.361470;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = -0.440495;


  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalB);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));

    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
