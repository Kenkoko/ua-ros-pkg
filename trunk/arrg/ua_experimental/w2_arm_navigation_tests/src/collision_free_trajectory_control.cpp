#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointExecutorActionClient;

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_controller");
  boost::thread spin_thread(&spinThread);
  JointExecutorActionClient *traj_action_client_ = new JointExecutorActionClient("l_arm_controller/joint_trajectory_action");

  while(!traj_action_client_->waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
    if(!ros::ok()) {
      exit(0);
    }
  }

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("shoulder_pitch_joint");
  goal.trajectory.joint_names.push_back("shoulder_yaw_joint");
  goal.trajectory.joint_names.push_back("shoulder_roll_joint");
  goal.trajectory.joint_names.push_back("elbow_pitch_joint");
  goal.trajectory.joint_names.push_back("wrist_roll_joint");
  goal.trajectory.joint_names.push_back("wrist_pitch_joint");
  goal.trajectory.joint_names.push_back("wrist_yaw_joint");

  goal.trajectory.points.resize(1);
  for(unsigned int i=0; i < 7; i++)
    goal.trajectory.points[0].positions.push_back(0.0);
  goal.trajectory.points[0].positions[0] = -1.57/2.0;
  goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

  traj_action_client_->sendGoal(goal);
  ROS_INFO("Sent goal");

  while(!traj_action_client_->getState().isDone() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  return 0;
}
