#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_arm/MoveArmAction.h>
#include <move_arm/ActuateGripperAction.h>
#include <mapping_msgs/AttachedObject.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_models/kinematic_state.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <manipulation_msgs/JointTraj.h>
#include <manipulation_srvs/IKService.h>
#include <experimental_controllers/TrajectoryStart.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "wubble_teleop/TargetPosition.h"
    
const std::string arm = "r";
const std::string group = "right_arm";

class ArmControlServer
{
private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<move_arm::MoveArmAction> *move_arm;
    actionlib::SimpleActionClient<move_arm::ActuateGripperAction> *gripper;

    std::vector<std::string> names;

    ros::ServiceServer service_;

public:
    ArmControlServer(const ros::NodeHandle& nh)     
    : nh_(nh),
      names(7)
    {
        std::cout << "BEGIN CONSTRUCTOR" << std::endl;
        
        move_arm = new actionlib::SimpleActionClient<move_arm::MoveArmAction>(nh_, "move_" + group);
        gripper = new actionlib::SimpleActionClient<move_arm::ActuateGripperAction>(nh_, "actuate_gripper_" + group);

        names[0] = arm + "_shoulder_pan_joint";
        names[1] = arm + "_shoulder_lift_joint";
        names[2] = arm + "_upper_arm_roll_joint";
        names[3] = arm + "_elbow_flex_joint";
        names[4] = arm + "_forearm_roll_joint";
        names[5] = arm + "_wrist_flex_joint";
        names[6] = arm + "_wrist_roll_joint";

        std::cout << "END CONSTRUCTOR" << std::endl;
    }

    void init() 
    {
        service_ = nh_.advertiseService("arm_go", &ArmControlServer::go, this);
    }   

    bool go(wubble_teleop::TargetPosition::Request   &req,
            wubble_teleop::TargetPosition::Response  &res)
    {
        std::cout << "IN GO" << std::endl;

        double allowed_time = 5.0;
        
        std::vector<double> nrs;
        // Position
        nrs.push_back(req.x);
        nrs.push_back(req.y);
        nrs.push_back(req.z);
        // Rotation (Quaternion?)
        nrs.push_back(0);
        nrs.push_back(0);
        nrs.push_back(0);
        nrs.push_back(1);

        std::string link = "r_wrist_roll_link"; 
        move_arm::MoveArmGoal g;
        ArmControlServer::setupGoalEEf(link, nrs, g);

        std::cout << "Moving " << link << " to " << nrs[0] << ", " << nrs[1] << ", " << nrs[2] << ", " <<
            nrs[3] << ", " << nrs[4] << ", " << nrs[5] << ", " << nrs[6] << "..." << std::endl;        

        bool finished_within_time;
        move_arm->sendGoal(g);
        finished_within_time = move_arm->waitForGoalToFinish(ros::Duration(allowed_time));

        if (!finished_within_time)
        {
            move_arm->cancelGoal();
            std::cerr << "Timed out achieving goal" << std::endl;
        }
        else 
        {
            std::cout << "Final state is " << move_arm->getTerminalState().toString() << std::endl;
        }

        res.success = finished_within_time;
        return true;
    }

    static void setupGoalEEf(const std::string &link, const std::vector<double> &pz, move_arm::MoveArmGoal &goal)
    {
        goal.goal_constraints.pose_constraint.resize(1);
        goal.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z +
	    + motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;
        goal.goal_constraints.pose_constraint[0].link_name = link;
        goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
        goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "/base_link";
        goal.goal_constraints.pose_constraint[0].pose.pose.position.x = pz[0];
        goal.goal_constraints.pose_constraint[0].pose.pose.position.y = pz[1];
        goal.goal_constraints.pose_constraint[0].pose.pose.position.z = pz[2];

        goal.goal_constraints.pose_constraint[0].pose.pose.orientation.x = pz[3];
        goal.goal_constraints.pose_constraint[0].pose.pose.orientation.y = pz[4];
        goal.goal_constraints.pose_constraint[0].pose.pose.orientation.z = pz[5];
        goal.goal_constraints.pose_constraint[0].pose.pose.orientation.w = pz[6];

        goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
        goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.005;
        goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.01;
        goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
        goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.005;
        goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.005;

        goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.005;
        goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.005;
        goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.005;
        goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.005;
        goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.005;
        goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.005;

        goal.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

        goal.contacts.resize(2);
        goal.contacts[0].links.push_back("r_gripper_l_finger_link");
        goal.contacts[0].links.push_back("r_gripper_r_finger_link");
        goal.contacts[0].links.push_back("r_gripper_l_finger_tip_link");
        goal.contacts[0].links.push_back("r_gripper_r_finger_tip_link");

        goal.contacts[0].depth = 0.04;
        goal.contacts[0].bound.type = mapping_msgs::Object::SPHERE;
        goal.contacts[0].bound.dimensions.push_back(0.3);
        goal.contacts[0].pose = goal.goal_constraints.pose_constraint[0].pose;

        goal.contacts[1].links.push_back("r_gripper_palm_link");
        goal.contacts[1].links.push_back("r_wrist_roll_link");
        goal.contacts[1].depth = 0.02;
        goal.contacts[1].bound.type = mapping_msgs::Object::SPHERE;
        goal.contacts[1].bound.dimensions.push_back(0.2);
        goal.contacts[1].pose = goal.goal_constraints.pose_constraint[0].pose;
    }
};


// END CLASS




int main(int argc, char **argv)
{
    // Below is the new main

    ros::init(argc, argv, "arm_server", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ArmControlServer server(nh);
    server.init();    

    ros::spin();

    return 0;
}

