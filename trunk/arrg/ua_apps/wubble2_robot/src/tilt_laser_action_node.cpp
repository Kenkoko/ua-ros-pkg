#include <string>

#include <boost/scoped_ptr.hpp>
#include <wubble2_robot/WubbleLaserTiltAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Float64.h>
#include <pr2_msgs/LaserScannerSignal.h>
#include <dynamixel_hardware_interface/JointState.h>
#include <dynamixel_hardware_interface/SetVelocity.h>

class LaserTiltAction
{

public:
    LaserTiltAction()
    {
        private_nh_ = ros::NodeHandle("~");
        
        // Report success if error reaches below threshold
        private_nh_.param<double>("error_threshold", error_threshold_, 0.0175);
        signal_ = 1;
    }
     
    bool initialize()
    {
        std::string tilt_controller;
        if (!private_nh_.getParam("tilt_controller", tilt_controller))
        {
            ROS_ERROR("tilt_controller paramater not specified");
            return false;
        }
        
        tilt_state_sub_ = nh_.subscribe<dynamixel_hardware_interface::JointState>(tilt_controller + "/state", 100, &LaserTiltAction::processTiltState, this);
        tilt_command_pub_ = nh_.advertise<std_msgs::Float64>(tilt_controller + "/command", 100);
        laser_signal_pub_ = nh_.advertise<pr2_msgs::LaserScannerSignal>("laser_scanner_signal", 100);
        tilt_velocity_srv_ = nh_.serviceClient<dynamixel_hardware_interface::SetVelocity>(tilt_controller + "/set_velocity");
        
        ROS_INFO("LaserTiltAction waiting for %s to be advertised", (tilt_controller + "/set_velocity").c_str());
        tilt_velocity_srv_.waitForExistence();
        
        // Initialize tilt action server
        action_server_.reset(new WLTAS(nh_, "hokuyo_laser_tilt_action",
                                       boost::bind(&LaserTiltAction::processLaserTiltGoal, this, _1),
                                       false));
        action_server_->start();
        ROS_INFO("LaserTiltAction ready to accept goals");
        return true;
    }
    
    void processTiltState(const dynamixel_hardware_interface::JointStateConstPtr& msg)
    {
        tilt_state_ = *msg;
    }
    
    void processLaserTiltGoal(const wubble2_robot::WubbleLaserTiltGoalConstPtr& goal)
    {
        ros::Rate r(100);
        
        dynamixel_hardware_interface::SetVelocityRequest req;
        dynamixel_hardware_interface::SetVelocityResponse res;
        req.velocity = 2.0;
        
        if (!tilt_velocity_srv_.call(req, res))
        {
            ROS_ERROR("Unable to set tilt joint velocity");
            action_server_->setAborted();
            return;
        }

        std_msgs::Float64 msg;
        msg.data = goal->offset;
        tilt_command_pub_.publish(msg);
        ros::Duration(1).sleep();
        
        double delta = goal->amplitude - goal->offset;
        double target_speed = delta / goal->duration;
        ROS_DEBUG("delta = %f, target_speed = %f", delta, target_speed);
        
        ros::Duration timeout_threshold = ros::Duration(5.0) + ros::Duration(goal->duration);
        req.velocity = target_speed;
        
        if (!tilt_velocity_srv_.call(req, res))
        {
            ROS_ERROR("Unable to set tilt joint velocity");
            action_server_->setAborted();
            return;
        }
        
        result_.tilt_position = tilt_state_.position;
        
        // Tilt laser goal.tilt_cycles amount of times.
        while (ros::ok())
        {
            // Issue 2 commands for each cycle
            for (int j = 0; j < 2; ++j)
            {
                double target_tilt = 0.0;
                
                if (j % 2 == 0)
                {
                    target_tilt = goal->offset + goal->amplitude;    // Upper tilt limit
                    signal_ = 0;
                }
                else
                {
                    target_tilt = goal->offset;                      // Lower tilt limit
                    signal_ = 1;
                }
                
                // Publish target command to controller
                msg.data = target_tilt;
                tilt_command_pub_.publish(msg);
                ros::Time start_time = ros::Time::now();
                ros::Time current_time = start_time;
                
                while (fabs(target_tilt - tilt_state_.position) > error_threshold_)
                {
                    // Cancel execution if another goal was received (i.e. preempt requested)
                    if (action_server_->isPreemptRequested())
                    {
                        ROS_WARN("LaserTiltAction preempted, new goal received");
                        action_server_->setPreempted();
                        return;
                    }
                        
                    // Publish current tilt position as feedback
                    feedback_.tilt_position = tilt_state_.position;
                    action_server_->publishFeedback(feedback_);
                    
                    // Abort if timeout
                    current_time = ros::Time::now();
                    
                    if (current_time - start_time > timeout_threshold)
                    {
                        ROS_WARN("LaserTiltAction timed out");
                        action_server_->setAborted();
                        return;
                    }
                        
                    pr2_msgs::LaserScannerSignal s;
                    s.header.stamp = current_time;
                    s.signal = signal_;
                    laser_signal_pub_.publish(s);
                    
                    r.sleep();
                }
            }
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber tilt_state_sub_;
    ros::Publisher tilt_command_pub_;
    ros::Publisher laser_signal_pub_;
    ros::ServiceClient tilt_velocity_srv_;
    
    double error_threshold_;
    int signal_;
    dynamixel_hardware_interface::JointState tilt_state_;
    wubble2_robot::WubbleLaserTiltResult result_;
    wubble2_robot::WubbleLaserTiltFeedback feedback_;
    
    typedef actionlib::SimpleActionServer<wubble2_robot::WubbleLaserTiltAction> WLTAS;
    boost::scoped_ptr<WLTAS> action_server_;
    
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tilt_laser_action_server");
    LaserTiltAction lta;
    
    if (!lta.initialize())
    {
        ROS_ERROR("unable to initialize LaserTiltAction");
        return -1;
    }
    
    ros::spin();
    return 0;
}
