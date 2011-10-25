#include <vector>
#include <string>
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_hardware_interface/JointState.h>

class JointStateAggregator
{

public:
    JointStateAggregator()
    {
        private_nh_ = ros::NodeHandle("~");
        private_nh_.param<int>("rate", publish_rate_, 50);
    }
    
    bool initialize()
    {
        int num_static = 4;
        
        msg_.name.resize(num_static);
        msg_.position.resize(num_static);
        msg_.velocity.resize(num_static);
        msg_.effort.resize(num_static);
        
        msg_.name[0] = "base_caster_support_joint";
        msg_.name[1] = "caster_wheel_joint";
        msg_.name[2] = "base_link_left_wheel_joint";
        msg_.name[3] = "base_link_right_wheel_joint";
        
        XmlRpc::XmlRpcValue val;
        
        if (private_nh_.getParam("controllers", val))
        {
            if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("controllers parameter is not a list");
                return false;
            }
            
            for (int i = 0; i < val.size(); ++i)
            {
                controller_names_.push_back(static_cast<std::string>(val[i]));
            }
        }
        
        int num_controllers = controller_names_.size();
        controller_state_subs_.resize(num_controllers);
        msg_.name.resize(num_static+num_controllers);
        msg_.position.resize(num_static+num_controllers);
        msg_.velocity.resize(num_static+num_controllers);
        msg_.effort.resize(num_static+num_controllers);
        
        for (int i = 0; i < num_controllers; ++i)
        {
            controller_state_subs_[i] = nh_.subscribe<dynamixel_hardware_interface::JointState>(controller_names_[i] +  "/state", 100,
                                                                                                boost::bind(&JointStateAggregator::processControllerState, this, _1, i+num_static));
        }
        
        for (int i = 0; i < num_controllers; ++i)
        {
            ros::topic::waitForMessage<dynamixel_hardware_interface::JointState>(controller_names_[i] +  "/state");
        }
        
        joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
        
        return true;
    }
    
    void processControllerState(const dynamixel_hardware_interface::JointStateConstPtr& msg, int i)
    {
        msg_.name[i] = msg->name;
        msg_.position[i] = msg->position;
        msg_.velocity[i] = msg->velocity;
        msg_.effort[i] = msg->load;
    }
    
    void start()
    {
        ros::Rate r(publish_rate_);
        
        while (ros::ok())
        {
            msg_.header.stamp = ros::Time::now();
            joint_states_pub_.publish(msg_);
            
            ros::spinOnce();
            r.sleep();
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::V_Subscriber controller_state_subs_;
    ros::Publisher joint_states_pub_;
    
    int publish_rate_;
    std::vector<std::string> controller_names_;
    sensor_msgs::JointState msg_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_state_aggregator");
    JointStateAggregator jsa;
    
    if (!jsa.initialize())
    {
        ROS_ERROR("JointStateAggregator failed to initialize");
        return 1;
    }
    
    jsa.start();
    
    return 0;
}