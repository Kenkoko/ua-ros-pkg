/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Controller for diffdrive robots in gazebo.
 * Author: Tony Pratkanis
 * Date: 17 April 2009
 * SVN info: $Id$
 */

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <boost/bind.hpp>

class DiffDrive {
public:
  gazebo::PositionIface *posIface;
  ros::NodeHandle* rnh_;
  ros::Subscriber  sub_;
  ros::Publisher   pub_;
  
  void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    if (this->posIface) {
      this->posIface->Lock(1);
      this->posIface->data->cmdVelocity.pos.x = cmd_msg->linear.x;
      this->posIface->data->cmdVelocity.pos.y = cmd_msg->linear.y;
      this->posIface->data->cmdVelocity.yaw = cmd_msg->angular.z;
      this->posIface->Unlock();
    }
  }

  DiffDrive() {
    // get tf prefix
    ros::NodeHandle n;
    std::string tf_prefix_ = tf::getPrefixParam(n);
    
    gazebo::Client *client = new gazebo::Client();
    gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
    this->posIface = new gazebo::PositionIface();

    ros::NodeHandle private_nh("~");

    int serverId;
    std::string modelName;
    private_nh.param("gazebo_server_id", serverId, 0);
    private_nh.param("gazebo_model_name", modelName, std::string("robot_description"));
    
    /// Connect to the libgazebo server
    try {
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
      return;
    }
    
    /// Open the Simulation Interface
    try {
      simIface->Open(client, "default");
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
      return;
    }
    
    /// Open the Position interface
    try {
      this->posIface->Open(client, modelName + "::position_iface_0");
    } catch (std::string e) {
      std::cout << "Gazebo error: Unable to connect to the position interface\n" << e << "\n";
      return;
    }
    
    // Enable the motor
    this->posIface->Lock(1);
    this->posIface->data->cmdEnableMotors = 1;
    this->posIface->Unlock();

    this->rnh_ = new ros::NodeHandle();
    this->sub_ = rnh_->subscribe<geometry_msgs::Twist>("cmd_vel", 100, &DiffDrive::cmdVelCallBack,this);
    this->pub_ = rnh_->advertise<nav_msgs::Odometry>("erratic_odometry/odom", 1);
   
    // spawn 2 threads by default, ///@todo: make this a parameter
    ros::MultiThreadedSpinner s(2);
    boost::thread spinner_thread( boost::bind( &ros::spin, s ) );

    nav_msgs::Odometry odom;

    // setup transform publishers, need to duplicate pr2_odometry controller functionalities
    tf::TransformBroadcaster transform_broadcaster_ ;

    ros::Duration d; d.fromSec(0.01);
    
    while(rnh_->ok()) { 
      if (this->posIface) {
        this->posIface->Lock(1);

        // get current time
        ros::Time current_time_ = ros::Time::now();

        // getting data for base_footprint to odom transform
        btQuaternion qt;
        qt.setEulerZYX(this->posIface->data->pose.yaw, this->posIface->data->pose.pitch, this->posIface->data->pose.roll);
        btVector3 vt(this->posIface->data->pose.pos.x, this->posIface->data->pose.pos.y, this->posIface->data->pose.pos.z);
        tf::Transform base_footprint_to_odom(qt, vt);
        
        transform_broadcaster_.sendTransform(tf::StampedTransform(base_footprint_to_odom,ros::Time::now(), "odom", "base_footprint"));

        // publish odom topic
        odom.pose.pose.position.x = this->posIface->data->pose.pos.x;
        odom.pose.pose.position.y = this->posIface->data->pose.pos.y;

        gazebo::Quatern rot;
        rot.SetFromEuler(gazebo::Vector3(this->posIface->data->pose.roll,this->posIface->data->pose.pitch,this->posIface->data->pose.yaw));

        odom.pose.pose.orientation.x = rot.x;
        odom.pose.pose.orientation.y = rot.y;
        odom.pose.pose.orientation.z = rot.z;
        odom.pose.pose.orientation.w = rot.u;

        odom.twist.twist.linear.x = this->posIface->data->velocity.pos.x;
        odom.twist.twist.linear.y = this->posIface->data->velocity.pos.y;
        odom.twist.twist.angular.z = this->posIface->data->velocity.yaw;

        odom.header.frame_id = tf::resolve(tf_prefix_, "odom"); 
        odom.header.stamp = ros::Time::now();

        this->pub_.publish(odom); 

        this->posIface->Unlock();
      }
      d.sleep();
    }
  }
  
  ~DiffDrive() {
    delete this->rnh_;
  }
};




int main(int argc, char** argv) {
  ros::init(argc,argv,"gazebo_ros_diffdrive",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

  DiffDrive d;
  return 0;
}


