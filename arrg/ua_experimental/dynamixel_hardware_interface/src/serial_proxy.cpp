
// Author: Antons Rebguns

#include <stdint.h>
#include <time.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <XmlRpcValue.h>

#include <gearbox/flexiport/flexiport.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/serial_proxy.h>

#include <ros/ros.h>
#include <dynamixel_msgs/MotorState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace dynamixel_hardware_interface
{

SerialProxy::SerialProxy(std::string port_name,
                         std::string port_namespace,
                         std::string baud_rate,
                         int min_motor_id,
                         int max_motor_id,
                         double update_rate,
                         double diagnostics_rate,
                         int error_level_temp,
                         int warn_level_temp)
    :port_name_(port_name),
     port_namespace_(port_namespace),
     baud_rate_(baud_rate),
     min_motor_id_(min_motor_id),
     max_motor_id_(max_motor_id),
     update_rate_(update_rate),
     diagnostics_rate_(diagnostics_rate),
     error_level_temp_(error_level_temp),
     warn_level_temp_(warn_level_temp),
     freq_status_(diagnostic_updater::FrequencyStatusParam(&update_rate_, &update_rate_, 0.1, 25))
{
    current_state_ = dynamixel_msgs::MotorStateListPtr(new dynamixel_msgs::MotorStateList);

    motor_states_pub_ = nh_.advertise<dynamixel_msgs::MotorStateList>("motor_states/" + port_namespace_, 1000);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);
}

SerialProxy::~SerialProxy()
{
    if (feedback_thread_)
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            terminate_feedback_ = true;
        }
        feedback_thread_->join();
        delete feedback_thread_;
    }

    if (diagnostics_thread_)
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            terminate_diagnostics_ = true;
        }
        diagnostics_thread_->join();
        delete diagnostics_thread_;
    }

    delete dxl_io_;
}

void SerialProxy::connect()
{
    try
    {
        dxl_io_ = new DynamixelIO(port_name_, baud_rate_);
        findMotors();
    }
    catch (flexiport::PortException pex)
    {
        ROS_FATAL("%s", pex.what());
        ros::requestShutdown();
    }
    
    if (update_rate_ > 0)
    {
        terminate_feedback_ = false;
        feedback_thread_ = new boost::thread(boost::bind(&SerialProxy::updateMotorStates, this));
    }
    else
    {
        feedback_thread_ = NULL;
    }

    if (diagnostics_rate_ > 0)
    {
        terminate_diagnostics_ = false;
        diagnostics_thread_ = new boost::thread(boost::bind(&SerialProxy::publishDiagnosticInformation, this));
    }
    else
    {
        diagnostics_thread_ = NULL;
    }
}

DynamixelIO* SerialProxy::getSerialPort()
{
    return dxl_io_;
}

void SerialProxy::fillMotorParameters(int motor_id, uint16_t model_number)
{
    uint8_t firmware_version;
    uint8_t return_delay_time;
    uint16_t cw_angle;
    uint16_t ccw_angle;
    float min_voltage_limit;
    float max_voltage_limit;
    float voltage;

    dxl_io_->getFirmwareVersion(motor_id, firmware_version);
    dxl_io_->getReturnDelayTime(motor_id, return_delay_time);
    dxl_io_->getAngleLimits(motor_id, cw_angle, ccw_angle);
    dxl_io_->getVoltageLimits(motor_id, min_voltage_limit, max_voltage_limit);
    dxl_io_->getVoltage(motor_id, voltage);

    std::stringstream ss;
    ss << "dynamixel/" << port_namespace_ << "/" << motor_id << "/";
    std::string prefix = ss.str();

    nh_.setParam(prefix + "model_number", model_number);
    nh_.setParam(prefix + "model_name", getMotorModelName(model_number));
    nh_.setParam(prefix + "min_angle", cw_angle);
    nh_.setParam(prefix + "max_angle", ccw_angle);

    double torque_per_volt = getMotorModelParams(model_number, TORQUE_PER_VOLT);
    nh_.setParam(prefix + "torque_per_volt", torque_per_volt);
    nh_.setParam(prefix + "max_torque", torque_per_volt * voltage);

    double velocity_per_volt = getMotorModelParams(model_number, VELOCITY_PER_VOLT);
    nh_.setParam(prefix + "velocity_per_volt", velocity_per_volt);
    nh_.setParam(prefix + "max_velocity", velocity_per_volt * voltage);
    nh_.setParam(prefix + "radians_second_per_encoder_tick", velocity_per_volt * voltage / DXL_MAX_VELOCITY_ENCODER);

    int encoder_resolution = (int) getMotorModelParams(model_number, ENCODER_RESOLUTION);
    double range_degrees = getMotorModelParams(model_number, RANGE_DEGREES);
    double range_radians = range_degrees * M_PI / 180.0;

    nh_.setParam(prefix + "encoder_resolution", encoder_resolution);
    nh_.setParam(prefix + "range_degrees", range_degrees);
    nh_.setParam(prefix + "range_radians", range_radians);
    nh_.setParam(prefix + "encoder_ticks_per_degree", encoder_resolution / range_degrees);
    nh_.setParam(prefix + "encoder_ticks_per_radian", encoder_resolution / range_radians);
    nh_.setParam(prefix + "degrees_per_encoder_tick", range_degrees / encoder_resolution);
    nh_.setParam(prefix + "radians_per_encoder_tick", range_radians / encoder_resolution);
    
    // keep some parameters around for diagnostics
    std::vector<boost::any> static_info;
    static_info.resize(7);
    static_info[0] = getMotorModelName(model_number);
    static_info[1] = firmware_version;
    static_info[2] = return_delay_time;
    static_info[3] = min_voltage_limit;
    static_info[4] = max_voltage_limit;
    static_info[5] = cw_angle;
    static_info[6] = ccw_angle;

    motor_static_info_[motor_id] = static_info;
}

void SerialProxy::findMotors()
{
    ROS_INFO("Pinging motor IDs %d through %d...", min_motor_id_, max_motor_id_);

    XmlRpc::XmlRpcValue val;
    std::map<int, int> counts;
    std::stringstream ss;
    ss << "[";

    for (int motor_id = min_motor_id_; motor_id <= max_motor_id_; ++motor_id)
    {
        if (dxl_io_->ping(motor_id))
        {
            uint16_t model_number;
            dxl_io_->getModelNumber(motor_id, model_number);
            counts[model_number] += 1;
            fillMotorParameters(motor_id, model_number);

            motors_.push_back(motor_id);
            val[motors_.size()-1] = motor_id;
            ss << motor_id << ", ";
        }
    }

    std::string motors_found = ss.str();

    if (!motors_.empty())
    {
        motors_found.replace(motors_found.size()-2, 2, "]");
        ROS_INFO("Found motors with IDs: %s.", motors_found.c_str());
    }
    else
    {
        ROS_INFO("No motors found, aborting.");
        exit(1);
    }

    nh_.setParam("dynamixel/" + port_namespace_ + "/connected_ids", val);

    ss.str("");
    ss << "There are ";

    std::map<int, int>::iterator it;
    for (it = counts.begin(); it != counts.end(); ++it)
    {
        ss << (*it).second << " " << getMotorModelName((*it).first) << ", ";
    }

    std::string status = ss.str();
    status.replace(status.size()-2, 2, " servos connected");
    ROS_INFO("%s", status.c_str());
    ROS_INFO("Dynamixel Manager on port %s initialized", port_namespace_.c_str());
}

void SerialProxy::updateMotorStates()
{    
    current_state_->motor_states.resize(motors_.size());
    dynamixel_hardware_interface::DynamixelStatus status;
    ros::Rate rate(update_rate_);
    double r = 1.0e6/update_rate_;
    
    struct timespec ts_now;
    double start_time_usec, end_time_usec;
    int st = 0;
    
    while (nh_.ok())
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (terminate_feedback_) { break; }
        }

        clock_gettime(CLOCK_REALTIME, &ts_now);
        start_time_usec = ts_now.tv_sec * 1.0e6 + ts_now.tv_nsec / 1.0e3;
        
        for (size_t i = 0; i < motors_.size(); ++i)
        {
            int motor_id = motors_[i];

            if (dxl_io_->getFeedback(motor_id, status))
            {
                dynamixel_msgs::MotorState ms;
                ms.timestamp = status.timestamp;
                ms.id = motor_id;
                ms.goal = status.goal;
                ms.position = status.position;
                ms.error = status.position - status.goal;
                ms.speed = status.velocity;
                ms.load = status.load;
                ms.voltage = status.voltage;
                ms.temperature = status.temperature;
                ms.moving = status.moving;
                current_state_->motor_states[i] = ms;
            }
            else
            {
                ROS_DEBUG("Bad feedback received from motor %d on port %s", motor_id, port_namespace_.c_str());
            }
        }

        motor_states_pub_.publish(current_state_);
        freq_status_.tick();

        clock_gettime(CLOCK_REALTIME, &ts_now);
        end_time_usec = ts_now.tv_sec * 1.0e6 + ts_now.tv_nsec / 1.0e3;
        
        st = r - (end_time_usec - start_time_usec);
        
        if (st >= 1000)
        {
            st = (st / 1000) * 1000;
            usleep(st);
        }
        
        //rate.sleep();
    }
}

void SerialProxy::publishDiagnosticInformation()
{
    diagnostic_msgs::DiagnosticArray diag_msg;
    diagnostic_updater::DiagnosticStatusWrapper bus_status;
    ros::Rate rate(diagnostics_rate_);

    while (nh_.ok())
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (terminate_diagnostics_) { break; }
        }
        
        double error_rate = dxl_io_->read_error_count / (double) dxl_io_->read_count;

        bus_status.clear();
        bus_status.name = "Dynamixel Serial Bus (" + port_namespace_ + ")";
        bus_status.hardware_id = "Dynamixel Serial Bus on port " + port_name_;
        bus_status.add("Baud Rate", baud_rate_);
        bus_status.add("Min Motor ID", min_motor_id_);
        bus_status.add("Max Motor ID", max_motor_id_);
        bus_status.addf("Error Rate", "%0.5f", error_rate);
        bus_status.summary(bus_status.OK, "OK");
        
        freq_status_.run(bus_status);
        
        if (error_rate > 0.05)
        {
            bus_status.mergeSummary(bus_status.WARN, "Too many errors while reading/writing to/from Dynamixel bus");
        }
        
        diag_msg.status.clear();
        diag_msg.header.stamp = ros::Time::now();
        diag_msg.status.push_back(bus_status);
        
        for (size_t i = 0; i < current_state_->motor_states.size(); ++i)
        {
            dynamixel_msgs::MotorState motor_state = current_state_->motor_states[i];
            int mid = motor_state.id;
            
            // check if current motor state was already populated by updateMotorStates thread
            if (mid == 0) { continue; }
            
            std::string mid_str = boost::lexical_cast<std::string>(mid);
            
            diagnostic_updater::DiagnosticStatusWrapper motor_status;
            
            motor_status.name = "Robotis Dynamixel Motor " + mid_str + " on port " + port_namespace_;
            motor_status.hardware_id = "DXL-" + mid_str + "@" + port_namespace_;
            motor_status.add("Model Name", boost::any_cast<std::string>(motor_static_info_[mid][0]));
            motor_status.addf("Firmware Version", "%d", boost::any_cast<uint8_t>(motor_static_info_[mid][1]));
            motor_status.addf("Return Delay Time", "%d", boost::any_cast<uint8_t>(motor_static_info_[mid][2]));
            motor_status.addf("Minimum Voltage", "%0.1f", boost::any_cast<float>(motor_static_info_[mid][3]));
            motor_status.addf("Maximum Voltage", "%0.1f", boost::any_cast<float>(motor_static_info_[mid][4]));
            motor_status.addf("Minimum Position (CW)", "%d", boost::any_cast<uint16_t>(motor_static_info_[mid][5]));
            motor_status.addf("Maximum Position (CCW)", "%d", boost::any_cast<uint16_t>(motor_static_info_[mid][6]));
            
            motor_status.addf("Goal", "%d", motor_state.goal);
            motor_status.addf("Position", "%d", motor_state.position);
            motor_status.addf("Error", "%d", motor_state.error);
            motor_status.addf("Velocity", "%d", motor_state.speed);
            motor_status.addf("Load", "%d", motor_state.load);
            motor_status.addf("Voltage", "%0.1f", motor_state.voltage / 10.0f);
            motor_status.addf("Temperature", "%d", motor_state.temperature);
            motor_status.add("Moving", motor_state.moving ? "True" : "False");
            
            if (motor_state.temperature >= error_level_temp_)
            {
                motor_status.summary(motor_status.ERROR, "OVERHEATING");
            }
            else if (motor_state.temperature >= warn_level_temp_)
            {
                motor_status.summary(motor_status.WARN, "VERY HOT");
            }
            else
            {
                motor_status.summary(motor_status.OK, "OK");
            }
                
            diag_msg.status.push_back(motor_status);
        }
              
        diagnostics_pub_.publish(diag_msg);
        
        rate.sleep();
    }
}

}
