#ifndef NU_URC_ROVERHW_H
#define NU_URC_ROVERHW_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <nu_urc_msgs/SensorData.h>
#include <nu_urc_msgs/JointCommands.h>
#include <nu_urc_msgs/Imu.h>
#include <nu_urc_control/DriveHW.h>
#include <boost/shared_ptr.hpp>
#include <vector>

#define NUM_JOINTS 4
#define UPDATE_FREQ 100

namespace nu_urc_control {

class RoverHW : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:
  RoverHW();
  
  void update(const ros::TimerEvent& te);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& js);
  void sensorDataCallback(const nu_urc_msgs::SensorDataConstPtr& sd);
  
private:
  ros::NodeHandle nh_;
  ros::Timer update_timer_;
  
  boost::shared_ptr<nu_urc_control::DriveHW> drive_hw_;
  boost::shared_ptr<controller_manager::ControllerManager> cm_;
  
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  sensor_msgs::JointState robot_js_; 
  nu_urc_msgs::SensorData sensor_data_;
  
  ros::Subscriber js_subscriber_;
  ros::Subscriber sd_subscriber_;
  
  ros::Time prev_update_time_;
};

} // namespace nu_urc_control

#endif // NU_URC_ROVERHW_H
