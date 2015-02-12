#ifndef NU_URC_ROVERHW
#define NU_URC_ROVERHW

#include <ros/ros.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <nu_urc_msgs/SensorData.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <vector>

#define NUM_ACTUATORS 6

namespace nu_urc_control {

class RoverHW : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:
  RoverHW();
  
  void read();
  void write();
  
  void jointsCallback(const sensor_msgs::JointState::ConstPtr&);
  void sensorsCallback(const nu_urc_msgs::SensorData::ConstPtr&);
  
  void resetJoints();
private:
  hardware_interface::ActuatorStateInterface actuators_[NUM_ACTUATORS];
  sensor_msgs::JointState js_buffer_; 
  sensor_msgs::JointState js_; 
  sensor_msgs::NavSatFix gps_data_;
  sensor_msgs::Imu imu_data_;
  
  ros::NodeHandle nh_;
  ros::Publisher js_publisher_;
  ros::Subscriber js_subscriber_;
  ros::Subscriber sd_subscriber_;
};

} // namespace nu_urc_control

#endif // NU_URC_ROVERHW
