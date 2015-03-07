#ifndef NU_URC_DRIVEHW_H
#define NU_URC_DRIVEHW_H

#define NUM_DRIVE_JOINTS      4

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <nu_urc_msgs/JointCommands.h>
#include <algorithm>

namespace nu_urc_control {

class ArmHW
{
public:
  ArmHW();
  
  void init(hardware_interface::JointStateInterface& js_interface,
            hardware_interface::PositionyJointInterface& pj_interface);
  void read(const sensor_msgs::JointState& j_state);
  void write(ros::Duration time_diff);
  
private:
  ros::NodeHandle nh_;
  ros::Publisher jc_publisher_;
  
  nu_urc_msgs::JointCommands jc_;
  sensor_msgs::JointState js_;
};

} // namespace nu_urc_control

#endif // NU_URC_DRIVEHW_H
