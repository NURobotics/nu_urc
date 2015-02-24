#include <nu_urc_control/DriveHW.h>

namespace nu_urc_control {

DriveHW::DriveHW()
{
  // Populate the joint name
  js_.name.push_back("front_left_wheel");
  js_.name.push_back("front_right_wheel");
  js_.name.push_back("back_left_wheel");
  js_.name.push_back("back_right_wheel");
  
  // Resize joints with respect to joint name
  js_.position.resize(NUM_DRIVE_JOINTS);
  js_.velocity.resize(NUM_DRIVE_JOINTS);
  js_.effort.resize(NUM_DRIVE_JOINTS);
  
  jc_.names.push_back("front_left_wheel");
  jc_.names.push_back("front_right_wheel");
  jc_.names.push_back("back_left_wheel");
  jc_.names.push_back("back_right_wheel");
  jc_.commands.resize(NUM_DRIVE_JOINTS);
  
  jc_publisher_ = nh_.advertise<nu_urc_msgs::JointCommands>("/joints/drive/cmd",100);
}

void DriveHW::init(hardware_interface::JointStateInterface& js_interface, 
              hardware_interface::VelocityJointInterface& vj_interface) 
{
  // Register all of the joint handles
  for(int i = 0; i < js_.name.size(); i++) {
    std::vector<std::string>::const_iterator fit = find(js_.name.begin(), js_.name.end(), js_.name[i]);
    if(fit != js_.name.end()) {
      int index = fit - js_.name.begin();
      js_interface.registerHandle(
        hardware_interface::JointStateHandle(
          js_.name[i],
          &(js_.position[index]), 
          &(js_.velocity[index]), 
          &(js_.effort[index])));
      vj_interface.registerHandle(
        hardware_interface::JointHandle(
          js_interface.getHandle(js_.name[i]),
          &(jc_.commands[i])));
    } 
  }
}

void DriveHW::read(const sensor_msgs::JointState& j_state) {
  // Read in all of the drive joints from the robot's state
  for(int i = 0; i < js_.name.size(); i++) {
    std::vector<std::string>::const_iterator fit = find(j_state.name.begin(), j_state.name.end(), js_.name[i]);
    if(fit != j_state.name.end()) {
      int index = fit - j_state.name.begin();
      js_.position[i] = j_state.position[index];
      js_.velocity[i] = j_state.velocity[index];
      js_.effort[i] = j_state.effort[index];
    }
    else {
      // Call out an error message
    }
  }
}

void DriveHW::write(ros::Duration time_diff) {
  // Publish over rosserial
  jc_publisher_.publish(jc_);
}

} // namepace nu_urc_control
