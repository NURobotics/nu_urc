#include <nu_urc_control/VelocityPIDController.h>
#include <string>

using namespace std;

namespace nu_urc_control {

bool VelocityPIDController::init(hardware_interface::VelocityActuatorInterface* hw, ros::NodeHandle &nh)
{
  // Initialize PID integral and derivative measurement terms
  std::string actuator_name;
  if(!nh.getParam("actuator", actuator_name)) {
    ROS_ERROR("Did not set the actuator string parameter on the parameter server.");
    return false;
  }
  
  actuator_ = hw->getHandle(actuator_name);
  integral_error_ = 0.0;
  discrete_velocities_[1] = actuator_.getVelocity();
  discrete_times_[1] = ros::Time::now().toSec();
  discrete_error_[1] = discrete_error_[0] = 0.0;
  
  return true;
}

void VelocityPIDController::update(const ros::Time &t, const ros::Duration &d)
{
  discrete_velocities_[0] = discrete_velocities_[1];
  discrete_velocities_[1] = actuator_.getVelocity();
  discrete_times_[0] = discrete_times_[1];
  discrete_times_[1] = ros::Time::now().toSec();
  discrete_error_[0] = discrete_error_[1];
  
  double error = velocity_goal_ - discrete_velocities_[1];
  discrete_error_[1] = error;
  
  double time_diff = discrete_times_[1]-discrete_times_[0];
  integral_error_ += error*time_diff;
  double derivative_error = (discrete_error_[1]-discrete_error_[0])/time_diff;
  double velocity_control = P*error + I*integral_error_ + D*derivative_error;
  
  actuator_.setCommand(velocity_control);
}

void VelocityPIDController::set_pid(double p, double i, double d)
{
  // Make sure that change is safe
  P = p;
  I = i;
  D = d;
}

} // namespace nu_urc_control
