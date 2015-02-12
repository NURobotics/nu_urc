#ifndef NU_URC_VELOCITYPIDCONTROLLER_H
#define NU_URC_VELOCITYPIDCONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/actuator_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace nu_urc_control {

class VelocityPIDController : public controller_interface::Controller<hardware_interface::VelocityActuatorInterface>
{
public:
  bool init(hardware_interface::VelocityActuatorInterface* hw, ros::NodeHandle &nh);
  void update(const ros::Time &t, const ros::Duration &d);

  void starting(const ros::Time &t) {}
  void stopping(const ros::Time &t) {}

  void set_pid(double p, double i, double d);
  double get_p() { return P; }
  double get_i() { return I; }
  double get_d() { return D; }
private:
  hardware_interface::ActuatorHandle actuator_;
  
  double P;
  double I;
  double D; 
  
  double integral_error_;
  double discrete_velocities_[2];
  double discrete_error_[2];
  double discrete_times_[2];
  
  double velocity_goal_;
};

} // namespace nu_urc_control

PLUGINLIB_EXPORT_CLASS(nu_urc_control::VelocityPIDController, controller_interface::ControllerBase);



#endif // NU_URC_VELOCITYPIDCONTROLLER
