#include <nu_urc_control/RoverHW.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nu_urc_control");
  ros::NodeHandle nh;
  
  nu_urc_control::RoverHW rover_control;
  controller_manager::ControllerManager cm(&rover_control);
  
  ros::AsyncSpinner sp(2);
  sp.start();
  
  ros::Time t_previous = ros::Time::now();
  ros::Rate cycle_rate(100);

  while(ros::ok()) {
    const ros::Time t_current = ros::Time::now();
    const ros::Duration period = t_current - t_previous;
  
    rover_control.read();
    cm.update(t_current, period);
    rover_control.write();
    
    t_previous = t_current;
    cycle_rate.sleep();
  }
}
