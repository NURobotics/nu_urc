#include <nu_urc_control/RoverHW.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nu_urc_control");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner sp(8);
  sp.start();
  
  nu_urc_control::RoverHW rover_control;
  
  ros::spin();
  return 0;
}
