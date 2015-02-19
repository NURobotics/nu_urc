#include <nu_urc_control/RoverHW.h>
#include <string>

namespace nu_urc_control {

RoverHW::RoverHW()
{
  
  js_subscriber_ = nh_.subscribe("joints/state", 100, &RoverHW::jointStateCallback, this);
  sd_subscriber_ = nh_.subscribe("sensor/data", 100, &RoverHW::sensorDataCallback, this);
  
  drive_hw_.reset(new DriveHW());
  drive_hw_->init(js_interface_, vj_interface_);
  
  registerInterface(&js_interface_);
  registerInterface(&vj_interface_);
  
  cm_.reset(new controller_manager::ControllerManager(this, nh_));
  
  update_timer_ = nh_.createTimer(ros::Duration(1.0/UPDATE_FREQ), &RoverHW::update, this);
  prev_update_time_ = ros::Time::now();
}

void RoverHW::jointStateCallback(const sensor_msgs::JointStateConstPtr& js)
{
  // Swap joint state buffer
  robot_js_ = *js;
}

void RoverHW::sensorDataCallback(const nu_urc_msgs::SensorDataConstPtr& sd)
{
  // Swap out sensor data buffer
  sensor_data_ = *sd;
}

void RoverHW::update(const ros::TimerEvent& te)
{
  ros::Time curr_update_time = ros::Time::now();
  ros::Duration time_diff(te.current_real - te.last_real);
  
  drive_hw_->read(robot_js_);
  cm_->update(curr_update_time, time_diff);
  drive_hw_->write(time_diff); 
}

} // namespace nu_urc_control
