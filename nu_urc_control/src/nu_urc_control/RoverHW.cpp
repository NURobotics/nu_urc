#include <nu_urc_control/RoverHW.h>
#include <string>

namespace nu_urc_control {

RoverHW::RoverHW()
{
  resetJoints();
  for(int i = 0; i < js_.name.size(); i++) {
    hardware_interface::ActuatorStateHandle as_handle(js_.name[i].c_str(), &(js_.position[i]), &(js_.velocity[i]), &(js_.effort[i]));
    actuators_[i].registerHandle(as_handle);
    registerInterface(&(actuators_[i]));
  }
  
  js_publisher_ = nh_.advertise<sensor_msgs::JointState>("joints/set",10);
  js_subscriber_ = nh_.subscribe("joints/get", 100, &RoverHW::jointsCallback, this);
  sd_subscriber_ = nh_.subscribe("sensor/data", 100, &RoverHW::sensorsCallback, this);
}

void RoverHW::resetJoints()
{
  js_.name.resize(NUM_ACTUATORS);
  js_.name[0] = "front_left";
  js_.name[1] = "front_right";
  js_.name[2] = "mid_left";
  js_.name[3] = "mid_right";
  js_.name[4] = "back_left";
  js_.name[5] = "back_right";
  
  js_.position.resize(NUM_ACTUATORS);
  for(int i = 0; i < NUM_ACTUATORS; i++) js_.position[i] = 0.0;
  
  js_.velocity.resize(NUM_ACTUATORS);
  for(int i = 0; i < NUM_ACTUATORS; i++) js_.velocity[i] = 0.0;
  
  js_.effort.resize(NUM_ACTUATORS);
  for(int i = 0; i < NUM_ACTUATORS; i++) js_.effort[i] = 0.0;
  
  js_buffer_ = js_;
}

void RoverHW::jointsCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  // Check if the data for each of the joints exist and set accordingly
  for(std::vector<std::string>::iterator sit = js_buffer_.name.begin(); sit != js_buffer_.name.end(); sit++) {
    if(find(js->name.begin(), js->name.end(), *sit) != js->name.end()) {
      int index = find(js->name.begin(), js->name.end(), *sit)-js->name.begin();
      js_buffer_.position[index] = js->position[index];
      js_buffer_.velocity[index] = js->velocity[index];
      js_buffer_.effort[index] = js->effort[index];
    }
  }
}

void RoverHW::sensorsCallback(const nu_urc_msgs::SensorData::ConstPtr& sd)
{
  gps_data_ = sd->gps;
  imu_data_ = sd->imu;
}

void RoverHW::read()
{
  // Transfer the buffer data
  js_ = js_buffer_;
}

void RoverHW::write()
{
  // Write the date in as_store_ over rosserial
  // Publish actuator commands
  js_publisher_.publish(js_);
}

} // namespace nu_urc_control
