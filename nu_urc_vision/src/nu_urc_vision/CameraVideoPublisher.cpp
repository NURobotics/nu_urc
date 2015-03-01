#include <nu_urc_vision/CameraVideoPublisher.h>

using namespace nu_urc_vision;

CameraVideoPublisher::CameraVideoPublisher(std::string param_dir_) :
  nh_(),
  param_dir_path_(param_dir_),
  it_(nh_),
  camera_video_capture_(0),
  camera_image_publisher_(0),
  camera_calib_service_(0)
{
  // Set the parameter directory
  // TO DO:
  // Handle case in which the last character is not "/"
  nh_.param<std::string>("calib_dir", param_dir_path_, param_dir_path_);

  // Find the number of cameras available
  bool camera_limit_reached = false;
  do {
    cv::VideoCapture t_cap(camera_video_capture_.size());

    // Checking if video capture is working by loading an image
    // Using isOpened doesn't have enough of a delay
    cv::Mat t_image;
    t_cap >> t_image;
    camera_limit_reached = t_image.empty();

    if(!camera_limit_reached) camera_video_capture_.push_back(t_cap);
  } while(!camera_limit_reached);
  
  // Publish the images and details for each camera
  for(int i = 0; i < camera_video_capture_.size(); i++) {
    std::stringstream ss;
    ss << "camera_" << i;
    std::string camera_name = ss.str();

    image_transport::Publisher t_pub = it_.advertise(camera_name.c_str(), 1);
    camera_image_publisher_.push_back(t_pub);
  }

  // Define the service used to set the camera calibration parameters
  for(int i = 0; i < camera_video_capture_.size(); i++) {
    std::stringstream ss;
    ss << "camera_" << i << "/set_camera_info";
    std::string camera_service_name = ss.str();

    ros::ServiceServer t_camera_service = nh_.advertiseService<SetCameraInfoRequest, SetCameraInfoResponse>(camera_service_name.c_str(), boost::bind(&CameraVideoPublisher::setCameraInfo, this, _1, _2));
    camera_calib_service_.push_back( t_camera_service );
  }
}

void CameraVideoPublisher::publishCameraImages()
{
  // Iterator through all video captures and publish image
  for(int i = 0; i < camera_video_capture_.size(); i++) {
    cv::Mat image;
    if(camera_video_capture_[i].read(image)) {
      std_msgs::Header image_header;      // Defining image size and timestamp. seq deprecated.
      image_header.stamp = ros::Time::now();
      std::stringstream ss;
      ss << "camera_" << i;
      image_header.frame_id = ss.str();
      
      std::string image_encoding = "bgr8";    // Assumed default encoding
      
      cv_bridge::CvImage image_bridge(image_header, image_encoding, image);
      camera_image_publisher_[i].publish( *image_bridge.toImageMsg() );
    }
    else {
      ROS_ERROR("Could not read from camera %d.", i);
    }
  }
}

void CameraVideoPublisher::publishCameraImages(const ros::TimerEvent& te)
{
  publishCameraImages();
}

bool CameraVideoPublisher::setCameraInfo(SetCameraInfoRequest& req, SetCameraInfoResponse& res)
{
  // Extract the camera's id through the frame_id
  int camera_id = atoi(req.camera_info.header.frame_id.c_str());
  if(camera_id >= 0 && camera_id < camera_video_capture_.size()) {
    // Store parameters in configuration file
    std::stringstream image_filename;
    image_filename << param_dir_path_ << "camera_calib_" << camera_id << "_" << std::time(0) << ".dat";
    std::fstream f_handle(image_filename.str().c_str());  
    if(f_handle.is_open()) {
      f_handle << req.camera_info;
      res.success = true;
    }
    else {
      ROS_ERROR("Invalid filename for calibration parameters: %s.", image_filename.str().c_str());
      std::stringstream s_message;
      s_message << "Invalid filename for calibration parameters: " << image_filename.str() << ".";
      res.status_message = s_message.str();
      res.success = false;
    }
    res.success = true;
  }
  else {
    ROS_ERROR("Invalid frame_id %d.", camera_id);
    std::stringstream s_message;
    s_message << "Invalid frame_id " << camera_id << ".";
    res.status_message = s_message.str();
    res.success = false;
  }
  return res.success;
}
