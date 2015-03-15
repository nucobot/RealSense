#include <ros/ros.h>
#include <realsense_cam/realsense_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, camera_name_, camera_info_url_;
  int image_width_, image_height_, framerate_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  UsbCamNode() :
      node_("~")
  {

    // specify settings for images and camera for Realsense camera
    image_width_ = 640;
    image_height_ = 480;
    framerate_ = 30;

    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("realsense_camera"));
    node_.param("camera_name", camera_name_, std::string("realsense_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));
    // check for default camera info
    if (camera_info_url_.size() == 0)
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }

    ROS_INFO("Starting '%s' (%s) at %dx%d, %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, image_width_, image_height_, framerate_);
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    img_.encoding = "16UC1";

    // publish the image
    image_pub_.publish(img_, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (!take_and_send_image())
        ROS_WARN("Realsense camera did not respond in time.");
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
