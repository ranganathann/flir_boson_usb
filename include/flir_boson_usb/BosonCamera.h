/*
 * Copyright © 2019 AutonomouStuff, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef FLIR_BOSON_USB_BOSONCAMERA_H
#define FLIR_BOSON_USB_BOSONCAMERA_H

// C++ Includes
#include <string>
#include <memory>

// Linux system includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// OpenCV Includes
#include <opencv2/opencv.hpp>

// ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace flir_boson_usb
{

enum Encoding
{
  YUV = 0,
  RAW16 = 1
};

enum SensorTypes
{
  Boson320,
  Boson640
};

class BosonCamera : public rclcpp::Node
{
  public:
    explicit BosonCamera(const rclcpp::NodeOptions & options);
    virtual ~BosonCamera();

  private:
    void agcBasicLinear(const cv::Mat& input_16,
                      cv::Mat* output_8,
                      const int& height,
                      const int& width);
    bool openCamera();
    bool closeCamera();
    void captureAndPublish();

    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraPublisher image_pub_;
    cv_bridge::CvImage cv_img_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    int32_t width_, height_;
    int32_t fd_;
    int32_t i_;
    struct v4l2_capability cap_;
    int32_t frame_ = 0;                // First frame number enumeration
    int8_t thermal_sensor_name_[20];  // To store the sensor name
    struct v4l2_buffer bufferinfo_;
    void* buffer_start_;

    cv::Mat thermal16_, thermal16_linear_, thermal16_linear_zoom_,
            thermal_rgb_zoom_, thermal_luma_, thermal_rgb_;

    // Default Program options
    std::string frame_id_, dev_path_, camera_info_url_,
      video_mode_str_, sensor_type_str_;
    float frame_rate_;
    Encoding video_mode_;
    bool zoom_enable_;
    SensorTypes sensor_type_;
};

}  // namespace flir_boson_usb

#endif  // FLIR_BOSON_USB_BOSONCAMERA_H