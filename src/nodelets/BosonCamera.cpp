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

#include "flir_boson_usb/BosonCamera.h"

using namespace cv;
using namespace flir_boson_usb;

BosonCamera::BosonCamera(const rclcpp::NodeOptions & options)
: Node("boson_camera", options),
  cv_img_()
{
  // Declare parameters
  this->declare_parameter("frame_id", "boson_camera");
  this->declare_parameter("dev", "/dev/video0");
  this->declare_parameter("frame_rate", 60.0);
  this->declare_parameter("video_mode", "RAW16");
  this->declare_parameter("zoom_enable", false);
  this->declare_parameter("sensor_type", "Boson_640");
  this->declare_parameter("camera_info_url", "");

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  dev_path_ = this->get_parameter("dev").as_string();
  frame_rate_ = this->get_parameter("frame_rate").as_double();
  video_mode_str_ = this->get_parameter("video_mode").as_string();
  zoom_enable_ = this->get_parameter("zoom_enable").as_bool();
  sensor_type_str_ = this->get_parameter("sensor_type").as_string();
  camera_info_url_ = this->get_parameter("camera_info_url").as_string();

  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got dev: %s", dev_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got frame rate: %f", frame_rate_);
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got video mode: %s", video_mode_str_.c_str());
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got zoom enable: %s", (zoom_enable_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got sensor type: %s", sensor_type_str_.c_str());
  RCLCPP_INFO(this->get_logger(), "flir_boson_usb - Got camera_info_url: %s", camera_info_url_.c_str());

  bool exit = false;

  if (video_mode_str_ == "RAW16")
  {
    video_mode_ = RAW16;
  }
  else if (video_mode_str_ == "YUV")
  {
    video_mode_ = YUV;
  }
  else
  {
    exit = true;
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - Invalid video_mode value provided. Exiting.");
  }

  if (sensor_type_str_ == "Boson_320" ||
      sensor_type_str_ == "boson_320")
  {
    sensor_type_ = Boson320;
    camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "Boson320");
  }
  else if (sensor_type_str_ == "Boson_640" ||
           sensor_type_str_ == "boson_640")
  {
    sensor_type_ = Boson640;
    camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "Boson640");
  }
  else
  {
    exit = true;
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - Invalid sensor_type value provided. Exiting.");
  }

  if (camera_info_->validateURL(camera_info_url_))
  {
    camera_info_->loadCameraInfo(camera_info_url_);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "flir_boson_usb - camera_info_url could not be validated. Publishing with unconfigured camera.");
  }

  if (!exit)
  {
    exit = openCamera() ? exit : true;
  }

  if (!exit)
  {
    it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
    image_pub_ = it_->advertiseCamera("image_raw", 1);

    using namespace std::chrono_literals;
    auto timer_period = std::chrono::duration<double>(1.0 / frame_rate_);
    capture_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&BosonCamera::captureAndPublish, this));
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - Failed to initialize camera. Shutting down node.");
    rclcpp::shutdown();
  }
}

BosonCamera::~BosonCamera()
{
  closeCamera();
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void BosonCamera::agcBasicLinear(const Mat& input_16,
                               Mat* output_8,
                               const int& height,
                               const int& width)
{
  int i, j;  // aux variables

  // auxiliary variables for AGC calcultion
  unsigned int max1 = 0;         // 16 bits
  unsigned int min1 = 0xFFFF;    // 16 bits
  unsigned int value1, value2, value3, value4;

  // RUN a super basic AGC
  for (i = 0; i < height; i++)
  {
    for (j = 0; j < width; j++)
    {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;  // High Byte
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;      // Low Byte
      value3 = (value1 << 8) + value2;

      if (value3 <= min1)
        min1 = value3;

      if (value3 >= max1)
        max1 = value3;
    }
  }

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;     // High Byte
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;         // Low Byte
      value3 = (value1 << 8) + value2;
      value4 = ((255 * (value3 - min1))) / (max1 - min1);

      output_8->at<uchar>(i, j) = static_cast<uint8_t>(value4 & 0xFF);
    }
  }
}

bool BosonCamera::openCamera()
{
  // Open the Video device
  if ((fd_ = open(dev_path_.c_str(), O_RDWR)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - ERROR : OPEN. Invalid Video Device.");
    return false;
  }

  // Check VideoCapture mode is available
  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap_) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - ERROR : VIDIOC_QUERYCAP. Video Capture is not available.");
    return false;
  }

  if (!(cap_.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - The device does not handle single-planar video capture.");
    return false;
  }

  struct v4l2_format format;

  // Two different FORMAT modes, 8 bits vs RAW16
  if (video_mode_ == RAW16)
  {
    // I am requiring thermal 16 bits mode
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

    // Select the frame SIZE (will depend on the type of sensor)
    switch (sensor_type_)
    {
      case Boson320:  // Boson320
        width_ = 320;
        height_ = 256;
        break;
      case Boson640:  // Boson640
        width_ = 640;
        height_ = 512;
        break;
      default:  // Boson320
        width_ = 320;
        height_ = 256;
        break;
    }
  }
  else  // 8- bits is always 640x512 (even for a Boson 320)
  {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;  // thermal, works   LUMA, full Cr, full Cb
    width_ = 640;
    height_ = 512;
  }

  // Common varibles
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = width_;
  format.fmt.pix.height = height_;

  // request desired FORMAT
  if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_S_FMT error. The camera does not support the requested video format.");
    return false;
  }

  // we need to inform the device about buffers to use.
  // and we need to allocate them.
  // we'll use a single buffer, and map our memory using mmap.
  // All this information is sent using the VIDIOC_REQBUFS call and a
  // v4l2_requestbuffers structure:
  struct v4l2_requestbuffers bufrequest;
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 1;   // we are asking for one buffer

  if (ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_REQBUFS error. The camera failed to allocate a buffer.");
    return false;
  }

  // Now that the device knows how to provide its data,
  // we need to ask it about the amount of memory it needs,
  // and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
  // and its v4l2_buffer structure.

  memset(&bufferinfo_, 0, sizeof(bufferinfo_));

  bufferinfo_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo_.memory = V4L2_MEMORY_MMAP;
  bufferinfo_.index = 0;

  if (ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo_) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_QUERYBUF error. Failed to retreive buffer information.");
    return false;
  }

  // map fd+offset into a process location (kernel will decide due to our NULL). length and
  // properties are also passed
  buffer_start_ = mmap(NULL, bufferinfo_.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, bufferinfo_.m.offset);

  if (buffer_start_ == MAP_FAILED)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - mmap error. Failed to create a memory map for buffer.");
    return false;
  }

  // Fill this buffer with ceros. Initialization. Optional but nice to do
  memset(buffer_start_, 0, bufferinfo_.length);

  // Activate streaming
  int type = bufferinfo_.type;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_STREAMON error. Failed to activate streaming on the camera.");
    return false;
  }

  // Declarations for RAW16 representation
  // Will be used in case we are reading RAW16 format
  // Boson320 , Boson 640
  // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
  thermal16_ = Mat(height_, width_, CV_16U, buffer_start_);
  // OpenCV output buffer : Data used to display the video
  thermal16_linear_ = Mat(height_, width_, CV_8U, 1);

  // Declarations for 8bits YCbCr mode
  // Will be used in case we are reading YUV format
  // Boson320, 640 :  4:2:0
  int luma_height = height_+height_/2;
  int luma_width = width_;
  int color_space = CV_8UC1;

  // Declarations for Zoom representation
  // Will be used or not depending on program arguments
  thermal_luma_ = Mat(luma_height, luma_width,  color_space, buffer_start_);  // OpenCV input buffer
  // OpenCV output buffer , BGR -> Three color spaces :
  // (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)
  thermal_rgb_ = Mat(height_, width_, CV_8UC3, 1);

  return true;
}

bool BosonCamera::closeCamera()
{
  // Finish loop. Exiting.
  // Deactivate streaming
  int type = bufferinfo_.type;
  if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0 )
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_STREAMOFF error. Failed to disable streaming on the camera.");
    return false;
  };

  close(fd_);

  return true;
}

void BosonCamera::captureAndPublish()
{
  Size size(640, 512);

  auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_->getCameraInfo());
  ci->header.frame_id = frame_id_;

  // Put the buffer in the incoming queue.
  if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo_) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_QBUF error. Failed to queue the image buffer.");
    return;
  }

  // The buffer's waiting in the outgoing queue.
  if (ioctl(fd_, VIDIOC_DQBUF, &bufferinfo_) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "flir_boson_usb - VIDIOC_DQBUF error. Failed to dequeue the image buffer.");
    return;
  }

  if (video_mode_ == RAW16)
  {
    // -----------------------------
    // RAW16 DATA
    agcBasicLinear(thermal16_, &thermal16_linear_, height_, width_);

    // Display thermal after 16-bits AGC... will display an image
    if (!zoom_enable_)
    {
      // Threshold using Otsu's method, then use the result as a mask on the original image
      Mat mask_mat, masked_img;
      threshold(thermal16_linear_, mask_mat, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
      thermal16_linear_.copyTo(masked_img, mask_mat);

      // Normalize the pixel values to the range [0, 1] then raise to power (gamma). Then convert back for display.
      Mat d_out_img, norm_image, d_norm_image, gamma_corrected_image, d_gamma_corrected_image;
      double gamma = 0.8;
      masked_img.convertTo(d_out_img, CV_64FC1);
      normalize(d_out_img, d_norm_image, 0, 1, NORM_MINMAX, CV_64FC1);
      pow(d_out_img, gamma, d_gamma_corrected_image);
      d_gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);
      normalize(gamma_corrected_image, gamma_corrected_image, 0, 255, NORM_MINMAX, CV_8UC1);

      // Apply top hat filter
      int erosion_size = 5;
      Mat top_hat_img, kernel = getStructuringElement(MORPH_ELLIPSE,
          Size(2 * erosion_size + 1, 2 * erosion_size + 1));
      morphologyEx(gamma_corrected_image, top_hat_img, MORPH_TOPHAT, kernel);

      cv_img_.image = thermal16_linear_;
      cv_img_.header.stamp = this->now();
      cv_img_.header.frame_id = frame_id_;
      cv_img_.encoding = "mono8";
      auto pub_image = cv_img_.toImageMsg();

      ci->header.stamp = pub_image->header.stamp;
      image_pub_.publish(*pub_image, *ci);
    }
    else
    {
      resize(thermal16_linear_, thermal16_linear_zoom_, size);

      cv_img_.image = thermal16_linear_zoom_;
      cv_img_.header.stamp = this->now();
      cv_img_.header.frame_id = frame_id_;
      cv_img_.encoding = "mono8";
      auto pub_image = cv_img_.toImageMsg();

      ci->header.stamp = pub_image->header.stamp;
      image_pub_.publish(*pub_image, *ci);
    }
  }
  else  // Video is in 8 bits YUV
  {
    // ---------------------------------
    // DATA in YUV
    cvtColor(thermal_luma_, thermal_rgb_, COLOR_YUV2GRAY_I420, 0);

    cv_img_.image = thermal_rgb_;
    cv_img_.encoding = "mono8";
    cv_img_.header.stamp = this->now();
    cv_img_.header.frame_id = frame_id_;
    auto pub_image = cv_img_.toImageMsg();

    ci->header.stamp = pub_image->header.stamp;
    image_pub_.publish(*pub_image, *ci);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_usb::BosonCamera)