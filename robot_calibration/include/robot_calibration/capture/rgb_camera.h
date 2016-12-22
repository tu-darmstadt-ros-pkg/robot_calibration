//=================================================================================================
// Copyright (c) 2016, Martin Oehler, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef RGB_CAMERA_H
#define RGB_CAMERA_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/ExtendedCameraInfo.h>

namespace robot_calibration {

class RGBCameraInfoManager {
public:
  RGBCameraInfoManager() : camera_info_valid_(false) {}

  bool init(ros::NodeHandle& nh) {
    camera_info_subscriber_ = nh.subscribe("/head_camera/rgb/camera_info", 1, &RGBCameraInfoManager::cameraInfoCallback, this);

    // Wait for camera_info
    int count = 100;
    while (--count)
    {
      if (camera_info_valid_)
      {
        return true;
      }
      ROS_INFO("Waiting for CameraInfo.");
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    ROS_ERROR("CameraInfo receive timed out.");
    return false;
  }

  robot_calibration_msgs::ExtendedCameraInfo getCameraInfo() {
    robot_calibration_msgs::ExtendedCameraInfo info;
    info.camera_info = *camera_info_ptr_;
    return info;
  }

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfo::Ptr camera_info)
  {
    camera_info_ptr_ = camera_info;
    camera_info_valid_ = true;
  }

  ros::Subscriber camera_info_subscriber_;
  bool camera_info_valid_;

  sensor_msgs::CameraInfo::Ptr camera_info_ptr_;
};

}  // namespace robot_calibration

#endif
