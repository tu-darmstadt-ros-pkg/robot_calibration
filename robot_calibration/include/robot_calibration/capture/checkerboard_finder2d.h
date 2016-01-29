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

#ifndef CHECKERBOARD_FINDER_2D_H
#define CHECKERBOARD_FINDER_2D_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <robot_calibration/capture/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration/capture/rgb_camera.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

namespace robot_calibration
{

/**
 *  \brief This class processes the point cloud input to find a checkerboard
 */
class CheckerboardFinder2D : public FeatureFinder
{
public:
  CheckerboardFinder2D(ros::NodeHandle & nh);

  /**
   * \brief Attempts to find the checkerboard incoming data.
   * \param msg CalibrationData instance to fill in with point information.
   * \param points_x Number of checkerboard points in x
   * \param points_y Number of checkerboard points in y
   * \returns True if point has been filled in.
   */
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  bool findInternal(robot_calibration_msgs::CalibrationData * msg);

  void cameraCallback(const sensor_msgs::Image& image);
  bool waitForImage();

  ros::Subscriber image_sub_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher corners_pub_;   /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  sensor_msgs::Image image_;
  RGBCameraInfoManager rgb_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;        /// Size of checkerboard
  int points_y_;        /// Size of checkerboard

  double square_size_;     /// Size of a square on checkboard (in meters)

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
