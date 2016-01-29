#include <ros/ros.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration/capture/checkerboard_finder2d.h>
//#include "../robot_calibration/include/robot_calibration/capture/checkerboard_finder2d.h"

int main(int argc, char** argv) {
  ros::init(argc, argv,"checkerboard_finder_2d");

  robot_calibration_msgs::CalibrationData msg;

  ros::NodeHandle pnh("~");
  robot_calibration::CheckerboardFinder2D finder(pnh);

  finder.find(&msg);


  return 0;
}
