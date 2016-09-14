#include <robot_calibration/capture/checkerboard_finder2d.h>

namespace robot_calibration
{

CheckerboardFinder2D::CheckerboardFinder2D(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(false)
{
  ROS_INFO_STREAM("Starting Checkerboard finder 2D!");
  // Setup Scriber
  image_sub_ = nh.subscribe("image", 1, &CheckerboardFinder2D::cameraCallback, this);

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 5);
  nh.param<int>("points_y", points_y_, 4);
  nh.param<double>("size", square_size_, 0.0245);

  // Should we output debug image/cloud
  nh.param<bool>("debug", output_debug_, false);

  // Get sensor names
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where checkerboard points were seen
  corners_pub_ = nh.advertise<sensor_msgs::Image>("checkerboard_corners", 10);
  corner_pose_pub_ = nh.advertise<visualization_msgs::MarkerArray>("checkerboard_corner_poses", 10);

  // Setup to get camera depth info
  if (!rgb_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}

void CheckerboardFinder2D::publishCornerPoses(const std::vector<geometry_msgs::PointStamped>& features) {
  visualization_msgs::MarkerArray array;
  for (unsigned int i = 0; i < features.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = features[i].header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "corners";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = features[i].point.x;
    marker.pose.position.y = features[i].point.y;
    marker.pose.position.z = features[i].point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    array.markers.push_back(marker);
  }
  corner_pose_pub_.publish(array);
}

void CheckerboardFinder2D::cameraCallback(const sensor_msgs::Image &image)
{
  if (waiting_)
  {
    image_ = image;
    waiting_ = false;
  }
}

// Returns true if we got a message, false if we timeout
bool CheckerboardFinder2D::waitForImage()
{
  // Initial wait cycle so that camera is definitely up to date.
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get image");
  return !waiting_;
}

bool CheckerboardFinder2D::find(robot_calibration_msgs::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    if (findInternal(msg))
      return true;
  }
  ROS_INFO_STREAM("No checkerboard found in 50 frames.");
  return false;
}

bool CheckerboardFinder2D::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgb;
  geometry_msgs::PointStamped world;

  // Get cloud
  if (!waitForImage())
  {
    ROS_ERROR("No image data");
    return false;
  }

  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(image_, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Conversion failed: " << e.what());
    return false;
  }

  // Find checkerboard
  std::vector<cv::Point2f> corners;
  corners.resize(points_x_ * points_y_);
  cv::Size checkerboard_size(points_x_, points_y_);
  bool found = cv::findChessboardCorners(cv_image->image, checkerboard_size,
                                        corners, CV_CALIB_CB_ADAPTIVE_THRESH);

  ROS_INFO_STREAM("Found corners: " << corners.size());

  std::vector<cv::Point2f> corners_small;
  corners_small.push_back(corners[12]);

  cv::drawChessboardCorners(cv_image->image, checkerboard_size, cv::Mat(corners_small), found);
  sensor_msgs::ImagePtr corner_image_ptr(cv_image->toImageMsg());
  corners_pub_.publish(*corner_image_ptr);

  if (found) {
    ROS_INFO("Found the checkboard");

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;

    msg->observations[idx_cam].features.resize(points_x_ * points_y_);
    msg->observations[idx_chain].features.resize(points_x_ * points_y_);


    // Fill in the headers
    rgb.header = image_.header;
    world.header.frame_id = "base_checkerboard_link";

    // Fill in message
    for (size_t i = 0; i < corners.size(); ++i)
    {
      world.point.y = (i % points_x_) * square_size_ - points_x_*square_size_/2 + square_size_/2;
      world.point.z = (i / points_x_) * square_size_ - points_y_*square_size_/2 + square_size_/2;

      rgb.point.x = corners[i].x;
      rgb.point.y = corners[i].y;
      rgb.point.z = 0;

      // Do not accept NANs
      if (std::isnan(rgb.point.x) ||
          std::isnan(rgb.point.y) ||
          std::isnan(rgb.point.z))
      {
        ROS_ERROR_STREAM("NAN point on " << i);
        return false;
      }

      msg->observations[idx_cam].features[i] = rgb;
      msg->observations[idx_cam].ext_camera_info = rgb_camera_manager_.getCameraInfo();
      msg->observations[idx_chain].features[i] = world;
    }

    std::vector<geometry_msgs::PointStamped> features_small;
    features_small.push_back(msg->observations[1].features[12]);
    publishCornerPoses(features_small);

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[0].image = image_;
    }

    // Publish results

    // Found all points
    return true;
  }

  return false;
}

}  // namespace robot_calibration
