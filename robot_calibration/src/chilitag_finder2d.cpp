#include <robot_calibration/capture/chilitag_finder2d.h>

namespace robot_calibration
{

ChilitagFinder2D::ChilitagFinder2D(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(false)
{
  ROS_INFO_STREAM("Starting Chilitag finder 2D!");
  // Setup Scriber

  image_sub_ = nh.subscribe("image", 1, &ChilitagFinder2D::cameraCallback, this);

  detector_.setFilter(0, 0);
  detector_.setPerformance(chilitags::Chilitags::ROBUST);

  // Load parameters
  nh.param<double>("size", marker_size_, 0.03);
  nh.param<std::vector<int>>("tag_ids", tag_ids_, std::vector<int>());
  if (tag_ids_.size() == 0) {
    ROS_WARN_STREAM("No tag ids set for detection. Detecting all tags (not recommendet).");
  }

  // Should we output debug image/cloud
  nh.param<bool>("debug", output_debug_, false);

  // Get sensor names
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where chilitags were seen
  image_transport::ImageTransport it(nh);
  tag_pub_ = it.advertise("chilitags", 10);
  tag_pose_pub_ = nh.advertise<visualization_msgs::MarkerArray>("tag_poses", 10);

  // Setup to get camera depth info
  if (!rgb_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}

void ChilitagFinder2D::publishTagPoses(const std::vector<geometry_msgs::PointStamped>& features) {
  visualization_msgs::MarkerArray array;
  for (unsigned int i = 0; i < features.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = features[i].header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "chilitags";
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
  tag_pose_pub_.publish(array);
}

void ChilitagFinder2D::cameraCallback(const sensor_msgs::Image &image)
{
  if (waiting_)
  {
    image_ = image;
    waiting_ = false;
  }
}


// Returns true if we got a message, false if we timeout
bool ChilitagFinder2D::waitForImage()
{
  // Initial wait cycle so that camera is definitely up to date.
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count && ros::ok())
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

bool ChilitagFinder2D::find(robot_calibration_msgs::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    if (findInternal(msg))
      return true;
    if (!ros::ok())
      return false;
  }
  ROS_INFO_STREAM("No chilitags found in 50 frames.");
  return false;
}

bool ChilitagFinder2D::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  // Get image
  if (!waitForImage())
  {
    ROS_ERROR("No image data");
    return false;
  }

  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(image_);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Conversion failed: " << e.what());
    return false;
  }

  // Find chilitags
  chilitags::TagCornerMap map = detector_.find(cv_image->image, chilitags::Chilitags::DETECT_ONLY);

  bool found = (map.size() > 0);

  ROS_INFO_STREAM("Found " << map.size() << " tags.");
  std::stringstream ids_ss;
  ids_ss << "Found ids: ";
  for (auto& kv: map) {
    ids_ss << kv.first << " ";
  }
  ROS_INFO_STREAM(ids_ss.str());

  for (auto kv = map.cbegin(); kv != map.cend();  ) {
    // Check if id is in list of valid tag ids
    if (tag_ids_.size() > 0 && std::find(tag_ids_.begin(), tag_ids_.end(), kv->first) == tag_ids_.end()) {
      ROS_WARN_STREAM("Erasing id " << kv->first << " because it is not contained in the valid ids list.");
      map.erase(kv++);
    } else {
      ++kv;
    }
  }

  drawTags(cv_image->image, map);
  sensor_msgs::ImagePtr corner_image_ptr(cv_image->toImageMsg());
  tag_pub_.publish(*corner_image_ptr);

  if (found) {
    ROS_INFO("Found chilitags");

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;

    msg->observations[idx_cam].features.resize(map.size() * 4);
    msg->observations[idx_chain].features.resize(map.size() * 4);


    // Fill in the headers
    geometry_msgs::PointStamped rgb;
    geometry_msgs::PointStamped world;
    rgb.header = image_.header;

    // Fill in message
    int counter = 0;
    for (auto& kv : map)
    {
        world.header.frame_id = "chilitag" + std::to_string(kv.first) + "_link";

//        world.header.frame_id = "chilitag1_link";

        chilitags::Quad positions = kv.second;

        // 4 points per marker
        // top left
        world.point.x = -marker_size_/2.0;
        world.point.y = -marker_size_/2.0;
        world.point.z = 0;

        rgb.point.x = positions(0, 0);
        rgb.point.y = positions(0, 1);
        rgb.point.z = 0;

        msg->observations[idx_cam].features[counter] = rgb;
        msg->observations[idx_cam].ext_camera_info = rgb_camera_manager_.getCameraInfo();
        msg->observations[idx_chain].features[counter] = world;
        counter++;

        // top right
        world.point.x = marker_size_/2.0;
        world.point.y = -marker_size_/2.0;
        world.point.z = 0;

        rgb.point.x = positions(1, 0);
        rgb.point.y = positions(1, 1);
        rgb.point.z = 0;

        msg->observations[idx_cam].features[counter] = rgb;
        msg->observations[idx_cam].ext_camera_info = rgb_camera_manager_.getCameraInfo();
        msg->observations[idx_chain].features[counter] = world;
        counter++;

        // bottom right
        world.point.x = marker_size_/2.0;
        world.point.y = marker_size_/2.0;
        world.point.z = 0;

        rgb.point.x = positions(2, 0);
        rgb.point.y = positions(2, 1);
        rgb.point.z = 0;

        msg->observations[idx_cam].features[counter] = rgb;
        msg->observations[idx_cam].ext_camera_info = rgb_camera_manager_.getCameraInfo();
        msg->observations[idx_chain].features[counter] = world;
        counter++;

        // bottom left
        world.point.x = -marker_size_/2.0;
        world.point.y = marker_size_/2.0;
        world.point.z = 0;

        rgb.point.x = positions(3, 0);
        rgb.point.y = positions(3, 1);
        rgb.point.z = 0;

        msg->observations[idx_cam].features[counter] = rgb;
        msg->observations[idx_cam].ext_camera_info = rgb_camera_manager_.getCameraInfo();
        msg->observations[idx_chain].features[counter] = world;
        counter++;

      // Do not accept NANs
      if (std::isnan(rgb.point.x) ||
          std::isnan(rgb.point.y) ||
          std::isnan(rgb.point.z))
      {
        ROS_ERROR_STREAM("NAN point on id " << kv.first);
        return false;
      }

    }

    // Add debug cloud to message
//    if (output_debug_)
//    {
//      msg->observations[0].image = image_;
//    }

    // Found chilitags
    return true;
  }

  // Found no chilitags
  return false;
}

void ChilitagFinder2D::drawTags(cv::Mat& image, const chilitags::TagCornerMap map) {
  int line_length = 8;
  int line_width = 3;

  for (auto& kv : map) {
    chilitags::Quad q = kv.second;
    int x = q(0, 0);
    int y = q(0, 1);

    // horizontal line
    cv::Point p1h(x-line_length, y);
    cv::Point p2h(x + line_length, y);
    cv::line(image, p1h, p2h, CV_RGB(255, 0, 0), line_width);

    // vertical line
    cv::Point p1v(x, y-line_length);
    cv::Point p2v(x, y+line_length);
    cv::line(image, p1v, p2v, CV_RGB(255, 0, 0), line_width);
  }
}


}  // namespace robot_calibration
