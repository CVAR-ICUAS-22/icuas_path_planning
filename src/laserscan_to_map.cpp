#include "laserscan_to_map.hpp"

LaserscanToMap::LaserscanToMap() : it_(nh_)
{
  laserscan_sub_ = nh_.subscribe(LASERSCAN_TOPIC, 1, &LaserscanToMap::laserscanCallback, this);
  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1, &LaserscanToMap::positionCallback, this);
  image_publisher_ = it_.advertise(IMAGE_PUB_TOPIC, 1);

  ref_frame_ = REF_FRAME;
  img_h_ = IMG_H;
  img_w_ = IMG_W;
  laser_map_ = cv::Mat(img_h_, img_w_, CV_8UC1, cv::Scalar(0));
  grid_size_.width = int((img_w_ / OCC_GRID_SIZE));
  grid_size_.height = int((img_h_ / OCC_GRID_SIZE));
  occupancy_map_ = cv::Mat::ones(grid_size_.height, grid_size_.width, CV_8UC1) * 255;
}

LaserscanToMap::~LaserscanToMap() {}

void LaserscanToMap::run()
{

  if (laser_update_)
  {
    ROS_INFO("New laser measuments");
    laser_update_ = false;
    generateOccupancyMap();
  }

  if (send_occ_map_)
  {
    send_occ_map_ = false;
    sendMap(occupancy_map_);
  }

  showMap(laser_map_, "ICUAS laser map", true);
  showMap(occupancy_map_, "ICUAS occupancy map", false);
}

void LaserscanToMap::generateOccupancyMap()
{

  static cv::Mat last_occ_map_sent(grid_size_, CV_8UC1);
  // Convert _img into a binary image  thresholding it and compute the distance map (in meters)
  cv::Mat binary_map, distance_map, dist_normalized_map;
  float max_dist = OCC_MAX_DIST_PX;

  cv::Point2i img_drone_position;
  img_drone_position = coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);

  // Generate Distance Map
  cv::threshold(laser_map_, binary_map, 100, 255, cv::THRESH_BINARY_INV);
  cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, 3);
  dist_normalized_map = distance_map / max_dist;               // normalize distance map respect to the max dist
  dist_normalized_map.setTo(1.0f, dist_normalized_map > 1.0f); // clip image to 1.0

  // Generate Occupancy Map (option 1)
  // cv::Mat binary_occupancy_map;
  // cv::resize(dist_normalized_map, binary_occupancy_map, grid_size_, cv::INTER_MAX);
  // occupancy_map_.setTo(255, binary_occupancy_map > 0.9);
  // occupancy_map_.setTo(0, binary_occupancy_map <= 0.9);

  // Generate Occupancy Map (option 2)
  // cv::Mat binary_distance_map, binary_occupancy_map;
  // cv::threshold(dist_normalized_map, binary_distance_map, 0.5, 1, cv::THRESH_BINARY);
  // cv::resize(binary_distance_map, binary_occupancy_map, grid_size_, cv::INTER_MAX);
  // occupancy_map_.setTo(255, binary_occupancy_map > 0.9);
  // occupancy_map_.setTo(0, binary_occupancy_map <= 0.9);

  // TODO: Generate Occupancy Map (option 3): MAKE CONVOLUTION TO SOLVE THIS AND EVERYBODY WILL BE HAPPY
  cv::Mat binary_distance_map, binary_occupancy_map;
  cv::threshold(dist_normalized_map, binary_distance_map, 0.5, 1, cv::THRESH_BINARY);
  // cv::imshow("binary_distance_map", binary_distance_map);
  // cv::waitKey(0);

  // cv::threshold(dist_normalized_map, binary_distance_map, 0.5, 1, cv::THRESH_BINARY);
  for (int i = 0; i < grid_size_.height; i++)
  {
    for (int j = 0; j < grid_size_.width; j++)
    {
      int current_value = occupancy_map_.at<uchar>(i, j);
      if (current_value > 1) // Just for no occuped cells
      {
        int h_step = int(img_h_ / grid_size_.height);
        int w_step = int(img_w_ / grid_size_.width);
        cv::Mat submatrix = binary_distance_map(cv::Range(h_step * i, h_step * (i + 1)), cv::Range(w_step * j, w_step * (j + 1)));
        // ROS_INFO("Submatrix size: %d, %d", submatrix.size[0], submatrix.size[1]);
        // cv::imshow("submatrix", submatrix);
        // cv::waitKey(0);

        double min_value, max_value;
        cv::minMaxLoc(submatrix, &min_value, &max_value);

        if (min_value < 1)
        {
          occupancy_map_.at<uchar>(i, j) = 0;
          send_occ_map_ = true;
        }
      }
    }
  }

  if (send_occ_map_)
  {
    // Code drone position in map
    // TODO: COPY THE MAP
    occupancy_map_.at<uchar>(0, 0) = int((img_drone_position.x / OCC_GRID_SIZE)); // x
    occupancy_map_.at<uchar>(0, 1) = int((img_drone_position.y / OCC_GRID_SIZE)); // y
    ROS_INFO("New occupancy map available");
  }

  // // Check occupancy map changes
  // cv::Mat diff;
  // cv::absdiff(occupancy_map_, last_occ_map_sent, diff);
  // if (cv::countNonZero(diff) > 0)
  // {
  //   ROS_INFO("New occupancy map available");
  //   send_occ_map_ = true;
  //   occupancy_map_.copyTo(last_occ_map_sent);

  //   // Code drone position in map
  //   occupancy_map_.at<uchar>(0, 0) = int((img_drone_position.x / OCC_GRID_SIZE)); // x
  //   occupancy_map_.at<uchar>(0, 1) = int((img_drone_position.y / OCC_GRID_SIZE)); // y
  // }

  // auto drone_x = occupancy_map_.at<uchar>(0, 0);
  // auto drone_y = occupancy_map_.at<uchar>(0, 1);

  // ROS_INFO("Drone position: %d, %d", drone_x, drone_y);
}

void LaserscanToMap::laserscanCallback(const sensor_msgs::LaserScan &_msg)
{
  /*   Because the stamp of a sensor_msgs/LaserScan is the time of the first measurement,
    one cannot simply wait for a transform to target_frame at this stamp.
    Instead one also has to wait for a transform at the last measurement of the scan. */
  if (!tf_listener_.waitForTransform(
          _msg.header.frame_id,
          ref_frame_,
          _msg.header.stamp + ros::Duration().fromSec(_msg.ranges.size() * _msg.time_increment),
          ros::Duration(1.0)))
  {
    return;
  }

  sensor_msgs::PointCloud point_cloud;
  laser_projector_.transformLaserScanToPointCloud(ref_frame_, _msg, point_cloud, tf_listener_);

  // TODO: Check changes
  // Generate laser map
  cv::Point2i img_point;
  for (auto point : point_cloud.points)
  {
    if (point.z > Z_MIN_TH)
    {
      img_point = coord2img(point.x, point.y, img_h_, img_w_);
      int current_value = laser_map_.at<uchar>(img_point.x, img_point.y);
      if (current_value < 1)
      {
        laser_map_.at<uchar>(img_point.x, img_point.y) = 255;
        laser_update_ = true;
      }
    }
  }
}

void LaserscanToMap::sendMap(cv::Mat _occupancy_map)
{
  sensor_msgs::ImagePtr output_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", _occupancy_map).toImageMsg();
  image_publisher_.publish(output_image_msg);
}

void LaserscanToMap::showMap(const cv::Mat &_map, const std::string &_map_name, const bool _add_drone)
{
  cv::namedWindow(_map_name, cv::WINDOW_FREERATIO);
  // Generate image to show
  if (_add_drone)
  {
    cv::Point2i img_drone_position;
    img_drone_position = coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
    cv::Mat map_img = generateShowImg(_map, img_drone_position);
    cv::imshow(_map_name, map_img);
  }
  else
  {
    cv::imshow(_map_name, _map);
  }
  cv::waitKey(1);
}

// // Generate images to show
// cv::Mat laser_img = generateShowImg(binary_map, img_drone_position);
// cv::Mat distance_img = generateShowImg(dist_normalized_map, img_drone_position);

// cv::imshow("ICUAS laser map", laser_img);
// cv::imshow("ICUAS distance map", distance_img);

// cv::namedWindow("ICUAS bin occupancy map", cv::WINDOW_FREERATIO);
// cv::imshow("ICUAS bin occupancy map", binary_occupancy_map);

// cv::namedWindow("ICUAS occupancy map", cv::WINDOW_FREERATIO);
// cv::imshow("ICUAS occupancy map", occupancy_map_);
// cv::waitKey(3);
// }

void LaserscanToMap::positionCallback(const geometry_msgs::PointStamped &_msg)
{
  drone_position_.x = _msg.point.x;
  drone_position_.y = _msg.point.y;
}

cv::Mat LaserscanToMap::generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px)
{
  cv::Mat color_img, show_img;
  cv::cvtColor(_img, color_img, cv::COLOR_GRAY2BGR);

  cv::circle(color_img, cv::Point(_drone_px.y, _drone_px.x), 3, cv::Scalar(255, 0, 0), cv::FILLED);
  // cv::resize(color_img, show_img, cv::Size(400, 600));
  show_img = color_img;
  return show_img;
}

cv::Point2i coord2img(const float _x, const float _y, const int _img_h, const int _img_w)
{
  cv::Point2i img_point;
  float w = _img_w / 2;
  float h = _img_h / 2;
  float x = _x * 10;
  float y = _y * 10;
  img_point.x = int(h - x);
  img_point.y = int(w - y);

  return img_point;
}
