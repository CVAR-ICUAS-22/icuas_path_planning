#include "path_planner.hpp"

PathPlanner::PathPlanner() : it_(nh_)
{
  laserscan_sub_ = nh_.subscribe(LASERSCAN_TOPIC, 1, &PathPlanner::laserscanCallback, this);
  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1, &PathPlanner::positionCallback, this);
  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(WAYPOINT_TOPIC, 1);
  image_publisher_ = it_.advertise(IMAGE_PUB_TOPIC, 1);
  control_node_srv = nh_.advertiseService(CONTROLNODE_SRV, &PathPlanner::controlNodeSrv, this);
  // set_goal_srv = nh_.advertiseService(SETGOAL_SRV, &PathPlanner::setGoalSrv, this);
  set_goal_sub_ = nh_.subscribe(SETGOAL_TOPIC, 1, &PathPlanner::setGoalCallback, this);

  ref_frame_ = REF_FRAME;
  img_h_ = IMG_H;
  img_w_ = IMG_W;
  laser_map_ = cv::Mat(img_h_, img_w_, CV_8UC1, cv::Scalar(0));
  grid_size_.width = int((img_w_ / OCC_GRID_SIZE));
  grid_size_.height = int((img_h_ / OCC_GRID_SIZE));
  occupancy_map_ = cv::Mat::ones(grid_size_.height, grid_size_.width, CV_8UC1) * 255;
}

PathPlanner::~PathPlanner() {}

void PathPlanner::start()
{
  ROS_INFO("Node started");
  if (goal_position_ == cv::Point2f(0, 0))
  {
    goal_position_ = cv::Point2f(5, 0);
    goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);
    ROS_WARN("Goal position set to default: %f, %f", goal_position_.x, goal_position_.y);
  }
  force_generation_ = true;
}

void PathPlanner::stop()
{
  ROS_INFO("Node stopped");
  goal_set_ = false;
  goal_position_ = cv::Point2f(0, 0);
  goal_cell_ = cv::Point2i(0, 0);
}

void PathPlanner::run()
{
  if (!run_node_)
  {
    return;
  }

  if (laser_update_ || force_generation_)
  {
    ROS_DEBUG("New laser measuments");
    laser_update_ = false;
    generateOccupancyMap();
  }

  if (new_occupancy_map_ || force_generation_)
  {
    new_occupancy_map_ = false;
    ROS_DEBUG("New occupancy map available");
    // sendMap(occupancy_map_); // DEBUG
    checkCurrentPath();
  }

  if (generate_path_ || force_generation_)
  {
    generate_path_ = false;
    generateNewPath();
  }

  if (current_path_.size() > 0)
  {
    cv::Point2f next_point = grid2coord(current_path_[0], img_h_, img_w_);
    waypoint_pub_.publish(createTrajectoryFromPoint(next_point, 2.0, drone_position_));

    double distance = sqrt(pow(next_point.x - drone_position_.x, 2) + pow(next_point.y - drone_position_.y, 2));
    if (distance < DISTPOINT_TH)
    {
      // ROS_INFO("Next cell: %d, %d", current_path_[0].x, current_path_[0].y);
      // ROS_INFO("Next point: %.2f, %.2f", next_point.x, next_point.y);
      current_path_.erase(current_path_.begin());
    }
  }

  cv::Mat path_map = generatePathImg(occupancy_map_, drone_cell_, current_path_);
  sendMap(path_map);
  showMap(laser_map_, "ICUAS laser map", true);
  // showMap(occupancy_map_, "ICUAS occupancy map", false);
  // showMap(path_map, "ICUAS path map", false);

  if (force_generation_)
    force_generation_ = false;
}

void PathPlanner::generateOccupancyMap()
{
  static cv::Mat last_occ_map_sent(grid_size_, CV_8UC1);
  // Convert _img into a binary image  thresholding it and compute the distance map (in meters)
  cv::Mat binary_map, distance_map, dist_normalized_map;
  float max_dist = OCC_MAX_DIST_PX;

  // Generate Distance Map
  cv::threshold(laser_map_, binary_map, DISTANCEMAP_TH, 255, cv::THRESH_BINARY_INV);
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
  cv::threshold(dist_normalized_map, binary_distance_map, 0.6, 1, cv::THRESH_BINARY);

  // cv::imshow("dist_normalized_map", dist_normalized_map);
  // cv::imshow("binary_distance_map", binary_distance_map);
  // showMap(binary_distance_map, "binary_distance_map", true);

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

        double min_value, max_value;
        cv::minMaxLoc(submatrix, &min_value, &max_value);

        if (min_value < 1)
        {
          occupancy_map_.at<uchar>(i, j) = 0;
          new_occupancy_map_ = true;
        }
      }
    }
  }

  // // Check occupancy map changes
  // cv::Mat diff;
  // cv::absdiff(occupancy_map_, last_occ_map_sent, diff);
  // if (cv::countNonZero(diff) > 0)
  // {
  //   ROS_INFO("New occupancy map available");
  // }

  cv::Point2i img_drone_position;
  img_drone_position = coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
}

void PathPlanner::checkCurrentPath()
{
  // Check current path is not empty
  if (current_path_.size() == 0)
  {
    generate_path_ = true;
    return;
  }

  // Check if path points are free on the map
  for (auto &point : current_path_)
  {
    if (occupancy_map_.at<uchar>(point.x, point.y) == 0) // TODO: Check
    {
      generate_path_ = true;
      break;
    }
  }
}

void PathPlanner::generateNewPath()
{
  ROS_DEBUG("Generating new path");
  current_path_.clear();

  planner_algorithm_.setOriginPoint(drone_cell_);
  planner_algorithm_.setGoal(goal_cell_);
  planner_algorithm_.setOcuppancyGrid(occupancy_map_);

  current_path_ = planner_algorithm_.solveGraph();
}

// CALLBACK

void PathPlanner::laserscanCallback(const sensor_msgs::LaserScan &_msg)
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

void PathPlanner::positionCallback(const nav_msgs::Odometry &_msg)
{
  drone_position_.x = _msg.pose.pose.position.x;
  drone_position_.y = _msg.pose.pose.position.y;

  drone_cell_ = coord2grid(drone_position_.x, drone_position_.y, img_h_, img_w_);
}

bool PathPlanner::controlNodeSrv(std_srvs::SetBool::Request &_request, std_srvs::SetBool::Response &_response)
{
  if (run_node_ == _request.data)
  {
    _response.success = false;
    _response.message = "Run node already set to " + std::to_string(run_node_);
    return true;
  }

  run_node_ = _request.data;

  _response.success = true;
  _response.message = "Run node set to " + std::to_string(run_node_);

  if (run_node_)
  {
    start();
  }
  else
  {
    stop();
  }

  return true;
}

bool PathPlanner::setGoalSrv(path_planner::setGoalPoint::Request &_request, path_planner::setGoalPoint::Response &_response)
{
  goal_position_.x = _request.goal.point.x;
  goal_position_.y = _request.goal.point.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  _response.success = true;
  _response.message = "Goal position set to: " + std::to_string(goal_position_.x) + ", " + std::to_string(goal_position_.y);

  goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);

  goal_set_ = true;
  // force_generation_ = true;

  return true;
}

void PathPlanner::setGoalCallback(const geometry_msgs::PoseStamped &_msg)
{
  float distance = sqrt(pow(_msg.pose.position.x - drone_position_.x, 2) + pow(_msg.pose.position.y - drone_position_.y, 2));
  if (distance < 5.0)
  {
    return;
  }

  goal_position_.x = _msg.pose.position.x;
  goal_position_.y = _msg.pose.position.y;
  // goal_position_.x = _msg.point.x;
  // goal_position_.y = _msg.point.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);

  goal_set_ = true;
  // force_generation_ = true;
}

// PUBLISH

void PathPlanner::sendMap(cv::Mat _map)
{
  sensor_msgs::ImagePtr output_image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", _map).toImageMsg();
  image_publisher_.publish(output_image_msg);
}

// DEBUG

void PathPlanner::showMap(const cv::Mat &_map, const std::string &_map_name, const bool _add_drone)
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

cv::Mat PathPlanner::generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px)
{
  cv::Mat color_img, show_img;
  cv::cvtColor(_img, color_img, cv::COLOR_GRAY2BGR);

  cv::circle(color_img, cv::Point(_drone_px.y, _drone_px.x), 3, cv::Scalar(255, 0, 0), cv::FILLED);
  // cv::resize(color_img, show_img, cv::Size(400, 600));
  show_img = color_img;
  return show_img;
}

cv::Mat generatePathImg(const cv::Mat &_map, const cv::Point &_drone_px, const std::vector<cv::Point> &_path)
{
  cv::Mat path_img, color_img;

  path_img = _map.clone();

  for (auto &point : _path)
  {
    path_img.at<uchar>(point.x, point.y) = 125;
  }

  cv::cvtColor(path_img, color_img, cv::COLOR_GRAY2BGR);
  color_img.at<cv::Vec3b>(_drone_px.x, _drone_px.y) = cv::Vec3b(0, 0, 255);

  return color_img;
}

// AUXILIAR FUNCTION

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

cv::Point2i coord2grid(const float _x, const float _y, const int _img_h, const int _img_w)
{
  cv::Point2i img_drone_position, grid_drone_position;
  img_drone_position = coord2img(_x, _y, _img_h, _img_w);

  grid_drone_position.x = int((img_drone_position.x / OCC_GRID_SIZE)); // x
  grid_drone_position.y = int((img_drone_position.y / OCC_GRID_SIZE)); // y

  return grid_drone_position;
}

cv::Point2f grid2coord(const cv::Point2i &_point, const int _img_h, const int _img_w)
{
  cv::Point2f coord_point;

  coord_point.x = ((_img_h / 2) - (_point.x * OCC_GRID_SIZE)) / 10;
  coord_point.y = ((_img_w / 2) - (_point.y * OCC_GRID_SIZE)) / 10;
  // ROS_INFO("First x: %f, y: %f", coord_point.x, coord_point.y);
  coord_point.x -= (float(OCC_GRID_SIZE) / 2) / 10;
  coord_point.y -= (float(OCC_GRID_SIZE) / 2) / 10;
  // ROS_INFO("Second x: %f, y: %f", coord_point.x, coord_point.y);

  return coord_point;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint createTrajectoryFromPoint(const cv::Point2f &_point, const float _height, const cv::Point2f &_current_position)
{
  float yaw = atan2(_point.y - _current_position.y, _point.x - _current_position.x);
  // convert yaw to quaternion
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
  trajectory_point.transforms.resize(1);
  trajectory_point.transforms[0].translation.x = _point.x;
  trajectory_point.transforms[0].translation.y = _point.y;
  trajectory_point.transforms[0].translation.z = _height;
  trajectory_point.transforms[0].rotation.x = q.x();
  trajectory_point.transforms[0].rotation.y = q.y();
  trajectory_point.transforms[0].rotation.z = q.z();
  trajectory_point.transforms[0].rotation.w = q.w();
  // trajectory_point.
  trajectory_point.velocities.resize(1);
  // trajectory_point.velocities[0].x = 0;
  // trajectory_point.velocities[0].y = 0;
  // trajectory_point.velocities[0].z = 0;
  trajectory_point.accelerations.resize(1);
  // trajectory_point.accelerations[0].x = 0;
  // trajectory_point.accelerations[0].y = 0;
  // trajectory_point.accelerations[0].z = 0;
  trajectory_point.time_from_start = ros::Duration(0.1);
  return trajectory_point;
}
