#include "path_planner.hpp"
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

PathPlanner::PathPlanner() : it_(nh_)
{
  laserscan_sub_ = nh_.subscribe(LASERSCAN_TOPIC, 1, &PathPlanner::laserscanCallback, this);
  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1, &PathPlanner::positionCallback, this);

  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(WAYPOINT_TOPIC, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
  has_ended_pub_ = nh_.advertise<std_msgs::Bool>(PATHPLANNER_HAS_ENDED_TOPIC, 1);
  image_publisher_ = it_.advertise(IMAGE_PUB_TOPIC, 1);

  control_node_srv = nh_.advertiseService(CONTROLNODE_SRV, &PathPlanner::controlNodeSrv, this);
  set_goal_srv = nh_.advertiseService(SETGOAL_SRV, &PathPlanner::setGoalSrv, this);

  float map_h;
  float map_w;
  float security_distance;

  nh_.getParam("path_planner/map_h", map_h);
  nh_.getParam("path_planner/map_w", map_w);
  nh_.getParam("path_planner/img_resolution", img_resolution_);
  nh_.getParam("path_planner/occmap_resolution", occmap_resolution_);
  nh_.getParam("path_planner/z_min_th", z_min_th_);
  nh_.getParam("path_planner/ref_frame", ref_frame_);

  nh_.getParam("path_planner/fly_height", fly_height_);
  nh_.getParam("path_planner/security_distance", security_distance);
  nh_.getParam("path_planner/next_point_reached_dist", next_point_reached_dist_);
  // SPEED CONTROLLER
  nh_.getParam("path_planner/speed_controller", speed_controller_);
  nh_.getParam("path_planner/max_control_speed", max_control_speed_);

  ROS_INFO("map_h: %.2f", map_h);
  ROS_INFO("map_w: %.2f", map_w);
  ROS_INFO("img_resolution: %.2f", img_resolution_);
  ROS_INFO("occmap_resolution: %.2f", occmap_resolution_);
  ROS_INFO("z_min_th: %.2f", z_min_th_);
  ROS_INFO("ref_frame: %s", ref_frame_.c_str());

  ROS_INFO("fly_height: %.2f", fly_height_);
  ROS_INFO("security distance: %.2f", security_distance);
  ROS_INFO("next_point_reached_dist: %.2f", next_point_reached_dist_);
  // SPEED CONTROLLER
  std::string controller_str = speed_controller_ ? "SPEED" : "POSITION";
  ROS_INFO("controller: %s", controller_str.c_str());
  if (speed_controller_)
  {
    ROS_INFO("control_speed: %.2f", max_control_speed_);
  }

  img_size_.width = map_w / img_resolution_;
  img_size_.height = map_h / img_resolution_;
  laser_map_ = cv::Mat(img_size_.height, img_size_.width, CV_8UC1, cv::Scalar(0));
  grid_size_.width = int((map_w / occmap_resolution_));
  grid_size_.height = int((map_h / occmap_resolution_));
  occupancy_map_ = cv::Mat::ones(grid_size_.height, grid_size_.width, CV_8UC1) * 255;
  security_distance_th_= security_distance / img_resolution_;
  // SPEED CONTROLLER
  if (speed_controller_)
  {
    speed_control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(SPEEDCONTROL_TOPIC, 1);
  }
}

PathPlanner::~PathPlanner() {}

void PathPlanner::start()
{
  ROS_INFO("Node started");
  run_node_ = true;
  generate_path_ = true;
  goal_reached_ = false;
  no_solution_ = false;
  attempts_count_ = 0;
  // force_generation_ = true;
}

void PathPlanner::stop()
{
  ROS_INFO("Node stopped");
  run_node_ = false;
}

void PathPlanner::run()
{
  static bool send_waypoint = false;
  // force_generation_ = true; // DEBUG

  if (laser_update_ || force_generation_)
  {
    ROS_DEBUG("New laser measuments");
    laser_update_ = false;
    generateOccupancyMap();
  }

  if (!run_node_)
  {
    cv::Mat path_map = generatePathImg(occupancy_map_, drone_cell_, goal_cell_, current_path_, ref_waypoints_);
    sendMap(path_map);
    return;
  }

  // if (goal_reached_) {
  //   endNavigation();
  // }

  if (no_solution_)
  {
    // static cv::Point2f hover_position = drone_position_;
    ROS_WARN_ONCE("Solution not found");
    // sendWaypoint(hover_position, 0.0);
    // if (attempts_count_ < max_attempts_)
    // {
    //   ROS_INFO("Trying again");
    //   prepareNewAttempt();
    //   attempts_count_++;
    //   generate_path_ = true;
    //   no_solution_ = false;
    // }
    // else {
    endNavigation();
    return;
    // }
  }

  if (new_occupancy_map_ || force_generation_)
  {
    // ROS_INFO("New occupancy map available");
    new_occupancy_map_ = false;
    // sendMap(occupancy_map_); // DEBUG
    checkCurrentPath();
  }

  if (generate_path_ || force_generation_)
  {
    generate_path_ = false;
    if (!cellIsAvailable(drone_cell_)) {
        drone_cell_ = findNearestAvailableCell(drone_cell_, drone_max_distance_);
    }
    generateNewPath();
    optimizePath();
    send_waypoint = true;
  }

  cv::Mat path_map = generatePathImg(occupancy_map_, drone_cell_, goal_cell_, current_path_, ref_waypoints_);
  sendMap(path_map);

  // Check if the goal has been reached
  float distance = sqrt(pow(goal_position_.x - drone_position_.x, 2) +
                        pow(goal_position_.y - drone_position_.y, 2));
  if (distance < goal_reached_dist_) {
    ROS_INFO("Goal reached");
    goal_reached_ = true;
    endNavigation();
    return; 
  }

  if (ref_waypoints_.size() > 0)
  {
    float sending_yaw;
    cv::Point2f next_point = grid2coord(ref_waypoints_[0]);
    double distance_to_next_point =
        sqrt(pow(next_point.x - drone_position_.x, 2) +
             pow(next_point.y - drone_position_.y, 2));

    // IF NEXT POINT IS REACHED: GET THE NEXT POINT
    if (distance_to_next_point < next_point_reached_dist_)
    {
      // Do not check future point
      if (!check_future_point_)
        ref_waypoints_.erase(ref_waypoints_.begin());

      if (check_future_point_)
      {
        // Check future point
        cv::Point2f future_point = grid2coord(ref_waypoints_[1]);

        float future_yaw =
            atan2(future_point.y - drone_position_.y, future_point.x - drone_position_.x);
        float distance_to_future_point = sqrt(pow(future_point.x - drone_position_.x, 2) +
                                              pow(future_point.y - drone_position_.y, 2));

        // IF DRONE IS LOOKING TO THE FUTURE POINT OR HAS ALREADY REACHED IT: SEND THE NEXT POINT
        if (abs(future_yaw - drone_yaw_) < M_PI_4 ||
            distance_to_future_point < next_point_reached_dist_)
        {
          ref_waypoints_.erase(ref_waypoints_.begin());
          send_waypoint = false;
        }
        // ELSE: SEND THE CURRENT POINT
        else
        {
          sending_yaw = future_yaw;
          send_waypoint = true;
        }
      }
    }
    // // IF NEXT POINT IS NOT REACHED: CONTINUE SENDIING THE SAME POINT
    else
    {
      float next_yaw = atan2(next_point.y - drone_position_.y, 
                             next_point.x - drone_position_.x);
      sending_yaw = next_yaw;
      send_waypoint = true;
    }

    if (send_waypoint)
    {
      sendWaypoint(next_point, sending_yaw);
    }
  }

  if (force_generation_)
    force_generation_ = false;
}

void PathPlanner::endNavigation()
{
  std_msgs::Bool msg;
  msg.data = goal_reached_;
  has_ended_pub_.publish(msg);
  ROS_INFO("End navigation");
  // current_path_.clear();
  // ref_waypoints_.clear();
  goal_reached_ = false;
  // run_node_ = false; // This should works, but doesnt
}

void PathPlanner::prepareNewAttempt()
{
  ROS_INFO("Cleaning goal zone");
  int goal_zone_clean_radious = attempts_count_;
  cv::circle(occupancy_map_, goal_cell_, goal_zone_clean_radious, cv::Scalar(0), -1);
}

void PathPlanner::sendWaypoint(const cv::Point2f &_next_point, const float _sending_yaw)
{
  if (!speed_controller_)
  {
    waypoint_pub_.publish(createTrajectoryFromPointMsg(_next_point, fly_height_, _sending_yaw));
    return;
  }
  if (speed_controller_)
  {
    if (goal_reached_ || no_solution_)
    {
      pose_pub_.publish(createPoseStampedMsg(_next_point, fly_height_, _sending_yaw));
    }
    else {
      speed_control_pub_.publish(createSpeedReferenceMsg(drone_position_, _next_point, max_control_speed_)); 
    }
  }
}

void PathPlanner::generateOccupancyMap()
{
  static cv::Mat last_occ_map_sent(grid_size_, CV_8UC1);
  // Convert _img into a binary image  thresholding it and compute the distance map (in meters)
  cv::Mat binary_map, distance_map, dist_normalized_map;

  // Generate Distance Map
  cv::threshold(laser_map_, binary_map, LASER2BIN_TH, 255, cv::THRESH_BINARY_INV);
  cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, 3);
  dist_normalized_map = distance_map / security_distance_th_; // normalize distance map respect to the max dist
  dist_normalized_map.setTo(1.0f, dist_normalized_map > 1.0f); // clip image to 1.0

  // Generate Occupancy Map: MAKE CONVOLUTION TO SOLVE THIS AND EVERYBODY WILL BE HAPPY
  cv::Mat binary_distance_map, binary_occupancy_map;
  cv::threshold(dist_normalized_map, binary_distance_map, DIST2BIN_TH, 1, cv::THRESH_BINARY);

  // std::vector<cv::Mat> maps;
  // maps.emplace_back(laser_map_);
  // maps.emplace_back(binary_map);
  // maps.emplace_back(distance_map);
  // maps.emplace_back(dist_normalized_map);
  // showCombinedMap(maps, "combined maps");
  
  // Crop the map around the drone_cell_
  cv::Point2i drone_px = coord2img(drone_position_.x, drone_position_.y);
  float ego_radious = 2.5;
  int crop_distance = int(ego_radious / img_resolution_);

  cv::Range r1(cv::Range(std::max(0, drone_px.x - crop_distance), std::min(drone_px.x + crop_distance, img_size_.height-1)));
  cv::Range r2(cv::Range(std::max(0, drone_px.y - crop_distance), std::min(drone_px.y + crop_distance, img_size_.width-1))); 

  cv::Mat ego_map = binary_map(r1,r2);

  // cv::Mat ego_map = binary_map(cv::Rect(drone_px.x - crop_distance, drone_px.y - crop_distance,
  //                                       2 * crop_distance, 2 * crop_distance));
  // cv::Mat ego_distance_map = binary_distance_map(cv::Rect(drone_px.x - crop_distance, drone_px.y - crop_distance,
  //                                                         2 * crop_distance, 2 * crop_distance));
  // cv::Mat ego_occupancy_map = binary_occupancy_map(cv::Rect(drone_px.x - crop_distance, drone_px.y - crop_distance,
  //                                                           2 * crop_distance, 2 * crop_distance));

  // showMap(ego_map, "egomap", false);
  // std::vector<cv::Mat> maps;
  // maps.emplace_back(ego_map);
  // maps.emplace_back(ego_distance_map);
  // maps.emplace_back(ego_occupancy_map);
  // showCombinedMap(maps, "combined maps");

  for (int i = 0; i < grid_size_.height; i++)
  {
    for (int j = 0; j < grid_size_.width; j++)
    {
      int h_step = int(img_size_.height / grid_size_.height);
      int w_step = int(img_size_.width / grid_size_.width);
      cv::Mat submatrix = binary_distance_map(cv::Range(h_step * i, h_step * (i + 1)),
                                              cv::Range(w_step * j, w_step * (j + 1)));

      double min_value, max_value;
      cv::minMaxLoc(submatrix, &min_value, &max_value);
      // count the number of zero pixels
      int count = cv::countNonZero(submatrix);
      int zeros = submatrix.cols * submatrix.rows - count;
      float rate = float(zeros) / (submatrix.cols * submatrix.rows);

      if (rate > 0.3)
      {
        occupancy_map_.at<uchar>(i, j) = 0;
        new_occupancy_map_ = true;
      }
    }
  }
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
    if (occupancy_map_.at<uchar>(point.x, point.y) == 0)
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

void PathPlanner::optimizePath()
{
  ROS_DEBUG("Optimizing path");
  ref_waypoints_.clear();

  if (current_path_.size() == 0)
  {
    ROS_ERROR("Path is empty");
    no_solution_ = true;
    return;
  }

  bool optimize = false;
  if (!optimize)
  {
    ref_waypoints_ = current_path_;
    return;
  }

  if (!current_path_.size())
  {
    ROS_ERROR("QUEEEEEEEEEEEEEEEEEEE");
  }
  ref_waypoints_.emplace_back(current_path_[0]);

  cv::Point2i path_point;

  for (int i = 1; i < current_path_.size(); i++)
  {
    path_point = current_path_[i];
    if (path_point.x == ref_waypoints_.back().x || path_point.y == ref_waypoints_.back().y)
    {
      continue;
    }
    // if (path_point.x != ref_waypoints_.back().x && path_point.y != ref_waypoints_.back().y)
    // {
    //   ROS_INFO("Path point: %.2i, %.2i", path_point.x, path_point.y);
    //   ROS_INFO("Last ref point: %.2i, %.2i", ref_waypoints_.back().x, ref_waypoints_.back().y);
    //   continue;
    // }
    // else
    // {
    ref_waypoints_.emplace_back(current_path_[i - 1]);
    // }
  }

  ref_waypoints_.emplace_back(current_path_[current_path_.size() - 1]);
}

// CALLBACK

/* std::vector<float> removeRangeOutliers(const std::vector<float> &_ranges) {

  const int n_samples_without_nan = 2;
  std::vector<float> filtered_ranges;
  filtered_ranges.reserve(_ranges.size());

  int last_nan_index_ = 0;

  for (int i = 0; i < _ranges.size() - n_samples_without_nan; i++) {
    const int displacement = i + n_samples_without_nan;
    if (std::isnan(_ranges[displacement])) {
      last_nan_index_ = displacement;
      continue;
    }
    if (std::abs(last_nan_index_ - i) > n_samples_without_nan) {
      filtered_ranges.emplace_back(_ranges[i]);
    } else {
      filtered_ranges.emplace_back(std::nan(""));
    }
  }

  for (int i = _ranges.size() - n_samples_without_nan; i < _ranges.size(); i++) {
    filtered_ranges.emplace_back(std::nan(""));
  }

  // check if size is correct
  if (filtered_ranges.size() != _ranges.size()) {
    ROS_ERROR("Filtered ranges size is not equal to original ranges size");
  }

  return filtered_ranges;
} */

float computeMedian(const std::deque<float> &_ranges)
{
  // create ordered vector
  std::vector<float> ordered_ranges;
  ordered_ranges.reserve(_ranges.size());
  int n_nans = 0;
  for (auto &range : _ranges)
  {
    if (std::isnan(range))
    {
      n_nans++;
      continue;
    }
    ordered_ranges.emplace_back(range);
  }
  if (n_nans > _ranges.size() - n_nans)
  {
    return std::nan("");
  }
  std::sort(ordered_ranges.begin(), ordered_ranges.end());
  return ordered_ranges[ordered_ranges.size() / 2];
}

std::vector<float> removeRangeOutliers(const std::vector<float> &_ranges)
{
  int n_samples_median = 9;
  if (n_samples_median % 2 == 0)
  {
    n_samples_median++;
  }

  std::vector<float> filtered_ranges;
  std::deque<float> median_queue;

  filtered_ranges.reserve(_ranges.size());

  for (int i = 0; i < n_samples_median; i++)
  {
    median_queue.push_back(_ranges[i]);
  }

  for (int i = 0; i < _ranges.size(); i++)
  {
    if ((i < n_samples_median / 2 + 1) || (i > _ranges.size() - n_samples_median / 2 - 1))
    {
      filtered_ranges.emplace_back(std::nanf(""));
      continue;
    }

    median_queue.push_back(_ranges[i + n_samples_median / 2]);
    if (median_queue.size() >= n_samples_median)
    {
      filtered_ranges.emplace_back(computeMedian(median_queue));
      median_queue.pop_front();
    }
    else
    {
      filtered_ranges.emplace_back(_ranges[i]);
    }
  }

  // check if size is correct
  if (filtered_ranges.size() != _ranges.size())
  {
    ROS_ERROR("Filtered ranges size is not equal to original ranges size");
  }

  return filtered_ranges;
}

sensor_msgs::LaserScan filterLaserScan(const sensor_msgs::LaserScan &msg)
{
  sensor_msgs::LaserScan filtered_scan;
  filtered_scan.header = msg.header;
  filtered_scan.angle_min = msg.angle_min;
  filtered_scan.angle_max = msg.angle_max;
  filtered_scan.angle_increment = msg.angle_increment;
  filtered_scan.range_min = msg.range_min;
  filtered_scan.range_max = msg.range_max;
  filtered_scan.intensities = msg.intensities;

  filtered_scan.ranges = std::move(removeRangeOutliers(msg.ranges));

  return filtered_scan;
}

void show_image_resized(const std::string &_title, const cv::Mat &_image)
{
  cv::Mat map_resized = cv::Mat::zeros(_image.size() * 5, _image.type());
  cv::resize(_image, map_resized, map_resized.size(), 0, 0, cv::INTER_NEAREST);
  cv::imshow(_title, map_resized);
}

cv::Mat &filterLaserMap(cv::Mat &mat)
{
  static cv::Mat prev_mat = cv::Mat::zeros(mat.size(), mat.type());
  static cv::Mat out = cv::Mat::zeros(mat.size(), mat.type());

  const int maximum_value = 255;
  const float value_to_set = 0.6;
  const float increment_per_obstacle = 0.05;
  const float decrement_per_free = 0.1;
  const float threshold = value_to_set * maximum_value;

  // filter a mask with a threshold
  cv::Mat free_space_mask, obstacle_mask;
  cv::threshold(mat, free_space_mask, 0.5 * maximum_value, 1, cv::THRESH_BINARY_INV);
  cv::threshold(mat, obstacle_mask, 0.5 * maximum_value, 1, cv::THRESH_BINARY);

  ROS_INFO_ONCE("Filtering occupancy map");

  cv::Mat prev_mat_free_space;
  cv::threshold(prev_mat, prev_mat_free_space, value_to_set * maximum_value, 1,
                cv::THRESH_BINARY_INV);
  cv::Mat combined_mask = cv::Mat::zeros(mat.size(), mat.type());
  cv::multiply(prev_mat_free_space, free_space_mask, combined_mask);
  cv::multiply(combined_mask, cv::Scalar(decrement_per_free * maximum_value), combined_mask);
  cv::subtract(prev_mat, combined_mask, prev_mat);

  cv::multiply(obstacle_mask, cv::Scalar(increment_per_obstacle * maximum_value), obstacle_mask);
  cv::add(prev_mat, obstacle_mask, prev_mat);

  prev_mat = cv::max(prev_mat, 0.0f);
  prev_mat = cv::min(prev_mat, maximum_value);

  out = prev_mat.clone();
  cv::threshold(out, out, threshold, maximum_value, cv::THRESH_BINARY);
  return out;
}

void PathPlanner::laserscanCallback(const sensor_msgs::LaserScan &_msg)
{
  // auto laser_filtered = filterLaserScan(_msg);
  auto &laser_filtered = _msg;

  // test publishing the filtered laser scan
  static ros::Publisher filtered_laser_scan_pub =
      nh_.advertise<sensor_msgs::LaserScan>("/laser_scan_filtered", 1);
  filtered_laser_scan_pub.publish(laser_filtered);

  /*   Because the stamp of a sensor_msgs/LaserScan is the time of the first measurement,
    one cannot simply wait for a transform to target_frame at this stamp.
    Instead one also has to wait for a transform at the last measurement of the scan. */
  if (!tf_listener_.waitForTransform(
          _msg.header.frame_id, ref_frame_,
          _msg.header.stamp + ros::Duration().fromSec(_msg.ranges.size() * _msg.time_increment),
          ros::Duration(1.0)))
  {
    return;
  }

  sensor_msgs::PointCloud point_cloud;
  laser_projector_.transformLaserScanToPointCloud(ref_frame_, laser_filtered, point_cloud,
                                                  tf_listener_);

  static cv::Mat local_laser_map = cv::Mat::zeros(laser_map_.size(), laser_map_.type());
  cv::Point2i img_point;
  for (auto point : point_cloud.points)
  {
    if (point.z > z_min_th_)
    {
      img_point = coord2img(point.x, point.y);
      local_laser_map.at<uchar>(img_point.x, img_point.y) = 255;
      laser_update_ = true;
    }
  }

  laser_map_ = filterLaserMap(local_laser_map);
  local_laser_map = cv::Mat::zeros(laser_map_.size(), laser_map_.type());
}

void PathPlanner::positionCallback(const nav_msgs::Odometry &_msg)
{
  drone_position_.x = _msg.pose.pose.position.x;
  drone_position_.y = _msg.pose.pose.position.y;
  drone_yaw_ = tf::getYaw(_msg.pose.pose.orientation);

  drone_cell_ = coord2grid(drone_position_);
}

bool PathPlanner::controlNodeSrv(std_srvs::SetBool::Request &_request,
                                 std_srvs::SetBool::Response &_response)
{
  // ROS_INFO("PathPlanner controlNode called");

  if (run_node_ == _request.data)
  {
    _response.success = false;
    _response.message = "Run node already set to " + std::to_string(run_node_);
    std::cout << "Run node already set to " << std::to_string(run_node_) << std::endl;
    return true;
  }

  run_node_ = _request.data;

  _response.success = true;
  _response.message = "Run node set to " + std::to_string(run_node_);
  std::cout << "Run node set to " << std::to_string(run_node_) << std::endl;

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

bool PathPlanner::setGoalSrv(path_planner::setGoalPoint::Request &_request,
                             path_planner::setGoalPoint::Response &_response)
{
  goal_position_.x = _request.goal.point.x;
  goal_position_.y = _request.goal.point.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  goal_cell_ = coord2grid(goal_position_);

  if (!cellIsAvailable(goal_cell_))
  {
    ROS_INFO("Goal cell is occupied. Finding the nearest cell available");
    goal_cell_ = findNearestAvailableCell(goal_cell_, goal_max_distance_);
  }

  if (!cellIsAvailable(goal_cell_)) {
    ROS_INFO("Goal position unreacheable");
    _response.success = false;
    _response.message = "Goal position unreacheable"; 
    return true;
  }

  goal_position_ = grid2coord(goal_cell_);
  std::cout << "New goal: " << goal_position_ << std::endl;

  _response.success = true;
  _response.message = "Goal position set to: " + std::to_string(goal_position_.x) + ", " +
                      std::to_string(goal_position_.y);

  start();

  return true;
}

bool PathPlanner::cellIsAvailable(const cv::Point2i &_cell)
{
  return !(occupancy_map_.at<uchar>(_cell.x, _cell.y) == 0);
}

cv::Point2i PathPlanner::findNearestAvailableCell(const cv::Point2i &_cell, const float _max_distance)
{
  cv::Point2i nearest_cell = _cell;
  int radious = 1;
  while (!cellIsAvailable(nearest_cell))
  {
    for (int i = -radious; i <= radious; i++)
    {
      for (int j = -radious; j <= radious; j++)
      {
        nearest_cell.x = _cell.x + i;
        nearest_cell.y = _cell.y + j;
        if (cellIsAvailable(nearest_cell))
        {
          return nearest_cell;
        }
      }
    }
    if ((radious * occmap_resolution_) < _max_distance) {
      radious++;
    }
    else {
      nearest_cell = _cell;
      break;
    }
  }
  return nearest_cell;
}
///////////////////////////////////////////////////////
// PUBLISH

void PathPlanner::sendMap(const cv::Mat &_map)
{
  sensor_msgs::ImagePtr output_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "rgb8", _map).toImageMsg();
  image_publisher_.publish(output_image_msg);
}

// DEBUG

void PathPlanner::showMap(const cv::Mat &_map, const std::string &_map_name,
                          const bool _add_drone)
{
  // cv::namedWindow(_map_name, cv::WINDOW_FREERATIO);
  // // Generate image to show
  // if (_add_drone)
  // {
  //   cv::Point2i img_drone_position;
  //   img_drone_position = coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
  //   cv::Mat map_img = generateShowImg(_map, img_drone_position);
  //   cv::imshow(_map_name, map_img);
  // }
  // else
  // {
   cv::imshow(_map_name, _map);
  // }
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

cv::Mat PathPlanner::generatePathImg(const cv::Mat &_map, 
                        const cv::Point2i &_drone_px,
                        const cv::Point2i &_goal_px,
                        const std::vector<cv::Point2i> &_path,
                        const std::vector<cv::Point2i> &_waypoints)
{
  cv::Mat path_img, color_img;

  path_img = occupancy_map_.clone();

  if (run_node_){

  for (auto &point : current_path_)
  {
    path_img.at<uchar>(point.x, point.y) = 125;
  }
  }

  cv::cvtColor(path_img, color_img, cv::COLOR_GRAY2BGR);

  if (run_node_) {
  for (auto &waypoint : ref_waypoints_)
  {
    color_img.at<cv::Vec3b>(waypoint.x, waypoint.y) = cv::Vec3b(0, 255, 0);
  }

  color_img.at<cv::Vec3b>(goal_cell_.x, goal_cell_.y) = cv::Vec3b(255, 0, 0);
  }

  color_img.at<cv::Vec3b>(drone_cell_.x, drone_cell_.y) = cv::Vec3b(0, 0, 255);

  return color_img;
}

void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name)
{
  cv::Mat combined_map_top, combined_map_bot, combined_map;

  for (int i = 0; i < maps.size(); i++)
  {
    if (i == 0)
    {
      combined_map_top = maps[i];
    }
    else if (i < 2)
    {
      cv::hconcat(combined_map_top, maps[i], combined_map_top);
    }

    if (i == 2)
    {
      combined_map_bot = maps[i];
    }
    else if (i > 2 && i < 5)
    {
      cv::hconcat(combined_map_bot, maps[i], combined_map_bot);
    }
  }

  // std::vector<cv::Mat> m_top(maps.begin(), maps.begin() + 2);
  // std::vector<cv::Mat> m_bot(maps.begin() + 3, maps.end());
  // cv::hconcat(m_top, combined_map_top);
  cv::imshow("map_top", combined_map_top);
  // cv::hconcat(m_bot, combined_map_bot);
  cv::imshow("map_bot", combined_map_bot);
  // cv::vconcat(combined_map_top, combined_map_bot, combined_map);
  // cv::imshow(window_name, combined_map);
  cv::waitKey(1);
}

// AUXILIAR FUNCTION

cv::Point2i PathPlanner::coord2img(const float _x, const float _y)
{
  cv::Point2i img_point;
  float x = _x / img_resolution_;
  float y = _y / img_resolution_;

  // swamping x and y
  img_point.x = int(img_size_.height - y);
  img_point.y = int(x);

  // saturate with _img_h and _img_w
  img_point.x = std::max(img_point.x, 0);
  img_point.x = std::min(img_point.x, img_size_.height - 1);

  img_point.y = std::max(img_point.y, 0);
  img_point.y = std::min(img_point.y, img_size_.width - 1);

  return img_point;
}

cv::Point2i PathPlanner::coord2grid(const cv::Point2f &_point)
{
  cv::Point2i grid_position;
  float x = _point.x / occmap_resolution_;
  float y = _point.y / occmap_resolution_;
  // swamping x and y
  grid_position.x = int(grid_size_.height - y);
  grid_position.y = int(x);
  // saturate with _img_h and _img_w
  grid_position.x = std::max(grid_position.x, 0);
  grid_position.x = std::min(grid_position.x, grid_size_.height - 1);

  grid_position.y = std::max(grid_position.y, 0);
  grid_position.y = std::min(grid_position.y, grid_size_.width - 1);

  return grid_position;
}

cv::Point2f PathPlanner::grid2coord(const cv::Point2i &_point)
{
  cv::Point2f coord_point;
  coord_point.x = (double)_point.y * occmap_resolution_;
  coord_point.y = (double)(grid_size_.height - _point.x) * occmap_resolution_;
  return coord_point;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint createTrajectoryFromPointMsg(
    const cv::Point2f &_point, const float _height, const float _yaw)
{
  // convert yaw to quaternion
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, _yaw);

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
  trajectory_point.transforms.resize(1);
  trajectory_point.transforms[0].translation.x = _point.x;
  trajectory_point.transforms[0].translation.y = _point.y;
  trajectory_point.transforms[0].translation.z = _height;
  trajectory_point.transforms[0].rotation.x = q.x();
  trajectory_point.transforms[0].rotation.y = q.y();
  trajectory_point.transforms[0].rotation.z = q.z();
  trajectory_point.transforms[0].rotation.w = q.w();
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

// Begin of SPEED_CONTROLLER
geometry_msgs::TwistStamped createSpeedReferenceMsg(cv::Point2d _current_position,
                                                    cv::Point2d _target_position, float _speed)
{
  cv::Point2d speed_to_follow = (_target_position - _current_position);
  float speed_to_follow_z = 0;
  Eigen::Vector3d speed_to_follow_vector(speed_to_follow.x, speed_to_follow.y, speed_to_follow_z);
  speed_to_follow_vector = speed_to_follow_vector.normalized() * _speed;

  geometry_msgs::TwistStamped speed_to_follow_msg;
  speed_to_follow_msg.twist.linear.x = speed_to_follow_vector(0);
  speed_to_follow_msg.twist.linear.y = speed_to_follow_vector(1);
  speed_to_follow_msg.twist.linear.z = speed_to_follow_vector(2);
  speed_to_follow_msg.header.stamp = ros::Time::now();

  return speed_to_follow_msg;
}

geometry_msgs::PoseStamped createPoseStampedMsg(const cv::Point2f &_point, const float _height,
                                                const float _yaw)
{
  // convert yaw to quaternion
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, _yaw);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = _point.x;
  pose_msg.pose.position.y = _point.y;
  pose_msg.pose.position.z = _height;
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  return pose_msg;
}

// End of SPEED_CONTROLLER
