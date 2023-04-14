#include "path_planner.hpp"

#include "geometry_msgs/PoseStamped.h"

PathPlanner::PathPlanner() : it_(nh_) {
  occupancy_image_sub_ = nh_.subscribe(
      OCCUPANCY_IMAGE_TOPIC, 1, &PathPlanner::occupancyImageCallback, this);

  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1,
                                     &PathPlanner::positionCallback, this);

  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
      WAYPOINT_TOPIC, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);

  image_publisher_ = it_.advertise(IMAGE_PUB_TOPIC, 1);
  control_node_srv =
      nh_.advertiseService(CONTROLNODE_SRV, &PathPlanner::controlNodeSrv, this);
  // set_goal_srv = nh_.advertiseService(SETGOAL_SRV, &PathPlanner::setGoalSrv,
  // this);
  set_goal_sub_ =
      nh_.subscribe(SETGOAL_TOPIC, 1, &PathPlanner::setGoalCallback, this);
  float map_h;
  float map_w;
  float max_distance;
  float occ_map_grid_size;
  float goal_position_x, goal_position_y;

  nh_.getParam("path_planner/map_h", map_h);
  nh_.getParam("path_planner/map_w", map_w);
  nh_.getParam("path_planner/img_resolution", img_resolution_);
  nh_.getParam("path_planner/occ_grid_size", occ_map_grid_size);
  nh_.getParam("path_planner/z_min_th", z_min_th_);
  nh_.getParam("path_planner/ref_frame", ref_frame_);

  nh_.getParam("path_planner/goal_position_x", goal_position_x);
  nh_.getParam("path_planner/goal_position_y", goal_position_y);
  nh_.getParam("path_planner/fly_height", fly_height_);
  nh_.getParam("path_planner/security_distance", max_distance);
  nh_.getParam("path_planner/next_point_reached_dist",
               next_point_reached_dist_);
  nh_.getParam("path_planner/speed_controller", speed_controller_);
  nh_.getParam("path_planner/max_control_speed", max_control_speed_);
  nh_.getParam("path_planner/x_safe_zone", x_safe_zone_);

  ROS_INFO("map_h: %.2f", map_h);
  ROS_INFO("map_w: %.2f", map_w);
  ROS_INFO("img_resolution: %.2f", img_resolution_);
  ROS_INFO("occ_grid_size: %.2f", occ_map_grid_size);
  ROS_INFO("z_min_th: %.2f", z_min_th_);
  ROS_INFO("ref_frame: %s", ref_frame_.c_str());
  ROS_INFO("fly_height: %.2f", fly_height_);
  ROS_INFO("security distance: %.2f", max_distance);
  ROS_INFO("next_point_reached_dist: %.2f", next_point_reached_dist_);
  std::string controller_str = speed_controller_ ? "SPEED" : "POSITION";
  ROS_INFO("controller: %s", controller_str.c_str());
  if (speed_controller_) {
    ROS_INFO("control_speed: %.2f", max_control_speed_);
  }

  // ref_frame_ = REF_FRAME;
  img_h_ = map_h * img_resolution_;
  img_w_ = map_w * img_resolution_;
  occ_grid_size_ = occ_map_grid_size * img_resolution_;
  grid_size_.width = int((map_w / occ_map_grid_size));
  grid_size_.height = int((map_h / occ_map_grid_size));
  occupancy_map_ =
      cv::Mat::zeros(grid_size_.height, grid_size_.width, CV_8UC1) * 255;
  max_distance_th_ = max_distance * img_resolution_;

  if (speed_controller_) {
    speed_control_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>(SPEEDCONTROL_TOPIC, 1);
  }

  // Node start
  // goal_position_ = cv::Point2f(6, 0);
  goal_position_ = cv::Point2f(goal_position_x, goal_position_y);
  goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);
  ROS_WARN("Goal position set to default: %f, %f", goal_position_.x,
           goal_position_.y);
}

PathPlanner::~PathPlanner() {}

void PathPlanner::start() {
  ROS_INFO("Node started");
  // goal_position_ = cv::Point2f(6, 0);
  // goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_,
  // img_w_); ROS_WARN("Goal position set to default: %f, %f", goal_position_.x,
  // goal_position_.y); if (goal_position_ == cv::Point2f(0, 0))
  // {
  //   goal_position_ = cv::Point2f(5, 0);
  //   goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_,
  //   img_w_); ROS_WARN("Goal position set to default: %f, %f",
  //   goal_position_.x, goal_position_.y);
  // }
  force_generation_ = true;
}

void PathPlanner::stop() {
  ROS_INFO("Node stopped");
  goal_set_ = false;
  goal_position_ = cv::Point2f(0, 0);
  goal_cell_ = cv::Point2i(0, 0);
}

void PathPlanner::run() {
  static bool send_waypoint = false;
  // force_generation_ = true; // DEBUG

  // OLD
  // if (laser_update_ || force_generation_) {
  //   ROS_DEBUG("New laser measuments");
  //   laser_update_ = false;
  //   generateOccupancyMap();
  // }

  // if (new_occupancy_map_ || force_generation_) {
  //   new_occupancy_map_ = false;
  //   ROS_DEBUG("New occupancy map available");
  //   // sendMap(occupancy_map_); // DEBUG
  //   checkCurrentPath();
  // }

  cv::Mat path_map = generatePathImg(occupancy_map_, drone_cell_, current_path_,
                                     ref_waypoints_);
  sendMap(path_map);
  showMap(path_map, "RUN MAP", false);

  if (no_solution_) {
    static cv::Point2f hover_position = drone_position_;
    ROS_WARN_ONCE("Solution not found");
    sendWaypoint(hover_position, 0.0);
    return;
  }

  if (generate_path_ || force_generation_) {
    generate_path_ = false;
    generateNewPath();
    optimizePath();
    send_waypoint = true;
  }

  if (!run_node_) {
    return;
  }

  if (!ending_maze_) {
    if (drone_position_.x > x_safe_zone_) {
      ending_maze_ = true;
      ROS_WARN("Ending maze");
    }
  }
  // TESTING YAW
  // if (current_path_.size() > 0 & send_waypoint)
  if (ref_waypoints_.size() > 0) {
    float sending_yaw;
    cv::Point2f next_point = grid2coord(ref_waypoints_[0], img_h_, img_w_);
    double distance_to_next_point =
        sqrt(pow(next_point.x - drone_position_.x, 2) +
             pow(next_point.y - drone_position_.y, 2));

    // IF NEXT POINT IS REACHED: GET THE NEXT POINT
    if (distance_to_next_point < next_point_reached_dist_) {
      // Do not check future point
      if (!check_future_point_)
        ref_waypoints_.erase(ref_waypoints_.begin());

      if (check_future_point_) {
        // Check future point
        cv::Point2f future_point =
            grid2coord(ref_waypoints_[1], img_h_, img_w_);

        float future_yaw = atan2(future_point.y - drone_position_.y,
                                 future_point.x - drone_position_.x);
        float distance_to_future_point =
            sqrt(pow(future_point.x - drone_position_.x, 2) +
                 pow(future_point.y - drone_position_.y, 2));

        // IF DRONE IS LOOKING TO THE FUTURE POINT OR HAS ALREADY REACHED IT:
        // SEND THE NEXT POINT
        if (abs(future_yaw - drone_yaw_) < M_PI_4 ||
            distance_to_future_point < next_point_reached_dist_) {
          ref_waypoints_.erase(ref_waypoints_.begin());
          send_waypoint = false;
        }
        // ELSE: SEND THE CURRENT POINT
        else {
          sending_yaw = future_yaw;
          send_waypoint = true;
          // waypoint_pub_.publish(createTrajectoryFromPointMsg(next_point,
          // fly_height_, future_yaw));
        }
      }
    }
    // // IF NEXT POINT IS NOT REACHED: CONTINUE SENDIING THE SAME POINT
    else {
      float next_yaw = atan2(next_point.y - drone_position_.y,
                             next_point.x - drone_position_.x);
      sending_yaw = next_yaw;
      // waypoint_pub_.publish(createTrajectoryFromPointMsg(next_point,
      // fly_height_, next_yaw));
      send_waypoint = true;
    }

    if (send_waypoint) {
      sendWaypoint(next_point, sending_yaw);
    }
    // float next_yaw = atan2(next_point.y - drone_position_.y, next_point.x -
    // drone_position_.x); if (!speed_controller_) {
    //   waypoint_pub_.publish(createTrajectoryFromPointMsg(next_point,
    //   fly_height_, sending_yaw));
    // }
    // if (speed_controller_) {
    //   if (ending_maze_) {
    //     ROS_INFO("Sending pose %f, %f", next_point.x, next_point.y);
    //     pose_pub_.publish(createPoseStampedMsg(next_point, fly_height_,
    //     sending_yaw));
    //   } else {
    //     speed_control_pub_.publish(
    //         createSpeedReferenceMsg(drone_position_, next_point,
    //         max_control_speed_));
    //   }
    // }

    // if (SPEED_CONTROLLER)
    //   speed_control_pub_.publish(createSpeedReferenceMsg(drone_position_,
    //   next_point, control_speed_));

    // if (distance < NEXT_POINT_REACHED_DIST)
    // {
    //   ref_waypoints_.erase(ref_waypoints_.begin());
    //   send_waypoint = true;
    // }
    // }
  }

  // showMap(occupancy_map_, "ICUAS occupancy map", false);
  // showMap(path_map, "ICUAS path map", false);

  if (force_generation_)
    force_generation_ = false;
}

void PathPlanner::sendWaypoint(const cv::Point2f _next_point,
                               const float _sending_yaw) {
  if (!speed_controller_) {
    waypoint_pub_.publish(
        createTrajectoryFromPointMsg(_next_point, fly_height_, _sending_yaw));
  }
  if (speed_controller_) {
    if (ending_maze_ || no_solution_) {
      // ROS_INFO("Sending pose %f, %f", _next_point.x, _next_point.y);
      pose_pub_.publish(
          createPoseStampedMsg(_next_point, fly_height_, _sending_yaw));
    } else {
      speed_control_pub_.publish(createSpeedReferenceMsg(
          drone_position_, _next_point, max_control_speed_));
    }
  }
}

void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name) {
  cv::Mat combined_map_top, combined_map_bot, combined_map;
  // for (auto m : maps)
  // {
  //   ROS_INFO("Map size: %d, %d, %d", m.size().height, m.size().width,
  //   m.channels()); ROS_INFO("Map type: %d", m.type());
  // }

  for (int i = 0; i < maps.size(); i++) {
    if (i == 0) {
      combined_map_top = maps[i];
    } else if (i < 2) {
      cv::hconcat(combined_map_top, maps[i], combined_map_top);
    }

    if (i == 2) {
      combined_map_bot = maps[i];
    } else if (i > 2 && i < 5) {
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

void PathPlanner::checkCurrentPath() {
  // Check current path is not empty
  if (current_path_.size() == 0) {
    generate_path_ = true;
    return;
  }

  // Check if path points are free on the map
  for (auto &point : current_path_) {
    if (occupancy_map_.at<uchar>(point.x, point.y) == 0) {
      generate_path_ = true;
      break;
    }
  }
}

void PathPlanner::generateNewPath() {
  ROS_DEBUG("Generating new path");
  current_path_.clear();

  planner_algorithm_.setOriginPoint(drone_cell_);
  planner_algorithm_.setGoal(goal_cell_);
  planner_algorithm_.setOcuppancyGrid(occupancy_map_);

  current_path_ = planner_algorithm_.solveGraph();
}

void PathPlanner::optimizePath() {
  ROS_DEBUG("Optimizing path");
  ref_waypoints_.clear();

  if (current_path_.size() == 0) {
    ROS_ERROR("Path is empty");
    no_solution_ = true;
    return;
  }

  bool optimize = true;
  if (!optimize) {
    ref_waypoints_ = current_path_;
    return;
  }

  if (!current_path_.size()) {
    ROS_ERROR("QUEEEEEEEEEEEEEEEEEEE");
  }
  ref_waypoints_.emplace_back(current_path_[0]);

  cv::Point2i path_point;

  for (int i = 1; i < current_path_.size(); i++) {
    path_point = current_path_[i];
    // ROS_INFO("Path point: %.2i, %.2i", path_point.x, path_point.y);
    // ROS_INFO("Last ref point: %.2i, %.2i", ref_waypoints_.back().x,
    // ref_waypoints_.back().y);
    if (path_point.x == ref_waypoints_.back().x ||
        path_point.y == ref_waypoints_.back().y) {
      continue;
    }

    // if (path_point.x != ref_waypoints_.back().x && path_point.y !=
    // ref_waypoints_.back().y)
    // {
    //   ROS_INFO("Path point: %.2i, %.2i", path_point.x, path_point.y);
    //   ROS_INFO("Last ref point: %.2i, %.2i", ref_waypoints_.back().x,
    //   ref_waypoints_.back().y); continue;
    // }
    // else
    // {
    ref_waypoints_.emplace_back(current_path_[i - 1]);
    // }
  }

  ref_waypoints_.emplace_back(current_path_[current_path_.size() - 1]);
}

// CALLBACK

void show_image_resized(const std::string &_title, const cv::Mat &_image) {
  cv::Mat map_resized = cv::Mat::zeros(_image.size() * 5, _image.type());
  cv::resize(_image, map_resized, map_resized.size(), 0, 0, cv::INTER_NEAREST);
  cv::imshow(_title, map_resized);
}

void PathPlanner::occupancyImageCallback(const sensor_msgs::Image &_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  occupancy_map_ = cv_ptr->image;
}

void PathPlanner::positionCallback(const geometry_msgs::PoseStamped &_msg) {
  drone_position_.x = _msg.pose.position.x;
  drone_position_.y = _msg.pose.position.y;

  drone_yaw_ = tf::getYaw(_msg.pose.orientation);

  drone_cell_ =
      coord2grid(drone_position_.x, drone_position_.y, img_h_, img_w_);
}

bool PathPlanner::controlNodeSrv(std_srvs::SetBool::Request &_request,
                                 std_srvs::SetBool::Response &_response) {
  if (run_node_ == _request.data) {
    _response.success = false;
    _response.message = "Run node already set to " + std::to_string(run_node_);
    return true;
  }

  run_node_ = _request.data;

  _response.success = true;
  _response.message = "Run node set to " + std::to_string(run_node_);

  if (run_node_) {
    start();
  } else {
    stop();
  }

  return true;
}

bool PathPlanner::setGoalSrv(path_planner::setGoalPoint::Request &_request,
                             path_planner::setGoalPoint::Response &_response) {
  goal_position_.x = _request.goal.point.x;
  goal_position_.y = _request.goal.point.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  _response.success = true;
  _response.message =
      "Goal position set to: " + std::to_string(goal_position_.x) + ", " +
      std::to_string(goal_position_.y);

  goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);

  goal_set_ = true;
  // force_generation_ = true;

  return true;
}

void PathPlanner::setGoalCallback(const geometry_msgs::PoseStamped &_msg) {
  float distance = sqrt(pow(_msg.pose.position.x - drone_position_.x, 2) +
                        pow(_msg.pose.position.y - drone_position_.y, 2));
  if (distance < 5.0) {
    return;
  }

  goal_position_.x = _msg.pose.position.x;
  goal_position_.y = _msg.pose.position.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  goal_cell_ = coord2grid(goal_position_.x, goal_position_.y, img_h_, img_w_);

  goal_set_ = true;
  // force_generation_ = true;
}

// PUBLISH

void PathPlanner::sendMap(cv::Mat _map) {
  sensor_msgs::ImagePtr output_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "rgb8", _map).toImageMsg();
  image_publisher_.publish(output_image_msg);
}

// DEBUG

void PathPlanner::showMap(const cv::Mat &_map, const std::string &_map_name,
                          const bool _add_drone) {
  cv::namedWindow(_map_name, cv::WINDOW_FREERATIO);
  // Generate image to show
  if (_add_drone) {
    cv::Point2i img_drone_position;
    img_drone_position =
        coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
    cv::Mat map_img = generateShowImg(_map, img_drone_position);
    cv::imshow(_map_name, map_img);
  } else {
    cv::imshow(_map_name, _map);
  }
  cv::waitKey(1);
}

cv::Mat PathPlanner::generateShowImg(const cv::Mat &_img,
                                     const cv::Point2i &_drone_px) {
  cv::Mat color_img, show_img;
  cv::cvtColor(_img, color_img, cv::COLOR_GRAY2BGR);

  cv::circle(color_img, cv::Point(_drone_px.y, _drone_px.x), 3,
             cv::Scalar(255, 0, 0), cv::FILLED);
  // cv::resize(color_img, show_img, cv::Size(400, 600));
  show_img = color_img;
  return show_img;
}

cv::Mat generatePathImg(const cv::Mat &_map, const cv::Point2i &_drone_px,
                        const std::vector<cv::Point2i> &_path,
                        const std::vector<cv::Point2i> &_waypoints) {
  cv::Mat path_img, color_img;

  path_img = _map.clone();

  for (auto &point : _path) {
    path_img.at<uchar>(point.x, point.y) = 125;
  }

  cv::cvtColor(path_img, color_img, cv::COLOR_GRAY2BGR);

  for (auto &waypoint : _waypoints) {
    color_img.at<cv::Vec3b>(waypoint.x, waypoint.y) = cv::Vec3b(0, 255, 0);
  }

  color_img.at<cv::Vec3b>(_drone_px.x, _drone_px.y) = cv::Vec3b(0, 0, 255);

  return color_img;
}

// AUXILIAR FUNCTION

cv::Point2i PathPlanner::coord2img(const float _x, const float _y,
                                   const int _img_h, const int _img_w) {
  cv::Point2i img_point;
  // float w = _img_w / 2;
  // float h = _img_h / 2;
  float x = _x / img_resolution_;
  float y = _y / img_resolution_;
  img_point.x = int(x);
  img_point.y = int(_img_h - y);

  // saturate with _img_h and _img_w

  img_point.x = std::max(img_point.x, 0);
  img_point.x = std::min(img_point.x, _img_h - 1);

  img_point.y = std::max(img_point.y, 0);
  img_point.y = std::min(img_point.y, _img_w - 1);

  return img_point;
}

cv::Point2i PathPlanner::coord2grid(const float _x, const float _y,
                                    const int _img_h, const int _img_w) {
  ROS_INFO("Real pose: %f %f", _x, _y);

  cv::Point2i img_drone_position, grid_drone_position;
  img_drone_position = coord2img(_x, _y, _img_h, _img_w);
  ROS_INFO("Image pose: %f %f", img_drone_position.x, img_drone_position.y);

  grid_drone_position.x = int((img_drone_position.x / occ_grid_size_)); // x
  grid_drone_position.y = int((img_drone_position.y / occ_grid_size_)); // y
  ROS_INFO("Grid pose: %f %f", grid_drone_position.x, grid_drone_position.y);

  return grid_drone_position;
}

cv::Point2f PathPlanner::grid2coord(const cv::Point2i &_point, const int _img_h,
                                    const int _img_w) {
  cv::Point2f coord_point;

  // ROS_INFO("Point: %d, %d", _point.x, _point.y);
  // ROS_INFO("Img h: %d, w: %d", _img_h / 2, _img_w / 2);
  // ROS_INFO("%d, %d", _point.x * OCC_GRID_SIZE, _point.y * OCC_GRID_SIZE);
  coord_point.x =
      (((double)_img_h / 2.0) - (_point.x * occ_grid_size_)) / img_resolution_;
  coord_point.y =
      (((double)_img_w / 2.0) - (_point.y * occ_grid_size_)) / img_resolution_;
  // ROS_INFO("First x: %f, y: %f", coord_point.x, coord_point.y);
  coord_point.x -= (occ_grid_size_ / 2) / img_resolution_;
  coord_point.y -= (occ_grid_size_ / 2) / img_resolution_;
  // ROS_INFO("Second x: %f, y: %f", coord_point.x, coord_point.y);

  // saturate the coordinates
  // coord_point.x = std::max((double)coord_point.x,(double) _img_h / 2.0);
  // coord_point.x = std::min((double)coord_point.x,(double) -_img_h / 2.0);
  // coord_point.y = std::max((double)coord_point.y,(double) _img_w / 2.0);
  // coord_point.y = std::min((double)coord_point.y,(double) -_img_w / 2.0);

  // coord_point.x = (int) coord_point.x;
  // coord_point.y = (int) coord_point.y;

  return coord_point;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint
createTrajectoryFromPointMsg(const cv::Point2f &_point, const float _height,
                             const float _yaw) {
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

// Begin of SPEED_CONTROLLER
geometry_msgs::TwistStamped
createSpeedReferenceMsg(cv::Point2d _current_position,
                        cv::Point2d _target_position, float _speed) {
  cv::Point2d speed_to_follow = (_target_position - _current_position);
  float speed_to_follow_z = 0;
  Eigen::Vector3d speed_to_follow_vector(speed_to_follow.x, speed_to_follow.y,
                                         speed_to_follow_z);
  speed_to_follow_vector = speed_to_follow_vector.normalized() * _speed;

  geometry_msgs::TwistStamped speed_to_follow_msg;
  speed_to_follow_msg.twist.linear.x = speed_to_follow_vector(0);
  speed_to_follow_msg.twist.linear.y = speed_to_follow_vector(1);
  speed_to_follow_msg.twist.linear.z = speed_to_follow_vector(2);
  speed_to_follow_msg.header.stamp = ros::Time::now();

  return speed_to_follow_msg;
}

geometry_msgs::PoseStamped createPoseStampedMsg(const cv::Point2f &_point,
                                                const float _height,
                                                const float _yaw) {
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
