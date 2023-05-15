#include "path_planner.hpp"


PathPlanner::PathPlanner() : it_(nh_) {
  occupancy_image_sub_ = nh_.subscribe(
      OCCUPANCY_IMAGE_TOPIC, 1, &PathPlanner::occupancyImageCallback, this);

  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1,
                                     &PathPlanner::positionCallback, this);

  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
      WAYPOINT_TOPIC, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
  has_ended_pub_ = nh_.advertise<std_msgs::Bool>(PATHPLANNER_HAS_ENDED_TOPIC, 1);
  image_publisher_ = it_.advertise(IMAGE_PUB_TOPIC, 1);

  reset_octomap_ = nh_.serviceClient<std_srvs::Empty>(RESET_OCTOMAP_SRV);

  control_node_srv =
      nh_.advertiseService(CONTROLNODE_SRV, &PathPlanner::controlNodeSrv, this);
  set_goal_srv =
      nh_.advertiseService(SETGOAL_SRV, &PathPlanner::setGoalSrv, this);

  float map_h;
  float map_w;

  nh_.getParam("path_planner/map_h", map_h);
  nh_.getParam("path_planner/map_w", map_w);
  nh_.getParam("path_planner/occmap_resolution", occmap_resolution_);

  nh_.getParam("path_planner/fly_height", fly_height_);
  nh_.getParam("path_planner/next_point_reached_dist",
               next_point_reached_dist_);

  ROS_INFO("map_h: %.2f", map_h);
  ROS_INFO("map_w: %.2f", map_w);
  ROS_INFO("resolution: %.2f", occmap_resolution_);
  ROS_INFO("fly_height: %.2f", fly_height_);
  ROS_INFO("next_point_reached_dist: %.2f", next_point_reached_dist_);

  occmap_h_ = int(map_h / occmap_resolution_);
  occmap_w_ = int(map_w / occmap_resolution_);

  occ_map_received_ = false;

  // nh_.getParam("path_planner/speed_controller", speed_controller_);
  // nh_.getParam("path_planner/max_control_speed", max_control_speed_);
  //
  // std::string controller_str = speed_controller_ ? "SPEED" : "POSITION";
  // ROS_INFO("controller: %s", controller_str.c_str());
  // if (speed_controller_) {
  //   ROS_INFO("control_speed: %.2f", max_control_speed_);
  // }
  // if (speed_controller_) {
  //   speed_control_pub_ =
  //       nh_.advertise<geometry_msgs::TwistStamped>(SPEEDCONTROL_TOPIC, 1);
  // }

}

PathPlanner::~PathPlanner() {}

void PathPlanner::start() {
  ROS_INFO("Node started");
  run_node_ = true;
  generate_path_ = true;
  goal_reached_ = false;
  no_solution_ = false;
}

void PathPlanner::stop() {
  ROS_INFO("Node stopped");
  goal_position_ = cv::Point2f(0, 0);
  goal_cell_ = cv::Point2i(0, 0);
  run_node_ = false;
}

void PathPlanner::run() {
  static bool send_waypoint = false;

  if (!occ_map_received_) {
    return;
  }

  cv::Mat path_map = generatePathImg(occupancy_map_, drone_cell_, current_path_,
                                     ref_waypoints_);
  sendMap(path_map);
  showMap(path_map, "PATH PLANNER MAP", false);

  if (!run_node_) {
    return;
  }

  if (new_occupancy_map_)
  {
    new_occupancy_map_ = false;
    checkCurrentPath();
  }

  if (no_solution_) {
    static cv::Point2f hover_position = drone_position_;
    ROS_WARN("Solution not found");
    sendWaypoint(hover_position, 0.0);
    endNavigation();
    return;
  }

  if (generate_path_) {
    generate_path_ = false;
    generateNewPath();
    optimizePath();
    send_waypoint = true;
  }

  // Check if the goal has been reached
  float distance = sqrt(pow(goal_position_.x - drone_position_.x, 2) +
                        pow(goal_position_.y - drone_position_.y, 2));
  if (distance < goal_reached_dist_) {
    ROS_INFO("Goal reached");
    goal_reached_ = true;
    endNavigation();
    return; 
  }
  
  if (ref_waypoints_.size() > 0) {
    float sending_yaw;
    cv::Point2f next_point = map2coord(ref_waypoints_[0]);

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
            map2coord(ref_waypoints_[1]);

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
        }
      }
    }
    // // IF NEXT POINT IS NOT REACHED: CONTINUE SENDIING THE SAME POINT
    else {
      float next_yaw = atan2(next_point.y - drone_position_.y,
                             next_point.x - drone_position_.x);
      sending_yaw = next_yaw;
      send_waypoint = true;
    }

    if (send_waypoint) {
      sendWaypoint(next_point, sending_yaw);
    }
  }

  // showMap(occupancy_map_, "ICUAS occupancy map", false);
  // showMap(path_map, "ICUAS path map", false);
}

void PathPlanner::endNavigation()
{
  std_msgs::Bool msg;
  msg.data = goal_reached_;
  has_ended_pub_.publish(msg);
  ROS_INFO("End navigation");
}

void PathPlanner::sendWaypoint(const cv::Point2f _next_point,
                               const float _sending_yaw) {
    waypoint_pub_.publish(
        createTrajectoryFromPointMsg(_next_point, fly_height_, _sending_yaw));
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
  current_path_.clear();
  // int n_attempts = 10;

  // Try to find a solution many times before give up
  // for (int i=0; i < n_attempts; i++) {
    // ROS_INFO("Generating new path");
  planner_algorithm_.setOriginPoint(drone_cell_);
  planner_algorithm_.setGoal(goal_cell_);
  planner_algorithm_.setOcuppancyGrid(occupancy_map_);
  current_path_ = planner_algorithm_.solveGraph();

    // if (current_path_.size() == 0) {
    //   ROS_INFO("Path not found, regenerating occupancy map");
      // ROS_INFO("Path not found, generating occupancy map %d/%d", i, n_attempts);
      // std_srvs::Empty reset;
      // if (reset_octomap_.call(reset)){
      //   ROS_INFO("Call to service successful");
      //   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

      // }
      // continue;
    // }
  //   break;
  // }
}

void PathPlanner::optimizePath() {
  ROS_DEBUG("Optimizing path");
  ref_waypoints_.clear();

  if (current_path_.size() == 0) {
    ROS_ERROR("Path is empty");
    no_solution_ = true;
    return;
  }

  if (!optimize_path_) {
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

void PathPlanner::occupancyImageCallback(const sensor_msgs::Image &_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  occupancy_map_ = cv_ptr->image;
  new_occupancy_map_ = true;
  occ_map_received_ = true;

  return;
}

void PathPlanner::positionCallback(const geometry_msgs::PoseStamped &_msg) {
  drone_position_.x = _msg.pose.position.x;
  drone_position_.y = _msg.pose.position.y;

  drone_yaw_ = tf::getYaw(_msg.pose.orientation);
  drone_cell_ = coord2map(drone_position_);
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
  if (run_node_) {
    _response.success = false;
    _response.message = "Node already performing path planning";
    ROS_WARN("Node already performing path planning");
    return false;
  }
  std_srvs::Empty reset;
  // reset_octomap_.call(reset);

  goal_position_.x = _request.goal.point.x;
  goal_position_.y = _request.goal.point.y;

  ROS_INFO("Goal position: %f, %f", goal_position_.x, goal_position_.y);

  _response.success = true;
  _response.message =
      "Goal position set to: " + std::to_string(goal_position_.x) + ", " +
      std::to_string(goal_position_.y);

  goal_cell_ = coord2map(goal_position_);

  start();

  return true;
}

// PUBLISH

void PathPlanner::sendMap(cv::Mat _map) {
  sensor_msgs::ImagePtr output_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "rgb8", _map).toImageMsg();
  image_publisher_.publish(output_image_msg);
}

// DEBUG

void show_image_resized(const std::string &_title, const cv::Mat &_image) {
  cv::Mat map_resized = cv::Mat::zeros(_image.size() * 5, _image.type());
  cv::resize(_image, map_resized, map_resized.size(), 0, 0, cv::INTER_NEAREST);
  cv::imshow(_title, map_resized);
}

void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name) {
  cv::Mat combined_map_top, combined_map_bot, combined_map;

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

void PathPlanner::showMap(const cv::Mat &_map, const std::string &_map_name,
                          const bool _add_drone) {
  cv::namedWindow(_map_name, cv::WINDOW_FREERATIO);
  // Generate image to show
  if (_add_drone) {
    // cv::Point2i img_drone_position;
    // img_drone_position =
    //     coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
    // cv::Mat map_img = generateShowImg(_map, img_drone_position);
    // cv::imshow(_map_name, map_img);
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
//
cv::Point2i PathPlanner::coord2map(const cv::Point2f &_point) {

  cv::Point2i map_point;
  float x = _point.x / occmap_resolution_;
  float y = _point.y / occmap_resolution_;

  // swaping x and y
  map_point.x = int(occmap_h_ - y);
  map_point.y = int(x);
  // saturate with _img_h and _img_w
  map_point.x = std::max(map_point.x, 0);
  map_point.x = std::min(map_point.x, occmap_h_ - 1);

  map_point.y = std::max(map_point.y, 0);
  map_point.y = std::min(map_point.y, occmap_w_ - 1);

  return map_point;
}

cv::Point2f PathPlanner::map2coord(const cv::Point2i &_point) {

  cv::Point2f coord_point;
  coord_point.x = _point.y * occmap_resolution_;
  coord_point.y = (occmap_h_ - _point.x) * occmap_resolution_;

  return coord_point; // in meters
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
