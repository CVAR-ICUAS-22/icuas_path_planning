#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <deque>
#include "A_star_algorithm.hpp"
#include "path_planner/setGoalPoint.h"

#define REF_FRAME "world"
#define LASERSCAN_TOPIC "scan"
#define DRONEPOSITION_TOPIC "odometry"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define SPEEDCONTROL_TOPIC "motion_reference/speed"
#define IMAGE_PUB_TOPIC "occupancy_map"
#define CONTROLNODE_SRV "path_planning/run"
#define SETGOAL_SRV "path_planning/set_goal"
#define SETGOAL_TOPIC "artag_pose"

// SIMULATION
// #define MAP_H 26.0          // in m
// #define MAP_W 16.0          // in m
// REAL
#define MAP_H 26.0          // in m
#define MAP_W 16.0          // in m

#define IMG_RESOLUTION 10.0 // in px/m

#define MAX_DISTANCE_TH 10 // in px
#define OCC_GRID_SIZE 3.0  // 5.0  // in px = 0.1m

#define LASER2BIN_TH 100 // 100 // [0-255]
#define DIST2BIN_TH 0.7  // [0.0-1.0]

#define NEXT_POINT_REACHED_DIST 0.5 // in m
#define Z_MIN_TH 1.0                // in m

#define SPEED_CONTROLLER 0 // Activate Speed Controller [0/1]
#define CONTROL_SPEED 0    // in m/s
#define FLY_HEIGHT 2.0     // in m

class PathPlanner
{
public:
  PathPlanner();
  ~PathPlanner();
  void run();
  void start();
  void stop();
  void setGoal();

  ros::NodeHandle nh_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber droneposition_sub_;
  ros::Publisher waypoint_pub_;

  ros::ServiceServer control_node_srv;
  // ros::ServiceServer set_goal_srv;
  ros::Subscriber set_goal_sub_;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;

  tf::TransformListener tf_listener_;

  laser_geometry::LaserProjection laser_projector_;
  AStarPlanner planner_algorithm_;

  void laserscanCallback(const sensor_msgs::LaserScan &_msg);
  void positionCallback(const nav_msgs::Odometry &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request, std_srvs::SetBool::Response &_response);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request, path_planner::setGoalPoint::Response &_response);
  void setGoalCallback(const geometry_msgs::PoseStamped &_msg);

  bool laser_update_ = false;
  bool new_occupancy_map_ = false;
  bool generate_path_ = false;
  bool run_node_ = false;
  bool goal_set_ = false;
  bool force_generation_ = false;

  int img_h_;
  int img_w_;
  cv::Size grid_size_;
  std::string ref_frame_;

  cv::Mat laser_map_;
  cv::Mat occupancy_map_;
  std::vector<float> laser_mesuraments;
  std::vector<cv::Point2i> current_path_;
  std::vector<cv::Point2i> ref_waypoints_;

  cv::Point2f drone_position_;
  cv::Point2f goal_position_;
  float drone_yaw_;
  float fly_height_;

  cv::Point2i drone_cell_;
  cv::Point2i goal_cell_;

  // Begin of SPEED_CONTROLLER
  ros::Publisher speed_control_pub_;
  float control_speed_;
  // End of SPEED_CONTROLLER

  void generateOccupancyMap();
  void checkCurrentPath();
  void generateNewPath();
  void optimizePath();

  void sendMap(cv::Mat _map);
  void showMap(const cv::Mat &_map, const std::string &_map_name, const bool _add_drone);

  cv::Mat generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px);
};

cv::Point2i coord2img(const float _x, const float _y, const int _img_h, const int _img_w);
cv::Point2i coord2grid(const float _x, const float _y, const int _img_h, const int _img_w);
trajectory_msgs::MultiDOFJointTrajectoryPoint createTrajectoryFromPointMsg(const cv::Point2f &_point, const float _height, const float _yaw);
geometry_msgs::TwistStamped createSpeedReferenceMsg(cv::Point2d _current_position, cv::Point2d _target_position, float _speed);

cv::Point2f grid2coord(const cv::Point2i &_point, const int _img_h, const int _img_w);
cv::Mat generatePathImg(const cv::Mat &_map, const cv::Point2i &_drone_px, const std::vector<cv::Point2i> &_path, const std::vector<cv::Point2i> &_waypoints);
void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name);

#endif // PATH_PLANNER_HPP_
