#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <string>

#include "A_star_algorithm.hpp"
#include "path_planner/setGoalPoint.h"
#include "tf/transform_listener.h"

#define LASERSCAN_TOPIC "scan"
#define DRONEPOSITION_TOPIC "odometry"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define SPEEDCONTROL_TOPIC "motion_reference/speed"
#define POSE_TOPIC "motion_reference/pose"
#define OCCMAP_PUB_TOPIC "occupancy_map"
#define EGOMAP_PUB_TOPIC "ego_map"
#define PATHPLANNER_HAS_ENDED_TOPIC "path_planning/has_ended"

#define CONTROLNODE_SRV "path_planning/run"
#define SETGOAL_SRV "path_planning/set_goal"

#define LASER2BIN_TH 100      // 100 // [0-255]
#define DIST2BIN_TH 0.7       // [0.0-1.0]
#define LASER_FILTER_MARGIN 0 // in pixels, see laserscanCallback

class PathPlanner {
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
  ros::Publisher has_ended_pub_;

  ros::ServiceServer control_node_srv;
  ros::ServiceServer set_goal_srv;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;
  image_transport::Publisher egomap_publisher_;

  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  tf::TransformListener tf_listener_;

  laser_geometry::LaserProjection laser_projector_;
  AStarPlanner planner_algorithm_;

  void laserscanCallback(const sensor_msgs::LaserScan &_msg);
  void positionCallback(const nav_msgs::Odometry &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request,
                      std_srvs::SetBool::Response &_response);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request,
                  path_planner::setGoalPoint::Response &_response);
  void endNavigation();

  // PARAMETERS
  float img_resolution_ = 0.1;          // meters/pixel
  float occmap_resolution_ = 0.5;       // meters/pixel
  float security_distance_th_ = 2.0;    // meters
  float z_min_th_ = 1.0;                // meters
  float z_max_th_ = 2.0;                // meters
  float fly_height_ = 2.0;              // meters
  float next_point_reached_dist_ = 0.5; // meters
  float goal_reached_dist_ = 1.0;       // meters
  float drone_max_distance_ = 5.0;      // meters
  float goal_max_distance_ = 5.0;       // meters
  float ego_radious_ = 2.5;             // meters
  std::string ref_frame_ = "world";
  bool check_future_point_ = false;
  int max_attempts_ = 5;
  int attempts_count_ = 0;
  bool center_origin_ = true;

  int laser_filter_margin_ = LASER_FILTER_MARGIN;
  bool reset_occ_map_enabled_ = false;

  bool run_node_ = false;
  bool goal_reached_ = false;
  bool no_solution_ = false;
  bool laser_update_ = false;
  bool new_occupancy_map_ = false;
  bool generate_path_ = false;
  bool force_generation_ = false;
  cv::Size img_size_;
  cv::Size grid_size_;
  float drone_yaw_;
  cv::Point2f drone_position_;
  cv::Point2f goal_position_;
  cv::Point2i drone_cell_;
  cv::Point2i goal_cell_;
  cv::Mat laser_map_;
  cv::Mat occupancy_map_;
  std::vector<float> laser_mesuraments_;
  std::vector<cv::Point2i> current_path_;
  std::vector<cv::Point2i> ref_waypoints_;

  // Begin of SPEED_CONTROLLER
  ros::Publisher speed_control_pub_;
  ros::Publisher pose_pub_;
  bool speed_controller_ = false;
  float max_control_speed_ = 0.0;
  // End of SPEED_CONTROLLER

  void generateOccupancyMap();
  void resetOccupancyMap();
  void checkCurrentPath();
  void generateNewPath();
  void optimizePath();
  void sendWaypoint(const cv::Point2f &_next_point, const float _sending_yaw);
  void prepareNewAttempt();
  bool cellIsAvailable(const cv::Point2i &_cell);
  cv::Point2i findNearestAvailableCell(const cv::Point2i &_cell,
                                       const float _max_distance);

  void sendMap(const cv::Mat &_map);
  void showMap(const cv::Mat &_map, const std::string &_map_name,
               const bool _add_drone);

  cv::Mat generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px);
  cv::Point2i coord2img(const float _x, const float _y);
  cv::Point2i coord2grid(const cv::Point2f &_point);
  cv::Point2f grid2coord(const cv::Point2i &_point);

  cv::Mat generatePathImg(const cv::Mat &_map, const cv::Point2i &_drone_px,
                          const cv::Point2i &_goal_px,
                          const std::vector<cv::Point2i> &_path,
                          const std::vector<cv::Point2i> &_waypoints);
};

trajectory_msgs::MultiDOFJointTrajectoryPoint
createTrajectoryFromPointMsg(const cv::Point2f &_point, const float _height,
                             const float _yaw);
geometry_msgs::TwistStamped
createSpeedReferenceMsg(cv::Point2d _current_position,
                        cv::Point2d _target_position, float _speed);
geometry_msgs::PoseStamped createPoseStampedMsg(const cv::Point2f &_point,
                                                const float _height,
                                                const float _yaw);

void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name);

#endif // PATH_PLANNER_HPP_
