#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <opencv2/opencv.hpp>
#include <string>

#include "A_star_algorithm.hpp"
#include "path_planner/setGoalPoint.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#define OCCUPANCY_IMAGE_TOPIC "/occupancy_image"
#define PROJECTED_MAP_TOPIC "/projected_map"
#define LASERSCAN_TOPIC "scan"
#define DRONEPOSITION_TOPIC "position"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define SPEEDCONTROL_TOPIC "motion_reference/speed"
#define POSE_TOPIC "motion_reference/pose"
#define IMAGE_PUB_TOPIC "occupancy_map"
#define CONTROLNODE_SRV "path_planning/run"
#define SETGOAL_SRV "path_planning/set_goal"
#define SETGOAL_TOPIC "path_planning/set_goal"

#define LASER2BIN_TH 100 // 100 // [0-255]
#define DIST2BIN_TH 0.7  // [0.0-1.0]

class PathPlanner {
public:
  PathPlanner();
  ~PathPlanner();
  void run();
  void start();
  void stop();
  void setGoal();

  ros::NodeHandle nh_;
  ros::Subscriber occupancy_image_sub_;
  ros::Subscriber projected_map_sub_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber droneposition_sub_;
  ros::Publisher waypoint_pub_;

  ros::ServiceServer control_node_srv;
  // ros::ServiceServer set_goal_srv;
  // ros::Subscriber set_goal_sub_;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;

  tf::TransformListener tf_listener_;

  laser_geometry::LaserProjection laser_projector_;
  AStarPlanner planner_algorithm_;

  void occupancyImageCallback(const sensor_msgs::Image &_msg);
  void laserscanCallback(const sensor_msgs::LaserScan &_msg);
  void positionCallback(const geometry_msgs::PoseStamped &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request,
                      std_srvs::SetBool::Response &_response);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request,
                  path_planner::setGoalPoint::Response &_response);
  void setGoalCallback(const geometry_msgs::PoseStamped &_msg);

  bool laser_update_ = false;
  bool new_occupancy_map_ = false;
  bool generate_path_ = false;
  bool run_node_ = false;
  bool goal_set_ = false;
  bool force_generation_ = false;
  bool check_future_point_ = false;
  bool ending_maze_ = false;
  bool no_solution_ = false;

  int img_h_;
  int img_w_;
  float img_resolution_;
  float occ_grid_size_;
  float z_min_th_;
  std::string ref_frame_;
  cv::Size grid_size_;

  cv::Mat laser_map_;
  cv::Mat occupancy_map_;

  std::vector<float> laser_mesuraments_;
  std::vector<cv::Point2i> current_path_;
  std::vector<cv::Point2i> ref_waypoints_;

  cv::Point2f drone_position_;
  cv::Point2f goal_position_;
  float drone_yaw_;
  float fly_height_;
  float max_distance_th_;
  float next_point_reached_dist_;
  float x_safe_zone_;

  cv::Point2i drone_cell_;
  cv::Point2i goal_cell_;

  // Begin of SPEED_CONTROLLER
  ros::Publisher speed_control_pub_;
  ros::Publisher pose_pub_;
  bool speed_controller_;
  float max_control_speed_;
  // End of SPEED_CONTROLLER

  void generateOccupancyMap();
  void checkCurrentPath();
  void generateNewPath();
  void optimizePath();
  void sendWaypoint(const cv::Point2f _next_point, const float _sending_yaw);

  void sendMap(cv::Mat _map);
  void showMap(const cv::Mat &_map, const std::string &_map_name,
               const bool _add_drone);

  cv::Mat generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px);
  cv::Point2i coord2img(const float _x, const float _y, const int _img_h,
                        const int _img_w);
  cv::Point2i coord2grid(const float _x, const float _y, const int _img_h,
                         const int _img_w);
  cv::Point2f grid2coord(const cv::Point2i &_point, const int _img_h,
                         const int _img_w);
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

cv::Mat generatePathImg(const cv::Mat &_map, const cv::Point2i &_drone_px,
                        const std::vector<cv::Point2i> &_path,
                        const std::vector<cv::Point2i> &_waypoints);
void showCombinedMap(std::vector<cv::Mat> maps, std::string window_name);

#endif // PATH_PLANNER_HPP_
