#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>

#include "A_star_algorithm.hpp"
#include "path_planner/setGoalPoint.h"

#define DRONEPOSITION_TOPIC "pose"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define POSE_TOPIC "motion_reference/pose"
#define OCCUPANCY_IMAGE_TOPIC "/occupancy_image"
#define IMAGE_PUB_TOPIC "occupancy_map"
#define PATHPLANNER_HAS_ENDED_TOPIC "path_planning/has_ended"
#define SPEEDCONTROL_TOPIC "motion_reference/speed"

#define CONTROLNODE_SRV "path_planning/run"
#define SETGOAL_SRV "path_planning/set_goal"
#define RESET_OCTOMAP_SRV "/octomap_server/reset"

#define OCC2BIN_TH 100 // 100 // [0-255]
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
  ros::Subscriber droneposition_sub_;

  ros::Publisher waypoint_pub_;
  ros::Publisher has_ended_pub_;

  ros::ServiceServer control_node_srv;
  ros::ServiceServer set_goal_srv;
  ros::ServiceClient reset_octomap_;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;

  AStarPlanner planner_algorithm_;

  void occupancyImageCallback(const sensor_msgs::Image &_msg);
  void positionCallback(const geometry_msgs::PoseStamped &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request,
                      std_srvs::SetBool::Response &_response);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request,
                  path_planner::setGoalPoint::Response &_response);
  void endNavigation();

  bool new_occupancy_map_ = false;
  bool generate_path_ = false;
  bool run_node_ = false;
  bool check_future_point_ = true;
  bool no_solution_ = false;
  bool goal_reached_ = false;
  bool occ_map_received_ = false;
  bool optimize_path_ = false;

  int occmap_h_;
  int occmap_w_;
  float occmap_resolution_;
  float drone_yaw_;
  float fly_height_;
  float next_point_reached_dist_;
  float goal_reached_dist_ = 1.0;

  cv::Point2f drone_position_;
  cv::Point2f goal_position_;
  cv::Point2i drone_cell_;
  cv::Point2i goal_cell_;

  std::vector<cv::Point2i> current_path_;
  std::vector<cv::Point2i> ref_waypoints_;

  cv::Mat occupancy_map_;

  // Begin of SPEED_CONTROLLER
  ros::Publisher speed_control_pub_;
  ros::Publisher pose_pub_;
  bool speed_controller_;
  float max_control_speed_;
  // End of SPEED_CONTROLLER

  void checkCurrentPath();
  void generateNewPath();
  void optimizePath();
  void sendWaypoint(const cv::Point2f _next_point, const float _sending_yaw);

  void sendMap(cv::Mat _map);
  void showMap(const cv::Mat &_map, const std::string &_map_name,
               const bool _add_drone);

  cv::Mat generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px);
  cv::Point2i coord2map(const cv::Point2f &_point); 
  cv::Point2f map2coord(const cv::Point2i &_point);
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
