#ifndef LASERSCAN_TO_MAP_HPP_
#define LASERSCAN_TO_MAP_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#define LASERSCAN_TOPIC "scan"
#define DRONEPOSITION_TOPIC "position"
#define REF_FRAME "world"
#define IMG_H 260
#define IMG_W 160
#define Z_MIN_TH 1.0

class LaserscanToMap
{
public:
  LaserscanToMap();
  ~LaserscanToMap();

  ros::NodeHandle nh_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber droneposition_sub_;
  std::vector<float> laser_mesuraments;
  std::string ref_frame_;

  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection laser_projector_;

  void laserscanCallback(const sensor_msgs::LaserScan &_msg);
  void positionCallback(const geometry_msgs::PointStamped &_msg);
  // void generateMap();
  int img_h_;
  int img_w_;
  cv::Mat map_;

  void showMap(const cv::Mat &_img);
  cv::Point2f drone_position_;
};

cv::Point2i coord2img(const float _x, const float _y, const int _img_h, const int _img_w);

#endif // LASERSCAN_TO_MAP_HPP_
