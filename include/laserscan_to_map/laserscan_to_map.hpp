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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
// #include <opencv2/imgproc/imgproc.hpp>

#define LASERSCAN_TOPIC "scan"
#define DRONEPOSITION_TOPIC "position"
#define REF_FRAME "world"
#define IMAGE_PUB_TOPIC "occupancy_map"
#define IMG_H 260
#define IMG_W 160
#define Z_MIN_TH 1.0
#define OCC_MAX_DIST_PX 10
#define OCC_GRID_SIZE 5

class LaserscanToMap
{
public:
  LaserscanToMap();
  ~LaserscanToMap();
  void run();

  ros::NodeHandle nh_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber droneposition_sub_;
  std::vector<float> laser_mesuraments;
  std::string ref_frame_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;

  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection laser_projector_;

  bool laser_update_ = false;
  bool send_occ_map_ = false;

  void laserscanCallback(const sensor_msgs::LaserScan &_msg);
  void positionCallback(const geometry_msgs::PointStamped &_msg);
  // void generateMap();
  int img_h_;
  int img_w_;
  cv::Size grid_size_;
  cv::Mat laser_map_;
  cv::Mat occupancy_map_;
  cv::Point2f drone_position_;

  void generateOccupancyMap();
  cv::Mat generateShowImg(const cv::Mat &_img, const cv::Point2i &_drone_px);
  void showMap(const cv::Mat &_map, const std::string &_map_name, const bool _add_drone);

  void sendMap(cv::Mat _map);
};

cv::Point2i coord2img(const float _x, const float _y, const int _img_h, const int _img_w);

#endif // LASERSCAN_TO_MAP_HPP_
