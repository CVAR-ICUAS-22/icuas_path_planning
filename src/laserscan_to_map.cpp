#include "laserscan_to_map.hpp"

LaserscanToMap::LaserscanToMap()
{
  laserscan_sub_ = nh_.subscribe(LASERSCAN_TOPIC, 1, &LaserscanToMap::laserscanCallback, this);
  droneposition_sub_ = nh_.subscribe(DRONEPOSITION_TOPIC, 1, &LaserscanToMap::positionCallback, this);

  ref_frame_ = REF_FRAME;
  img_h_ = IMG_H;
  img_w_ = IMG_W;
  map_ = cv::Mat(img_h_, img_w_, CV_8UC1, cv::Scalar(0));
}

LaserscanToMap::~LaserscanToMap() {}

void LaserscanToMap::laserscanCallback(const sensor_msgs::LaserScan &_msg)
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
      map_.at<uchar>(img_point.x, img_point.y) = 255;
    }
  }

  showMap(map_);
}

void LaserscanToMap::showMap(const cv::Mat &_img)
{
  cv::Mat color_map, img_map;
  cv::cvtColor(_img, color_map, cv::COLOR_GRAY2BGR);
  // _img.copyTo(map_img);
  cv::Point2i img_drone_position;
  img_drone_position = coord2img(drone_position_.x, drone_position_.y, img_h_, img_w_);
  cv::circle(color_map, cv::Point(img_drone_position.y, img_drone_position.x), 3, cv::Scalar(255, 0, 0), cv::FILLED);
  cv::resize(color_map, img_map, cv::Size(400, 600));

  cv::imshow("ICUAS challenge map", img_map);
  cv::waitKey(3);
}

void LaserscanToMap::positionCallback(const geometry_msgs::PointStamped &_msg)
{
  drone_position_.x = _msg.point.x;
  drone_position_.y = _msg.point.y;
}

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
