/*!********************************************************************************
 * \brief     ICUAS Path Planner Implementation
 * \authors   CVAR-UPM
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "icuas_path_planner.hpp"

namespace icuas_path_planning {

PathPlanner::PathPlanner() {
  // ROS Service getGraph
  goNodeService = nh_.advertiseService("go_node", &PathPlanner::goNode, this);

  // ROS Publisher
  // Creates has ended publisher
  has_ended_pub_ = nh_.advertise<std_msgs::Bool>("path_planning/has_ended", 1);

  // Creates controller publisher
  motion_reference_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("motion_reference/pose", 1);

  // Creates odometry subscriber
  odom_sub_      = nh_.subscribe("odometry", 1, &PathPlanner::odomCallback, this);
  odom_received_ = false;
}

PathPlanner::~PathPlanner() {}

bool PathPlanner::goNode(icuas_msgs::GoNode::Request &req, icuas_msgs::GoNode::Response &res) {
  // TODO(Miguel): implement this

  res.success = true;
  geometry_msgs::PoseStamped waypoint;
  switch (req.node_id) {
    case 0:
      waypoint =
          generatePoseStamped(initial_position_.x, initial_position_.y, initial_position_.z, M_PI);
      break;
    case 1:
      waypoint = generatePoseStamped(1.0, 5.0, 2.0, M_PI_2);
      break;
    case 2:
      waypoint = generatePoseStamped(10.0, 6.0, 5.0, -M_PI_2);
      break;
    case 3:
      waypoint = generatePoseStamped(10.0, 10.0, 3.0, 0.0);
      break;
    default:
      res.success = false;
      break;
  }

  path_.clear();
  path_.push_back(waypoint);
  motion_reference_pose_ = path_.front();

  ROS_INFO("Go Node callback - Going to node %d", req.node_id);

  std_msgs::Bool has_ended_msg;
  has_ended_msg.data = false;
  has_ended_pub_.publish(has_ended_msg);
  ROS_INFO("Go Node callback - has_ended published");

  return true;
}

void PathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose_ = msg->pose.pose;

  if (!odom_received_) {
    initial_position_ = current_pose_.position;
  }

  odom_received_ = true;
}

void PathPlanner::run() {
  if (!odom_received_) {
    return;
  }

  if (checkFinished()) {
    return;
  }
  motion_reference_pose_pub_.publish(path_.front());
}

bool PathPlanner::checkFinished() {
  // Check if the next point is reached
  if (odom_received_) {
    if (path_.size() == 0) {
      odom_received_ = false;
      std_msgs::Bool has_ended_msg;
      has_ended_msg.data = true;
      has_ended_pub_.publish(has_ended_msg);
      return true;
    }

    // 3D euclidean distance
    auto distance = sqrt(pow(path_.front().pose.position.x - current_pose_.position.x, 2) +
                         pow(path_.front().pose.position.y - current_pose_.position.y, 2) +
                         pow(path_.front().pose.position.z - current_pose_.position.z, 2));

    if (distance < GO_TO_THRESHOLD) {
      ROS_INFO("Point reached");
      path_.pop_front();
    }
  }
  return false;
}

geometry_msgs::PoseStamped generatePoseStamped(double x, double y, double z, double yaw) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp    = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf::Quaternion quaternion;
  double roll  = 0.0;
  double pitch = 0.0;
  quaternion.setRPY(roll, pitch, yaw);
  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();
  return pose;
}

}  // namespace icuas_path_planning
