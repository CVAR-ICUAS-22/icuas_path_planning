/*!********************************************************************************
 * \brief     ICUAS Path Planner
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

#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <deque>

#include <geometry_msgs/PoseStamped.h>
#include <icuas_msgs/goNode.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#define GO_TO_THRESHOLD 0.5

namespace icuas_path_planning {

class PathPlanner {
private:
  // ROS
  ros::NodeHandle nh_;

  // ROS Service goNode
  ros::ServiceServer goNodeService;

  // ROS Publisher
  ros::Publisher motion_reference_pose_pub_;
  ros::Publisher has_ended_pub_;

  // ROS Subscribers
  ros::Subscriber odom_sub_;
  bool odom_received_ = false;

  // State
  geometry_msgs::Point initial_position_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::PoseStamped motion_reference_pose_;
  std::deque<geometry_msgs::PoseStamped> path_;

public:
  PathPlanner();
  ~PathPlanner();

  void run();

private:
  bool goNode(icuas_msgs::goNode::Request &req, icuas_msgs::goNode::Response &res);

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  bool checkFinished();
};

geometry_msgs::PoseStamped generatePoseStamped(double x, double y, double z, double yaw);

}  // namespace icuas_path_planning

#endif  // PATH_PLANNER_HPP_
