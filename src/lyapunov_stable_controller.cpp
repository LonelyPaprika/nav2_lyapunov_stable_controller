// Copyright (c) 2024 LonelyPaprika
// Author (s) Théo Combelles <theo.combelles@gmail.com>
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_lyapunov_stable_controller/lyapunov_stable_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_lyapunov_stable_controller {

void LyapunovStableController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                         std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                                         const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent;
    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel",
                                      rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
                                      rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_drift", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_collision_detection",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_allowed_time_to_collision",
                                      rclcpp::ParameterValue(1.0));

    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_drift", max_angular_drift_);
    node->get_parameter(plugin_name_ + ".use_collision_detection", use_collision_detection_);
    node->get_parameter(plugin_name_ + ".max_allowed_time_to_collision", max_allowed_time_to_collision_);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    // initialize collision checker and set costmap
    collision_checker_ =
        std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
    collision_checker_->setCostmap(costmap_);
}

void LyapunovStableController::activate() {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type lyapunov_stable_controller::LyapunovStableController\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_activate();
}

void LyapunovStableController::deactivate() {
    RCLCPP_INFO(
        logger_,
        "Dectivating controller: %s of type lyapunov_stable_controller::LyapunovStableController\"  %s",
        plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_deactivate();
}

void LyapunovStableController::cleanup() {
    RCLCPP_INFO(logger_,
                "Cleaning up controller: %s of type lyapunov_stable_controller::LyapunovStableController",
                plugin_name_.c_str());
    global_pub_.reset();
}

void LyapunovStableController::setPlan(const nav_msgs::msg::Path& path) {
    global_pub_->publish(path);
    global_plan_ = path;
}

void LyapunovStableController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    (void)speed_limit;
    (void)percentage;
}

geometry_msgs::msg::TwistStamped LyapunovStableController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker) {
    (void)velocity;
    (void)goal_checker;

    auto goal_pose = selectGoal(transformGlobalPlan(pose));

    auto cmd = computeVelocity(goal_pose);

    checkCollision(pose, cmd);

    return generateTwistStampedMsg(pose.header.frame_id, cmd);
}

geometry_msgs::msg::Pose LyapunovStableController::selectGoal(const nav_msgs::msg::Path& transformed_plan) {
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(),
        [&](const auto& ps) { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_; });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
        goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    return goal_pose_it->pose;
}

geometry_msgs::msg::Twist LyapunovStableController::computeVelocity(
    const geometry_msgs::msg::Pose& goal_pose) {
    auto cmd = geometry_msgs::msg::Twist();

    auto k_lin = 1.0;
    auto k_ang = 3.0;

    auto angle_error = std::atan2(goal_pose.position.y, goal_pose.position.x);

    if (goal_pose.position.x > 0) {
        if (std::abs(angle_error) < max_angular_drift_) {
            cmd.linear.x = desired_linear_vel_ * std::cos(angle_error);
            cmd.angular.z = -k_lin * goal_pose.position.y + k_ang * angle_error;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = max_angular_vel_ * sign(angle_error);
        }

    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = max_angular_vel_ * sign(angle_error);
    }

    return cmd;
}

void LyapunovStableController::checkCollision(const geometry_msgs::msg::PoseStamped& pose,
                                              const geometry_msgs::msg::Twist& cmd) {
    if (use_collision_detection_ && isCollisionImminent(pose, cmd.linear.x, cmd.angular.z, 1.0)) {
        throw nav2_core::PlannerException("LyapunovStableController detected collision ahead!");
    }
}

geometry_msgs::msg::TwistStamped LyapunovStableController::generateTwistStampedMsg(
    const std_msgs::msg::Header::_frame_id_type& frame_id, const geometry_msgs::msg::Twist& cmd) {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = cmd.linear.x;
    cmd_vel.twist.angular.z = max(-1.0 * abs(max_angular_vel_), min(cmd.angular.z, abs(max_angular_vel_)));
    return cmd_vel;
}

nav_msgs::msg::Path LyapunovStableController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped& pose) {
    // implementation taken from navigation2_tutorials/nav2_pure_pursuit_controller.

    if (global_plan_.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav_2d_utils::transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose,
                                     transform_tolerance_)) {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    double dist_threshold =
        std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        nav2_util::geometry_utils::min_by(global_plan_.poses.begin(), global_plan_.poses.end(),
                                          [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) {
                                              return euclidean_distance(robot_pose, ps);
                                          });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end =
        std::find_if(transformation_begin, end(global_plan_.poses), [&](const auto& global_plan_pose) {
            return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
        // We took a copy of the pose, let's lookup the transform at the current time
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        nav_2d_utils::transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose,
                                    transform_tolerance_);
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
                   transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool LyapunovStableController::isCollisionImminent(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                   const double& linear_vel, const double& angular_vel,
                                                   const double& carrot_dist) {
    // implementation taken from nav2_regulated_pure_pursuit_controller.
    // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
    // odom frame and the carrot_pose is in robot base frame.

    // check current point is OK
    if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y,
                    tf2::getYaw(robot_pose.pose.orientation))) {
        return true;
    }

    // visualization messages
    nav_msgs::msg::Path arc_pts_msg;
    arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
    arc_pts_msg.header.stamp = robot_pose.header.stamp;
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
    pose_msg.header.stamp = arc_pts_msg.header.stamp;

    double projection_time = 0.0;
    if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
        // rotating to heading at goal or toward path
        // Equation finds the angular distance required for the largest
        // part of the robot radius to move to another costmap cell:
        // theta_min = 2.0 * sin ((res/2) / r_max)
        // via isosceles triangle r_max-r_max-resolution,
        // dividing by angular_velocity gives us a timestep.
        double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
        projection_time = 2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
    } else {
        // Normal path tracking
        projection_time = costmap_->getResolution() / fabs(linear_vel);
    }

    const geometry_msgs::msg::Point& robot_xy = robot_pose.pose.position;
    geometry_msgs::msg::Pose2D curr_pose;
    curr_pose.x = robot_pose.pose.position.x;
    curr_pose.y = robot_pose.pose.position.y;
    curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

    // only forward simulate within time requested
    int i = 1;
    while (i * projection_time < max_allowed_time_to_collision_) {
        i++;

        // apply velocity at curr_pose over distance
        curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
        curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
        curr_pose.theta += projection_time * angular_vel;

        // check if past carrot pose, where no longer a thoughtfully valid command
        if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
            break;
        }

        // store it for visualization
        pose_msg.pose.position.x = curr_pose.x;
        pose_msg.pose.position.y = curr_pose.y;
        pose_msg.pose.position.z = 0.01;
        arc_pts_msg.poses.push_back(pose_msg);

        // check for collision at the projected pose
        if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
            return true;
        }
    }

    return false;
}

bool LyapunovStableController::inCollision(const double& x, const double& y, const double& theta) {
    // implementation taken from nav2_regulated_pure_pursuit_controller.

    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my)) {
        RCLCPP_WARN_THROTTLE(
            logger_, *(clock_), 30000,
            "The dimensions of the costmap is too small to successfully check for "
            "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
            "increase your costmap size.");
        return false;
    }

    double footprint_cost =
        collision_checker_->footprintCostAtPose(x, y, theta, costmap_ros_->getRobotFootprint());
    if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
        return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

}  // namespace nav2_lyapunov_stable_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_lyapunov_stable_controller::LyapunovStableController, nav2_core::Controller)
