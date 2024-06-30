/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Théo Combelles <theo.combelles@gmail.com>
 *
 */

#ifndef NAV2_LYAPUNOV_STABLE_CONTROLLER__LYAPUNOV_STABLE_CONTROLLER_HPP_
#define NAV2_LYAPUNOV_STABLE_CONTROLLER__LYAPUNOV_STABLE_CONTROLLER_HPP_

#include <cmath>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "utils.hpp"

using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace nav2_lyapunov_stable_controller {

struct CmdVelocity {
    double linear_vel;
    double angular_vel;
};

class LyapunovStableController : public nav2_core::Controller {
   public:
    LyapunovStableController() = default;
    ~LyapunovStableController() override = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                   const std::shared_ptr<tf2_ros::Buffer> tf,
                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;

    void cleanup() override;
    void setPlan(const nav_msgs::msg::Path& path) override;

    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                             const geometry_msgs::msg::Twist& velocity,
                                                             nav2_core::GoalChecker* goal_checker) override;

   protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);

    CmdVelocity computeVelocity(const geometry_msgs::msg::Pose& goal_pose);

    geometry_msgs::msg::TwistStamped generateTwistMsg(const geometry_msgs::msg::PoseStamped& pose,
                                                      const CmdVelocity& cmd);

    void checkCollision(const geometry_msgs::msg::PoseStamped& pose, const CmdVelocity& cmd);

    geometry_msgs::msg::Pose selectGoal(const nav_msgs::msg::Path& transformed_plan);

    /**
     * @brief Whether collision is imminent
     * @param robot_pose Pose of robot
     * @param carrot_pose Pose of carrot
     * @param linear_vel linear velocity to forward project
     * @param angular_vel angular velocity to forward project
     * @param carrot_dist Distance to the carrot for PP
     * @return Whether collision is imminent
     */
    bool isCollisionImminent(const geometry_msgs::msg::PoseStamped&, const double&, const double&,
                             const double&);

    /**
     * @brief checks for collision at projected pose
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @param theta orientation of Yaw
     * @return Whether in collision
     */
    bool inCollision(const double& x, const double& y, const double& theta);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("LyapunovStableController")};
    rclcpp::Clock::SharedPtr clock_;

    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;
    double max_angular_drift_;
    rclcpp::Duration transform_tolerance_{0, 0};
    bool use_collision_detection_;
    double max_allowed_time_to_collision_;

    nav_msgs::msg::Path global_plan_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>
        collision_checker_;
};
}  // namespace nav2_lyapunov_stable_controller

#endif  // NAV2_LYAPUNOV_STABLE_CONTROLLER__LYAPUNOV_STABLE_CONTROLLER_HPP_