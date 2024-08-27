#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

#include "epuck_driver_interfaces/action/simple_movement.hpp"
#include "epuck_driver_interfaces/srv/change_robot_state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define TURN_SPEED 250 // steps per min
#define DRIVE_SPEED 500 // steps per min
#define ANGLE_TOLERANCE 0.001 // difference to goal angle in rad
#define DISTANCE_TOLERANCE 0.005 // difference to goal distance in cm

class EpuckMovementController : public rclcpp::Node {
    public:
        EpuckMovementController();
        ~EpuckMovementController();
    private:
        bool goal_running_ = false;

        geometry_msgs::msg::Pose current_pose_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Client<epuck_driver_interfaces::srv::ChangeRobotState>::SharedPtr robot_control_srv_;
        std::shared_ptr<rclcpp::Rate> update_rate_;

        rclcpp_action::Server<epuck_driver_interfaces::action::SimpleMovement>::SharedPtr action_server_;
        
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const epuck_driver_interfaces::action::SimpleMovement::Goal> goal);
        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void execute_movement(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void odomCB(const std::shared_ptr<const nav_msgs::msg::Odometry> data);

        //helper
        tf2::Quaternion msgToQuat(geometry_msgs::msg::Pose const &pose);
};
