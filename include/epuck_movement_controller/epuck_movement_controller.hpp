#pragma once

#include <memory>

#include "epuck_driver_interfaces/action/simple_movement.hpp"
#include "epuck_driver_interfaces/srv/change_robot_state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int16.hpp>

#include "epuck_movement_controller/pid.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32.hpp>

#define TURN_SPEED 250 // steps per min
#define DRIVE_SPEED 500 // steps per min
#define ANGLE_TOLERANCE 0.001 // difference to goal angle in rad
#define DISTANCE_TOLERANCE 0.005 // difference to goal distance in cm
#define TOF_LAG_DISTANCE 0 // sensor tolerance due to input lag in mm
#define TOF_APPROACH_TOLERANCE 05 // tolerance for tof approach that is okay in mm
#define TOF_WINDOW 7
#define PID_GOAL_TOLERANCE 5

class EpuckMovementController : public rclcpp::Node {
    public:
        EpuckMovementController();
        ~EpuckMovementController();
    private:
        bool goal_running_ = false;
        float pid_output_,goal_delta_;
        std::vector<int> tof_accum_;
        std::shared_ptr<PIDController<float>> pid_;

        geometry_msgs::msg::Pose current_pose_;
        int current_tof_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr tof_sub_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr clean_tof_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_output_pub_;
        rclcpp::Client<epuck_driver_interfaces::srv::ChangeRobotState>::SharedPtr robot_control_srv_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pid_p_srv_,pid_i_srv_,pid_d_srv_;
        std::shared_ptr<rclcpp::Rate> update_rate_;


        rclcpp_action::Server<epuck_driver_interfaces::action::SimpleMovement>::SharedPtr action_server_;
        
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const epuck_driver_interfaces::action::SimpleMovement::Goal> goal);
        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void executeMovement(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void executeTofApproach(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle);
        void odomCB(const std::shared_ptr<const nav_msgs::msg::Odometry> data);
        void tofCB(const std::shared_ptr<const std_msgs::msg::Int16> data);

        //helper
        tf2::Quaternion msgToQuat(geometry_msgs::msg::Pose const &pose);
        unsigned long pidTimeFunction();
        void srvCBp(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
        void srvCBi(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
        void srvCBd(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
};
