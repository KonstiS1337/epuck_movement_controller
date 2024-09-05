#include "epuck_movement_controller/epuck_movement_controller.hpp"

EpuckMovementController::EpuckMovementController() : rclcpp::Node("epuck_movement_controller_node"){
    this->declare_parameter<std::string>("epuck_name","epuck"); // set the correct name of epuck here
    std::string epuck_name = this->get_parameter("epuck_name").as_string();
    action_server_ = rclcpp_action::create_server<epuck_driver_interfaces::action::SimpleMovement>(
                        this,
                        epuck_name + "/movement_goal",
                        std::bind(&EpuckMovementController::handleGoal,this,std::placeholders::_1,std::placeholders::_2),
                        std::bind(&EpuckMovementController::handleCancel,this,std::placeholders::_1),
                        std::bind(&EpuckMovementController::handleAccepted,this,std::placeholders::_1),
                        rcl_action_server_get_default_options());
    
    update_rate_=std::make_shared<rclcpp::Rate>(10.0);


    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(epuck_name + "/odom",1,std::bind(&EpuckMovementController::odomCB,this,std::placeholders::_1));
    tof_sub_ = this->create_subscription<std_msgs::msg::Int16>(epuck_name + "/tof",1,std::bind(&EpuckMovementController::tofCB,this,std::placeholders::_1));

    robot_control_srv_ = this->create_client<epuck_driver_interfaces::srv::ChangeRobotState>(epuck_name + "/robot_control");
    
    return;
}

EpuckMovementController::~EpuckMovementController()  {
    RCLCPP_INFO(this->get_logger(),"Destroying node.");
    return;
}

void EpuckMovementController::odomCB(const std::shared_ptr<const nav_msgs::msg::Odometry> data) {
    current_pose_ = data->pose.pose;
    //RCLCPP_INFO(this->get_logger(),"Odom data updated %f with x %f and y %f",data->header.stamp.nanosec,current_pose_.position.x,current_pose_.position.y);
    return;
}

void EpuckMovementController::tofCB(const std::shared_ptr<const std_msgs::msg::Int16> data) {
    current_tof_ = data->data;
    return;
}


rclcpp_action::GoalResponse EpuckMovementController::handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const epuck_driver_interfaces::action::SimpleMovement::Goal> goal) {
    if(goal_running_) {
        RCLCPP_INFO(this->get_logger(),"New goal rejected because there is a current goal active.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EpuckMovementController::handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle) {
    RCLCPP_INFO(this->get_logger(),"Received cancel call.");
    goal_running_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void EpuckMovementController::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle){
    goal_running_ = true;
    if(goal_handle->get_goal()->tof_approach) std::thread{std::bind(&EpuckMovementController::executeTofApproach, this, std::placeholders::_1), goal_handle}.detach();
    else std::thread{std::bind(&EpuckMovementController::executeMovement, this, std::placeholders::_1), goal_handle}.detach();
    return;
}

void EpuckMovementController::executeMovement(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle) {
    RCLCPP_INFO(this->get_logger(),"Executing drive goal.");
    auto goal = goal_handle->get_goal();
    auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    request_left->module = request_left->MODULE_LEFT_MOTOR;
    request_right->module = request_right->MODULE_RIGHT_MOTOR;
    //adjusting angle
    if(goal->angle) { //requested angle is != 0
        tf2::Quaternion turn_quat,current_quat,final_quat;
        turn_quat.setRPY(0,0,goal->angle); //goal->angle
        RCLCPP_INFO(this->get_logger(),"turn w is %f with z %f",turn_quat.getW(),turn_quat.getZ());
        current_quat = msgToQuat(current_pose_);
        final_quat = turn_quat * current_quat;
        

        request_right->value = goal->angle > 0 ? TURN_SPEED : - TURN_SPEED;
        request_left->value = goal->angle < 0 ? TURN_SPEED : - TURN_SPEED;
        robot_control_srv_->async_send_request(request_left);
        robot_control_srv_->async_send_request(request_right);

        RCLCPP_INFO(this->get_logger(),"Final quat with w %f, x %f, y %f, z %f",final_quat.getW(),final_quat.getX(),final_quat.getY(),final_quat.getZ());
        while(!goal_handle->is_canceling() && rclcpp::ok()) {
            current_quat = msgToQuat(current_pose_);
            tf2::Quaternion error = current_quat.inverse() * final_quat;

            RCLCPP_INFO(this->get_logger(),"Error is %f",abs(1 - abs(error.getW())));
            if(abs(1 - abs(error.getW())) < ANGLE_TOLERANCE) {
                request_left->value  = 0;
                request_right->value  = 0;
                robot_control_srv_->async_send_request(request_left);
                robot_control_srv_->async_send_request(request_right);
                break;
            }
            update_rate_->sleep(); //sleeping one second for the robot to stop
        }
    } 
    RCLCPP_INFO(this->get_logger(),"Passed angle.");
    if(goal->distance > 0 && !goal_handle -> is_canceling()) {
        // driving straight
        geometry_msgs::msg::Pose starting_pose = current_pose_;

        request_left->value = DRIVE_SPEED;
        request_right->value = DRIVE_SPEED;
        robot_control_srv_->async_send_request(request_left);
        robot_control_srv_->async_send_request(request_right);
        RCLCPP_INFO(this->get_logger(),"Sending speed.");

        while(!goal_handle->is_canceling() && rclcpp::ok()) {
            float sum1 = pow(current_pose_.position.x - starting_pose.position.x,2);
            float sum2 = pow(current_pose_.position.y - starting_pose.position.y,2);
            float value = sqrt(sum1 + sum2);
            //RCLCPP_INFO(this->get_logger(),"Calculated value %f.",value);
            //RCLCPP_INFO(this->get_logger(),"Current pose: %f , %f and starting: %f , %f",current_pose_.position.x,current_pose_.position.y,starting_pose.position.x,starting_pose.position.y);
            if( value >= goal->distance) {
                RCLCPP_INFO(this->get_logger(),"Stopping robot.");
                request_left->value  = 0;
                request_right->value  = 0;
                robot_control_srv_->async_send_request(request_left);
                robot_control_srv_->async_send_request(request_right);
                break;
            }
            update_rate_->sleep(); //sleeping one second for the robot to stop
        }
    } 
    //stopping the robot
    request_left->value  = 0;
    request_right->value  = 0;
    robot_control_srv_->async_send_request(request_left);
    robot_control_srv_->async_send_request(request_right);

    auto res = std::make_shared<epuck_driver_interfaces::action::SimpleMovement::Result>();
    RCLCPP_INFO(this->get_logger(),"Stopped robot - creating result");
    if(goal_handle->is_canceling()) {
        res->success = false;
        goal_handle->canceled(res);
    }
    else {
        res->success = true;
        res->distance_driven = 0.0;
        goal_handle->succeed(res);
        RCLCPP_INFO(this->get_logger(),"Set goal handle to true");
    }
    goal_running_ = false;

    RCLCPP_INFO(this->get_logger(),"Exit");
    return;
}

void EpuckMovementController::executeTofApproach(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle) {
    RCLCPP_INFO(this->get_logger(),"Executing tof goal.");
    auto goal = goal_handle->get_goal();
    auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    request_left->module = request_left->MODULE_LEFT_MOTOR;
    request_right->module = request_right->MODULE_RIGHT_MOTOR;
    request_left->value = DRIVE_SPEED;
    request_right->value = DRIVE_SPEED;
    robot_control_srv_->async_send_request(request_left);
    robot_control_srv_->async_send_request(request_right);
    bool slow_mode_active = false;

    while(!goal_handle->is_canceling() && rclcpp::ok()) {
        if(current_tof_ - TOF_LAG_DISTANCE <= goal->distance * 1000) { //converting m to mm
            RCLCPP_INFO(this->get_logger(),"Reached limit with %i",current_tof_);
            request_left->value = 0;
            request_right->value = 0;
            robot_control_srv_->async_send_request(request_left);
            robot_control_srv_->async_send_request(request_right);
            break;
        }
        if(!slow_mode_active && current_tof_ - TOF_LAG_DISTANCE <= goal->distance * 1000 + 30) {
            RCLCPP_INFO(this->get_logger(),"Entering slow movement");
            request_left->value = DRIVE_SPEED*0.5;
            request_right->value = DRIVE_SPEED*0.5;
            robot_control_srv_->async_send_request(request_left);
            robot_control_srv_->async_send_request(request_right);
            slow_mode_active = true;
        }
        update_rate_->sleep();
    } 

    //stopping the robot
    request_left->value  = 0;
    request_right->value  = 0;
    robot_control_srv_->async_send_request(request_left);
    robot_control_srv_->async_send_request(request_right);

    auto res = std::make_shared<epuck_driver_interfaces::action::SimpleMovement::Result>();

    if(goal_handle->is_canceling()) {
        res->success = false;
        goal_handle->canceled(res);
    }
    else {
        res->success = true;
        goal_handle->succeed(res);
    }
    goal_running_ = false;
    return;
}

tf2::Quaternion EpuckMovementController::msgToQuat(geometry_msgs::msg::Pose const &pose) {
    tf2::Quaternion out;
    out.setW(pose.orientation.w);
    out.setX(pose.orientation.x);
    out.setY(pose.orientation.y);
    out.setZ(pose.orientation.z);
    return out;
}

int main(int argc,char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<EpuckMovementController> node = std::make_shared<EpuckMovementController>();
    RCLCPP_INFO(node->get_logger(),"Start spinning of node.");
	rclcpp::spin(node);
    
   return 0;
}

