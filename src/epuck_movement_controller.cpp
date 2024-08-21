#include "epuck_movement_controller/epuck_movement_controller.hpp"

EpuckMovementController::EpuckMovementController() : rclcpp::Node("epuck_movement_controller_node"){
    this->declare_parameter<std::string>("epuck_name","epuck"); // set the correct name of epuck here
    action_server_ = rclcpp_action::create_server<epuck_driver_interfaces::action::SimpleMovement>(
                        this,
                        "~/movement_goal",
                        std::bind(&EpuckMovementController::handleGoal,this,std::placeholders::_1,std::placeholders::_2),
                        std::bind(&EpuckMovementController::handleCancel,this,std::placeholders::_1),
                        std::bind(&EpuckMovementController::handleAccepted,this,std::placeholders::_1),
                        rcl_action_server_get_default_options());
    
    std::string epuck_name = this->get_parameter("epuck_name").as_string();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(epuck_name + "/odom",1,std::bind(&EpuckMovementController::odomCB,this,std::placeholders::_1));
    robot_control_srv_ = this->create_client<epuck_driver_interfaces::srv::ChangeRobotState>(epuck_name + "/robot_control");
    return;
}

EpuckMovementController::~EpuckMovementController()  {
    RCLCPP_INFO(this->get_logger(),"Destroying node.");
    return;
}

void EpuckMovementController::odomCB(nav_msgs::msg::Odometry::ConstSharedPtr const &data) {
    current_pose_ = data->pose.pose;
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
    return rclcpp_action::CancelResponse::ACCEPT;
}

void EpuckMovementController::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle){
    goal_running_ = true;
    std::thread{std::bind(&EpuckMovementController::execute_movement, this, std::placeholders::_1), goal_handle}.detach();
    return;
}

void EpuckMovementController::execute_movement(const std::shared_ptr<rclcpp_action::ServerGoalHandle<epuck_driver_interfaces::action::SimpleMovement>>  goal_handle) {
    auto goal = goal_handle->get_goal();
    auto request_left = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    auto request_right = std::make_shared<epuck_driver_interfaces::srv::ChangeRobotState::Request>();
    //adjusting angle
    if(goal->angle) { //requested angle is != 0
        tf2::Quaternion turn_quat,current_quat,final_quat;
        turn_quat.setRPY(0,0,goal->angle);
        current_quat = msgToQuat(current_pose_);
        
        final_quat = turn_quat * current_quat;

        request_left->module = request_left->MODULE_LEFT_MOTOR;
        request_right->module = request_right->MODULE_RIGHT_MOTOR;
        request_left->value = goal->angle > 0 ? TURN_SPEED : - TURN_SPEED;
        request_right->value = goal->angle < 0 ? TURN_SPEED : - TURN_SPEED;
        robot_control_srv_->async_send_request(request_left);
        robot_control_srv_->async_send_request(request_right);

        while(!goal_handle->is_canceling() && rclcpp::ok()) {
            tf2::Quaternion error = final_quat * current_quat.inverse();
            if(abs(1 - error.getW()) < ANGLE_TOLERANCE) {
                request_left->value  = 0;
                request_right->value  = 0;
                robot_control_srv_->async_send_request(request_left);
                robot_control_srv_->async_send_request(request_right);
                break;
            }
        }
        sleep(1); //sleeping one second for the robot to stop
    } 
    if(goal->distance > 0 && !goal_handle -> is_canceling()) {
        // driving straight
        geometry_msgs::msg::Pose starting_pose = current_pose_;

        request_left->value = DRIVE_SPEED;
        request_right->value = DRIVE_SPEED;
        robot_control_srv_->async_send_request(request_left);
        robot_control_srv_->async_send_request(request_right);

        while(!goal_handle->is_canceling() && rclcpp::ok()) {
            if(sqrt(pow(current_pose_.position.x - starting_pose.position.x,2) + pow(current_pose_.position.y - starting_pose.position.y,2)) < DISTANCE_TOLERANCE) {
                request_left->value  = 0;
                request_right->value  = 0;
                robot_control_srv_->async_send_request(request_left);
                robot_control_srv_->async_send_request(request_right);
            }
        }
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
