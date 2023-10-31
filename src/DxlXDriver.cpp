


#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_driver/control_table.hpp>
#include <dynamixel_driver/DxlXDriver.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;


DxlXDriver::DxlXDriver(): 
    Node("dxl_x_driver_node"), 
    dxl_instruction_(Dxl_instruction::INSTRUCTION_INITIAL), 
    dxl_is_ready_(false),
    dxl_error_(0),
    goal_velocity_(VELOCITY_LIMIT-50)
{
    // In ros2 versions after Dashing, parameters defined in yaml have higher priority than default value specified here.
    this -> declare_parameter("dxl_id", 1);
    this -> declare_parameter("baudrate", 9600);
    this -> declare_parameter("device_file", "/dev/ttyUSB0");
    dxl_id_ = this -> get_parameter("dxl_id").as_int();
    baudrate_ = this -> get_parameter("baudrate").as_int();
    device_file_ = this -> get_parameter("device_file").as_string().c_str();

    // parameters defined in yaml are used here. dont worry.
    portHandler_ = dynamixel::PortHandler::getPortHandler(device_file_);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("dxl_present_velocity", 10);
    timer_ = this -> create_wall_timer(500ms, std::bind(&DxlXDriver::timer_callback, this));
    subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>(
        "dxl_cmd_vel", 10, std::bind(&DxlXDriver::subscription_callback, this, _1));

    stand_up();
}

DxlXDriver::~DxlXDriver() {
    step_zero_velocity();
    step_disactivate();
}


void DxlXDriver::timer_callback()
{
    /* check instruction */
    switch (dxl_instruction_) {
        case Dxl_instruction::INSTRUCTION_ACTIVATE:
            step_activate();
            // break;
        case Dxl_instruction::INSTRUCTION_RUN_VELOCITY:
            dxl_instruction_ = Dxl_instruction::INSTRUCTION_RUN_VELOCITY;
            step_run_velocity(goal_velocity_);
            break;
        case Dxl_instruction::INSTRUCTION_STOP:
            step_zero_velocity();
            break;
        case Dxl_instruction::INSTRUCTION_DISACTIVATE:
            step_disactivate();
            break;
        default:
            RCLCPP_ERROR(this -> get_logger(), "unknown instruction state");
            break;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = step_read_velocity();
    msg.linear.y = step_read_present_current();

    publisher_ -> publish(msg);
}

void DxlXDriver::subscription_callback(geometry_msgs::msg::Twist msg){
    /* set goal velocity */
    // goal_velocity_ = msg.linear.y;   // example
    goal_velocity_ = VELOCITY_LIMIT - 20;

    return;
}

void DxlXDriver::stand_up(){
    /* open port */
    if (portHandler_ -> openPort()){
        RCLCPP_INFO(this -> get_logger(), "succeed to open port");
    } else {
        RCLCPP_ERROR(this -> get_logger(), "fail to open port");
    }

    /* set baudrate */
    if (portHandler_ -> setBaudRate(baudrate_)) {
        RCLCPP_INFO(this -> get_logger(), "succeed to set baudrate");
    } else {
        RCLCPP_ERROR(this -> get_logger(), "failed to set baudrate");
    }

    /* set operating mode */
    int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
        portHandler_, dxl_id_, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error_);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error_ != 0) {
        RCLCPP_ERROR(this -> get_logger(), "failed to set operating mode");
    } else {
        RCLCPP_INFO(this -> get_logger(), "Operating mode: velocity control mode.");
    }

    /* set velocity limit */
    dxl_comm_result = packetHandler_ -> write4ByteTxRx(
        portHandler_, dxl_id_, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error_);
    if (dxl_comm_result != 0 || dxl_error_ != 0) {
        RCLCPP_ERROR(this -> get_logger(), "failed to set velocity limit");
    } else {
        RCLCPP_INFO(this -> get_logger(), "succeed to set velocity limit");
    }

    /* need further implementation */
    dxl_instruction_ = Dxl_instruction::INSTRUCTION_ACTIVATE;
}

bool DxlXDriver::step_activate(){
    if (dxl_is_ready_) {
        RCLCPP_INFO(this -> get_logger(), "dxl is already ready");
        return true;
    } else {
        /* enable torque */
        int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
            portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_);
        if (dxl_comm_result != 0 || dxl_error_ != 0){
            RCLCPP_ERROR(this -> get_logger(), "failed to enable dxl torque");
            dxl_is_ready_ = false;
            return false;
        } else {
            dxl_is_ready_ = true;
            RCLCPP_INFO(this -> get_logger(), "dxl: activate");
            return true;
        }   
    }
}

bool DxlXDriver::step_run_velocity(double goal_velocity){
    if (dxl_is_ready_) {
        /* set motor goal velocity */
        int dxl_comm_result = packetHandler_ -> write4ByteTxRx(
            portHandler_, dxl_id_, ADDR_GOAL_VELOCITY, goal_velocity, &dxl_error_);
        if (dxl_comm_result != 0 || dxl_error_ != 0){
            RCLCPP_ERROR(this -> get_logger(), "failed to set goal velocity");
            return false;
        }
        return true;
    } else {
        RCLCPP_ERROR(this -> get_logger(), "dxl is not ready");
        return false;
    }
}

bool DxlXDriver::step_zero_velocity(){
    if (dxl_is_ready_) {
        /* set motor velocity zero */
        int dxl_comm_result = packetHandler_ -> write4ByteTxRx(
            portHandler_, dxl_id_, ADDR_GOAL_VELOCITY, 0, &dxl_error_);
        if (dxl_comm_result != 0 || dxl_error_ != 0){
            RCLCPP_ERROR(this -> get_logger(), "failed to set zero velocity");
            return false;
        } else {
            RCLCPP_INFO(this -> get_logger(), "set zero velocity");
            return true;
        }
    } else {
        RCLCPP_ERROR(this -> get_logger(), "dxl is not ready");
        return false;
    }
}

double DxlXDriver::step_read_velocity(){
    /* read motor velocity */
    uint32_t dxl_present_velocity = 0;
    int dxl_comm_result = packetHandler_ -> read4ByteTxRx(
        portHandler_, dxl_id_, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error_); 
    if (dxl_comm_result != 0 || dxl_error_ != 0) {
        RCLCPP_ERROR(this -> get_logger(), "failed to read present velocity");
        return 0.;  // false
    } else {
        RCLCPP_INFO(this -> get_logger(), "present velocity: %u[rpm]", dxl_present_velocity);
        return (double)dxl_present_velocity;
    }
}

double DxlXDriver::step_read_present_current(){ // changed type 
    /* read present current */
    uint16_t dxl_present_current = 0;
    int dxl_comm_result = packetHandler_ -> read2ByteTxRx(
        portHandler_, dxl_id_, ADDR_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error_); 
    if (dxl_comm_result != 0 || dxl_error_ != 0) {
        RCLCPP_ERROR(this -> get_logger(), "failed to read present current");
        return 0.;  // false
    } else {
        RCLCPP_INFO(this -> get_logger(), "present current: %u[mA]", dxl_present_current);
        return (double)dxl_present_current;
    }
}

bool DxlXDriver::step_disactivate(){
    if (dxl_is_ready_) {
        /* disable torque*/
        int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
            portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_);
        if (dxl_comm_result != 0 || dxl_error_ != 0){
            RCLCPP_ERROR(this -> get_logger(), "failed to enable dxl torque");
            return false;
        } else {
            RCLCPP_INFO(this -> get_logger(), "dxl: disactivate");
            dxl_is_ready_ = false;
            return true;
        }
    } else {
        RCLCPP_ERROR(this -> get_logger(), "dxl is already torque off");
        return true;
    }
}

// int dxl_id_;
// int baudrate_;
// const char* device_file_;
// Dxl_instruction dxl_instruction_;
// bool dxl_is_ready_;
// uint8_t dxl_error_;
// double goal_velocity_;
// dynamixel::PortHandler *portHandler_;
// dynamixel::PacketHandler *packetHandler_;
// rclcpp::TimerBase::SharedPtr timer_;
// rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
// rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;







