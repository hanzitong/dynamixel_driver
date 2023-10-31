

#ifndef DYNAMIXEL_DRIVER_DXLXDRIVER_H
#define DYNAMIXEL_DRIVER_DXLXDRIVER_H


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>


class DxlXDriver : public rclcpp::Node
{
public:
    enum class Dxl_instruction : uint8_t {
        INSTRUCTION_INITIAL,
        INSTRUCTION_ACTIVATE,
        INSTRUCTION_RUN_VELOCITY,
        INSTRUCTION_RUN_POSITION,
        INSTRUCTION_STOP,
        INSTRUCTION_DISACTIVATE,
    };

    DxlXDriver();
    ~DxlXDriver();
private:
    void timer_callback();
    void subscription_callback(geometry_msgs::msg::Twist msg);
    void stand_up();
    bool step_activate();
    bool step_run_velocity(double goal_velocity);
    bool step_zero_velocity();
    double step_read_velocity();
    double step_read_present_current();
    bool step_disactivate();

    int dxl_id_;
    int baudrate_;
    const char* device_file_;
    Dxl_instruction dxl_instruction_;
    bool dxl_is_ready_;
    uint8_t dxl_error_;
    double goal_velocity_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};


#endif // DYNAMIXEL_DRIVER_DXLXDRIVER_H

