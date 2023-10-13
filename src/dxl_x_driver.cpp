
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
/* MEMO: where to install dynamixel_sdk by default Makefile
        library: /usr/local/lib
        header: /usr/local/include
*/

#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

/* ===== basic settings ===== */
#define PROTOCOL_VERSION        2.0
#define DXL_ID                  1
#define BAUDRATE                9600
#define DEVICENAME              "/dev/ttyUSB0"

/* ===== control table address ===== */
#define ADDR_OPERATING_MODE       11
#define ADDR_CURRENT_LIMIT        38
#define ADDR_VELOCITY_LIMIT       44
// #define ADDR_MAX_POSITION_LIMIT   48
// #define ADDR_MIN_POSITION_LIMIT   52
#define ADDR_TORQUE_ENABLE        64
#define ADDR_VELOCITY_I_GAIN      76
#define ADDR_VELOCITY_P_GAIN      78
// #define ADDR_POSITION_D_GAIN      80
// #define ADDR_POSITION_I_GAIN      82
// #define ADDR_POSITION_P_GAIN      84
#define ADDR_GOAL_VELOCITY        104
// #define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_CURRENT        126
#define ADDR_PRESENT_VELOCITY       128
// #define ADDR_PRESENT_POSITION       132

/* ===== control table value ===== */
#define VELOCITY_CONTROL_MODE   1
// #define POSITION_CONTROL_MODE   3
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define VELOCITY_LIMIT          210
// #define MAXIMUM_POSITION_LIMIT  4095
// #define MINIMUM_POSITION_LIMIT  0
#define DXL_MOVING_STATUS_THRESHOLD 20


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

    DxlXDriver(): Node("dxl_x_driver_node")
    {
        this -> declare_parameter("dxl_id", DXL_ID);
        this -> declare_parameter("baudrate", BAUDRATE);
        this -> declare_parameter("device_file", DEVICENAME);

        dxl_instruction_ = Dxl_instruction::INSTRUCTION_INITIAL;
        dxl_is_ready_ = false;
        dxl_error_ = 0;
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        timer_ = this -> create_wall_timer(500ms, std::bind(&DxlXDriver::callback, this));
        // publisher_ = this -> create_publisher<std_msg::aaaaaa>("dxl_present_velocity", 10);
        // subscription_ = this -> create_subscription<geometry_msgs::twist>(
        //     "dxl_goal_velocity", 10, std::bind(DxlXDriver::callback, this, _1));

        /* open port */
        if (portHandler_ -> openPort()){
            RCLCPP_INFO(this -> get_logger(), "succeed to open port");
        } else {
            RCLCPP_ERROR(this -> get_logger(), "fail to open port");
        }

        /* set baudrate */
        if (portHandler_ -> setBaudRate(BAUDRATE)) {
            RCLCPP_INFO(this -> get_logger(), "succeed to set baudrate");
        } else {
            RCLCPP_ERROR(this -> get_logger(), "failed to set baudrate");
        } 

        /* set operating mode */
        int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
            portHandler_, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error_);
        if (dxl_comm_result != COMM_SUCCESS || dxl_error_ != 0) {
            RCLCPP_ERROR(this -> get_logger(), "failed to set operating mode");
        } else {
            RCLCPP_INFO(this -> get_logger(), "Operating mode: velocity control mode.");
        }

        /* set velocity limit */
        dxl_comm_result = packetHandler_ -> write4ByteTxRx(
            portHandler_, DXL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error_);
        if (dxl_comm_result != 0 || dxl_error_ != 0) {
            RCLCPP_ERROR(this -> get_logger(), "failed to set velocity limit");
        } else {
            RCLCPP_INFO(this -> get_logger(), "succeed to set velocity limit");
        }

        /* need further implementation */
        dxl_instruction_ = Dxl_instruction::INSTRUCTION_ACTIVATE;
    }

    ~DxlXDriver() {
        step_zero_velocity();
        step_disactivate();
    }

private:
    void callback()
    {
        // geometry_msgs::msg::twist msg;
        // RCLCPP_INFO(this -> get_logger(), "id:%s, baudrate:%s, port:%s, mode:%s, limit:%s", my_param.c_str());

        /* check instruction */
        switch (dxl_instruction_) {
            case Dxl_instruction::INSTRUCTION_ACTIVATE:
                step_activate();
                dxl_instruction_ = Dxl_instruction::INSTRUCTION_RUN_VELOCITY;
                // break;
            case Dxl_instruction::INSTRUCTION_RUN_VELOCITY:
                step_run_velocity();
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

        step_read_velocity();

        // publisher_ -> publish(msg);
    }

    void step_activate(){
        if (dxl_is_ready_) {
            RCLCPP_INFO(this -> get_logger(), "dxl is already ready");
        } else {
            /* enable torque */
            int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
                portHandler_, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_);
            if (dxl_comm_result != 0 || dxl_error_ != 0){
                RCLCPP_ERROR(this -> get_logger(), "failed to enable dxl torque");
                dxl_is_ready_ = false;
            } else {
                dxl_is_ready_ = true;
                RCLCPP_INFO(this -> get_logger(), "dxl: activate");
            }   
        }
    }

    void step_run_velocity(){
        if (dxl_is_ready_) {
            /* set motor goal velocity */
            int dxl_comm_result = packetHandler_ -> write4ByteTxRx(
                portHandler_, DXL_ID, ADDR_GOAL_VELOCITY, VELOCITY_LIMIT-20, &dxl_error_);
            if (dxl_comm_result != 0 || dxl_error_ != 0){
                RCLCPP_ERROR(this -> get_logger(), "failed to set goal velocity");
            }
        } else {
            RCLCPP_ERROR(this -> get_logger(), "dxl is not ready");
        }
    }

    void step_zero_velocity(){
        if (dxl_is_ready_) {
            /* set motor velocity zero */
            int dxl_comm_result = packetHandler_ -> write4ByteTxRx(
                portHandler_, DXL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error_);
            if (dxl_comm_result != 0 || dxl_error_ != 0){
                RCLCPP_ERROR(this -> get_logger(), "failed to set zero velocity");
            } else {
                RCLCPP_INFO(this -> get_logger(), "set zero velocity");
            }
        } else {
            RCLCPP_ERROR(this -> get_logger(), "dxl is not ready");
        }
    }

    void step_read_velocity(){
        /* read motor velocity */
        uint16_t dxl_present_velocity = 0;
        int dxl_comm_result = packetHandler_ -> read4ByteTxRx(
            portHandler_, DXL_ID, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error_); 
        if (dxl_comm_result != 0 || dxl_error_ != 0) {
            RCLCPP_ERROR(this -> get_logger(), "failed to read present velocity");
        } else {
            RCLCPP_INFO(this -> get_logger(), "present velocity: %u[rpm]", dxl_present_velocity);
        }
    }

    void step_disactivate(){
        if (dxl_is_ready_) {
            /* disable torque*/
            int dxl_comm_result = packetHandler_ -> write1ByteTxRx(
                portHandler_, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_);
            if (dxl_comm_result != 0 || dxl_error_ != 0){
                RCLCPP_ERROR(this -> get_logger(), "failed to enable dxl torque");
            } else {
                RCLCPP_INFO(this -> get_logger(), "dxl: disactivate");
                dxl_is_ready_ = false;
            }
        } else {
            RCLCPP_ERROR(this -> get_logger(), "dxl is already torque off");
        }
    }

    // void change_instruction(){
    //     dxl_instruction_ = ;
    // }

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<geometry_msgs::msg::twist>::SharedPtr publisher_;

    uint8_t dxl_error_;
    // uint8_t dxl_instruction_;
    Dxl_instruction dxl_instruction_;
    bool dxl_is_ready_;
    int velocity_value;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlXDriver>());
    rclcpp::shutdown();

    return 0;
}
