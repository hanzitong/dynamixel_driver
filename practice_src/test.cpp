#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <bitset>
#include <chrono>
#include <thread>
using namespace std::this_thread;   // sleep_for()
using namespace std::chrono_literals;   // ns, us, ms, s, h, etc

#include <dynamixel_sdk/dynamixel_sdk.h>
/* MEMO: where to install dynamixel_sdk by default Makefile
        library: /usr/local/lib
        header: /usr/local/include
*/

/* basic settings */
#define PROTOCOL_VERSION        2.0
#define DXL_ID                  1
#define BAUDRATE                9600
#define DEVICENAME              "/dev/ttyUSB1"
#define ESC_ASCII_VALUE         0x1b
#define STDIN_FILENO            0          // nanikore

/* control table address */
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
// #define ADDR_PRESENT_POSITION       132

/* control table value */
#define VELOCITY_CONTROL_MODE   1
// #define POSITION_CONTROL_MODE   3
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define VELOCITY_LIMIT          210
// #define MAXIMUM_POSITION_LIMIT  4095
// #define MINIMUM_POSITION_LIMIT  0
#define DXL_MOVING_STATUS_THRESHOLD 20


int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);     // STDIN_FILENO: file descriptor
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}


int main(int argc, char* argv[])
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    uint16_t dxl_model_number = 0;
    int index = 0;
    // int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};
    // int32_t dxl_present_position = 0;
    int32_t dxl_present_velocity = 0;

    /* open port */
    if (portHandler -> openPort()){
        std::cout << "succeed to open the port" << std::endl;
    } else {
        std::cout << "fail to open the port" << std::endl;

        return 0;
    }

    /* set port baudrate */
    if (portHandler -> setBaudRate(BAUDRATE)) {
        std::cout << "succeed to set baudrate" << std::endl;
    } else {
        std::cout << "fail to set baudrate" << std::endl;

        return 0;
    } 

    /* ping to dynamixel */
    dxl_comm_result = packetHandler -> ping(portHandler, DXL_ID, &dxl_model_number, &dxl_error);
    // std::cout << "ping" << std::endl;
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout << "fail ping " << std::endl;
        std::cout << packetHandler -> getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0){
        std::cout << "success ping " << std::endl;
        std::cout << packetHandler -> getRxPacketError(dxl_error);
        std::cout << static_cast<int>(dxl_error) << std::endl;
    } else {
        std::cout << "ping succeed. ID:" << DXL_ID << "dynamixel model number:" << dxl_model_number << std::endl;
    }

    /* set operating mode */
    dxl_comm_result = packetHandler -> write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        printf("Operating mode: velocity control mode. \n");
    }


    /* enable dynamixel torque */
    dxl_comm_result = packetHandler -> write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << packetHandler -> getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << packetHandler -> getRxPacketError(dxl_error) << std::endl;
    } else {
        std::cout << "enable dynamixel torque" << std::endl;
    }

    /* write velocity limit */
    dxl_comm_result = packetHandler -> write4ByteTxRx(portHandler, DXL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    if (dxl_comm_result != 0 || dxl_error != 0) {
        std::cout << "fail to set velocity limit" << std::endl;
    } else {
        std::cout << "succeed to set velocity limit" << std::endl;
    }


    /* disable dynamixel torque */
    dxl_comm_result = packetHandler -> write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cout << packetHandler -> getTxRxResult(dxl_comm_result) << std::endl;
    } else if (dxl_error != 0) {
        std::cout << packetHandler -> getRxPacketError(dxl_error) << std::endl;;
    } else {
        std::cout << "disable dynamixel torque" << std::endl;
    }


    /* close port */
    portHandler -> closePort();
    std::cout << "closed port" << std::endl;

    return 0;
}



