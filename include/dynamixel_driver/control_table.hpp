


#ifndef DYNAMIXEL_DRIVER_CONTROL_TABLE_H
#define DYNAMIXEL_DRIVER_CONTROL_TABLE_H


/* ===== basic settings ===== */
#define PROTOCOL_VERSION        2.0

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


#endif  // DYNAMIXEL_DRIVER_CONTROL_TABLE_H




