#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

/* USER Settings -------------------------------------------------------------*/
#include <stdint.h>
#include "can.h"
#include "SW_control_task.h"
#define DEFAULT_SET_VECTOR_SPEED_ONLY_CNT 1 /* 默认仅设置速度的次数 */
#define VALUE_OF_SCALED_POWER_COEFFICIENT_WHEN_SET_VECTOR_SPEED_ONLY -114514

#define TOQUE_COEFFICIENT 1.99688994e-6 // (20/16384)*(0.3)*(1/14)/9.55
#define K1 1.23e-07                     // 1为默认数值
#define K2 1.453e-07                    // 1为默认数值
#define CONSTANT 1.0f                   // 功率模型中的常数
// 用来存放其他舵轮回传的速度和扭矩
//  电机参数结构体
//  typedef struct {
//      float motion_part_speed;
//      float directive_part_speed;
//      uint16_t motion_part_torque;
//      uint16_t directive_part_torque;
//  } Power_Calc_Data_t;
//  /* 舵小板之间的通信 ------------------------------------------------------------*/
//  //烂烂，想改了,真烂完了，忙完最晚寒假全都改一遍
//     CAN_TxHeaderTypeDef agv_board_CAN_TxHeaderStruct;
//  typedef uint8_t (*agv_board_tx_ptr)(void *, uint16_t , uint8_t[]);
//  typedef uint8_t (*agv_board_rx_ptr)(void *, uint16_t *, uint8_t[]);
//     uint32_t  agv_board_pTxMailbox;
//  static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[]);
//  typedef struct
//  {
//  	/** essential  **/
//  	agv_board_tx_ptr	tx_cmd;

// 	/** customizable  **/
// 	void *handle;
// } agv_communication_ctx_t;

// agv_communication_ctx_t steering_communication_ctx;

// 轮组ID枚举
typedef enum
{
  WHEEL_ID_1 = 0x02A,
  WHEEL_ID_2 = 0x02B,
  WHEEL_ID_3 = 0x02C,
  WHEEL_ID_4 = 0x02D
} WheelID_e;

// 轮组数据结构体
typedef struct
{
  float motion_power;    // 动力轮速度
  float directive_power; // 转向轮速度
} WheelData_t;

// CAN通信配置结构体
// typedef struct {
//     CAN_HandleTypeDef* hcan;         // CAN句柄
//     CAN_TxHeaderTypeDef tx_header;   // 发送帧头
//     uint32_t tx_mailbox;             // 发送邮箱
// } WheelCAN_t;

// 轮组通信管理结构体
typedef struct
{
  WheelID_e id;                  // 轮组ID
  WheelData_t data;              // 轮组数据
                                 // WheelCAN_t* can;        // CAN配置
  CAN_HandleTypeDef *hcan;       // CAN句柄
  CAN_TxHeaderTypeDef tx_header; // 发送帧头
} WheelComm_t;

// 函数声明

/* Exported types ------------------------------------------------------------*/
#define CONVERT_CONSTANT 9.55 // 转化系数
typedef struct
{
  /* data */

  float power_limit_max; // 功率限制最大值

  float sum_power; // 用来存放总功率

  float scaled_power_coefficient_32; // 功率缩放系数

  float expect_total_power_32;
  float expect_directive_part_power_32;
  float expect_motion_part_power_32;
  float scaled_directive_part_power_32;
  float scaled_motion_part_power_32;

} chassis_power_control_t;

extern chassis_power_control_t chassis_power_control;
float expert_power_calculate(void);
float scaled_power_calculate(void);
float calculate_torque_current_according_to_scaled_power(float scaled_power);
// void wheel_can_init(WheelCAN_t* wheel_can, CAN_HandleTypeDef* hcan);
uint8_t *wheel_data_pack(WheelComm_t *wheel_comm, steering_wheel_t *steering_wheel, uint8_t *buffer);
uint8_t wheel_data_send(WheelComm_t *wheel_comm, steering_wheel_t *steering_wheel);
//  void Chassis_Power_Control_Init(void);
// extern Power_Calc_Data_t power_calc_data[4];
void calculate_true_power(WheelComm_t *wheel_comm, chassis_power_control_t *chassis_power_control);
uint8_t CAN_Send_Data(uint16_t ID, uint8_t *Data,chassis_power_control_t *chassis_power_control);
void wheel_data_unpack(WheelComm_t *wheel_comm, uint8_t rxdata[8]);
void limit_power_data_unpack(chassis_power_control_t *chassis_power_control, uint8_t rxdata[8]);
void wheel_data_init(WheelComm_t *wheel_comm, WheelID_e can_id);
extern WheelComm_t wheel_data[4];
#endif