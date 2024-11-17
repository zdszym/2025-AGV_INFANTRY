#include "chassis_power_control.h"

// 结构体实例化
chassis_power_control_t chassis_power_control;
WheelComm_t wheel_data[4] = {0};

// 因为时间问题，舵小板之间的通信也写在此文件夹，后面有时间会更正
int sign(int x)
{
    if (x > 0)
        return 1;
    else if (x == 0)
        return 0;
    else
        return -1;
}

float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

float calculate_torque_current_according_to_scaled_power(float scaled_power)
{

    float b, c, temp;
    b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm - scaled_power + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        temp = (-b + Sqrt(b * b - 4 * K1 * c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque <= temp)
        {
            steering_wheel.motion_part.motor.M3508_kit.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (temp > 16000)
            {
                steering_wheel.motion_part.motor.M3508_kit.command.torque = 16000;
            }
            else
                steering_wheel.motion_part.motor.M3508_kit.command.torque = (int)temp;
        }
    }
    else
    {
        temp = (-b - Sqrt(b * b - 4 * K1 * c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque >= temp)
        {
            steering_wheel.motion_part.motor.M3508_kit.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (temp < -16000)
            {
                steering_wheel.motion_part.motor.M3508_kit.command.torque = -16000;
            }
            else
                steering_wheel.motion_part.motor.M3508_kit.command.torque = (int)temp;
        }
    }
}

float expert_power_calculate(void)
{
    //    chassis_power_control.expect_motion_part_power_32 =
    //        steering_wheel.motion_part.motor.command.torque * TOQUE_COEFFICIENT * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm +
    //        K1 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm +
    //        K2 * steering_wheel.motion_part.motor.command.torque * steering_wheel.motion_part.motor.command.torque + CONSTANT;

    // power=机械功率+速度相关项损耗+转矩相关项损耗+常数
    chassis_power_control.expect_motion_part_power_32 =
        steering_wheel.motion_part.motor.command.torque * TOQUE_COEFFICIENT * __fabs(steering_wheel.motion_part.command.protocol_speed) * 14 * sign(steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm) +
        K1 * steering_wheel.motion_part.command.protocol_speed * 14 * steering_wheel.motion_part.command.protocol_speed * 14 +
        K2 * steering_wheel.motion_part.motor.command.torque * steering_wheel.motion_part.motor.command.torque + CONSTANT;

    chassis_power_control.expect_directive_part_power_32 =
        steering_wheel.directive_part.motor.command.torque * TOQUE_COEFFICIENT * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm +
        K2 * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm +
        K1 * steering_wheel.directive_part.motor.command.torque * steering_wheel.directive_part.motor.command.torque + CONSTANT;

    // chassis_power_control.expect_total_power_32 = chassis_power_control.expect_motion_part_power_32;
}

float m_b, m_c, m_temp, d_b, d_c, d_temp;
float scaled_power_calculate(void)
{

    chassis_power_control.scaled_power_coefficient_32 = chassis_power_control.power_limit_max / chassis_power_control.expect_total_power_32;
    chassis_power_control.scaled_directive_part_power_32 = chassis_power_control.expect_directive_part_power_32 * chassis_power_control.scaled_power_coefficient_32;
    chassis_power_control.scaled_motion_part_power_32 = chassis_power_control.expect_motion_part_power_32 * chassis_power_control.scaled_power_coefficient_32;
    m_b = steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    m_c = K2 * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.motion_part.motor.M3508_kit.feedback.current_rotor_rpm;
    m_c = m_c - chassis_power_control.scaled_motion_part_power_32 + CONSTANT;
    if (steering_wheel.motion_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        m_temp = (-m_b + Sqrt(m_b * m_b - 4 * K1 * m_c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque <= m_temp)
        {
            steering_wheel.motion_part.motor.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (m_temp > 16000)
            {
                steering_wheel.motion_part.motor.command.torque = 16000;
            }
            else
                steering_wheel.motion_part.motor.command.torque = (int)m_temp;
        }
    }
    else
    {
        m_temp = (-m_b - Sqrt(m_b * m_b - 4 * K1 * m_c)) / (2 * K1);

        if (steering_wheel.motion_part.motor.command.torque >= m_temp)
        {
            steering_wheel.motion_part.motor.command.torque = steering_wheel.motion_part.motor.command.torque;
        }
        else
        {
            if (m_temp < -16000)
            {
                steering_wheel.motion_part.motor.command.torque = -16000;
            }
            else
                steering_wheel.motion_part.motor.command.torque = (int)m_temp;
        }
    }
    d_b = steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * TOQUE_COEFFICIENT;
    d_c = K2 * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm * steering_wheel.directive_part.motor.M3508_kit.feedback.current_rotor_rpm - chassis_power_control.scaled_directive_part_power_32 + CONSTANT;
    if (steering_wheel.directive_part.motor.command.torque > 0) // Selection of the calculation formula according to the direction of the original moment
    {
        d_temp = (-d_b + Sqrt(d_b * d_b - 4 * K1 * d_c)) / (2 * K1);

        if (steering_wheel.directive_part.motor.command.torque <= d_temp)
        {
            steering_wheel.directive_part.motor.command.torque = steering_wheel.directive_part.motor.command.torque;
        }
        else
        {
            if (d_temp > 16000)
            {
                steering_wheel.directive_part.motor.command.torque = 16000;
            }
            else
                steering_wheel.directive_part.motor.command.torque = (int)d_temp;
        }
    }
    else
    {
        d_temp = (-d_b - Sqrt(d_b * d_b - 4 * K1 * d_c)) / (2 * K1);

        if (steering_wheel.directive_part.motor.command.torque >= d_temp)
        {
            steering_wheel.directive_part.motor.command.torque = steering_wheel.directive_part.motor.command.torque;
        }
        else
        {
            if (d_temp < -16000)
            {
                steering_wheel.directive_part.motor.command.torque = -16000;
            }
            else
                steering_wheel.directive_part.motor.command.torque = (int)d_temp;
        }
    }
}

// 通信部份
//  void agv_feedback_handler(Power_Calc_Data_t *power_calc_data, uint8_t rxdata[8]){
//   power_calc_data->motion_part_speed= (rxdata[0]<<8)|rxdata[1];
//   power_calc_data->motion_part_torque= (rxdata[2]<<8)|rxdata[3];
//   power_calc_data->directive_part_speed= (rxdata[4]<<8)|rxdata[5];
//   power_calc_data->directive_part_torque= (rxdata[6]<<8)|rxdata[7];
//  }
//  static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[])
//  {
//      #if defined(STM32F105) || (STM32F407)
//          agv_board_CAN_TxHeaderStruct.StdId = CAN_ID;
//          return HAL_CAN_AddTxMessage(handle, &agv_board_CAN_TxHeaderStruct, aData, &agv_board_pTxMailbox);
//      #endif
//  }
//  void agv_communication_init(void)
//  {
//      /* Initialize transmission functions */
//      steering_communication_ctx.tx_cmd =platform_trans;
//  	steering_communication_ctx.handle = &STEERING_COMMUNICATION_HANDLE;
//      /* Initialize CAN driver interface */
//      #if defined(STM32F105) || (STM32F407)
//  	 agv_board_CAN_TxHeaderStruct.StdId = 0;
//          agv_board_CAN_TxHeaderStruct.ExtId = 0;
//          agv_board_CAN_TxHeaderStruct.DLC = 8;
//          agv_board_CAN_TxHeaderStruct.IDE = CAN_ID_STD; // 使用拓展帧
//          agv_board_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
//          agv_board_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
//      #endif
//  	Steering_Communication_SubscribeList_Init(); // 初始化订阅队列
//  }

// uint8_t briter_encoder_command_transmit(agv_communication_ctx_t *ctx, Power_Calc_Data_t *power_calc_data)
// {
// 	uint8_t dataA[8];
// 	memcpy(&dataA, &power_calc_data->command, 8);
// 	return ctx->tx_cmd(encoder->parameter.handle, encoder->parameter.CAN_ID, dataA);

// }

// 数据打包函数
uint8_t *wheel_data_pack(WheelComm_t *wheel_comm, steering_wheel_t *steering_wheel, uint8_t *buffer)
{
    expert_power_calculate();
    // 直接使用memcpy复制两个float数据到buffer
    memcpy(buffer, &chassis_power_control.expect_motion_part_power_32, 4);
    memcpy(buffer + 4, &chassis_power_control.expect_directive_part_power_32, 4);
    return buffer;
}

// 发送单个轮组数据
uint8_t wheel_data_send(WheelComm_t *wheel_comm, steering_wheel_t *steering_wheel)
{
    uint8_t data[8];
    uint32_t used_mailbox;

    uint8_t *pack_data = wheel_data_pack(wheel_comm, steering_wheel, data);

    return (HAL_CAN_AddTxMessage(&hcan2, &wheel_comm->tx_header, pack_data, &used_mailbox));
}

// 数据解包函数
void wheel_data_unpack(WheelComm_t *wheel_comm, uint8_t rxdata[8])
{
    float motion_power, directive_power;

    // 从buffer还原两个float数据
    memcpy(&motion_power, rxdata, 4);
    memcpy(&directive_power, rxdata + 4, 4);

    // 存储解析出的功率值
    wheel_comm->data.motion_power = motion_power;
    wheel_comm->data.directive_power = directive_power;
    // 发送函数，
}
// 最大功率数据解包函数
void limit_power_data_unpack(chassis_power_control_t *chassis_power_control, uint8_t rxdata[8])
{
    uint16_t limit_power = 0;

    memcpy(&limit_power, rxdata, 4);

    chassis_power_control->power_limit_max = limit_power;
}
float sum = 0;

void calculate_total_power(WheelComm_t *wheel_comm)
{
    chassis_power_control.expect_total_power_32 = 0;
#ifdef AGV_BOARD_A
    // 先计算A舵的预期功率
    expert_power_calculate();
    wheel_comm[0].data.motion_power = chassis_power_control.expect_motion_part_power_32;
    wheel_comm[0].data.directive_power = chassis_power_control.expect_directive_part_power_32;
    // 计算每个轮组的预期功率
    for (uint8_t i = 0; i < 4; i++)
    {

        chassis_power_control.expect_total_power_32 += wheel_comm[i].data.motion_power + wheel_comm[i].data.directive_power;
    }
#endif
#ifdef AGV_BOARD_B
    // 计算每个轮组的预期功率
    expert_power_calculate();
    wheel_comm[1].data.motion_power = chassis_power_control.expect_motion_part_power_32;
    wheel_comm[1].data.directive_power = chassis_power_control.expect_directive_part_power_32;
    // 计算每个轮组的预期功率
    for (uint8_t i = 0; i < 4; i++)
    {

        chassis_power_control.expect_total_power_32 += wheel_comm[i].data.motion_power + wheel_comm[i].data.directive_power;
    }
#endif
#ifdef AGV_BOARD_C
    // 计算每个轮组的预期功率
    expert_power_calculate();
    wheel_comm[2].data.motion_power = chassis_power_control.expect_motion_part_power_32;
    wheel_comm[2].data.directive_power = chassis_power_control.expect_directive_part_power_32;
    // 计算每个轮组的预期功率
    for (uint8_t i = 0; i < 4; i++)
    {

        chassis_power_control.expect_total_power_32 += wheel_comm[i].data.motion_power + wheel_comm[i].data.directive_power;
    }
#endif
#ifdef AGV_BOARD_D
    // 计算每个轮组的预期功率
    expert_power_calculate();
    wheel_comm[3].data.motion_power = chassis_power_control.expect_motion_part_power_32;
    wheel_comm[3].data.directive_power = chassis_power_control.expect_directive_part_power_32;
    // 计算每个轮组的预期功率
    for (uint8_t i = 0; i < 4; i++)
    {

        chassis_power_control.expect_total_power_32 += wheel_comm[i].data.motion_power + wheel_comm[i].data.directive_power;
    }
#endif
}

// 功率分配

void calculate_true_power(WheelComm_t *wheel_comm, chassis_power_control_t *chassis_power_control)
{
    calculate_total_power(wheel_comm); // 先计算总功率

    // 计算功率缩放系数
    chassis_power_control->scaled_power_coefficient_32 =
        chassis_power_control->power_limit_max / chassis_power_control->expect_total_power_32;
    chassis_power_control->scaled_directive_part_power_32 = chassis_power_control->scaled_power_coefficient_32 * chassis_power_control->expect_directive_part_power_32;
    chassis_power_control->scaled_motion_part_power_32 = chassis_power_control->scaled_power_coefficient_32 * chassis_power_control->expect_motion_part_power_32;
}
// uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
// {
//     CAN_TxHeaderTypeDef tx_header;
//     uint32_t used_mailbox;

//     // 检测传参是否正确
//     assert_param(hcan != NULL);

//     tx_header.StdId = ID;
//     tx_header.ExtId = 0;
//     tx_header.IDE = 0;
//     tx_header.RTR = 0;
//     tx_header.DLC = Length;

//     return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
// }
void wheel_data_init(WheelComm_t *wheel_comm, WheelID_e can_id)
{
    wheel_comm->tx_header.StdId = can_id;
    wheel_comm->tx_header.ExtId = 0;
    wheel_comm->tx_header.IDE = 0;
    wheel_comm->tx_header.RTR = 0;
    wheel_comm->tx_header.DLC = 8;
    wheel_comm->tx_header.TransmitGlobalTime = DISABLE;
    wheel_comm->data.motion_power = 0;
    wheel_comm->data.directive_power = 0;
}