/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef TSK_INTERACTION_H
#define TSK_INTERACTION_H

/* Includes ------------------------------------------------------------------*/
#define GIMBAL
#include "dvc_dr16.h"
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "crt_booster.h"
#include "dvc_imu.h"
#include "tsk_config_and_callback.h"
#include "dvc_supercap.h"
#include "dvc_VT13.h"
/* Exported macros -----------------------------------------------------------*/

// #define CHASSIS
#define GIMBAL
#define AGV
// #define POWER_LIMIT

// #define IMAGE_VT12
#define IMAGE_VT13
/* Exported types ------------------------------------------------------------*/

class Class_Chariot;
/**
 * @brief 底盘通讯状态
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE,
};
/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
    DR16_Control_Type_NONE,

};

/**
 * @brief VT13控制数据来源
 *
 */
enum Enum_VT13_Control_Type
{
    VT13_Control_Type_REMOTE = 0,
    VT13_Control_Type_KEYBOARD,
    VT13_Control_Type_NONE,
};

/**
 * @brief 机器人是否离线 控制模式有限自动机
 *
 */
class Class_FSM_Alive_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

class Class_FSM_Alive_Control_VT13 : public Class_FSM
{
public:
    uint8_t Start_Flag = 0; // 记录第一次上电开机，以初始化状态
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};
// 添加活动控制器枚举类型
enum Enum_Active_Controller
{
    Controller_NONE = 0,
    Controller_DR16,
    Controller_VT13
};

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
#ifdef CHASSIS
    // 裁判系统

#ifdef POWER_LIMIT
    // 超级电容
    Class_Supercap Supercap;

    // 底盘随动PID环
    Class_PID PID_Chassis_Fllow;

    // 获取yaw电机编码器值 用于底盘和云台坐标系的转换
    Class_DJI_Motor_GM6020 Motor_Yaw;
#endif

#endif
    // 底盘
    Class_Tricycle_Chassis Chassis;

    Class_Referee Referee;

#ifdef GIMBAL
    // 遥控器
    Class_DR16 DR16;
    // 上位机
    Class_MiniPC MiniPC;
    // 云台
    Class_Gimbal Gimbal;
    // 发射机构
    Class_Booster Booster;
#ifdef IMAGE_VT13
    Class_VT13 VT13;
#endif // VT13

    // 遥控器离线保护控制状态机
    Class_FSM_Alive_Control FSM_Alive_Control;
    Class_FSM_Alive_Control_VT13 FSM_Alive_Control_VT13;

    friend class Class_FSM_Alive_Control;
    friend class Class_FSM_Alive_Control_VT13;

    inline Enum_DR16_Control_Type Get_DR16_Control_Type();
#endif

    void Init(float __DR16_Dead_Zone = 0);

#ifdef CHASSIS
    void CAN_Chassis_Control_RxCpltCallback();

#elif defined(GIMBAL)
    void CAN_Gimbal_RxCpltCallback(uint8_t *data);
    void CAN_Gimbal_TxCpltCallback();
    void TIM_Control_Callback();

    inline void DR16_Offline_Cnt_Plus();

    inline uint16_t Get_DR16_Offline_Cnt();
    inline void Clear_DR16_Offline_Cnt();
    inline Enum_Chassis_Control_Type Get_Pre_Chassis_Control_Type();
    inline Enum_Gimbal_Control_Type Get_Pre_Gimbal_Control_Type();
    inline Enum_Booster_Control_Type Get_Pre_Booster_Control_Type();

    inline void Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
#endif

    void TIM_Calculate_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN2_Manage_Object;
    // 发送缓存区
    uint8_t *CAN_Tx_Data = CAN2_Gimbal_Tx_Data;
    uint8_t *CAN_Rx_Data = CAN2_Chassis_Tx_Data;
    // 遥控器拨动的死区, 0~1
    float DR16_Dead_Zone;

    // 常量
    // 底盘标定参考正方向角度(数据来源yaw电机)
    float Reference_Angle = 2.45360231;
    // 底盘转换后的角度（数据来源yaw电机）
    float Chassis_Angle;

    // DR16底盘加速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Small = 0.001f;
    // DR16底盘减速灵敏度系数(0.001表示底盘加速度最大为1m/s2)
    float DR16_Keyboard_Chassis_Speed_Resolution_Big = 0.01f;

    // DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
    float DR16_Yaw_Angle_Resolution = 0.005f * PI * 57.29577951308232;
    // DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
    float DR16_Pitch_Angle_Resolution = 0.0035f * PI * 57.29577951308232;

    // DR16云台yaw灵敏度系数(0.001PI表示yaw速度最大时为1rad/s)
    float DR16_Yaw_Resolution = 0.003f * PI;
    // DR16云台pitch灵敏度系数(0.001PI表示pitch速度最大时为1rad/s)
    float DR16_Pitch_Resolution = 0.003f * 120;

    // DR16鼠标云台yaw灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Yaw_Angle_Resolution = 500.0f;
    // DR16鼠标云台pitch灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Pitch_Angle_Resolution = 500.0f;

    // 迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Yaw_Angle_Resolution = 0.003f;
    // 迷你主机云台pitch自瞄控制系数
    float MiniPC_Autoaiming_Pitch_Angle_Resolution = 0.003f;
    uint8_t Shoot_Flag = 0;

    // 内部变量
    // 遥控器离线计数
    uint16_t DR16_Offline_Cnt = 0;
    // 读变量

    // 写变量    // 写变量
    uint32_t Chassis_Alive_Flag = 0;
    uint32_t Pre_Chassis_Alive_Flag = 0;

    // 读写变量
    Enum_Chassis_Status Chassis_Status = Chassis_Status_DISABLE;
    // 底盘 云台 发射机构 前一帧控制类型
    Enum_Chassis_Control_Type Pre_Chassis_Control_Type = Chassis_Control_Type_FLLOW;
    Enum_Gimbal_Control_Type Pre_Gimbal_Control_Type = Gimbal_Control_Type_NORMAL;
    Enum_Booster_Control_Type Pre_Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

    Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_NONE;
    Enum_VT13_Control_Type VT13_Control_Type = VT13_Control_Type_NONE;
    // 内部函数
    // 当前活动的控制器
    Enum_Active_Controller Active_Controller = Controller_NONE;

    // 判断当前活动的控制器
    void Judge_Active_Controller();
    // 获取当前活动的控制器类型
    Enum_Active_Controller Get_Active_Controller();
    // 获取DR16控制类型
    // Enum_DR16_Control_Type Get_DR16_Control_Type();
    // 获取VT13控制类型
    Enum_VT13_Control_Type Get_VT13_Control_Type();
    // 内部函数
    void Judge_DR16_Control_Type();
    void Judge_VT13_Control_Type();
    void Control_Chassis();
    void Control_Gimbal();
    void Control_Booster();
};

/**
 * @brief 获取前一帧底盘控制类型
 *
 * @return Enum_Chassis_Control_Type 前一帧底盘控制类型
 */

Enum_Chassis_Control_Type Class_Chariot::Get_Pre_Chassis_Control_Type()
{
    return (Pre_Chassis_Control_Type);
}

/**
 * @brief 获取前一帧云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 前一帧云台控制类型
 */

Enum_Gimbal_Control_Type Class_Chariot::Get_Pre_Gimbal_Control_Type()
{
    return (Pre_Gimbal_Control_Type);
}

/**
 * @brief 获取前一帧发射机构控制类型
 *
 * @return Enum_Booster_Control_Type 前一帧发射机构控制类型
 */
Enum_Booster_Control_Type Class_Chariot::Get_Pre_Booster_Control_Type()
{
    return (Pre_Booster_Control_Type);
}

/**
 * @brief 设置前一帧底盘控制类型
 *
 * @param __Chassis_Control_Type 前一帧底盘控制类型
 */
void Class_Chariot::Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Pre_Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设置前一帧云台控制类型
 *
 * @param __Gimbal_Control_Type 前一帧云台控制类型
 */
void Class_Chariot::Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Pre_Gimbal_Control_Type = __Gimbal_Control_Type;
}

/**
 * @brief 设置前一帧发射机构控制类型
 *
 * @param __Booster_Control_Type 前一帧发射机构控制类型
 */
void Class_Chariot::Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Pre_Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief DR16离线计数加一
 */
void Class_Chariot::DR16_Offline_Cnt_Plus()
{
    DR16_Offline_Cnt++;
}

/**
 * @brief 获取DR16离线计数
 *
 * @return uint16_t DR16离线计数
 */
uint16_t Class_Chariot::Get_DR16_Offline_Cnt()
{
    return (DR16_Offline_Cnt);
}

/**
 * @brief DR16离线计数置0
 *
 */
void Class_Chariot::Clear_DR16_Offline_Cnt()
{
    DR16_Offline_Cnt = 0;
}

/**
 * @brief 获取DR16控制数据来源
 *
 * @return Enum_DR16_Control_Type DR16控制数据来源
 */

Enum_DR16_Control_Type Class_Chariot::Get_DR16_Control_Type()
{
    return (DR16_Control_Type);
}

#ifdef AGV
typedef __packed enum {
    CHASSIS_MODE_NOFORCE = 0x00u,
    CHASSIS_MODE_TOPANGLE = 0x02u,
    CHASSIS_MODE_ABSOLUTE = 0x01u,
    CHASSIS_MODE_PRECISE = 0x03u,
} CHASSIS_MODE_E;

typedef __packed struct
{
    float vx;
    float vy;
    float vw;
} CHASSIS_VELOCITY_T;

typedef __packed struct
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;
    bool follow_flag;
    bool invert_flag;

} CHASSIS_SEND_T;

typedef __packed struct
{
    CHASSIS_MODE_E mode;
    CHASSIS_VELOCITY_T velocity;

} CHASSIS_RECEIVE_T;

typedef __packed struct
{
    CHASSIS_SEND_T send;
    CHASSIS_RECEIVE_T receive;

} CHASSIS_T;

void Chassis_Connection_Init(void);
extern CHASSIS_T chassis;
void Chassis_Connection_Task(void);
void Can_Send_Task(int8_t ms_count);
void Can_Connection_Init(void);
#endif

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
