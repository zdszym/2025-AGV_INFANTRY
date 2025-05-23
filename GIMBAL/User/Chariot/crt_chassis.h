/**
 * @file crt_chassis.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 底盘电控
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_sampler.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "alg_power_limit.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    FOLLOW_ON = 0x01,
    FOLLOW_OFF = 0x00,
} Enum_FOLLOW_FLAG_E;

/**
 * @brief
 *
 */
enum Enum_Chassis_Invert_Flag : uint8_t
{
    Chassis_Invert_OFF = 0,
    Chassis_Invert_ON,
};

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type : uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN,
    Chassis_Control_Type_ANTI_SPIN,
};

typedef enum
{
    SUPERCAP_ON = 0x01,
    SUPERCAP_OFF = 0x00,
} Enum_SUPERCAP_FLAG_E;

typedef enum
{
    UI_INIT_ON = 0x01,
    UI_INIT_OFF = 0x00,
} Enum_UI_INIT_FLAG_E;

enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0,
    Sprint_Status_ENABLE,
};
/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
// omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    // 功率限制
    Class_Power_Limit Power_Limit;

    // 裁判系统
    Class_Referee *Referee;

    // 下方转动电机
    Class_DJI_Motor_C620 Motor_Wheel[4];

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 4.0f, float __Steer_Power_Ratio = 0.5);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Steer_Power();
    inline float Get_Target_Steer_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Spin_Omega();
    inline float Get_Chassis_UI_Init_flag();
    inline Enum_SUPERCAP_FLAG_E Get_Supercap_State(void);
    inline Enum_Chassis_Invert_Flag Get_Chassis_Invert_Flag(void);

    inline void Set_Chassis_Invert_Flag(Enum_Chassis_Invert_Flag __Chassis_Invert_Flag);
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Chassis_UI_Init_flag(Enum_UI_INIT_FLAG_E __UI_init_flag);
    inline void Set_Supercap_State(Enum_SUPERCAP_FLAG_E __Supercap_State);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback();

    Enum_Sprint_Status Sprint_Status = Sprint_Status_DISABLE;

protected:
    // 初始化相关常量

    // 速度X限制
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;
    // 舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    // 底盘小陀螺模式角速度
    float Spin_Omega = 5.0f;
    // 常量

    // 电机理论上最大输出
    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    // 内部变量

    // 舵向电机目标值
    float Target_Steer_Angle[3];
    // 转动电机目标值
    float Target_Wheel_Omega[4];

    // 读变量

    // 当前总功率
    float Now_Power = 0.0f;
    // 当前舵向电机功率
    float Now_Steer_Power = 0.0f;
    // 可使用的舵向电机功率
    float Target_Steer_Power = 0.0f;
    // 当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    // 可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_FOLLOW_FLAG_E FOLLOW_FLAG = FOLLOW_ON;
    Enum_SUPERCAP_FLAG_E SUPERCAP_FLAG = SUPERCAP_OFF;
    Enum_UI_INIT_FLAG_E UI_INIT_FLAG = UI_INIT_OFF;
    Enum_Chassis_Invert_Flag Chassis_Invert_Flag = Chassis_Invert_OFF;
    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 内部函数

    // void Power_Limit_Steer();
    // void Power_Limit_Wheel();
    // void Power_Limit();
    void Speed_Resolution();
};

/* Exported variables --------------------------------------------------------*/

// 三轮车底盘参数

// 轮组半径
const float WHEEL_RADIUS = 0.0520f;

// 轮距中心长度
const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

// 前心距中心长度
const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

// 前后轮距
const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

// 前轮距前心
const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

// 轮组方位角
const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

// 轮子直径
const float WHELL_DIAMETER = 13.200000f;

// 底盘半宽
const float HALF_WIDTH = 0.15000000f;

// 底盘半长
const float HALF_LENGTH = 0.15000000f;

// 转速转角速度	1 rpm = 2pi/60 rad/s
const float RPM2RAD = 0.104720f;

// 转速转线速度	vel = rpn*pi*D/60  cm/s
const float RPM2VEL = 0.806342f;

// 线速度转转度  //1.240168
const float VEL2RPM = 1.240168f;

// 齿轮箱减速比;
const float M3508_REDUCTION_RATIO = 13.733f;
/* Exported function declarations --------------------------------------------*/
Enum_Chassis_Invert_Flag Class_Tricycle_Chassis::Get_Chassis_Invert_Flag(void)
{
    return Chassis_Invert_Flag;
}

void Class_Tricycle_Chassis::Set_Chassis_Invert_Flag(Enum_Chassis_Invert_Flag __Chassis_Invert_Flag)
{
    Chassis_Invert_Flag = __Chassis_Invert_Flag;
}

void Class_Tricycle_Chassis::Set_Supercap_State(Enum_SUPERCAP_FLAG_E __SUPERCAP_FLAG)
{
    SUPERCAP_FLAG = __SUPERCAP_FLAG;
}

Enum_SUPERCAP_FLAG_E Class_Tricycle_Chassis::Get_Supercap_State(void)
{
    return SUPERCAP_FLAG;
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Tricycle_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Tricycle_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Tricycle_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

float Class_Tricycle_Chassis::Get_Chassis_UI_Init_flag()
{

    return (UI_INIT_FLAG);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Tricycle_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Tricycle_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

void Class_Tricycle_Chassis::Set_Chassis_UI_Init_flag(Enum_UI_INIT_FLAG_E __UI_init_flag)
{
    UI_INIT_FLAG = __UI_init_flag;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
