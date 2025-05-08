/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
#ifdef CHASSIS

    // 裁判系统
    Referee.Init(&huart6);

    // 底盘
    Chassis.Referee = &Referee;
    Chassis.Init(10.0f, 10.0f, 2.0f);

    // 底盘随动PID环初始化
    PID_Chassis_Fllow.Init(20.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.05f);

#ifdef POWER_LIMIT
    // 串口超电
    Supercap.Init_UART(&huart1);
#endif

#elif defined(GIMBAL)

    Chassis.Set_Velocity_X_Max(3.5f);
    Chassis.Set_Velocity_Y_Max(3.5f);

    // 遥控器
    DR16.Init(&huart3);
    DR16_Dead_Zone = __DR16_Dead_Zone;

    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);
    FSM_Alive_Control_VT13.Chariot = this;
    FSM_Alive_Control_VT13.Init(5, 0);
    // 初始化活动控制器为无
    Active_Controller = Controller_NONE;
    // 云台
    Gimbal.Init();

    // 发射机构
    Booster.Referee = &Referee;
    Booster.Init();

    // 上位机
    MiniPC.Init(&MiniPC_USB_Manage_Object);
    MiniPC.IMU = &Gimbal.Boardc_BMI;
    MiniPC.Referee = &Referee;
    MiniPC.Gimbal_Pitch_Motor_GM6020 = &Gimbal.Motor_Pitch;
    MiniPC.Gimbal_Yaw_Motor_GM6020 = &Gimbal.Motor_Yaw;

    // select the MiniPC object
    Gimbal.MiniPC = &MiniPC;

#endif
}

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Control_RxCpltCallback()
{
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 底盘和云台夹角（弧度制）
    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y;
    // 底盘控制类型
    Enum_Chassis_Control_Type tmp_chassis_control_type;
    // 底盘角速度
    float target_omega;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(int16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(int16_t));
    memcpy(&tmp_chassis_control_type, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());

    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Chassis.Motor_Yaw.Get_Now_Angle();
    derta_angle = Chassis_Angle - Reference_Angle;

    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));

    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(tmp_chassis_control_type);

    // 底盘控制方案
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        target_omega = Chassis.Get_Spin_Omega();
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        Chassis.PID_Chassis_Fllow.Set_Target(Reference_Angle);
        Chassis.PID_Chassis_Fllow.Set_Now(Chassis.Motor_Yaw.Get_Now_Angle());
        Chassis.PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        target_omega = Chassis.PID_Chassis_Fllow.Get_Out();
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        target_omega = 0;
        chassis_velocity_x = 0;
        chassis_velocity_y = 0;
    }
    else
    {
        target_omega = 0;
    }

    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(-target_omega);
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_RxCpltCallback(uint8_t *data)
{

    Enum_Referee_Data_Robots_ID robo_id;
    Enum_Referee_Game_Status_Stage game_stage;
    uint16_t Booster_17mm_1_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t tmp_shooter_speed;
    float Shooter_Speed;

    robo_id = (Enum_Referee_Data_Robots_ID)CAN_Manage_Object->Rx_Buffer.Data[0];
    game_stage = (Enum_Referee_Game_Status_Stage)CAN_Manage_Object->Rx_Buffer.Data[1];
    memcpy(&Shooter_Barrel_Heat_Limit, CAN_Manage_Object->Rx_Buffer.Data + 2, sizeof(uint16_t));
    memcpy(&Booster_17mm_1_Heat, CAN_Manage_Object->Rx_Buffer.Data + 4, sizeof(uint16_t));
    memcpy(&tmp_shooter_speed, CAN_Manage_Object->Rx_Buffer.Data + 6, sizeof(uint16_t));
    Shooter_Speed = tmp_shooter_speed / 10.0f;
    Referee.Robot_Status.Robot_ID = robo_id;
    Referee.Game_Status.Stage_Enum = game_stage;
    Referee.Robot_Status.Booster_17mm_1_Heat_Max = Shooter_Barrel_Heat_Limit;
    Referee.Robot_Power_Heat.Booster_17mm_1_Heat = Booster_17mm_1_Heat;
    Referee.Robot_Booster.Speed = Shooter_Speed;
}

float gimbal_velocity_x = 0, gimbal_velocity_y = 0;

float testtt = 0;
/**
 * @brief can云台向底盘发送数据
 *
 */
void Class_Chariot::CAN_Gimbal_TxCpltCallback()
{
    // 云台坐标系速度目标值 float

    float relative_angle = 0;

    int chassis_velocity_x = 0, chassis_velocity_y = 0, chassis_velocity_w = 0;

    float gimbal_angle = 0, chassis_angle = 0;

    float tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0;
    // 设定速度

    gimbal_velocity_x = Gimbal.Get_Gimbal_Speed_X();
    gimbal_velocity_y = Gimbal.Get_Gimbal_Speed_Y();

    gimbal_angle = Gimbal.Get_Gimbal_Head_Angle();
    chassis_angle = Gimbal.Motor_Yaw.Get_Now_Angle();
    relative_angle = gimbal_angle - chassis_angle;
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        relative_angle += 0.005 * PI * 2 * 0.6; // 0.003可以使功率上限为80的底盘正常运动
    }

    tmp_chassis_velocity_x = gimbal_velocity_x * cos(relative_angle) + gimbal_velocity_y * sin(relative_angle);
    tmp_chassis_velocity_y = -gimbal_velocity_x * sin(relative_angle) + gimbal_velocity_y * cos(relative_angle);

    if (DR16.Get_DR16_Status() != DR16_Status_DISABLE && DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
    {
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ANTI_SPIN);
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE)
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
    }

    memcpy(CAN2_0x150_Tx_Data, &tmp_chassis_velocity_x, sizeof(float));
    memcpy(CAN2_0x150_Tx_Data + 4, &tmp_chassis_velocity_y, sizeof(float));

    /*
    需要传
    Enum_Chassis_Control_Type 2bit
    Enum_Vision_Mode 1bit
    Enum_Sprint_Status 1bit
    Enum_Booster_User_Control_Type 1bit
    Enum_Gimbal_Control_Type 2bit
    Enum_MiniPC_Status 1bit
    Enum_UI_INIT_FLAG_E 1bit
    tmp_gimbal_pitch 2byte
    */
    uint16_t flags;
    uint16_t tmp_gimbal_pitch;
    uint16_t tmp_fric_omega_left;
    uint16_t tmp_fric_omega_right;
    flags =
        (uint16_t)Chassis.Get_Chassis_Control_Type() |
        ((uint16_t)MiniPC.Get_Vision_Mode() << 2) |
        ((uint16_t)Chassis.Sprint_Status << 3) |
        ((uint16_t)Booster.Get_Booster_User_Control_Type() << 4) |
        ((uint16_t)Gimbal.Get_Gimbal_Control_Type() << 5) |
        ((uint16_t)MiniPC.Get_MiniPC_Status() << 7) |
        ((uint16_t)Chassis.Get_Chassis_UI_Init_flag() << 8) |
        ((uint16_t)Chassis.Get_Chassis_Invert_Flag() << 9) |
        ((uint16_t)MiniPC.Get_Antispin_Type() << 10);

    tmp_gimbal_pitch = Math_Float_To_Int(Gimbal.Motor_Pitch.Get_True_Angle_Pitch(), -30.0f, 30.0f, 0, 0x7FFF);
    tmp_fric_omega_left = (uint16_t)abs(Booster.Motor_Friction_Left.Get_Now_Omega());
    tmp_fric_omega_right = (uint16_t)abs(Booster.Motor_Friction_Right.Get_Now_Omega());

    memcpy(CAN2_0x152_Tx_Data, &flags, sizeof(flags));
    memcpy(CAN2_0x152_Tx_Data + 2, &tmp_gimbal_pitch, sizeof(tmp_gimbal_pitch));
    memcpy(CAN2_0x152_Tx_Data + 4, &tmp_fric_omega_left, sizeof(tmp_fric_omega_left));
    memcpy(CAN2_0x152_Tx_Data + 6, &tmp_fric_omega_right, sizeof(tmp_fric_omega_right));

    // CAN2_0x152_Tx_Data[0] = Chassis.Get_Chassis_Control_Type();
    // CAN2_0x152_Tx_Data[1] = MiniPC.Get_Vision_Mode();
    // CAN2_0x152_Tx_Data[2] = Chassis.Sprint_Status;
    // CAN2_0x152_Tx_Data[3] = Booster.Get_Booster_User_Control_Type(); // 摩擦轮状态
    // CAN2_0x152_Tx_Data[4] = Gimbal.Get_Gimbal_Control_Type();        // 云台状态
    // CAN2_0x152_Tx_Data[5] = MiniPC.Get_MiniPC_Status();
    // CAN2_0x152_Tx_Data[6] = Booster.Get_Booster_Jamming_Type();
    // CAN2_0x152_Tx_Data[7] = Chassis.Get_Chassis_UI_Init_flag();
}

#endif

/**
 * @brief 判断当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
    // 检查DR16是否有输入
    Judge_DR16_Control_Type();

    // 检查VT13是否有输入
    Judge_VT13_Control_Type();

    // 判断当前活动的控制器
    if (VT13_Control_Type != VT13_Control_Type_NONE)
    {
        Active_Controller = Controller_VT13;
    }
    else if (DR16_Control_Type != DR16_Control_Type_NONE)
    {
        Active_Controller = Controller_DR16;
    }
    else
    {
        Active_Controller = Controller_NONE;
    }
}

/**
 * @brief 底盘控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y;
    float vt13_l_x = 0, vt13_l_y = 0;
    // 云台坐标系速度目标值 float
    float gimbal_velocity_x = 0, gimbal_velocity_y = 0;
    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        Chassis.Sprint_Status = Sprint_Status_ENABLE;
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ANTI_SPIN);
            }
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        Chassis.Sprint_Status = Sprint_Status_ENABLE;
        if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        // 排除遥控器死区
        vt13_l_x = (Math_Abs(VT13.Get_Left_X()) > DR16_Dead_Zone) ? VT13.Get_Left_X() : 0;
        vt13_l_y = (Math_Abs(VT13.Get_Left_Y()) > DR16_Dead_Zone) ? VT13.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        gimbal_velocity_x = vt13_l_x * sqrt(1.0f - vt13_l_y * vt13_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        gimbal_velocity_y = vt13_l_y * sqrt(1.0f - vt13_l_x * vt13_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (VT13.Get_Switch() == VT13_Switch_Status_Left)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Middle)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {
            if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED)
            {
                Chassis.Sprint_Status = Sprint_Status_ENABLE;
                // Chassis.Set_Supercap_State(SUPERCAP_ON);
            }
            else
            {
                Chassis.Sprint_Status = Sprint_Status_DISABLE;
                Chassis.Set_Supercap_State(SUPERCAP_OFF);
            }
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
            {
                gimbal_velocity_x -= Chassis.Get_Velocity_X_Max();
            }
            if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
            {
                gimbal_velocity_x += Chassis.Get_Velocity_X_Max();
            }
            if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
            {
                gimbal_velocity_y += Chassis.Get_Velocity_Y_Max();
            }
            if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
            {
                gimbal_velocity_y -= Chassis.Get_Velocity_Y_Max();
            }

            if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (DR16.Get_Keyboard_Key_R() == DR16_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_ON);
            }
            else
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_OFF);
            }
        }
        else if (Active_Controller == Controller_VT13)
        {
            if (VT13.Get_Keyboard_Key_Shift() == VT13_Key_Status_PRESSED) // 按住shift加速
            {
                Chassis.Sprint_Status = Sprint_Status_ENABLE;
                // Chassis.Set_Supercap_State(SUPERCAP_ON);
            }
            else
            {
                Chassis.Sprint_Status = Sprint_Status_DISABLE;
                Chassis.Set_Supercap_State(SUPERCAP_OFF);
            }
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }
            if (VT13.Get_Keyboard_Key_A() == VT13_Key_Status_PRESSED) // x轴
            {
                gimbal_velocity_x -= Chassis.Get_Velocity_X_Max();
            }
            if (VT13.Get_Keyboard_Key_D() == VT13_Key_Status_PRESSED)
            {
                gimbal_velocity_x += Chassis.Get_Velocity_X_Max();
            }
            if (VT13.Get_Keyboard_Key_W() == VT13_Key_Status_PRESSED) // y轴
            {
                gimbal_velocity_y += Chassis.Get_Velocity_Y_Max();
            }
            if (VT13.Get_Keyboard_Key_S() == VT13_Key_Status_PRESSED)
            {
                gimbal_velocity_y -= Chassis.Get_Velocity_Y_Max();
            }

            if (VT13.Get_Keyboard_Key_E() == VT13_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (VT13.Get_Keyboard_Key_R() == VT13_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_ON);
            }
            else
            {
                Chassis.Set_Chassis_UI_Init_flag(UI_INIT_OFF);
            }
        }
    }

    Gimbal.Set_Gimbal_Speed_X(gimbal_velocity_x);
    Gimbal.Set_Gimbal_Speed_Y(gimbal_velocity_y);
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{

    // 角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch;
    // 遥控器摇杆值
    float dr16_y = 0, dr16_r_y = 0;
    float vt13_y = 0, vt13_r_y = 0;
    // 获取当前角度值
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下自瞄
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
            tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
            // 遥控器操作逻辑
            tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * 5;
            tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Resolution * 10;
        }
        else // 非自瞄模式
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // 遥控器操作逻辑
            tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * 10;
            tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Resolution * 10;
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > DR16_Dead_Zone) ? VT13.Get_Right_Y() : 0;

        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_DISABLE)
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        if (VT13.Get_Button_Right() == VT13_Button_TRIG_FREE_PRESSED) //
        {
            if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
                tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
            }
            else if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC)
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }

        // 遥控器操作逻辑
        tmp_gimbal_yaw -= vt13_y * DR16_Yaw_Angle_Resolution;
        tmp_gimbal_pitch += vt13_r_y * DR16_Pitch_Angle_Resolution;
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {
            if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_pitch = 0;
            }

            // 长按右键  开启自瞄
            if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
                    {
                        tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                        tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    }
                }
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;

            // C键按下 一键调头
            if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
                if (Chassis.Get_Chassis_Invert_Flag() == Chassis_Invert_OFF)
                    Chassis.Set_Chassis_Invert_Flag(Chassis_Invert_ON);
                else
                {
                    Chassis.Set_Chassis_Invert_Flag(Chassis_Invert_OFF);
                }
            }
            // V键按下 自瞄模式中切换四点和五点模式
            if (DR16.Get_Keyboard_Key_V() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
                {
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
                }
                else
                {
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
                }
            }
            // Z键按下 切换反小陀螺开关
            if (DR16.Get_Keyboard_Key_Z() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.MiniPC->Get_Antispin_Type() == Antispin_On)
                {
                    Gimbal.MiniPC->Set_Antispin_Type(Antispin_Off);
                }
                else
                {
                    Gimbal.MiniPC->Set_Antispin_Type(Antispin_On);
                }
            }
        }
        else if (Active_Controller == Controller_VT13)
        {
            if (VT13.Get_Keyboard_Key_Q() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_pitch = 0;
            }

            // 长按右键  开启自瞄
            if (VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
                    {
                        tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                        tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    }
                }
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            tmp_gimbal_yaw -= VT13.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += VT13.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;

            // C键按下 一键调头
            if (VT13.Get_Keyboard_Key_C() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
                if (Chassis.Get_Chassis_Invert_Flag() == Chassis_Invert_OFF)
                    Chassis.Set_Chassis_Invert_Flag(Chassis_Invert_ON);
                else
                {
                    Chassis.Set_Chassis_Invert_Flag(Chassis_Invert_OFF);
                }
            }
            // Z键按下 切换反小陀螺开关
            if (VT13.Get_Keyboard_Key_Z() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.MiniPC->Get_Antispin_Type() == Antispin_On)
                {
                    Gimbal.MiniPC->Set_Antispin_Type(Antispin_Off);
                }
                else
                {
                    Gimbal.MiniPC->Set_Antispin_Type(Antispin_On);
                }
            }
            // V键按下 自瞄模式中切换四点和五点模式
            if (VT13.Get_Keyboard_Key_V() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
                {
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
                }
                else
                {
                    Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
                }
            }
        }
    }
    else if (DR16.Get_DR16_Status() == DR16_Status_DISABLE && VT13.Get_VT13_Status() == VT13_Status_DISABLE) // 失能
    {
        // 遥控器离线
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    }

    // 设定角度
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}

/*上位机和正常公用云台的tmp，*/
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
uint8_t test_cnt = 0;
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 左上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {

            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);

            if (DR16.Get_Yaw() > -0.2f && DR16.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (DR16.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (DR16.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {

        // 开启摩擦轮和发射机构
        if (VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED)
        {
            test_cnt++;
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            }
        }
        if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
        {
            if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {
            if (DR16.Get_Keyboard_Key_B() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
                }
                else
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
                }
            }
            // 按下ctrl键 开启摩擦轮
            if (DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_DISABLE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                }
            }

            // 按下鼠标左键 单发
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    }
                }
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                {
                    if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    }
                    else
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                }
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }

        if (Active_Controller == Controller_VT13)
        {
            if (VT13.Get_Keyboard_Key_B() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
                }
                else
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
                }
            }
            // 按下ctrl键 开启摩擦轮
            if (VT13.Get_Keyboard_Key_Ctrl() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_DISABLE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                }
            }

            // 按下鼠标左键 单发

            // 按下鼠标左键 单发
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_TRIG_FREE_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    }
                }
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    }
                    else
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                }
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }
    }
}
#endif

/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS

    // 各个模块的分别解算
    Chassis.TIM_Calculate_PeriodElapsedCallback();

    Supercap.Set_Now_Power(Chassis.Referee->Get_Chassis_Power());
    Supercap.Set_Limit_Power(Chassis.Referee->Get_Chassis_Power_Max());
    Supercap.TIM_UART_PeriodElapsedCallback();

#elif defined(GIMBAL)

    // 各个模块的分别解算

    Gimbal.TIM_Calculate_PeriodElapsedCallback();

    Booster.TIM_Calculate_PeriodElapsedCallback();
    // 传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();

    this->CAN_Gimbal_TxCpltCallback();

#endif
}
/**
 * @brief 判断DR16控制数据来源
 *
 */
#ifdef GIMBAL
void Class_Chariot::Judge_DR16_Control_Type()
{
    if (DR16.Get_Left_X() != 0 ||
        DR16.Get_Left_Y() != 0 ||
        DR16.Get_Right_X() != 0 ||
        DR16.Get_Right_Y() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_REMOTE;
    }
    else if (DR16.Get_Mouse_X() != 0 ||
             DR16.Get_Mouse_Y() != 0 ||
             DR16.Get_Mouse_Z() != 0 ||
             DR16.Get_Keyboard_Key_A() != 0 ||
             DR16.Get_Keyboard_Key_D() != 0 ||
             DR16.Get_Keyboard_Key_W() != 0 ||
             DR16.Get_Keyboard_Key_S() != 0 ||
             DR16.Get_Keyboard_Key_Shift() != 0 ||
             DR16.Get_Keyboard_Key_Ctrl() != 0 ||
             DR16.Get_Keyboard_Key_Q() != 0 ||
             DR16.Get_Keyboard_Key_E() != 0 ||
             DR16.Get_Keyboard_Key_R() != 0 ||
             DR16.Get_Keyboard_Key_F() != 0 ||
             DR16.Get_Keyboard_Key_G() != 0 ||
             DR16.Get_Keyboard_Key_Z() != 0 ||
             DR16.Get_Keyboard_Key_C() != 0 ||
             DR16.Get_Keyboard_Key_V() != 0 ||
             DR16.Get_Keyboard_Key_B() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
    else
    {
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            DR16_Control_Type = DR16_Control_Type_NONE;
    }
}

void Class_Chariot::Judge_VT13_Control_Type()
{
    if (VT13.Get_Left_X() != 0 ||
        VT13.Get_Left_Y() != 0 ||
        VT13.Get_Right_X() != 0 ||
        VT13.Get_Right_Y() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_REMOTE;
    }
    else if (VT13.Get_Mouse_X() != 0 ||
             VT13.Get_Mouse_Y() != 0 ||
             VT13.Get_Mouse_Z() != 0 ||
             VT13.Get_Keyboard_Key_A() != 0 ||
             VT13.Get_Keyboard_Key_D() != 0 ||
             VT13.Get_Keyboard_Key_W() != 0 ||
             VT13.Get_Keyboard_Key_S() != 0 ||
             VT13.Get_Keyboard_Key_Shift() != 0 ||
             VT13.Get_Keyboard_Key_Ctrl() != 0 ||
             VT13.Get_Keyboard_Key_Q() != 0 ||
             VT13.Get_Keyboard_Key_E() != 0 ||
             VT13.Get_Keyboard_Key_R() != 0 ||
             VT13.Get_Keyboard_Key_F() != 0 ||
             VT13.Get_Keyboard_Key_G() != 0 ||
             VT13.Get_Keyboard_Key_Z() != 0 ||
             VT13.Get_Keyboard_Key_C() != 0 ||
             VT13.Get_Keyboard_Key_V() != 0 ||
             VT13.Get_Keyboard_Key_B() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_KEYBOARD;
    }
    else
    {
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            VT13_Control_Type = VT13_Control_Type_NONE;
    }
}
#endif
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    // 判断DR16控制数据来源
    Judge_DR16_Control_Type();
    Judge_VT13_Control_Type();
    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif
/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static int mod50 = 0;
    mod50++;
    if (mod50 == 100)
    {
#ifdef CHASSIS

        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

#ifdef POWER_LIMIT
        Supercap.TIM_Alive_PeriodElapsedCallback();
#endif

        for (auto &wheel : Chassis.Motor_Wheel)
        {
            wheel.TIM_Alive_PeriodElapsedCallback();
        }

#elif defined(GIMBAL)
        VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE && VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Pitch_LK6010.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();

        MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

#endif

        mod50 = 0;
    }
}

/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    // 离线检测状态
    case (0):
    {
        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }

        // 转移为 在线状态
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
        if (Status[Now_Status_Serial].Time > 1000)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(1);
        }
    }
    break;
    // 遥控器关闭状态
    case (1):
    {
        // 离线保护
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
            Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }
    }
    break;
    // 遥控器在线状态
    case (2):
    {
        // 转移为 刚离线状态
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(3);
        }
    }
    break;
    // 刚离线状态
    case (3):
    {
        // 记录离线检测前控制模式
        Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
        Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

        // 无条件转移到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    // 遥控器串口错误状态
    case (4):
    {
        HAL_UART_DMAStop(&huart3); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}

void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    // 离线检测状态
    case (0):
    {
        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart6.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }

        // 转移为 在线状态
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
        if (Status[Now_Status_Serial].Time > 1000)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(1);
        }
    }
    break;
    // 遥控器关闭状态
    case (1):
    {
        // 离线保护
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
        {
            Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
            Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart6.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }
    }
    break;
    // 遥控器在线状态
    case (2):
    {
        // 转移为 刚离线状态
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(3);
        }
    }
    break;
    // 刚离线状态
    case (3):
    {
        // 记录离线检测前控制模式
        Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
        Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

        // 无条件转移到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    // 遥控器串口错误状态
    case (4):
    {
        HAL_UART_DMAStop(&huart6); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
