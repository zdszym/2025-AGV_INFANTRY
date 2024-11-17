#include "buzzer.h"
#include "SW_control_task.h"
#include "chassis_power_control.h"
steering_wheel_t steering_wheel;
steering_wheel_t *steering_wheel_p = &steering_wheel;
    int16_t probe;
void SW_control_task(void)
{

//    if (steering_wheel.parameter.enable)
//    {
        
        Steering_Wheel_CommandUpdate(&steering_wheel);
        Steering_Wheel_StatusUpdate(&steering_wheel);
        Steering_Wheel_CommandTransmit(&steering_wheel);
#ifdef AGV_BOARD_A
wheel_data_send(&wheel_data[0],&steering_wheel);

#endif
#ifdef AGV_BOARD_B
wheel_data_send(&wheel_data[1],&steering_wheel);
#endif
#ifdef AGV_BOARD_C
wheel_data_send(&wheel_data[2],&steering_wheel);
#endif
#ifdef AGV_BOARD_D
wheel_data_send(&wheel_data[3],&steering_wheel);
#endif
        // ɾ������
        // probe = steering_wheel.directive_part.motor.M3508_kit.parameter.bus->motor[1].feedback.LSB_rotor_rpm;
//    }
//    else
//    {
        //M3508_gear_set_torque_current_lsb(&steering_wheel_p->directive_part.motor.M3508_kit, 0, SEND_COMMAND_NOW);
        //M3508_gear_set_torque_current_lsb(&steering_wheel_p->motion_part.motor.M3508_kit, 0, SEND_COMMAND_NOW);
//    }
}

void SW_subscribe_task(void)
{
    steering_communication_SubscribeList_Scheduler(&steering_wheel);
}

void SW_control_task_init(void)
{
    buzzer_init_example();
    Steering_Wheel_HandleInit(&steering_wheel);
    steering_communication_init();
    wheel_data_init(&wheel_data[0],WHEEL_ID_1);
    wheel_data_init(&wheel_data[1],WHEEL_ID_2);
    wheel_data_init(&wheel_data[2],WHEEL_ID_3);
    wheel_data_init(&wheel_data[3],WHEEL_ID_4);
}
