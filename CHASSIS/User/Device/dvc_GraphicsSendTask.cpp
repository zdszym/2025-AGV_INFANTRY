/**********************************************************************************************************
 * @�ļ�     Graphics_Send.c
 * @˵��     ����ϵͳͼ�η���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2023.5.1
 **********************************************************************************************************/
#include "dvc_GraphicsSendTask.h"
#include <stm32f4xx.h>
#include <string.h>
#include "usart.h"
#include <stdio.h>

#define CAP_GRAPHIC_NUM 9 // �������ݵ���ͼ����ʾϸ�ָ���
#define Robot_ID 46
unsigned char JudgeSend[SEND_MAX_SIZE];
JudgeReceive_t JudgeReceiveData;
JudgeReceive_t Last_JudgeReceiveData;
// extern SuperPower superpower;
F405_typedef F405;
#define Robot_ID 46

int pitch_change_flag;
int cap_percent_change_flag;
int BigFrictSpeed_change_flag;
int Pitch_change_flag;
int vol_change_array[CAP_GRAPHIC_NUM];
float last_cap_vol;
short lastBigFrictSpeed;

/**********************************************************************************************************
 * @�ļ�     Graphics_Send.c
 * @����     2023.4


�ο���Robomaster ����Э�鸽¼v1.4



����ϵͳͨ��Э��

	֡�ײ�					����id(����UI��0x0301)		���ݶΣ��ײ�+���ݣ�			β��2�ֽ�У��λ CRC16
*********************		*********************		*********************		*********************
*					*		*					*		*					*		*					*
*	frame_header	*		*	cmd_id			*		*	data			*		*	frame_tail		*
*	(5 bytes)		*	+	*	(2 bytes)		*	+	*	(n bytes)		*	+	*	(2 bytes)		*
*					*		*					*		*					*		*	  				*
*********************		*********************		*********************		*********************



**********************************************************************************************************/

/*			��������				*/
uint8_t Transmit_Pack[128];				   // ����ϵͳ����֡
uint8_t data_pack[DRAWING_PACK * 7] = {0}; // ���ݶβ���
uint8_t DMAsendflag;
/**********************************************************************************************************
 *�� �� ��: Send_UIPack
 *����˵��: ��������UI���ݰ������ݶ��ײ������ݣ�x
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/

void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID, uint16_t receiverID, uint8_t *data, uint16_t pack_len)
{
	student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = data_cmd_id;
	custom_interactive_header.send_ID = SendID;
	custom_interactive_header.receiver_ID = receiverID;

	uint8_t header_len = sizeof(custom_interactive_header); // ���ݶ��ײ�����

	memcpy((void *)(Transmit_Pack + 7), &custom_interactive_header, header_len); // �����ݶε����ݶν��з�װ����װ���ף�
	memcpy((void *)(Transmit_Pack + 7 + header_len), data, pack_len);			 // ������֡�����ݶν��з�װ����װ���ݣ�

	Send_toReferee(0x0301, pack_len + header_len); // �����������֡����
}

/**********************************************************************************************************
 *�� �� ��: Send_toReferee
 *����˵��: �������֡�����͸�����ϵͳ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Send_toReferee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len;

	// ֡�ײ���װ
	{
		Transmit_Pack[0] = 0xA5;
		memcpy(&Transmit_Pack[1], (uint8_t *)&data_len, sizeof(data_len)); // ���ݶ���data�ĳ���
		Transmit_Pack[3] = seq++;
		Append_CRC8_Check_Sum(Transmit_Pack, HEADER_LEN); // ֡ͷУ��CRC8
	}

	// ����ID
	memcpy(&Transmit_Pack[HEADER_LEN], (uint8_t *)&cmd_id, CMD_LEN);

	// β������У��CRC16
	Append_CRC16_Check_Sum(Transmit_Pack, Frame_Length);

	// 对于状态变化类消息，增加发送次数为3次，提高可靠性
	uint8_t send_cnt = (cmd_id == Drawing_Char_ID) ? 3 : 1;
	while (send_cnt)
	{
		send_cnt--;
		// 将超时时间从5ms增加到50ms，提高通信稳定性
		HAL_UART_Transmit(&huart6, (uint8_t *)Transmit_Pack, Frame_Length, 10);
		DMAsendflag = 1;

		// 添加短暂延时，避免连续发送导致丢包
		if (send_cnt > 0)
		{
			for (volatile uint16_t i = 0; i < 1000; i++)
				; // 简单延时
		}
	}
}

/**********************************************************************************************************
 *�� �� ��: Deleta_Layer
 *����˵��: ���ͼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Deleta_Layer(uint8_t layer, uint8_t deleteType)
{
	static client_custom_graphic_delete_t Delete_Graphic; // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	Delete_Graphic.layer = layer;
	Delete_Graphic.operate_tpye = deleteType;
	Send_UIPack(Drawing_Delete_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, (uint8_t *)&Delete_Graphic, sizeof(Delete_Graphic)); // ���ַ�
}

/**********************************************************************************************************
 *�� �� ��: CharGraphic_Draw
 *����˵��: �õ��ַ�ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *CharGraphic_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[])
{

	static graphic_data_struct_t drawing;  // ����Ϊ��̬��������������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CHAR; // 7Ϊ�ַ�����
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;

	drawing.start_angle = size; // �����С
	drawing.end_angle = len;	// �ַ�����
	drawing.width = line_width;

	for (uint8_t i = DRAWING_PACK; i < DRAWING_PACK + 30; i++)
		data_pack[i] = 0;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Char_Draw
 *����˵��: �����ַ�
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Char_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[], uint8_t *str_data)
{
	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = CharGraphic_Draw(0, Op_Type, startx, starty, size, len, line_width, color, name);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	memset(&data_pack[DRAWING_PACK], 0, 30);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)str_data, len);
	Send_UIPack(Drawing_Char_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK + 30); // �����ַ�
}

/**********************************************************************************************************
 *�� �� ��: FloatData_Draw
 *����˵��: �õ����Ƹ���ͼ�νṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *FloatData_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, float data_f, uint8_t size, uint8_t valid_bit, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing; // ����Ϊ��̬��������������������ʱ�����ٸñ����ڴ�
	static int32_t Data1000;
	Data1000 = (int32_t)(data_f * 1000);
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_FLOAT; // 5Ϊ��������
	drawing.width = line_width;		   // �߿�
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;	   // �����С
	drawing.end_angle = valid_bit; // ��Чλ��

	drawing.radius = Data1000 & 0x03ff;
	drawing.end_x = (Data1000 >> 10) & 0x07ff;
	drawing.end_y = (Data1000 >> 21) & 0x07ff;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Line_Draw
 *����˵��: ֱ��ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Line_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_LINE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.end_x = endx;
	drawing.end_y = endy;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Rectangle_Draw
 *����˵��: ����ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Rectangle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_RECTANGLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.end_x = endx;
	drawing.end_y = endy;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Circle_Draw
 *����˵��: Բ��ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Circle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint32_t radius, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CIRCLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.radius = radius;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Lanelines_Init
 *����˵��: �����߳�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	graphic_data_struct_t *P_graphic_data;
	// ��һ��������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.41, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.31, 0, 4, Orange, LaneLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	// �ڶ���������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.59, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.69, 0, 4, Orange, LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // ������ͼ��
}

/**********************************************************************************************************
 *�� �� ��: Shootlines_Init
 *����˵��: ǹ�ڳ�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void ShootLines_Init(void)
{
	static uint8_t ShootLineName1[] = "SL1";
	static uint8_t ShootLineName2[] = "SL2";
	static uint8_t ShootLineName3[] = "SL3";
	static uint8_t ShootLineName4[] = "SL4";
	static uint8_t ShootLineName5[] = "SL5";
	static uint8_t ShootLineName6[] = "SL6";
	static uint8_t ShootLineName7[] = "SL7";
	graphic_data_struct_t *P_graphic_data;

#if Robot_ID == 44
	// 四米射击点
	P_graphic_data = Circle_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 19, SCREEN_WIDTH * 0.5 - 12, 2, 4, Green, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	//    //四米射击圈
	//	P_graphic_data = Circle_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-14, SCREEN_WIDTH * 0.5-10 , 40 , 3 , Green , ShootLineName2);
	//	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	// 4m横线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 41, SCREEN_WIDTH * 0.5 - 12, SCREEN_LENGTH * 0.5 - 27, SCREEN_WIDTH * 0.5 - 12, 1, Green, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 11, SCREEN_WIDTH * 0.5 - 12, SCREEN_LENGTH * 0.5 + 3, SCREEN_WIDTH * 0.5 - 12, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 4m竖线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 19, SCREEN_WIDTH * 0.5 - 4, SCREEN_LENGTH * 0.5 - 19, SCREEN_WIDTH * 0.5 + 10, 1, Green, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 19, SCREEN_WIDTH * 0.5 - 20, SCREEN_LENGTH * 0.5 - 19, SCREEN_WIDTH * 0.5 - 34, 1, Green, ShootLineName6);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 凑数
	P_graphic_data = Line_Draw(1, Op_None, 0, 0, SCREEN_LENGTH * 0.5 - 14, SCREEN_WIDTH * 0.5 - 24, 1, Green, ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // 画两个图形
#elif Robot_ID == 45
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 40, SCREEN_WIDTH * 0.5 - 72, SCREEN_LENGTH * 0.5 + 40, SCREEN_WIDTH * 0.5 - 72, 1, Green, ShootLineName3);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5, SCREEN_WIDTH * 0.5 - 40, SCREEN_LENGTH * 0.5, SCREEN_WIDTH * 0.5 - 112, 1, Green, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 30, SCREEN_WIDTH * 0.5 - 92, SCREEN_LENGTH * 0.5 + 30, SCREEN_WIDTH * 0.5 - 92, 1, Green, ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 20, SCREEN_WIDTH * 0.5 - 112, SCREEN_LENGTH * 0.5 + 20, SCREEN_WIDTH * 0.5 - 112, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5); // 画两个图形
#elif Robot_ID == 46
	float x_bias = 0;
	float y_bias = 0;
	// 射击横线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, SCREEN_LENGTH * 0.5 + 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, 1, Green, ShootLineName3);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, SCREEN_LENGTH * 0.5 + 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, 1, Green, ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Circle_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, 9, 4, Red_Blue, ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK); // 发射UI中间加一个圆圈辅助瞄准

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, SCREEN_LENGTH * 0.5 + 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 射击竖线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 40 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5); // 画两个图形
#elif Robot_ID == 47
	float x_bias = +14;
	float y_bias = -30;

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, SCREEN_LENGTH * 0.5 + 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, 1, Green, ShootLineName3);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 40 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 117 + y_bias, 1, Green, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	//    P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.5-30+x_bias, SCREEN_WIDTH * 0.5-102+y_bias, SCREEN_LENGTH * 0.5+30+x_bias , SCREEN_WIDTH * 0.5-102+y_bias ,1 , Green , ShootLineName1);
	//	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 20 + x_bias, SCREEN_WIDTH * 0.5 - 117 + y_bias, SCREEN_LENGTH * 0.5 + 20 + x_bias, SCREEN_WIDTH * 0.5 - 117 + y_bias, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 35 + x_bias, SCREEN_WIDTH * 0.5 - 87 + y_bias, SCREEN_LENGTH * 0.5 + 35 + x_bias, SCREEN_WIDTH * 0.5 - 87 + y_bias, 1, Green, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5); // 画两个图形
#endif
}

/**********************************************************************************************************
 *�� �� ��: CarPosture_Change
 *����˵��: ������̬����
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint16_t RectCenterX = SCREEN_LENGTH * 0.4;
uint16_t RectCenterY = SCREEN_WIDTH * 0.7;
uint16_t startX, startY, endX, endY;
float angle;
float angle1;
// 修改函数声明，添加夹角参数
void CarPosture_Change(short Yaw_100, uint8_t Init_Cnt)
{
	static uint8_t LaneLineName1[] = "po1";
	static uint8_t LaneLineName2[] = "po2";
	static uint8_t LaneLineName3[] = "po3";
	static uint8_t LaneLineName4[] = "po4";
	static uint8_t LaneLineName5[] = "po5";
	static uint8_t LaneLineName6[] = "po6";
	static uint8_t LaneLineName7[] = "po7";
	static uint8_t AngleName[] = "ang";
	graphic_data_struct_t *P_graphic_data;

	static uint16_t len = 50;
	static uint16_t centerx = 200, centery = 700;
	angle = (Yaw_100 / 100.0f) * PI / 180.0f + PI;
	angle1 = (Yaw_100 / 100.0f);

	if (angle1 < 0)
		angle1 += 360;
	uint8_t optype = Init_Cnt == 0 ? Op_Change : Op_Add;

	// 获取云台与底盘夹角（角度制）
	float chassis_gimbal_diff = JudgeReceiveData.Chassis_Gimbal_Diff;

	// 绘制云台朝向（固定向上）
	P_graphic_data = Line_Draw(0, optype, centerx,
							   centery,
							   centerx,
							   centery + len, 4, Green, LaneLineName2);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 绘制底盘朝向（根据与云台的夹角旋转）
	float chassis_angle_rad = chassis_gimbal_diff * PI / 180.0f;
	P_graphic_data = Line_Draw(0, optype, centerx,
							   centery,
							   centerx + len * sin(chassis_angle_rad),
							   centery + len * cos(chassis_angle_rad), 4, Orange, LaneLineName3);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 显示角度值
	static uint8_t angle_str[8];
	snprintf((char *)angle_str, sizeof(angle_str), "%.1f", chassis_gimbal_diff);
	Char_Draw(0, optype, centerx - 20, centery - 30, 20, strlen((char *)angle_str), 2, Yellow, AngleName, angle_str);

	P_graphic_data = Line_Draw(0, optype, 200, 700, 200, 800, 8, Pink, LaneLineName6); // 枪口标识线
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
}
void FrictSpeed_Draw(uint16_t omega_left, uint16_t omega_right, uint8_t Init_Cnt)
{
	static uint8_t FricSpeedValueName_left[] = "fsl";  // 数值的独立名称
	static uint8_t FricSpeedValueName_right[] = "fsr"; // 数值的独立名称
	static uint16_t last_omega_left = 0;			   // 添加静态变量记录上次的值
	static uint16_t last_omega_right = 0;
	uint8_t optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 处理低转速情况，当转速低于阈值时认为已停止
	if (omega_left < 200)
		omega_left = 0;
	if (omega_right < 200)
		omega_right = 0;

	// 恢复条件更新逻辑，确保数值变化时才更新显示
	if (abs((int)last_omega_left - (int)omega_left) > 10 ||
		abs((int)last_omega_right - (int)omega_right) > 10 ||
		(last_omega_left > 0 && omega_left == 0) ||
		(last_omega_left == 0 && omega_left > 0) ||
		(last_omega_right > 0 && omega_right == 0) ||
		(last_omega_right == 0 && omega_right > 0) ||
		Init_Cnt > 0)
	{
		// 将整数转换为字符串
		uint8_t speed_str_left[8];
		snprintf((char *)speed_str_left, sizeof(speed_str_left), "%d", omega_left);
		uint8_t speed_str_right[8];
		snprintf((char *)speed_str_right, sizeof(speed_str_right), "%d", omega_right);

		// 动态更新数值
		if (omega_left < 600)
		{
			Char_Draw(0, optype,
					  0.9 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, // 数值位置 (X, Y)
					  20, strlen((char *)speed_str_left), 2,
					  Green, FricSpeedValueName_left, speed_str_left);
		}
		else
		{
			Char_Draw(0, optype,
					  0.9 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, // 数值位置 (X, Y)
					  20, strlen((char *)speed_str_left), 2,
					  Orange, FricSpeedValueName_left, speed_str_left);
		}

		// if (omega_right < 600)
		// {
		// 	Char_Draw(0, optype, 0.933 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, 20, strlen((char *)speed_str_right), 2, Green, FricSpeedValueName_right, speed_str_right);
		// }
		// else
		// {
		// 	Char_Draw(0, optype, 0.933 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, 20, strlen((char *)speed_str_right), 2, Orange, FricSpeedValueName_right, speed_str_right);
		// }

		// 更新记录的值
		last_omega_left = omega_left;
		last_omega_right = omega_right;
	}
}
/*�������ݵ�������*/
void CapDraw(float CapVolt, uint8_t Init_Flag)
{
	static float Length;
	static uint8_t CapName1[] = "Out";
	static uint8_t CapName2[] = "In";

	graphic_data_struct_t *P_graphic_data;
	if (Init_Flag)
	{
		P_graphic_data = Rectangle_Draw(0, Op_Add, 0.3495 * SCREEN_LENGTH, 0.1125 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.1385 * SCREEN_WIDTH, 5, Cyan, CapName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, Op_Add, 0.35 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 0.65 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 27, Green, CapName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2);
	}
	else
	{
		Length = CapVolt * (0.3 * SCREEN_LENGTH);
		P_graphic_data = Line_Draw(0, Op_Change, 0.35 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 0.35 * SCREEN_LENGTH + Length, 0.125 * SCREEN_WIDTH, 27, Green, CapName2);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
	}
}

/*字符变化发送*/
void CharChange(uint8_t Init_Flag)
{
	uint8_t BulletOff[] = "OFF";
	uint8_t BulletOn[] = "ON";

	uint8_t FrictionOff[] = "OFF";
	uint8_t FrictionOn[] = "ON";

	uint8_t AutoLost[] = "LOST";
	uint8_t AutoOn[] = "ON";

	uint8_t FireAuto[] = "AUTO";
	uint8_t FireManual[] = "MANUAL";

	uint8_t SPIN[] = "SPIN";
	uint8_t FOLLOW[] = "FOLLOW";
	uint8_t Chassis_Off[] = "OFF";

	uint8_t INIT[] = "INIT";

	uint8_t JAMM[] = "JAMMING!!!";

	/*弹舱状态改变*/
	// static uint8_t BulletChangeName[] = "bul";
	// if (Init_Flag)
	// {
	// 	Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(INIT), 2, Green, BulletChangeName, INIT);
	// }
	// else
	// {
	// 	switch (JudgeReceiveData.Bullet_Status)
	// 	{
	// 	case 1:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(BulletOn), 2, Green, BulletChangeName, BulletOn);
	// 		break;
	// 	case 0:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(BulletOff), 2, Pink, BulletChangeName, BulletOff);
	// 		break;
	// 	}
	// }

	/*冲刺状态*/

	/*摩擦轮状态改变*/
	// static uint8_t FrictionChangeName[] = "mcl";
	// if (Init_Flag)
	// {
	// 	Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOff), 2, Pink, FrictionChangeName, INIT);
	// }
	// else
	// {
	// 	switch (JudgeReceiveData.Fric_Status)
	// 	{
	// 	case 0:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOff), 2, Pink, FrictionChangeName, FrictionOff);
	// 		break;
	// 	case 1:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOn), 2, Green, FrictionChangeName, FrictionOn);
	// 		break;
	// 	}
	// }

	/*自瞄状态改变*/
	// static uint8_t AutoChangeName[] = "aim";
	// if (Init_Flag)
	// {
	// 	Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(INIT), 2, Pink, AutoChangeName, INIT);
	// }
	// else
	// {
	// 	switch (JudgeReceiveData.MiniPC_Aim_Status)
	// 	{
	// 	case 0:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(AutoLost), 2, Pink, AutoChangeName, AutoLost);
	// 		break;
	// 	case 1:
	// 		Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(AutoOn), 2, Green, AutoChangeName, AutoOn);
	// 		break;
	// 	}
	// }
	static uint8_t optype;
	/*底盘状态改变*/
	static uint8_t ChassisChangeName[] = "cha";
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	switch (JudgeReceiveData.Chassis_Control_Type)
	{
	case 0:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(Chassis_Off), 2, Pink, ChassisChangeName, Chassis_Off);
		break;
	case 1:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(FOLLOW), 2, Green, ChassisChangeName, FOLLOW);
		break;
	case 2:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(SPIN), 2, Orange, ChassisChangeName, SPIN);
		break;
	}
}

/**********************************************************************************************************
 *�� �� ��: Char_Init
 *����˵��: �ַ����ݳ�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Char_Init(void)
{
	static uint8_t PitchName[] = "pit";
	static uint8_t GimbalName[] = "gim";
	static uint8_t FrictionName[] = "fri";
	static uint8_t AutoName[] = "aim";
	static uint8_t CapStaticName[] = "cpt";
	static uint8_t FireName[] = "frm";
	static uint8_t FricSpeedName[] = "fsp";
	static uint8_t GimbalStatusLabelName[] = "gsl"; // 云台状态标签名称
	static uint8_t BoosterModeLabelName[] = "bml";	// 发射机构模式标签名称

	//	/*              PITCH�ַ�*/
	//	uint8_t pitch_char[] = "PITCH :";
	//	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, 20, sizeof(pitch_char), 2, Yellow, PitchName, pitch_char);

	//	/*              GIMBAL�ַ�*/
	//	uint8_t bullet_char[] = "BULLET :";
	//	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(bullet_char), 2, Yellow, GimbalName, bullet_char);

	//	/*              FRICTION�ַ�*/
	//	uint8_t friction_char[] = "FRICTION :";
	//	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(friction_char), 2, Yellow, FrictionName, friction_char);

	//	/*              ARMOR�ַ�*/
	//	uint8_t auto_char[] = "AUTO :";
	//	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(auto_char), 2, Yellow, AutoName, auto_char);

	/*              FIREMODE�ַ�*/
	uint8_t fire_char[] = "CHASSIS :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(fire_char), 2, Yellow, FireName, fire_char);

	/*              CAP�ַ�*/
	uint8_t cap_char[] = "CAP :      %";
	Char_Draw(0, Op_Add, 0.40 * SCREEN_LENGTH, 0.1 * SCREEN_WIDTH, 30, sizeof(cap_char), 2, Yellow, CapStaticName, cap_char);

	uint8_t fric_speed_label[] = "OMEGA :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, 20, sizeof(fric_speed_label), 2, Yellow, FricSpeedName, fric_speed_label);

	/*              BOOSTER MODE�ַ�*/
	uint8_t booster_mode_label[] = "BOOSTER:";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(booster_mode_label), 2, Yellow, BoosterModeLabelName, booster_mode_label);

	/*              GIMBAL状态标签            */
	uint8_t gimbal_status_label[] = "GIMBAL :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(gimbal_status_label), 2, Yellow, GimbalStatusLabelName, gimbal_status_label);
}

void MiniPC_Aim_Change(uint8_t Init_Cnt)
{
	/*自瞄获取状态*/
	static uint8_t Auto_Aim_ChangeName[] = "Aim";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Minipc_Status)
	{
	case 1:
		P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.3 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.8 * SCREEN_WIDTH, 2, Green, Auto_Aim_ChangeName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
	case 0:
		P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.3 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.8 * SCREEN_WIDTH, 2, Pink, Auto_Aim_ChangeName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
	}
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *�� �� ��: PitchUI_Change
 *����˵��: Pitch�ǶȻ���
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void PitchUI_Change(float Pitch, uint8_t Init_Cnt)
{
	static uint8_t PitchName[] = "Pit";
	static uint8_t optype;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	graphic_data_struct_t *P_graphic_data;

	if (Pitch > 0)
	{
		P_graphic_data = FloatData_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, Pitch, 20, 4, 2, Green, PitchName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	}
	else
	{
		P_graphic_data = FloatData_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, Pitch, 20, 4, 2, Orange, PitchName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	}

	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *�� �� ��: CapUI_Change
 *����˵��: ���ݵ�������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void CapUI_Change(float CapVolt, uint8_t Init_Cnt)
{
	static uint8_t CapName[] = "cpv";
	static uint8_t optype;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = FloatData_Draw(0, optype, 0.42 * SCREEN_LENGTH + 100, 0.1 * SCREEN_WIDTH, CapVolt * 100, 30, 4, 2, Orange, CapName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK); // ���ַ�
}

/**********************************************************************************************************
 *函 数 名: RadarDoubleDamage_Draw
 *功能说明: 显示雷达双倍易伤状态
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void RadarDoubleDamage_Draw(uint8_t Init_Cnt)
{
	static uint8_t RadarDamageChangeName[] = "rdd";
	static uint8_t optype;
	static uint8_t READY[] = "READY";
	static uint8_t EMPTY[] = "     ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	if (JudgeReceiveData.Radar_Double_Damage_Flag)
	{
		Char_Draw(0, optype, 0.5 * SCREEN_LENGTH, 0.7 * SCREEN_WIDTH, 30, sizeof(READY), 3, Green, RadarDamageChangeName, READY);
	}
	else
	{
		Char_Draw(0, optype, 0.5 * SCREEN_LENGTH, 0.7 * SCREEN_WIDTH, 30, sizeof(EMPTY), 3, Pink, RadarDamageChangeName, EMPTY);
	}
}

/**********************************************************************************************************
 *函 数 名: GimbalStatus_Draw
 *功能说明: 显示云台状态
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void GimbalStatus_Draw(uint8_t Init_Cnt)
{
	static uint8_t GimbalStatusName[] = "gst";
	static uint8_t optype;
	static uint8_t DISABLE[] = "DISABLE";
	static uint8_t NORMAL[] = "NORMAL ";
	static uint8_t MINIPC[] = "MINIPC ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Gimbal_Control_Type)
	{
	case 0: // Gimbal_Control_Type_DISABLE
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(DISABLE), 2, Pink, GimbalStatusName, DISABLE);
		break;
	case 1: // Gimbal_Control_Type_NORMAL
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(NORMAL), 2, Green, GimbalStatusName, NORMAL);
		break;
	case 2: // Gimbal_Control_Type_MINIPC
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(MINIPC), 2, Orange, GimbalStatusName, MINIPC);
		break;
	default:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(DISABLE), 2, Pink, GimbalStatusName, DISABLE);
		break;
	}
}

/**********************************************************************************************************
 *函 数 名: BoosterMode_Draw
 *功能说明: 显示发射机构用户控制类型
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void BoosterMode_Draw(uint8_t Init_Cnt)
{
	static uint8_t BoosterModeStatusName[] = "bms";
	static uint8_t optype;
	static uint8_t SINGLE[] = "SINGLE";
	static uint8_t MULTI[] = "MULTI ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Booster_User_Control_Type)
	{
	case 0: // Booster_User_Control_Type_SINGLE
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(SINGLE), 2, Green, BoosterModeStatusName, SINGLE);
		break;
	case 1: // Booster_User_Control_Type_MULTI
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(MULTI), 2, Orange, BoosterModeStatusName, MULTI);
		break;
	default:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(SINGLE), 2, Green, BoosterModeStatusName, SINGLE);
		break;
	}
}

/**********************************************************************************************************
 *�� �� ��: GraphicSendtask
 *����˵��: ͼ�η�������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint8_t Init_Cnt = 10;
// 添加UI更新频率控制计数器
static uint32_t ui_update_counter = 0;

// 添加状态变化标志
static uint8_t status_changed = 0;

// 添加UI更新状态枚举
typedef enum
{
	UI_STATE_IDLE = 0,		// 空闲状态
	UI_STATE_STATUS_UPDATE, // 状态更新状态
	UI_STATE_VALUE_UPDATE	// 数值更新状态
} UI_Update_State_t;

void GraphicSendtask(void)
{
	static UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
	static uint8_t status_update_retry = 0;			   // 状态更新重试次数
	static uint8_t last_status_type = 0;			   // 上次变化的状态类型
	static uint32_t last_update_time = 0;			   // 上次更新时间
	static uint32_t current_time = 0;				   // 当前时间

	// 获取当前时间（假设有HAL_GetTick函数）
	current_time = HAL_GetTick();

	// 初始化阶段发送所有UI元素
	if (Init_Cnt > 0)
	{
		CharChange(Init_Cnt);
		PitchUI_Change(JudgeReceiveData.Pitch_Angle, Init_Cnt);
		CapDraw(JudgeReceiveData.Supercap_Voltage, Init_Cnt);
		MiniPC_Aim_Change(Init_Cnt);
		FrictSpeed_Draw(JudgeReceiveData.booster_fric_omega_left, JudgeReceiveData.booster_fric_omega_right, Init_Cnt);
		CapUI_Change(JudgeReceiveData.Supercap_Voltage, Init_Cnt);
		// CarPosture_Change(25, Init_Cnt);
		BoosterMode_Draw(Init_Cnt);
		GimbalStatus_Draw(Init_Cnt);
		RadarDoubleDamage_Draw(Init_Cnt);

		Init_Cnt--;

		Char_Init();	   // 字符
		ShootLines_Init(); // 枪口线
		Lanelines_Init();  // 车道线

		// 初始化完成后，保存当前数据作为比较基准
		memcpy(&Last_JudgeReceiveData, &JudgeReceiveData, sizeof(JudgeReceive_t));

		return;
	}

	// 状态机处理
	switch (ui_state)
	{
	case UI_STATE_IDLE:
		// 检查是否有状态变化
		if (Last_JudgeReceiveData.Chassis_Control_Type != JudgeReceiveData.Chassis_Control_Type)
		{
			// 底盘控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 1;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Minipc_Status != JudgeReceiveData.Minipc_Status)
		{
			// MiniPC状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 2;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Booster_User_Control_Type != JudgeReceiveData.Booster_User_Control_Type)
		{
			// 发射机构用户控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 3;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Gimbal_Control_Type != JudgeReceiveData.Gimbal_Control_Type)
		{
			// 云台控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 4;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Radar_Double_Damage_Flag != JudgeReceiveData.Radar_Double_Damage_Flag)
		{
			// 雷达双倍易伤状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 5;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		// 如果没有状态变化，且距离上次数值更新已经过去足够时间，则进入数值更新状态
		if (current_time - last_update_time > 100) // 100ms更新一次数值
		{
			ui_state = UI_STATE_VALUE_UPDATE;
			last_update_time = current_time;
		}
		break;

	case UI_STATE_STATUS_UPDATE:
		// 根据状态类型发送对应的状态更新
		switch (last_status_type)
		{
		case 1: // 底盘控制类型
			CharChange(0);
			Last_JudgeReceiveData.Chassis_Control_Type = JudgeReceiveData.Chassis_Control_Type;
			break;
		case 2: // MiniPC状态
			MiniPC_Aim_Change(0);
			Last_JudgeReceiveData.Minipc_Status = JudgeReceiveData.Minipc_Status;
			break;
		case 3: // 发射机构用户控制类型
			BoosterMode_Draw(0);
			Last_JudgeReceiveData.Booster_User_Control_Type = JudgeReceiveData.Booster_User_Control_Type;
			break;
		case 4: // 云台控制类型
			GimbalStatus_Draw(0);
			Last_JudgeReceiveData.Gimbal_Control_Type = JudgeReceiveData.Gimbal_Control_Type;
			break;
		case 5: // 雷达双倍易伤状态
			RadarDoubleDamage_Draw(0);
			Last_JudgeReceiveData.Radar_Double_Damage_Flag = JudgeReceiveData.Radar_Double_Damage_Flag;
			break;
		}

		// 增加重试次数
		status_update_retry++;

		// 如果重试次数达到上限或者已经成功发送，则回到空闲状态
		if (status_update_retry >= 5)
		{
			ui_state = UI_STATE_IDLE;
			last_update_time = current_time;
		}
		else
		{
			// 设置下次重试时间
			last_update_time = current_time;
		}
		break;

	case UI_STATE_VALUE_UPDATE:
		// 只更新一个数值，避免占用太多通信资源
		static uint8_t value_update_index = 0;

		switch (value_update_index)
		{
		case 0: // 更新Pitch角度
			if (fabs(Last_JudgeReceiveData.Pitch_Angle - JudgeReceiveData.Pitch_Angle) > 0.1f)
			{
				PitchUI_Change(JudgeReceiveData.Pitch_Angle, 0);
				Last_JudgeReceiveData.Pitch_Angle = JudgeReceiveData.Pitch_Angle;
			}
			break;

		case 1: // 更新超级电容电压
			if (fabs(Last_JudgeReceiveData.Supercap_Voltage - JudgeReceiveData.Supercap_Voltage) > 0.01f)
			{
				CapDraw(JudgeReceiveData.Supercap_Voltage, 0);
				CapUI_Change(JudgeReceiveData.Supercap_Voltage, 0);
				Last_JudgeReceiveData.Supercap_Voltage = JudgeReceiveData.Supercap_Voltage;
			}
			break;

		case 2: // 更新摩擦轮转速
			FrictSpeed_Draw(JudgeReceiveData.booster_fric_omega_left, JudgeReceiveData.booster_fric_omega_right, 0);
			break;

		case 3: // 更新云台底盘夹角
			if (fabs(Last_JudgeReceiveData.Chassis_Gimbal_Diff - JudgeReceiveData.Chassis_Gimbal_Diff) > 1.0f)
			{
				// CarPosture_Change(25, 0);
				Last_JudgeReceiveData.Chassis_Gimbal_Diff = JudgeReceiveData.Chassis_Gimbal_Diff;
			}
			break;
		}

		// 更新索引，循环遍历所有数值
		value_update_index = (value_update_index + 1) % 4;

		// 回到空闲状态
		ui_state = UI_STATE_IDLE;
		last_update_time = current_time;
		break;
	}
}

