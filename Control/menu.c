#include "myfile.h"

// ����OLED��ʾ���ֺ���
extern void OLED_ShowChinese(int16_t X, int16_t Y, const char *Chinese, uint8_t FontSize);

/************************�˵�***************************/
uint8_t Key_Num=0;
uint8_t func_index = 0;
uint8_t Cursor=1;
int Key_Flag=0;
uint8_t Start_Run_Flag = 0; // ������Ѳ��������־��0��δ������1����������
uint8_t Delay_Run_Flag = 0; // �������ӳ�������־��0��δ������1���ӳ��У�





extern uint8_t Start_Flag;
extern uint8_t line_count;
extern uint8_t pass_start_area_count;
extern uint8_t in_start_area;
extern float Location;
extern uint8_t current_line;
extern uint8_t second_line_count;




/************************�ڲ�����***************************/
void Boot_animation(void);
void Homepage_1(void);
void Homepage_2(void);
void Homepage_3(void);
void Homepage_4(void);
void Homepage_5(void);
void Soft_Delay_ms(uint16_t ms)
{

    const uint32_t TICKS_PER_MS = 6000; 
    
    uint32_t total_ticks = (uint32_t)ms * TICKS_PER_MS;

    volatile uint32_t tick = total_ticks;
    while(tick--);
}

typedef struct
{ 
    uint8_t Current;	//��ǰ״̬������
    uint8_t Up;      		  //
		uint8_t Down;     	//	
    void (*current_operation)(void); //��ǰ״̬Ӧ��ִ�еĲ���
} Key_table;

void (*current_operation_index)(); //����һ������ָ��

Key_table table[100] = {    
    {0,5,1,(*Boot_animation)},  // 0���������� �� ��һҳ1������״̬��
    {1,0,2,(*Homepage_1)},      // 1����һҳ������״̬���� ��һҳ2
    {2,1,3,(*Homepage_2)},      // 2���ڶ�ҳ �� ��һҳ3
    {3,2,4,(*Homepage_3)},      // 3������ҳ �� ��һҳ4
    {4,3,5,(*Homepage_4)},      // 4������ҳ �� ��һҳ5
    {5,4,0,(*Homepage_5)},      // 5������ҳ �� ��һҳ0��ѭ���ؿ���������
};




// ����һ��״̬��ǣ���¼�Ƿ��ڡ��ȴ��궨��״̬
static uint8_t waiting_calib = 0;



void menu_operation()
{	
    // ���浱ǰ�������������ں����ж��Ƿ���Ҫ����
    void (*prev_operation)() = table[func_index].current_operation;

	
	if(Key_Num == 1)//��
	{         
        // �ȴ��궨ʱ����Ӧ���Ʋ����������л�ҳ�棩
        if(!waiting_calib)
        {
            func_index = table[func_index].Up;
            OLED_Clear();        
        }
	}
	if(Key_Num == 2)  //��
	{   
		// ����ǰ�ǿ�������ҳ�棨func_index=0������δ����
		if(func_index == 0 && Start_Run_Flag == 0 && !waiting_calib)
		{
			// �״ΰ����Ҽ������롰�ȴ��궨��״̬
			waiting_calib = 1;
			Delay_Run_Flag = 1;
			OLED_Clear();
			OLED_Printf(0, 17, OLED_8X16, "Waiting Calib...");
			OLED_Update();
		}
		else if(!waiting_calib) // �ȴ��궨ʱ����Ӧ���Ʋ���
		{
			func_index = table[func_index].Down;
			OLED_Clear();        
		}
	}

	// ������ڡ��ȴ��궨��״̬���ұ궨�����
	if(waiting_calib && gyro_calibration_done)
	{
		// �궨��ɣ�����Ѳ��
		OLED_Clear();
		OLED_Printf(0, 17, OLED_8X16, "Start Running!");
		OLED_Update();
		Soft_Delay_ms(1000); // ������ʾ������Ϣ
		OLED_Clear();
		Start_Run_Flag = 1;
		func_index = 1;
		waiting_calib = 0; // ���õȴ�״̬
	}

    // ִ�е�ǰ�������ȴ��궨ʱ�������������������⸲����ʾ��
    if(!waiting_calib || prev_operation != (*Boot_animation))
    {
        current_operation_index = table[func_index].current_operation;
        (*current_operation_index)();
    }

	// ÿ��ִ�к���հ���ֵ�������ظ�������
	Key_Num = 0;
}

/************************��ʾ����***************************/
//�������������Լ�����





void Boot_animation()
{
	OLED_Clear();
	
    OLED_ShowChinese(24, 8, "立", OLED_16X16);
    OLED_ShowChinese(40, 8, "功", OLED_16X16);
    OLED_ShowChinese(56, 8, "汪", OLED_16X16);
    OLED_ShowChinese(72, 8, "汪", OLED_16X16);
    OLED_ShowChinese(88, 8, "队", OLED_16X16);

    OLED_ShowChinese(48, 40, "小", OLED_16X16);
    OLED_ShowChinese(64, 40, "砾", OLED_16X16);

    OLED_Update();
	
}

// ��һҳ
void Homepage_1()    
{
    OLED_Printf(0, 1,  OLED_8X16, "Patrol:%02d", Patrol_State);       // Ѳ��״̬����ǰ״̬
    OLED_Printf(0, 17, OLED_8X16, "None:%02d", Noline_Flag);
    OLED_Printf(0, 34, OLED_8X16, "Speed:%+04d", Basic_Speed);        // �����ٶȣ���������ʾ��
    OLED_Printf(0, 51, OLED_8X16, "yaw:%+03.2f", yaw);        // ת��ʹ�ܣ�0-�رգ�1-������
    OLED_Update();
}

//�ڶ�ҳ
void Homepage_2()	
{
	
		OLED_Printf(0,1,OLED_8X16,"	Speed_L:%+05d",Speed_L);
		OLED_Printf(0,17,OLED_8X16,"Speed_R:%+05d",Speed_R);
		OLED_Printf(0,34,OLED_8X16,"PWM_L:%+02d",Speed_Out_L);
		OLED_Printf(0,51,OLED_8X16,"PWM_R:%+02d",Speed_Out_R);
		OLED_Update();
	
}
//����ҳ
void Homepage_3()	
{
	
		OLED_Printf(0,0,OLED_8X16,"	s_err:%+04d",sensor_err);
		OLED_Printf(0,17,OLED_8X16,"f_err:%+04d",final_err);
		OLED_Printf(0,34,OLED_8X16,"P_Out:%+04d",Place_Out);
		OLED_Printf(0,51,OLED_8X16,"gz:%+03d",GZ);
		OLED_Update();
	
}
//����ҳ
void Homepage_4()	
{
	
		OLED_Printf(0,0,OLED_8X16,"	s:%+06.2f",Location);
		OLED_Printf(0,17,OLED_8X16,"FE:%+02d",Element_Flag);
		OLED_Update();
	
}
//����ҳ
void Homepage_5()	
{
	
		OLED_Printf(0,0,OLED_8X16,"	FP:%+03d",Place_Enable);
		OLED_Printf(0,17,OLED_8X16,"FN:%+02d",Noline_Flag);
		OLED_Printf(0,34,OLED_8X16,"FS:%+02d",Stop_Flag);
		OLED_Printf(0,51,OLED_8X16,"yaw:%+03.2f",yaw);
		OLED_Update();
	
}


