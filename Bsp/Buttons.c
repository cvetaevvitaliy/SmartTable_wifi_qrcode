/**********
1�������¼���������ͨkey�ĵ�����
2��˫���¼���500ms��ͬһ�����������Σ�
3�������¼���ͬһ������������1000ms��ϵͳ�г����¼�Ϊ500ms����
4����ϰ������������ϰ���ͬʱ��ס
**********/

#include "Buttons.h"
#include "stm32f10x.h" 
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "bsp_leds.h"
#include "wifi_esp.h"
#include "wifi_esp_config.h"
#include <string.h>
BUTTON_T gtButtonUp,gtButtonDown,gtButtonLeft,gtButtonRight;

//typedef struct
//{
//        /* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
//        unsigned char  (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */
//        unsigned char  Count;                     /* �˲��������� */
//        unsigned char  FilterTime;                /* �˲�ʱ��(���255,��ʾ2550ms) */
//        unsigned short LongCount;                 /* ���������� */
//        unsigned short LongTime;                  /* �������³���ʱ��, 0��ʾ����ⳤ�� */
//        unsigned char  UpDownState;               /* ������ǰ״̬�����»��ǵ��� */
////        unsigned char  KeyCodeUp;                 /* ��������ļ�ֵ����, 0��ʾ����ⰴ������ */
////        unsigned char  KeyCodeDown;               /* �������µļ�ֵ����, 0��ʾ����ⰴ������ */
////        unsigned char  KeyCodeLong;               /* ���������ļ�ֵ����, 0��ʾ����ⳤ�� */
////        unsigned char  RepeatSpeed;               /* ������������ */
////        unsigned char  RepeatCount;               /* �������������� */
//}BUTTON_T; 
//typedef enum
//{
//        KEY_NONE = 0,                      		/* 0 ��ʾ�����¼� */ 
//        KEY_DOWN_Power,                        	/* ���������� */
//        KEY_UP_Power,                        	/* ���������� */
//        KEY_LONG_Power,                        	/* ���������� */
//		  KEY_DOUBLE_Power,
//        KEY_DOWN_Power_TAMPER        			/* ��ϼ��� �� ��ͬʱ���� */
//}KEY_STA_ENUM;

u8 gButtonIOdat;
static unsigned char key_up_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonUp.UpDownState = (gButtonIOdat & 0x08)>>3;                    // ������I/O��ƽ
    key_press = gtButtonUp.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // ������ʼ̬
        if (!key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬
        break;      
      case key_state_1:                      // ����������ȷ��̬
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
        }
        else
             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
             key_state = key_state_0;   // ת����������ʼ̬
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
        {
             key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
             key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
        }
        break;
      case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
        if (key_press) key_state = key_state_0; //�������ͷţ�ת����������ʼ̬
        break;
    }
    return key_return;
}
static unsigned char key_down_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonDown.UpDownState = (gButtonIOdat & 0x04)>>2;                    // ������I/O��ƽ
    key_press = gtButtonDown.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // ������ʼ̬
        if (!key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬
        break;      
      case key_state_1:                      // ����������ȷ��̬
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
        }
        else
             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
             key_state = key_state_0;   // ת����������ʼ̬
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
        {
             key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
             key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
        }
        break;
      case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
        if (key_press) key_state = key_state_0; //�������ͷţ�ת����������ʼ̬
        break;
    }
    return key_return;
}static unsigned char key_left_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonLeft.UpDownState = (gButtonIOdat & 0x02)>>1;                    // ������I/O��ƽ
    key_press = gtButtonLeft.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // ������ʼ̬
        if (!key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬
        break;      
      case key_state_1:                      // ����������ȷ��̬
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
        }
        else
             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
             key_state = key_state_0;   // ת����������ʼ̬
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
        {
             key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
             key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
        }
        break;
      case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
        if (key_press) key_state = key_state_0; //�������ͷţ�ת����������ʼ̬
        break;
    }
    return key_return;
}
static unsigned char key_right_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonRight.UpDownState = gButtonIOdat & 0x01;                    // ������I/O��ƽ
    key_press = gtButtonRight.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // ������ʼ̬
        if (!key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬
        break;      
      case key_state_1:                      // ����������ȷ��̬
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
        }
        else
             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
             key_state = key_state_0;   // ת����������ʼ̬
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
        {
             key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
             key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
        }
        break;
      case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
        if (key_press) key_state = key_state_0; //�������ͷţ�ת����������ʼ̬
        break;
    }
    return key_return;
}
/*=============
�м�㰴�������������õͲ㺯��һ�Σ�����˫���¼����жϣ������ϲ���ȷ���޼���������˫��������4�������¼���
���������ϲ�ѭ�����ã����10ms
===============*/

unsigned char key_up_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
     
    key_temp = key_up_driver();
     
    switch(key_m)
    {
        case key_state_0:
            if (key_temp == S_key )
            {
                 key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // �����޼�������������ԭ�¼�
            break;

        case key_state_1:
            if (key_temp == S_key)             // ��һ�ε���������϶�<500ms��
            {
                 key_return = D_key;           // ����˫�����¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            else                                
            {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
                      key_m = key_state_0;     // ���س�ʼ״̬
                 }
             }
             break;
    }
    return key_return;
}
unsigned char key_down_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
     
    key_temp = key_down_driver();
     
    switch(key_m)
    {
        case key_state_0:
            if (key_temp == S_key )
            {
                 key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // �����޼�������������ԭ�¼�
            break;

        case key_state_1:
            if (key_temp == S_key)             // ��һ�ε���������϶�<500ms��
            {
                 key_return = D_key;           // ����˫�����¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            else                                
            {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
                      key_m = key_state_0;     // ���س�ʼ״̬
                 }
             }
             break;
    }
    return key_return;
}
unsigned char key_left_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
     
    key_temp = key_left_driver();
     
    switch(key_m)
    {
        case key_state_0:
            if (key_temp == S_key )
            {
                 key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // �����޼�������������ԭ�¼�
            break;

        case key_state_1:
            if (key_temp == S_key)             // ��һ�ε���������϶�<500ms��
            {
                 key_return = D_key;           // ����˫�����¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            else                                
            {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
                      key_m = key_state_0;     // ���س�ʼ״̬
                 }
             }
             break;
    }
    return key_return;
}
unsigned char key_right_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
     
    key_temp = key_right_driver();
     
    switch(key_m)
    {
        case key_state_0:
            if (key_temp == S_key )
            {
                 key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // �����޼�������������ԭ�¼�
            break;

        case key_state_1:
            if (key_temp == S_key)             // ��һ�ε���������϶�<500ms��
            {
                 key_return = D_key;           // ����˫�����¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            else                                
            {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
                      key_m = key_state_0;     // ���س�ʼ״̬
                 }
             }
             break;
    }
    return key_return;
}

void Buttons_IOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);// 
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ر�jtag��ʹ��SWD��������SWDģʽ����
	
	GPIO_InitStructure.GPIO_Pin  = BUTTON_UP_PIN | BUTTON_DOWN_PIN |BUTTON_LEFT_PIN |BUTTON_RIGHT_PIN;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
 	GPIO_Init(BUTTON_UP_PORT, &GPIO_InitStructure); 
 
 }

////�ⲿ�жϳ�ʼ������
//void KEY_EXTIX_Init(void)
//{
// 
// 	  EXTI_InitTypeDef EXTI_InitStructure;
// 	  NVIC_InitTypeDef NVIC_InitStructure;

// 
////	GPIO_EXTILineConfig(USB_OUT_EXTI_PORT,USB_OUT_SOURCE);
////  	EXTI_InitStructure.EXTI_Line=	USB_OUT_EXTI_LINE;
////  	EXTI_InitStructure.EXTI_Mode = 	EXTI_Mode_Interrupt;	
////  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
////  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
////  	EXTI_Init(&EXTI_InitStructure);	 	 

////    
////  	GPIO_EXTILineConfig(CALL_KEY_EXTI_PORT,CALL_KEY_SOURCE);

////  	EXTI_InitStructure.EXTI_Line=	CALL_KEY_EXTI_LINE;
////  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
////  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
////  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
////  	EXTI_Init(&EXTI_InitStructure);	  	 

////		GPIO_EXTILineConfig(WIFI_KEY_EXTI_PORT,WIFI_KEY_SOURCE);

////   	EXTI_InitStructure.EXTI_Line	=	WIFI_KEY_EXTI_LINE;
////  	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;	
////  	EXTI_InitStructure.EXTI_Trigger = 	EXTI_Trigger_Falling ;  //EXTI_Trigger_Rising
////  	EXTI_InitStructure.EXTI_LineCmd = 	ENABLE;
////  	EXTI_Init(&EXTI_InitStructure);		 
////	
////	GPIO_EXTILineConfig(NORMAL_KEY_EXTI_PORT,NORMAL_KEY_SOURCE);
////	
////   	EXTI_InitStructure.EXTI_Line	=	NORMAL_KEY_EXTI_LINE;
////  	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;	
////  	EXTI_InitStructure.EXTI_Trigger = 	EXTI_Trigger_Falling ;  //EXTI_Trigger_Rising
////  	EXTI_InitStructure.EXTI_LineCmd = 	ENABLE;
////  	EXTI_Init(&EXTI_InitStructure);	
// 
//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			 
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�1
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								 
//  	NVIC_Init(&NVIC_InitStructure);  	   

//	 
//    
// 
//}
//��ʱ�� ��ʱ ɨ�谴��
void TimerButtonsConfig(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_DeInit(TIM3);                                           //���½�Timer����Ϊȱʡֵ

	TIM_InternalClockConfig(TIM3);                              //�����ڲ�ʱ�Ӹ�TIM2�ṩʱ��Դ      
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//36000-1;               //Ԥ��Ƶϵ��Ϊ36000-1������������ʱ��Ϊ72MHz/36000 = 2kHz       
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�      
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ü�����ģʽΪ���ϼ���ģʽ       
	TIM_TimeBaseStructure.TIM_Period = arr;             //0xffff;                  //���ü��������С�� �Ͳ���һ�������¼�
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);              //������Ӧ�õ�TIM2��
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	//����жϱ�־
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//���ж�
	TIM_Cmd(TIM3, ENABLE);
}
///**
//  * @brief  judge key down .
//  * @param  None
//* @retval  1: key down  0: nothing
//  */
//u8 IsKeyDownFuncButtonUp(void)
//{
//	
//	return ((~gButtonIOdat) & 0x01)? 1:0;
//}
///**
//  * @brief  judge key down .
//  * @param  None
//* @retval  1: key down  0: nothing
//  */
//u8 IsKeyDownFuncButtonDonw(void)
//{
//	
//	return ((~gButtonIOdat) & 0x02)? 1:0;
//}
///**
//  * @brief  judge key down .
//  * @param  None
//* @retval  1: key down  0: nothing
//  */
//u8 IsKeyDownFuncButtonLeft(void)
//{
//	
//	return ((~gButtonIOdat) & 0x04)? 1:0;
//}
///**
//  * @brief  judge key down .
//  * @param  None
//* @retval  1: key down  0: nothing
//  */
//u8 IsKeyDownFuncButtonRight(void)
//{
//	
//	return ((~gButtonIOdat) & 0x08)? 1:0;
//}
void ButtonsConfig(void)
{
	memset((u8*)&gtButtonUp,	0,sizeof(BUTTON_T));
	memset((u8*)&gtButtonDown,	0,sizeof(BUTTON_T));
	memset((u8*)&gtButtonLeft,	0,sizeof(BUTTON_T));
	memset((u8*)&gtButtonRight,	0,sizeof(BUTTON_T));
	Buttons_IOConfig();
	TimerButtonsConfig(99,7199);
}

void TIM3_IRQHandler(void)
{
	 
	struct  WIFI_Dev *pWifiDev = GetWifiDev();
	
	 if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) /*���TIM3�����жϷ������*/
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); /*���TIMx�����жϱ�־ */
		 
        gButtonIOdat = GPIO_ReadInputData(BUTTON_UP_PORT) & 0x0f;
		//KeyScan(io_dat);
		gtButtonUp.KeyState		= key_up_read();
		gtButtonDown.KeyState	= key_down_read();
		gtButtonLeft.KeyState	= key_left_read();
		gtButtonRight.KeyState	= key_right_read();
		
		switch(gtButtonUp.KeyState)
		{
			case N_key:
				break;
			case S_key:
				
				break;
			case D_key:
				
				break;
			case L_key:
			//	LED9_Toggle;  
				break;
			default:
				break; 
		}
		switch(gtButtonLeft.KeyState)
		{
			case N_key:
				break;
			case S_key:
				
				break;
			case D_key:
				
				break;
			case L_key:
				pWifiDev->wStatus = WIFI_SMART_LINK;  
				break;
			default:
				break; 
			
		}
		
    }
	
}


void EXTI1_IRQHandler(void)
{
	//Sys_delay_ms(10);    //����
 
//	EXTI_ClearITPendingBit( );  //���EXTI0��·����λ
}
 

void EXTI15_10_IRQHandler(void)
{
 
	
}
void SysStaCheck(void)
{
	 
}
void KeyScan(u8 buttons)
{ 
	
	
		 
}
//void KeyRead( void )
//{
//    unsigned char ReadData = PINB^0xff;   // 1
//    Trg = ReadData & (ReadData ^ Cont);      // 2
//    Cont = ReadData;                                // 3
//}
#if 0


#endif