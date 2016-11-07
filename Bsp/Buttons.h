#ifndef __BUTTONS_H
#define __BUTTONS_H


#define BUTTON_UP_PORT  		GPIOC
#define BUTTON_UP_PIN  			GPIO_Pin_3

#define BUTTON_DOWN_PORT  		GPIOC
#define BUTTON_DOWN_PIN  		GPIO_Pin_2

#define BUTTON_LEFT_PORT  		GPIOC
#define BUTTON_LEFT_PIN  		GPIO_Pin_1

#define BUTTON_RIGHT_PORT  		GPIOC
#define BUTTON_RIGHT_PIN		GPIO_Pin_0
#define BUTTON_RIGHT_EXTI_LINE  EXTI_Line0
#define BUTTON_FILTER_TIME     	1 		//1*10ms
#define BUTTON_LONG_TIME		100   	//100*10ms=1s
#define BUTTON_DOUBLE_TIME   	50
#define N_key    0             //�޼�
#define S_key    1             //����
#define D_key    2             //˫��
#define L_key    3             //����

#define key_state_0 	0
#define key_state_1 	1
#define key_state_2 	2
#define key_state_3		3


typedef struct
{
	unsigned char UpDownState;
	unsigned char KeyState;
}BUTTON_T;
//typedef struct
//{
//        /* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
//        unsigned char  (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */
//        unsigned char  Count;                     /* �˲��������� */
//        unsigned char  FilterTime;                /* �˲�ʱ��(���255,��ʾ2550ms) */
//        unsigned short LongCount;                 /* ���������� */
//        unsigned short LongTime;                  /* �������³���ʱ��, 0��ʾ����ⳤ�� */
//        unsigned char  UpDownState;               /* ������ǰ״̬�����»��ǵ��� */
//		unsigned char  KeyState; 
////        unsigned char  KeyCodeUp;                 /* ��������ļ�ֵ����, 0��ʾ����ⰴ������ */
////        unsigned char  KeyCodeDown;               /* �������µļ�ֵ����, 0��ʾ����ⰴ������ */
////        unsigned char  KeyCodeLong;               /* ���������ļ�ֵ����, 0��ʾ����ⳤ�� */
////        unsigned char  RepeatSpeed;               /* ������������ */
////        unsigned char  RepeatCount;               /* �������������� */
//}BUTTON_T; 
typedef enum
{
        KEY_NONE = 0,                      		/* 0 ��ʾ�����¼� */ 
        KEY_DOWN_Power,                        	/* ���������� */
        KEY_UP_Power,                        	/* ���������� */
        KEY_LONG_Power,                        	/* ���������� */
		KEY_DOUBLE_Power,
        KEY_DOWN_Power_TAMPER        			/* ��ϼ��� �� ��ͬʱ���� */
}KEY_STA_ENUM;

void ButtonsConfig(void);
#endif
