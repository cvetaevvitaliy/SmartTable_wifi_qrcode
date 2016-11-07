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
#define N_key    0             //无键
#define S_key    1             //单键
#define D_key    2             //双键
#define L_key    3             //长键

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
//        /* 下面是一个函数指针，指向判断按键手否按下的函数 */
//        unsigned char  (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */
//        unsigned char  Count;                     /* 滤波器计数器 */
//        unsigned char  FilterTime;                /* 滤波时间(最大255,表示2550ms) */
//        unsigned short LongCount;                 /* 长按计数器 */
//        unsigned short LongTime;                  /* 按键按下持续时间, 0表示不检测长按 */
//        unsigned char  UpDownState;               /* 按键当前状态（按下还是弹起） */
//		unsigned char  KeyState; 
////        unsigned char  KeyCodeUp;                 /* 按键弹起的键值代码, 0表示不检测按键弹起 */
////        unsigned char  KeyCodeDown;               /* 按键按下的键值代码, 0表示不检测按键按下 */
////        unsigned char  KeyCodeLong;               /* 按键长按的键值代码, 0表示不检测长按 */
////        unsigned char  RepeatSpeed;               /* 连续按键周期 */
////        unsigned char  RepeatCount;               /* 连续按键计数器 */
//}BUTTON_T; 
typedef enum
{
        KEY_NONE = 0,                      		/* 0 表示按键事件 */ 
        KEY_DOWN_Power,                        	/* 按键键按下 */
        KEY_UP_Power,                        	/* 按键键弹起 */
        KEY_LONG_Power,                        	/* 按键键长按 */
		KEY_DOUBLE_Power,
        KEY_DOWN_Power_TAMPER        			/* 组合键， 键 键同时按下 */
}KEY_STA_ENUM;

void ButtonsConfig(void);
#endif
