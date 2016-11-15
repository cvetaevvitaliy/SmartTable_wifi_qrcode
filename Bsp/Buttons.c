/**********
1、单击事件：就是普通key的单击；
2、双击事件：500ms内同一按键单击两次；
3、长按事件：同一按键长按超过1000ms（系统中长按事件为500ms）；
4、组合按键：两个以上按键同时按住
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
//        /* 下面是一个函数指针，指向判断按键手否按下的函数 */
//        unsigned char  (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */
//        unsigned char  Count;                     /* 滤波器计数器 */
//        unsigned char  FilterTime;                /* 滤波时间(最大255,表示2550ms) */
//        unsigned short LongCount;                 /* 长按计数器 */
//        unsigned short LongTime;                  /* 按键按下持续时间, 0表示不检测长按 */
//        unsigned char  UpDownState;               /* 按键当前状态（按下还是弹起） */
////        unsigned char  KeyCodeUp;                 /* 按键弹起的键值代码, 0表示不检测按键弹起 */
////        unsigned char  KeyCodeDown;               /* 按键按下的键值代码, 0表示不检测按键按下 */
////        unsigned char  KeyCodeLong;               /* 按键长按的键值代码, 0表示不检测长按 */
////        unsigned char  RepeatSpeed;               /* 连续按键周期 */
////        unsigned char  RepeatCount;               /* 连续按键计数器 */
//}BUTTON_T; 
//typedef enum
//{
//        KEY_NONE = 0,                      		/* 0 表示按键事件 */ 
//        KEY_DOWN_Power,                        	/* 按键键按下 */
//        KEY_UP_Power,                        	/* 按键键弹起 */
//        KEY_LONG_Power,                        	/* 按键键长按 */
//		  KEY_DOUBLE_Power,
//        KEY_DOWN_Power_TAMPER        			/* 组合键， 键 键同时按下 */
//}KEY_STA_ENUM;

u8 gButtonIOdat;
static unsigned char key_up_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonUp.UpDownState = (gButtonIOdat & 0x08)>>3;                    // 读按键I/O电平
    key_press = gtButtonUp.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // 按键初始态
        if (!key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态
        break;      
      case key_state_1:                      // 按键消抖与确认态
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
        }
        else
             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
             key_state = key_state_0;   // 转换到按键初始态
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
        {
             key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
             key_state = key_state_3;   // 转换到等待按键释放状态
        }
        break;
      case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
        if (key_press) key_state = key_state_0; //按键已释放，转换到按键初始态
        break;
    }
    return key_return;
}
static unsigned char key_down_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonDown.UpDownState = (gButtonIOdat & 0x04)>>2;                    // 读按键I/O电平
    key_press = gtButtonDown.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // 按键初始态
        if (!key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态
        break;      
      case key_state_1:                      // 按键消抖与确认态
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
        }
        else
             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
             key_state = key_state_0;   // 转换到按键初始态
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
        {
             key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
             key_state = key_state_3;   // 转换到等待按键释放状态
        }
        break;
      case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
        if (key_press) key_state = key_state_0; //按键已释放，转换到按键初始态
        break;
    }
    return key_return;
}static unsigned char key_left_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonLeft.UpDownState = (gButtonIOdat & 0x02)>>1;                    // 读按键I/O电平
    key_press = gtButtonLeft.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // 按键初始态
        if (!key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态
        break;      
      case key_state_1:                      // 按键消抖与确认态
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
        }
        else
             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
             key_state = key_state_0;   // 转换到按键初始态
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
        {
             key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
             key_state = key_state_3;   // 转换到等待按键释放状态
        }
        break;
      case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
        if (key_press) key_state = key_state_0; //按键已释放，转换到按键初始态
        break;
    }
    return key_return;
}
static unsigned char key_right_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;
	gtButtonRight.UpDownState = gButtonIOdat & 0x01;                    // 读按键I/O电平
    key_press = gtButtonRight.UpDownState;
    switch (key_state)
    {
      case key_state_0:                              // 按键初始态
        if (!key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态
        break;      
      case key_state_1:                      // 按键消抖与确认态
        if (!key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
        }
        else
             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
        break;      
      case key_state_2:
        if(key_press)
        {
             key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
             key_state = key_state_0;   // 转换到按键初始态
        }
        else if (++key_time >= BUTTON_LONG_TIME)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
        {
             key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
             key_state = key_state_3;   // 转换到等待按键释放状态
        }
        break;
      case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
        if (key_press) key_state = key_state_0; //按键已释放，转换到按键初始态
        break;
    }
    return key_return;
}
/*=============
中间层按键处理函数，调用低层函数一次，处理双击事件的判断，返回上层正确的无键、单键、双键、长键4个按键事件。
本函数由上层循环调用，间隔10ms
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
                 key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // 对于无键、长键，返回原事件
            break;

        case key_state_1:
            if (key_temp == S_key)             // 又一次单击（间隔肯定<500ms）
            {
                 key_return = D_key;           // 返回双击键事件，回初始状态
                 key_m = key_state_0;
            }
            else                                
            {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
                      key_m = key_state_0;     // 返回初始状态
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
                 key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // 对于无键、长键，返回原事件
            break;

        case key_state_1:
            if (key_temp == S_key)             // 又一次单击（间隔肯定<500ms）
            {
                 key_return = D_key;           // 返回双击键事件，回初始状态
                 key_m = key_state_0;
            }
            else                                
            {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
                      key_m = key_state_0;     // 返回初始状态
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
                 key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // 对于无键、长键，返回原事件
            break;

        case key_state_1:
            if (key_temp == S_key)             // 又一次单击（间隔肯定<500ms）
            {
                 key_return = D_key;           // 返回双击键事件，回初始状态
                 key_m = key_state_0;
            }
            else                                
            {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
                      key_m = key_state_0;     // 返回初始状态
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
                 key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
                 key_m = key_state_1;
            }
            else
                 key_return = key_temp;        // 对于无键、长键，返回原事件
            break;

        case key_state_1:
            if (key_temp == S_key)             // 又一次单击（间隔肯定<500ms）
            {
                 key_return = D_key;           // 返回双击键事件，回初始状态
                 key_m = key_state_0;
            }
            else                                
            {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
                 if(++key_time_1 >= BUTTON_DOUBLE_TIME)
                 {
                      key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
                      key_m = key_state_0;     // 返回初始状态
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
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin  = BUTTON_UP_PIN | BUTTON_DOWN_PIN |BUTTON_LEFT_PIN |BUTTON_RIGHT_PIN;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
 	GPIO_Init(BUTTON_UP_PORT, &GPIO_InitStructure); 
 
 }

////外部中断初始化函数
//void KEY_EXTIX_Init(void)
//{
// 
// 	  EXTI_InitTypeDef EXTI_InitStructure;
// 	  NVIC_InitTypeDef NVIC_InitStructure;

// 
////	GPIO_EXTILineConfig(USB_OUT_EXTI_PORT,USB_OUT_SOURCE);
////  	EXTI_InitStructure.EXTI_Line=	USB_OUT_EXTI_LINE;
////  	EXTI_InitStructure.EXTI_Mode = 	EXTI_Mode_Interrupt;	
////  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
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
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								 
//  	NVIC_Init(&NVIC_InitStructure);  	   

//	 
//    
// 
//}
//定时器 定时 扫描按键
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

	TIM_DeInit(TIM3);                                           //重新将Timer设置为缺省值

	TIM_InternalClockConfig(TIM3);                              //采用内部时钟给TIM2提供时钟源      
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//36000-1;               //预分频系数为36000-1，这样计数器时钟为72MHz/36000 = 2kHz       
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割      
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //设置计数器模式为向上计数模式       
	TIM_TimeBaseStructure.TIM_Period = arr;             //0xffff;                  //设置计数溢出大小， 就产生一个更新事件
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);              //将配置应用到TIM2中
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	//清除中断标志
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//开中断
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
	
	 if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) /*检查TIM3更新中断发生与否*/
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); /*清除TIMx更新中断标志 */
		 
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
	//Sys_delay_ms(10);    //消抖
 
//	EXTI_ClearITPendingBit( );  //清除EXTI0线路挂起位
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