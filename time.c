/**
  ******************************************************************************
  * @file    bsp_tim1.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2017-03-07
  * @brief   TIM1定时器
  ******************************************************************************
  * @attention
  * 实验平台: 平衡车
  * 输出频率控制电机和控制电机的正反转
  * PA8 --MOTB_PMW
	* PA11--MOTA_PMW
  * PB15--MOTB_IN2
  * PB14--MOTB_IN1
  * PB13--MOTA_IN1
  * PB12--MOTA_IN2
	* PA0--MOTB_ENCORDER
  * PA1--MOTB_ENCORDER
  * PB6--MOTA_ENCORDER
  * PB7--MOTA_ENCORDER
  ******************************************************************************
  */
  
#include "bsp_tim.h"

 /**
  * @brief  初始化控制电机速度PWM,控制电机方向,采集电机速度
  */
static const PinConfig_t MotorPinConfig[MOTOR_Max] =
{

	__PIN_CONFIG__( MOTA_IN2,  GPIOB, 12,  GPIO_Mode_Out_PP),
	__PIN_CONFIG__( MOTA_IN1,  GPIOB, 13,  GPIO_Mode_Out_PP),
	__PIN_CONFIG__( MOTB_IN1,  GPIOB, 14,  GPIO_Mode_Out_PP),
	__PIN_CONFIG__( MOTB_IN2,  GPIOB, 15,  GPIO_Mode_Out_PP),
	__PIN_CONFIG__(MOTA_ENCORDER1,  GPIOA, 0,  GPIO_Mode_IN_FLOATING),
	__PIN_CONFIG__(MOTA_ENCORDER2,  GPIOA, 1,  GPIO_Mode_IN_FLOATING),
  __PIN_CONFIG__(MOTB_ENCORDER1,  GPIOB, 6,  GPIO_Mode_IN_FLOATING),
  __PIN_CONFIG__(MOTB_ENCORDER2,  GPIOB, 7,  GPIO_Mode_IN_FLOATING),
};

const PinConfig_t *MotorpinConfig_ptr = NULL;

void Pin_Init(const PinConfig_t *config, int count)
{
	for(int i = 0; i < count; i++)
	{
		if(config->GPIOx)
		{
			GPIO_FastInit(config->GPIOx, (1 << config->pinIndex), config->mode,GPIO_Speed_50MHz);
			if(config->mode == GPIO_Mode_Out_PP || config->mode == GPIO_Mode_AF_PP)
			*config->addr = 0;
		}
		config++;
	}
}

void MotorPinInit(void)
{
	MotorpinConfig_ptr = MotorPinConfig;
  Pin_Init(MotorPinConfig, SIZEOF_ARR(MotorPinConfig));
}

void SetMotorPin(uint16_t pin, bool isEnable)
{
	__SET_MOTOR_PIN(pin, isEnable);
}

/**
  * @brief  配置TIM1
  * @param  无
  * @retval 无
  * TIMxCLK/CK_PSC --> TIMxCNT --> TIMx_ARR --> TIMxCNT 重新计数               
  * 信号周期=(TIMx_ARR +1 ) * 时钟周期
**/
static void TIM1_Mode_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);         /*开启GPIOA的外设时钟*/

	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_11;	      /*选择要控制的GPIOA引脚*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;   			      /*设置引脚模式为复用推挽输出*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 			      /*设置引脚速率为50MHz */
	GPIO_Init(GPIOA, &GPIO_InitStructure);		                    /*调用库函数，初始化GPIOA*/
	
	/* 设置PWM信号电平信号初始跳变值 */
 	u16 CCR1_Val = 0;        
 	u16 CCR4_Val = 0;

	/* 设置TIM1CLK 为 72MHZ */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
  TIM_TimeBaseStructure.TIM_Prescaler = 71;	                    //设置预分频：预分频72，即为1MHz
  TIM_TimeBaseStructure.TIM_Period = 999;                       //当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	    //设置时钟分频系数：不分频(这里用不到)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //向上计数模式 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;              //重复寄存器，用于自动更新pwm占空比  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);			          //使能定时器1
 
  /* PWM1 Mode configuration: Channel1 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             //配置为PWM模式1 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //当定时器计数值小于CCR1_Val时为高电平
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	                     //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //使能该通道输出
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	                     
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);				       //使能通道1

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	                     //设置通道4的电平跳变值，输出另外一个占空比的PWM
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	 //使能该通道输出
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);	                     
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);				       //使能通道4
	
  TIM_ARRPreloadConfig(TIM1, ENABLE);			                       // 使能TIM1重载寄存器ARR

  /*使能定时器1	*/
  TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	 
}

/**
  * @brief  配置TIM2
  * @param  无
  * @retval 无
  */
static void TIM2_Mode_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                   //自动装载寄存器的值，累计TIM_Period+1后产生一个更新或中断	 
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;	                 //设置预分频：不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	   //设置时钟分频系数：不分频(这里用不到)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);			         //使能定时器2
	 
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码模式3
	TIM_ICInitStructure.TIM_ICFilter = 10;							         // 输入的需要被捕获的信号的滤波系数
	TIM_ICInit(TIM2, &TIM_ICInitStructure);				               // 定时器输入捕获初始化

	TIM_SetCounter(TIM2,0);                                      //Reset counter
  TIM_Cmd(TIM2, ENABLE);                                       //使能定时器2
}

 /**
  * @brief  配置TIM4
  * @param  无
  * @retval 无
  */

static void TIM4_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                    //自动装载寄存器的值，累计TIM_Period+1后产生一个更新或中断	 
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;	                  //设置预分频：不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	    //设置时钟分频系数：不分频(这里用不到)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //向上计数模式 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);			          //使能定时器4

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码模式3
	TIM_ICInitStructure.TIM_ICFilter = 10;							          // 输入的需要被捕获的信号的滤波系数
	TIM_ICInit(TIM4, &TIM_ICInitStructure);				                // 定时器输入捕获初始化

	TIM_SetCounter(TIM4,0);                                       //Reset counter
	TIM_Cmd(TIM4, ENABLE);                                        //使能定时器4
}

  /**										
  * @brief  读取编码器的值
  * @param  无
  * @retval 无
  */
void Read_Encoder(uint16_t *Encoder)
{
	Encoder[0]=TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM2,0); 
	
	Encoder[1]=TIM_GetCounter(TIM4);
	TIM_SetCounter(TIM4,0);
}

 /**
  * @brief  配置TIM3
  * @param  无
  * @retval 无
  */
static void TIM3_Mode_Config(void)
 {
   // 开启定时器TIM3 CLOCK;内部时钟 CK_INT=72MHz
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	 
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
   TIM_TimeBaseStructure.TIM_Period = 9999;                       //自动装载寄存器的值，累计TIM_Period+1后产生一个更新或中断	 
   TIM_TimeBaseStructure.TIM_Prescaler = 71;	                    //设置预分频：预分频72，即为10KHz
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	      //设置时钟分频系数：不分频(这里用不到)
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数模式 
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;               //重复寄存器，用于自动更新pwm占空比  
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);			          //初始化定时器3
	 
	 TIM_OCInitTypeDef TIM_OCInitStructure;
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
   TIM_OCInitStructure.TIM_Pulse = 9999;
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
   TIM_OC1Init(TIM3, &TIM_OCInitStructure);
   TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
	 
	 NVIC_FastInit(TIM3_IRQn,0,0,ENABLE);
	 
	 TIM_ARRPreloadConfig(TIM3, ENABLE);
	 TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
	 TIM_ClearFlag(TIM3, TIM_FLAG_Update| TIM_FLAG_CC1);        //清除计数器中断标志位
   TIM_ITConfig(TIM3,TIM_IT_Update | TIM_IT_CC1,ENABLE);      //开启计数器中断
   TIM_Cmd(TIM3, ENABLE);                                     //使能定时器3
}

  /**
  * @brief  初始化函数
  * @param  无
  * @retval 无
  */
void TIM_Init(void)
{
	MotorPinInit();         //初始化电机的控制引脚
	TIM1_Mode_Config();	    //初始化控制电机的PWM输出
	TIM2_Mode_Config();     //初始化电机的编码器输入
	TIM4_Mode_Config();     //初始化电机的编码器输入
	TIM3_Mode_Config();     //初始化定时中断
}

/*********************************************END OF FILE**********************/
