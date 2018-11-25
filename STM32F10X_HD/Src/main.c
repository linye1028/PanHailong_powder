/*********************************
Project:HVOF Powder Controller
Date:20171210
错误代码：
Er01 伺服驱动器故障报警
Er02 加速完成1s后实际转速和设定转速相差超过0.5
Er08 累积工作时间超过2400分钟(40小时)提醒设备保养，同时按住UP和DOWN键可以消除错误提示
**********************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys.h"
#include "stdio.h"
#include "delay.h"
#include "24cxx.h"

#define DODA2_0    GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define DODA2_1    GPIO_SetBits(GPIOA, GPIO_Pin_12)
#define DORCK2_0    GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define DORCK2_1    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define DOCLK2_0    GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define DOCLK2_1    GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define DODA1_0    GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define DODA1_1    GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define DORCK1_0    GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define DORCK1_1    GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define DOCLK1_0    GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define DOCLK1_1    GPIO_SetBits(GPIOC, GPIO_Pin_13)

#define KEY_RUN IN0
#define Err_IN  IN1
#define Err_LED 2
#define Work_LED 1

int MotoSpeedSet=10; //设置转速
u16 MotoSpeedActual; //实测转速
u8 DisplayContent=0; //数码管显示的内容 0：实际转速，1：设置转速，2：实时时间,3错误代码
int MotoSpeedTemp=6; //加速时的转速 默认最低速度为6
u16 DisplayTime;
u16 ErrNO;
u8 Flag_ErrNO2=0;
u8 Flag_ErrNO8=0; //累积工作时间大于2400分钟，错误提示符
u8 Flag_Err_OUT=0;
unsigned char  Seg7[14]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0xFF,0x92,0x86,0xAF};//11:全灭，12:S

volatile u32 Timer1;	
u16 DOL=0,DOH=0;

u8 TIM6_100ms=0;
u8 TIM5_100ms_1=0;//给完成加速延时1s用，错误代码2用

unsigned int Puls=0;
unsigned int Puls_100ms[5];
u8 TIM5_10ms=0; //TIM5中断计数
u8 TIM3_i=0;//TIM3中断计数
unsigned char F_Down=0,F_Up=0;
unsigned TIM4_100ms=0;
u8 SET_AccelerateRun=0,SET_AccelerateFinish=0; //加速和完成加速的标识
u8 SET_DecelerateRun=0,SET_DecelerateFinish=0; //加速和完成加速的标识
u8 SET_AccelerateFinish_1sdelay=0; //完成加速后延时1s的标志位，为了错误代码2用的
u8 Flag_KeyRun=0,Flag_KeyRunStop=0;//加速和完成假设的标识
typedef struct 
{
	u8 hour;
	u8 min;
	u8 sec;			
	//公历日月年周
	u16 w_year;
	u8  w_month;
	u8  w_date;
	u8  week;		 
}tm;					 
 tm timer;
//月份数据表											 
u8 const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  
//平年的月份日期表
const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
u8 Flag_24C02Save=0;
u8 CurrentMinute=0; //当前分钟
u8 FormerMinute=0; //前一分钟
u16 BKP4Value=0;
uint32_t timer_second=0;
u8 Flag_timer_minute0=1;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void init_watchdog(void);
void setdol(void);
void setdoh(void);

void Timer3_Configuration(void);
void Timer2_Configuration(void);
void Timer4_Configuration(void);
void Timer5_Configuration(void);
void LEDdisplayInit(void);
void RTC_Configuration(void);
void Bkp_Init(void);

u8 Is_Leap_Year(u16);
u8 RTC_Set(u16 ,u8 ,u8 ,u8 ,u8 ,u8 );
u8 RTC_Get(void);
u8 RTC_Get_Week(u16 ,u8 ,u8 );
u8 RTC_Get_Week(u16 ,u8 ,u8 );
/*************************************************
函数: int main(void)
功能: main主函数
参数: 无
返回: 无
**************************************************/
int main(void)
{	

	u8 MotoSpeed_L=0,MotoSpeed_H=0;
	unsigned int AccumulateWorkMinute=0;
	
	SystemInit();
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	LEDdisplayInit();
	Timer3_Configuration();//100ms一次用来显示LED
	Timer2_Configuration();//产生高速脉冲驱动伺服驱动器
	Timer5_Configuration();//100ms读取一次编码器输入，检测速度,并加减速
	Timer4_Configuration(); //累加定时器

	EXTI_Configuration();
	delay_init(72);
	

	
	AT24CXX_Init();	
	AT24CXX_WP=1;
	DOH=Seg7[0];
	DOL=0x0;
	setdol();
	setdoh();
	
  if(BKP_ReadBackupRegister(BKP_DR1) != 0xAB) //初始化标志
  {
    RTC_Configuration();
		RTC_Set(1000,1,1,0,21,0);//1000年1月1日 0点0分0秒
    BKP_WriteBackupRegister(BKP_DR1, 0xAB);
		BKP_WriteBackupRegister(BKP_DR4,0);  //累积工作分钟寄存器清空    
  }	
	else //如果已经初始化，则开启BKP
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE );
		
		AccumulateWorkMinute=BKP_ReadBackupRegister(BKP_DR4);
	}

	
	MotoSpeed_L=AT24CXX_ReadOneByte(10);
	MotoSpeed_H=AT24CXX_ReadOneByte(100);
	MotoSpeedSet=MotoSpeed_H*256+MotoSpeed_L;

	DOL &=~(1<<0);
	DOL &= ~(1<<Work_LED);
	setdol();

	HSOE1=1;
	HSP1=1;
	HSP2=1;
	RTC_Get();
	CurrentMinute=timer.min;
	FormerMinute=CurrentMinute;
	while (1)
	{
		//24C02数据保存
		if(Flag_24C02Save==1)
		{
			Flag_24C02Save=0;
			AT24CXX_WP=0;
			EXTI->IMR &= ~(EXTI_Line0); 
			delay_ms(100);
			AT24CXX_WriteOneByte(10,MotoSpeedSet%256);
			AT24CXX_WriteOneByte(100,MotoSpeedSet/256);
			AT24CXX_WP=1;
			EXTI->IMR |= EXTI_Line0;
		}

		//长按按钮按键加速 释放后停止加速
		if(Button_down && F_Down) {F_Down=0;  TIM_Cmd(TIM4, DISABLE);  TIM4_100ms=0;}
		if(Button_up && F_Up) 		{F_Up=0;    TIM_Cmd(TIM4, DISABLE);  TIM4_100ms=0;}
		//RTC时间
		
		
		//如果拨到启动开始加速
		if(!KEY_RUN)
		{
			delay_ms(5);
			if((!KEY_RUN)  && (Flag_KeyRun==0))
			{
				Flag_KeyRunStop=0;
				Flag_KeyRun=1;
				SET_AccelerateRun=1;
				SET_AccelerateFinish=0;
				SET_AccelerateFinish_1sdelay=0;
				SET_DecelerateRun=0;
				DOL |= 1<<0;
				DOL |= 1<<Work_LED;
				setdol();	
			}
		}

		

		//如果拨到停机，立刻停止
		if(KEY_RUN)
		{
			delay_ms(5);
			if(KEY_RUN  && !Flag_KeyRunStop)
			{
				Flag_KeyRun=0;
				SET_AccelerateRun=0; 
				SET_AccelerateFinish=0;	
				SET_AccelerateFinish_1sdelay=0;
				SET_DecelerateRun=1;
				Flag_KeyRunStop=1;
			}
		}
		
		//如果有伺服错误
		if(Err_IN)
		{
			delay_ms(5);
			//如果有伺服错误
			if(Err_IN)
			{
				DisplayContent=3;
				ErrNO=1;	
				Flag_Err_OUT=0;
				if((DOL & (1<<Err_LED))==0x00)
				{
					DOL |= 1<<Err_LED;
					setdol();
				}
			}
		}
		//如果没有伺服错误
		if(!Err_IN)
		{
			delay_ms(5);
			//如果伺服错误消除
			if(!Err_IN && !Flag_Err_OUT)
			{
				Flag_Err_OUT=1;
				DisplayContent=0;
				if((DOL & (1<<Err_LED))!=0x00)
				{
					DOL &= ~(1<<Err_LED);
					setdol();	
				}
			}
			//如果没有伺服错误，并且在完成加速后，实际转速不等于设定转速，报警
			if(!Err_IN && SET_AccelerateFinish_1sdelay  &&(DisplayContent!=1)&& ((MotoSpeedSet-MotoSpeedActual>5)||(MotoSpeedActual-MotoSpeedSet>5)))
			{
				DisplayContent=3;
				ErrNO=2;
				Flag_ErrNO2=1;
				DOL |= 1<<Err_LED;
				setdol();				
			}
			if(!Err_IN && SET_AccelerateFinish_1sdelay && ((MotoSpeedSet-MotoSpeedActual<=5)&&(MotoSpeedActual-MotoSpeedSet<=5)) && Flag_ErrNO2)
			{
				DisplayContent=0;
				Flag_ErrNO2=0;
				if((DOL & (1<<Err_LED))!=0x00)
				{
					DOL &= ~(1<<Err_LED);
					setdol();		
				}
			}
		
		}
		
		
		
		//启动状态下开始累积工作时间，并存入BKP中，每分钟写入一次
		timer_second=RTC_GetCounter();
		if((timer_second%60==0) && Flag_timer_minute0==1)
		{
				Flag_timer_minute0=0;
				FormerMinute=CurrentMinute;
				if(!KEY_RUN)
				{
					PWR_BackupAccessCmd(ENABLE);
					BKP4Value=BKP_ReadBackupRegister(BKP_DR4)+1;
					BKP_WriteBackupRegister(BKP_DR4, BKP4Value);	
					AccumulateWorkMinute=BKP_ReadBackupRegister(BKP_DR4);
					PWR_BackupAccessCmd(DISABLE);
				}
		}
		if(timer_second%10!=0)
			Flag_timer_minute0=1;	

		//如果累积工作时间大于2400分钟，则提示Er08，此时如果同时按下UP和DOWN键，清空工作时间并消除错误提示
		if(AccumulateWorkMinute>2400)
		{
			ErrNO=8;
			DisplayContent=3;
			Flag_ErrNO8=1;
		}
		if(Flag_ErrNO8 && !Button_down && !Button_up)
		{
			Flag_ErrNO8=0;
			DisplayContent=0;
			RTC_Configuration();
			RTC_Set(1000,1,1,0,21,0);//1000年1月1日 0点0分0秒
			BKP_WriteBackupRegister(BKP_DR4,0);  //累积工作分钟寄存器清空 			
		}

		if(SET_AccelerateFinish)	
			TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
		else if(SET_AccelerateRun || SET_DecelerateRun)
				    TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedTemp)-1;	

	}

}


/*************************************************
函数: void RCC_Configuration(void)
功能: 复位和时钟控制 配置
参数: 无
返回: 无
**************************************************/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    //使能外部晶振
    RCC_HSEConfig(RCC_HSE_ON);
    //等待外部晶振稳定
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //如果外部晶振启动成功，则进行下一步操作
    if(HSEStartUpStatus==SUCCESS)
    {
        //设置HCLK（AHB时钟）=SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        //PCLK1(APB1) = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);
        //PCLK2(APB2) = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);
        //设置ADC时钟频率
        RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
        //FLASH时序控制
        //推荐值：SYSCLK = 0~24MHz   Latency=0
        //        SYSCLK = 24~48MHz  Latency=1
        //        SYSCLK = 48~72MHz  Latency=2
        FLASH_SetLatency(FLASH_Latency_2);
        //开启FLASH预取指功能
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        //PLL设置 SYSCLK/1 * 9 = 8*1*9 = 72MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        //RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
        //启动PLL
        RCC_PLLCmd(ENABLE);
        //等待PLL稳定
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        //系统时钟SYSCLK来自PLL输出
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //切换时钟后等待系统时钟稳定
        while(RCC_GetSYSCLKSource()!=0x08);
    }
		//启动GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
		
    //启动AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

    //以下是595的信号线 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


		//USART1_TX	 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //USART1_RX	 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
		//HSOE1 HSP1 HSP2 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//LED1234 推挽输出
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

		//PA0(HSPIN0) 高速脉冲输入捕获
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
		//UP DOWN SET PB4 PB3 PD2 input; WP OUTPP
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		//IN0-IN8 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);		
}

void EXTI_Configuration(void)
{
		EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	 //设置PA0（HSPIN0 ）为自动中断，下降沿中断
	 EXTI_ClearITPendingBit(EXTI_Line0);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	 //设置PB4（Up ）为自动中断，下降沿中断
	 EXTI_ClearITPendingBit(EXTI_Line4);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

	 //设置PB3（Down ）为自动中断，下降沿中断
	 EXTI_ClearITPendingBit(EXTI_Line3);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	 //设置PD2（Set ）为自动中断，下降沿中断
	 EXTI_ClearITPendingBit(EXTI_Line2);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           //4个抢占式优先级 4个副优先级

	//PA0 高速脉冲捕获计数
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;          //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//指定抢先优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);	

	//PA7 自动选择外部中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;          //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//指定抢先优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;          //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//指定抢先优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;          //指定中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	//指定抢先优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //指定响应优先级别0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);



	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//指定抢先优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	   //中断响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //打开中断
	NVIC_Init(&NVIC_InitStructure); 						   //初始化 					 


	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM3中断
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM2中断
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM2中断
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM2中断
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM2中断
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //指定抢占式优先级别1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //指定响应优先级别2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能TIM2中断
	NVIC_Init(&NVIC_InitStructure);	
}

void init_watchdog(void) {
	/* Check if the system has resumed from IWDG reset -------------------------*/
	//检查看门狗复位标志
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
	  /* IWDGRST flag set */
		      
	  /* Clear reset flags */
	  //清除复位标志
	  RCC_ClearFlag();
	
	 //LCD_text(50,90,0xF800,0,"Watchdog SET!");
	}
	/* IWDG timeout equal to 280 ms (the timeout may varies due to LSI frequency
	  dispersion) -------------------------------------------------------------*/
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	//允许IWDG被修改
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* IWDG counter clock: 40KHz(LSI) / 256 = 156 Hz */
	//设置IWDG时钟
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	/* Set counter reload value to 624 */
	//设置定时器重载值 共计4s
	IWDG_SetReload(780);
	
	/* Reload IWDG counter */
	//重载IWDG计数器
	IWDG_ReloadCounter();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	//使能IWDG
	IWDG_Enable();
}

void setdol(void)  //输出DOL数据函数	
{
	unsigned char i;
	for (i=0;i<16;i++)	
	{
		if (DOL&(1<<(15-i))) DODA1_1; else DODA1_0;
		DOCLK1_1;
		delay_us(100);
		DOCLK1_0;
	}
	//让595刷新输出数据
	delay_us(100);
	DORCK1_1;
	delay_us(100);
	DORCK1_0;
}
void setdoh(void)  //输出DOL数据函数	
{
	unsigned char i;

	for (i=0;i<16;i++)	
	{
		if (DOH&(1<<(15-i))) DODA2_1; else DODA2_0;
		DOCLK2_1;
		delay_us(100);
		DOCLK2_0;
	
	}

//让595刷新输出数据
	delay_us(100);
	DORCK2_1;
	delay_us(100);
	DORCK2_0;
}

//Tim3 1ms一次中断，用来显示
void Timer3_Configuration(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	    //打开TIM5定时器的时钟
  
  TIM_DeInit(TIM3);		                                    //TIMx寄存器重设为缺省值
  
  TIM_TimeBaseStructure.TIM_Period = 10-1;		            //自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx 时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //采样分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);                       //允许自动重装载寄存器（ARR）
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	                //允许TIM5溢出中断
  
  TIM_Cmd(TIM3, ENABLE);// 关闭时钟                                 //开启时钟 
}

//Tim2 定时器 用于延时停机计时 一次1ms 2K
void Timer2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	    //打开TIM2定时器的时钟
  
  TIM_DeInit(TIM2);		                                    //TIMx寄存器重设为缺省值
  
  TIM_TimeBaseStructure.TIM_Period =(u16)(342860/(unsigned long)MotoSpeedSet)-1;//500-1;		            //自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler=9-1;               //TIMx 时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //采样分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);                       //允许自动重装载寄存器（ARR）
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	                //允许TIM3溢出中断
  
  TIM_Cmd(TIM2, ENABLE);// 
}

void Timer4_Configuration(void) //100ms 按键累加
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	    //打开TIM3定时器的时钟
 
  TIM_DeInit(TIM4);		                                    //TIMx寄存器重设为缺省值
  
  TIM_TimeBaseStructure.TIM_Period = 1000;	            //自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx 时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //采样分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM4, ENABLE);                       //允许自动重装载寄存器（ARR）
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	                //允许TIM3溢出中断
  
	TIM_Cmd(TIM4, DISABLE);	                                //开启时钟 
}



//Tim5 10ms中断一次，读取Puls的值
void Timer5_Configuration(void) 
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	    //打开TIM5定时器的时钟
  
  TIM_DeInit(TIM5);		                                    //TIMx寄存器重设为缺省值
  
  TIM_TimeBaseStructure.TIM_Period = 1000-1;		            //自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx 时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //采样分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM5, ENABLE);                       //允许自动重装载寄存器（ARR）
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	                //允许TIM5溢出中断
  
  TIM_Cmd(TIM5, ENABLE);// 关闭时钟 

}


void LEDdisplayInit(void)
{
		GPIO_SetBits(GPIOC, GPIO_Pin_12);
		GPIO_SetBits(GPIOC, GPIO_Pin_11);
		GPIO_SetBits(GPIOC, GPIO_Pin_10);
		GPIO_SetBits(GPIOA, GPIO_Pin_15);		
}

void RTC_Configuration(void)
{
  /* Enable PWR and BKP clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();
 
  /* Enable LSE */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select LSE as RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  

  /* Enable RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  
  /* Enable the RTC Second */  
  RTC_ITConfig(RTC_IT_SEC, ENABLE);

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  
  /* Set RTC prescaler: set RTC period to 1sec */
  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}
u8 Is_Leap_Year(u16 year)
{			  
			   
	if(year%4==0) //必须能被4整除
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//如果以00结尾,还要能被400整除 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}	
//把输入的时钟转换为秒钟
//以1970年1月1日为基准
//1970~2099年为合法年份
//返回值:0,成功;其他:错误代码.
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec)
{		   
	u16 t;
	u32 seccount=0;
	if(syear<1970||syear>2099)return 1;	   
	for(t=1970;t<syear;t++)	//把所有年份的秒钟相加
	{
		if(Is_Leap_Year(t))seccount+=31622400;//闰年的秒钟数
		else seccount+=31536000;			  //平年的秒钟数
	}
	smon-=1;
	for(t=0;t<smon;t++)	   //把前面月份的秒钟数相加
	{
		seccount+=(u32)mon_table[t]*86400;//月份秒钟数相加
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//闰年2月份增加一天的秒钟数	   
	}
	seccount+=(u32)(sday-1)*86400;//把前面日期的秒钟数相加 
	seccount+=(u32)hour*3600;//小时秒钟数
    seccount+=(u32)min*60;	 //分钟秒钟数
	seccount+=sec;//最后的秒钟加上去
													    
	//设置时钟
    RTC_WaitForLastTask(); 
	RTC_SetCounter(seccount);
    RTC_WaitForLastTask();
	return 0;	    
}
//得到当前的时间
//返回值:0,成功;其他:错误代码.
u8 RTC_Get(void)
{
	static u16 daycnt=0;
	u32 timecount=0; 
	u32 temp=0;
	u16 temp1=0;	  
	   
     
    timecount = RTC_GetCounter();			 

	temp=timecount/86400;   //得到天数(秒钟数对应的)
	if(daycnt!=temp)//超过一天了
	{	  
		daycnt=temp;
		temp1=1970;	//从1970年开始
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//是闰年
			{
				if(temp>=366)temp-=366;//闰年的秒钟数
				else {temp1++;break;}  
			}
			else temp-=365;	  //平年 
			temp1++;  
		}   
		timer.w_year=temp1;//得到年份
		temp1=0;
		while(temp>=28)//超过了一个月
		{
			if(Is_Leap_Year(timer.w_year)&&temp1==1)//当年是不是闰年/2月份
			{
				if(temp>=29)temp-=29;//闰年的秒钟数
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//平年
				else break;
			}
			temp1++;  
		}
		timer.w_month=temp1+1;//得到月份
		timer.w_date=temp+1;  //得到日期 
	}
	temp=timecount%86400;     //得到秒钟数   	   
	timer.hour=temp/3600;     //小时
	timer.min=(temp%3600)/60; //分钟	
	timer.sec=(temp%3600)%60; //秒钟
	timer.week=RTC_Get_Week(timer.w_year,timer.w_month,timer.w_date);//获取星期   
	return 0;
}	 
//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//输入参数：公历年月日 
//返回值：星期号																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{	
	u16 temp2;
	u8 yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// 如果为21世纪,年份数加100  
	if (yearH>19)yearL+=100;
	// 所过闰年数只算1900年之后的  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
}

void Bkp_Init(void)
{
	RCC->APB1RSTR |= 1<<27;       
	RCC->APB1RSTR &= ~(1<<27);
	RCC->APB1ENR|=1<<28;        
	RCC->APB1ENR|=1<<27;     

}




//定时器2 用于延时停机报警
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&0X0001)//溢出中断
	{	
		HSP1=!HSP1;
		HSP2=!HSP2;
	}
	
	TIM2->SR&=~(1<<0);//清除中断标志位 	 
}


//定时器5 10ms一次  用于计算速度和加减速控制
void TIM5_IRQHandler(void)
{
	unsigned char x;
	u16 MotoSpeedActual_temp=0;
	if(TIM5->SR&0X0001)//溢出中断
	{	
		Puls_100ms[TIM5_10ms]=Puls;
		TIM5_10ms++;
		Puls=0;
		
		//SET_AccelerateFinish置位后2s后置位SET_AccelerateFinish_1sdelay
		if(SET_AccelerateFinish && (!SET_AccelerateFinish_1sdelay))
		{
			TIM5_100ms_1++;
			if(TIM5_100ms_1>20)
			{
				SET_AccelerateFinish_1sdelay=1;
				TIM5_100ms_1=0;
			}
		}		

		if(TIM5_10ms>5)
		{
			TIM5_10ms=0;
			MotoSpeedActual_temp=0;
			for(x=0;x<5;x++)
			{
				MotoSpeedActual_temp=MotoSpeedActual_temp+Puls_100ms[x];
			}
			MotoSpeedActual_temp=MotoSpeedActual_temp/5;
			MotoSpeedActual=(u16)(MotoSpeedActual_temp/29.1);
		}
		
		//加速控制
		if(SET_AccelerateRun==1)
		{
			MotoSpeedTemp=(MotoSpeedSet/20!=0)? (MotoSpeedTemp+MotoSpeedSet/20) : (MotoSpeedTemp+1);
			if(MotoSpeedTemp>=MotoSpeedSet)
			{
				MotoSpeedTemp=MotoSpeedSet;
				SET_AccelerateRun=0;
				SET_AccelerateFinish=1;
			}
		}
		//减速控制
		if(SET_DecelerateRun==1)
		{
			MotoSpeedTemp=(MotoSpeedSet/20!=0)? (MotoSpeedTemp-MotoSpeedSet/20) : (MotoSpeedTemp-1);
			if(MotoSpeedTemp<=6) ///此处修改20170701
			{
				MotoSpeedTemp=6;
				SET_DecelerateRun=0;
				DOL &= ~(1<<0);
				DOL &= ~(1<<Work_LED);
				setdol();
			}			
		}
	}
	TIM5->SR&=~(1<<0);//清除中断标志位 	 
}






//Tim7 每隔Reamer1Advanceduanxu/10S的时间中断一次，中断中就是关闭旋转油缸，延时0.5s后再前进
void TIM7_IRQHandler(void)
{
	if(TIM7->SR&0X0001)//溢出中断
	{	

	}
	
	TIM7->SR&=~(1<<0);//清除中断标志位 	 
}


//TIM3 显示中断 100ms显示一位
void TIM3_IRQHandler(void)
{
	int DisNum;
	char Num0,Num1,Num2,Num3;
	if(TIM3->SR&0X0001)//溢出中断
	{
		TIM3_i++;
		if(TIM3_i>=14)
			TIM3_i=10;
		switch(DisplayContent)
		{
			case 0 : DisNum=(u16)MotoSpeedActual; break; //实际转速
			case 1 : DisNum=(u16)(MotoSpeedSet); break; //设定转速
			case 2 : DisNum=(u16)DisplayTime; break; //实时时间
			case 3 : DisNum=(u16)ErrNO; break; //错误代码
			default : break;
		}
		if(TIM3_i==10)
		{
			Num3=DisNum/1000%10;
			GPIO_SetBits(GPIOA, GPIO_Pin_15);
			switch(DisplayContent)
			{
				case 0 : DOH=DOH=Seg7[10];break; //实测速度不显示最高位
				case 1 : DOH=Seg7[11];break; //设置速度最高位显示S
				case 2 : DOH=Seg7[Num3]; break; //时间最高位正常显示
				case 3 : DOH=Seg7[12]; break;  //错误代码第一位 E
				default:break;
			}
			setdoh();
			GPIO_ResetBits(GPIOC, GPIO_Pin_12);
		}

		if(TIM3_i==11)
		{
			Num2=DisNum/100%10;
			GPIO_SetBits(GPIOC, GPIO_Pin_12);
			switch(DisplayContent)
			{
				case 0 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //实际速度
				case 1 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //设定速度
				case 2 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //时间
				case 3 : DOH=Seg7[13]; break;  //错误代码第二位r
				default:break;
			}
			DOH=DisplayContent==2? (DOH & (~(1<<7))):(DOH);//显示时间的时候该位显示小数点
			setdoh();
			GPIO_ResetBits(GPIOC, GPIO_Pin_11);
		}

		if(TIM3_i==12)
		{
			Num1=DisNum/10%10;
			GPIO_SetBits(GPIOC, GPIO_Pin_11);
			DOH=Seg7[Num1];
			DOH=DisplayContent<2? (DOH & (~(1<<7))):(DOH); //实测速度和设定速度显示小数点
			setdoh();
			GPIO_ResetBits(GPIOC, GPIO_Pin_10);
		}

		if(TIM3_i==13)
		{
			Num0=DisNum%10;
			GPIO_SetBits(GPIOC, GPIO_Pin_10);
			DOH=Seg7[Num0];
			setdoh();
			GPIO_ResetBits(GPIOA, GPIO_Pin_15);
			
		}
	}
	TIM3->SR&=~(1<<0);//清除中断标志位 	 
}


//自动中断服务程序
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET) 
  {
		delay_ms(1);
		if(Button_down==0)
		{
			F_Down=1;
			TIM_Cmd(TIM4, ENABLE);
			DisplayContent=1;
			MotoSpeedSet=MotoSpeedSet-1;//Speed_LedSet+100;	
			//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			if(MotoSpeedSet<6)
			{
				MotoSpeedSet=0;
				DOL &=~(1<<0); //关闭伺服
				DOL &= ~(1<<Work_LED);
				setdol();
			}
		}	
  	EXTI_ClearFlag(EXTI_Line4);			       //清除中断标志（必须）
   	EXTI_ClearITPendingBit(EXTI_Line4);
	 }	
}
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET) 
  {
		delay_ms(1);
		if(Button_up==0)
		{
			F_Up=1;
			TIM_Cmd(TIM4, ENABLE);
			DisplayContent=1;
			MotoSpeedSet=MotoSpeedSet+1;
			//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			if(MotoSpeedSet>0 && MotoSpeedSet<6)
			{
				MotoSpeedSet=6;
				DOL |=1<<0; //开启伺服
				setdol();
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			}
				
			if(MotoSpeedSet>=300)
			{
				MotoSpeedSet=300;
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			}
				
		}	
  	EXTI_ClearFlag(EXTI_Line3);			       //清除中断标志（必须）
   	EXTI_ClearITPendingBit(EXTI_Line3);
	 }	
}
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET) 
  {
		delay_ms(1);
		if(Button_set==0)
		{ 
			DisplayContent=0;
			Flag_24C02Save=1;
		}	
  	EXTI_ClearFlag(EXTI_Line2);			       //清除中断标志（必须）
   	EXTI_ClearITPendingBit(EXTI_Line2);
	 }	
}
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET) 
  {
		delay_us(2);
		if(HSPIN0==0)
		{
			Puls++;
		}	
  	EXTI_ClearFlag(EXTI_Line0);			       //清除中断标志（必须）
   	EXTI_ClearITPendingBit(EXTI_Line0);
	 }	
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//溢出中断
	{	
		TIM4_100ms++;
		if(TIM4_100ms>20)
		{
			if(F_Up) 
			{
				MotoSpeedSet=MotoSpeedSet+TIM4_100ms/10;
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
				if(MotoSpeedSet>0 && MotoSpeedSet<6)
				{
					MotoSpeedSet=6;
					DOL |=1<<0; //开启伺服
					setdol();
					//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
				}
					
				if(MotoSpeedSet>=300)
				{
					MotoSpeedSet=300;
					//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
				}				

			}
			if(F_Down) 
			{
				MotoSpeedSet=MotoSpeedSet-TIM4_100ms/10;
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
				if(MotoSpeedSet<6)
				{
					MotoSpeedSet=0;
					DOL &=~(1<<0); //关闭伺服
					DOL &= ~(1<<Work_LED);
					setdol();
				}
			}			
		}

	}
	TIM4->SR&=~(1<<0);//清除中断标志位 	 
}

