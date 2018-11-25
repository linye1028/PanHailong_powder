/*********************************
Project:HVOF Powder Controller
Date:20171210
������룺
Er01 �ŷ����������ϱ���
Er02 �������1s��ʵ��ת�ٺ��趨ת������0.5
Er08 �ۻ�����ʱ�䳬��2400����(40Сʱ)�����豸������ͬʱ��סUP��DOWN����������������ʾ
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

int MotoSpeedSet=10; //����ת��
u16 MotoSpeedActual; //ʵ��ת��
u8 DisplayContent=0; //�������ʾ������ 0��ʵ��ת�٣�1������ת�٣�2��ʵʱʱ��,3�������
int MotoSpeedTemp=6; //����ʱ��ת�� Ĭ������ٶ�Ϊ6
u16 DisplayTime;
u16 ErrNO;
u8 Flag_ErrNO2=0;
u8 Flag_ErrNO8=0; //�ۻ�����ʱ�����2400���ӣ�������ʾ��
u8 Flag_Err_OUT=0;
unsigned char  Seg7[14]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0xFF,0x92,0x86,0xAF};//11:ȫ��12:S

volatile u32 Timer1;	
u16 DOL=0,DOH=0;

u8 TIM6_100ms=0;
u8 TIM5_100ms_1=0;//����ɼ�����ʱ1s�ã��������2��

unsigned int Puls=0;
unsigned int Puls_100ms[5];
u8 TIM5_10ms=0; //TIM5�жϼ���
u8 TIM3_i=0;//TIM3�жϼ���
unsigned char F_Down=0,F_Up=0;
unsigned TIM4_100ms=0;
u8 SET_AccelerateRun=0,SET_AccelerateFinish=0; //���ٺ���ɼ��ٵı�ʶ
u8 SET_DecelerateRun=0,SET_DecelerateFinish=0; //���ٺ���ɼ��ٵı�ʶ
u8 SET_AccelerateFinish_1sdelay=0; //��ɼ��ٺ���ʱ1s�ı�־λ��Ϊ�˴������2�õ�
u8 Flag_KeyRun=0,Flag_KeyRunStop=0;//���ٺ���ɼ���ı�ʶ
typedef struct 
{
	u8 hour;
	u8 min;
	u8 sec;			
	//������������
	u16 w_year;
	u8  w_month;
	u8  w_date;
	u8  week;		 
}tm;					 
 tm timer;
//�·����ݱ�											 
u8 const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�	  
//ƽ����·����ڱ�
const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
u8 Flag_24C02Save=0;
u8 CurrentMinute=0; //��ǰ����
u8 FormerMinute=0; //ǰһ����
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
����: int main(void)
����: main������
����: ��
����: ��
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
	Timer3_Configuration();//100msһ��������ʾLED
	Timer2_Configuration();//�����������������ŷ�������
	Timer5_Configuration();//100ms��ȡһ�α��������룬����ٶ�,���Ӽ���
	Timer4_Configuration(); //�ۼӶ�ʱ��

	EXTI_Configuration();
	delay_init(72);
	

	
	AT24CXX_Init();	
	AT24CXX_WP=1;
	DOH=Seg7[0];
	DOL=0x0;
	setdol();
	setdoh();
	
  if(BKP_ReadBackupRegister(BKP_DR1) != 0xAB) //��ʼ����־
  {
    RTC_Configuration();
		RTC_Set(1000,1,1,0,21,0);//1000��1��1�� 0��0��0��
    BKP_WriteBackupRegister(BKP_DR1, 0xAB);
		BKP_WriteBackupRegister(BKP_DR4,0);  //�ۻ��������ӼĴ������    
  }	
	else //����Ѿ���ʼ��������BKP
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
		//24C02���ݱ���
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

		//������ť�������� �ͷź�ֹͣ����
		if(Button_down && F_Down) {F_Down=0;  TIM_Cmd(TIM4, DISABLE);  TIM4_100ms=0;}
		if(Button_up && F_Up) 		{F_Up=0;    TIM_Cmd(TIM4, DISABLE);  TIM4_100ms=0;}
		//RTCʱ��
		
		
		//�������������ʼ����
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

		

		//�������ͣ��������ֹͣ
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
		
		//������ŷ�����
		if(Err_IN)
		{
			delay_ms(5);
			//������ŷ�����
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
		//���û���ŷ�����
		if(!Err_IN)
		{
			delay_ms(5);
			//����ŷ���������
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
			//���û���ŷ����󣬲�������ɼ��ٺ�ʵ��ת�ٲ������趨ת�٣�����
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
		
		
		
		//����״̬�¿�ʼ�ۻ�����ʱ�䣬������BKP�У�ÿ����д��һ��
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

		//����ۻ�����ʱ�����2400���ӣ�����ʾEr08����ʱ���ͬʱ����UP��DOWN������չ���ʱ�䲢����������ʾ
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
			RTC_Set(1000,1,1,0,21,0);//1000��1��1�� 0��0��0��
			BKP_WriteBackupRegister(BKP_DR4,0);  //�ۻ��������ӼĴ������ 			
		}

		if(SET_AccelerateFinish)	
			TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
		else if(SET_AccelerateRun || SET_DecelerateRun)
				    TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedTemp)-1;	

	}

}


/*************************************************
����: void RCC_Configuration(void)
����: ��λ��ʱ�ӿ��� ����
����: ��
����: ��
**************************************************/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    //ʹ���ⲿ����
    RCC_HSEConfig(RCC_HSE_ON);
    //�ȴ��ⲿ�����ȶ�
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //����ⲿ���������ɹ����������һ������
    if(HSEStartUpStatus==SUCCESS)
    {
        //����HCLK��AHBʱ�ӣ�=SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        //PCLK1(APB1) = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);
        //PCLK2(APB2) = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);
        //����ADCʱ��Ƶ��
        RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
        //FLASHʱ�����
        //�Ƽ�ֵ��SYSCLK = 0~24MHz   Latency=0
        //        SYSCLK = 24~48MHz  Latency=1
        //        SYSCLK = 48~72MHz  Latency=2
        FLASH_SetLatency(FLASH_Latency_2);
        //����FLASHԤȡָ����
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        //PLL���� SYSCLK/1 * 9 = 8*1*9 = 72MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        //RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
        //����PLL
        RCC_PLLCmd(ENABLE);
        //�ȴ�PLL�ȶ�
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        //ϵͳʱ��SYSCLK����PLL���
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //�л�ʱ�Ӻ�ȴ�ϵͳʱ���ȶ�
        while(RCC_GetSYSCLKSource()!=0x08);
    }
		//����GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
		
    //����AFIO
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

	// �ı�ָ���ܽŵ�ӳ�� GPIO_Remap_SWJ_JTAGDisable ��JTAG-DP ���� + SW-DP
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

    //������595���ź��� �������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


		//USART1_TX	 �������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //USART1_RX	 ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
		//HSOE1 HSP1 HSP2 �������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//LED1234 �������
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

		//PA0(HSPIN0) �����������벶��
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
		//IN0-IN8 ��������
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
	
	 //����PA0��HSPIN0 ��Ϊ�Զ��жϣ��½����ж�
	 EXTI_ClearITPendingBit(EXTI_Line0);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	 //����PB4��Up ��Ϊ�Զ��жϣ��½����ж�
	 EXTI_ClearITPendingBit(EXTI_Line4);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

	 //����PB3��Down ��Ϊ�Զ��жϣ��½����ж�
	 EXTI_ClearITPendingBit(EXTI_Line3);
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
	 //����PD2��Set ��Ϊ�Զ��жϣ��½����ж�
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           //4����ռʽ���ȼ� 4�������ȼ�

	//PA0 �������岶�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;          //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//ָ���������ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //ָ����Ӧ���ȼ���0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);	

	//PA7 �Զ�ѡ���ⲿ�ж�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;          //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//ָ���������ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //ָ����Ӧ���ȼ���0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;          //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//ָ���������ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //ָ����Ӧ���ȼ���0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;          //ָ���ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	//ָ���������ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //ָ����Ӧ���ȼ���0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);



	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //ͨ������Ϊ����1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;	//ָ���������ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	   //�ж���Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //���ж�
	NVIC_Init(&NVIC_InitStructure); 						   //��ʼ�� 					 


	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM3�ж�
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM2�ж�
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM2�ж�
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM2�ж�
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM2�ж�
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		 //ָ����ռʽ���ȼ���1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			 //ָ����Ӧ���ȼ���2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //ʹ��TIM2�ж�
	NVIC_Init(&NVIC_InitStructure);	
}

void init_watchdog(void) {
	/* Check if the system has resumed from IWDG reset -------------------------*/
	//��鿴�Ź���λ��־
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
	  /* IWDGRST flag set */
		      
	  /* Clear reset flags */
	  //�����λ��־
	  RCC_ClearFlag();
	
	 //LCD_text(50,90,0xF800,0,"Watchdog SET!");
	}
	/* IWDG timeout equal to 280 ms (the timeout may varies due to LSI frequency
	  dispersion) -------------------------------------------------------------*/
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	//����IWDG���޸�
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* IWDG counter clock: 40KHz(LSI) / 256 = 156 Hz */
	//����IWDGʱ��
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	/* Set counter reload value to 624 */
	//���ö�ʱ������ֵ ����4s
	IWDG_SetReload(780);
	
	/* Reload IWDG counter */
	//����IWDG������
	IWDG_ReloadCounter();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	//ʹ��IWDG
	IWDG_Enable();
}

void setdol(void)  //���DOL���ݺ���	
{
	unsigned char i;
	for (i=0;i<16;i++)	
	{
		if (DOL&(1<<(15-i))) DODA1_1; else DODA1_0;
		DOCLK1_1;
		delay_us(100);
		DOCLK1_0;
	}
	//��595ˢ���������
	delay_us(100);
	DORCK1_1;
	delay_us(100);
	DORCK1_0;
}
void setdoh(void)  //���DOL���ݺ���	
{
	unsigned char i;

	for (i=0;i<16;i++)	
	{
		if (DOH&(1<<(15-i))) DODA2_1; else DODA2_0;
		DOCLK2_1;
		delay_us(100);
		DOCLK2_0;
	
	}

//��595ˢ���������
	delay_us(100);
	DORCK2_1;
	delay_us(100);
	DORCK2_0;
}

//Tim3 1msһ���жϣ�������ʾ
void Timer3_Configuration(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	    //��TIM5��ʱ����ʱ��
  
  TIM_DeInit(TIM3);		                                    //TIMx�Ĵ�������Ϊȱʡֵ
  
  TIM_TimeBaseStructure.TIM_Period = 10-1;		            //�Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);                       //�����Զ���װ�ؼĴ�����ARR��
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	                //����TIM5����ж�
  
  TIM_Cmd(TIM3, ENABLE);// �ر�ʱ��                                 //����ʱ�� 
}

//Tim2 ��ʱ�� ������ʱͣ����ʱ һ��1ms 2K
void Timer2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	    //��TIM2��ʱ����ʱ��
  
  TIM_DeInit(TIM2);		                                    //TIMx�Ĵ�������Ϊȱʡֵ
  
  TIM_TimeBaseStructure.TIM_Period =(u16)(342860/(unsigned long)MotoSpeedSet)-1;//500-1;		            //�Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler=9-1;               //TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);                       //�����Զ���װ�ؼĴ�����ARR��
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	                //����TIM3����ж�
  
  TIM_Cmd(TIM2, ENABLE);// 
}

void Timer4_Configuration(void) //100ms �����ۼ�
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	    //��TIM3��ʱ����ʱ��
 
  TIM_DeInit(TIM4);		                                    //TIMx�Ĵ�������Ϊȱʡֵ
  
  TIM_TimeBaseStructure.TIM_Period = 1000;	            //�Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM4, ENABLE);                       //�����Զ���װ�ؼĴ�����ARR��
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	                //����TIM3����ж�
  
	TIM_Cmd(TIM4, DISABLE);	                                //����ʱ�� 
}



//Tim5 10ms�ж�һ�Σ���ȡPuls��ֵ
void Timer5_Configuration(void) 
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	    //��TIM5��ʱ����ʱ��
  
  TIM_DeInit(TIM5);		                                    //TIMx�Ĵ�������Ϊȱʡֵ
  
  TIM_TimeBaseStructure.TIM_Period = 1000-1;		            //�Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler=7200 - 1;               //TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM5, ENABLE);                       //�����Զ���װ�ؼĴ�����ARR��
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	                //����TIM5����ж�
  
  TIM_Cmd(TIM5, ENABLE);// �ر�ʱ�� 

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
			   
	if(year%4==0) //�����ܱ�4����
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400���� 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}	
//�������ʱ��ת��Ϊ����
//��1970��1��1��Ϊ��׼
//1970~2099��Ϊ�Ϸ����
//����ֵ:0,�ɹ�;����:�������.
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec)
{		   
	u16 t;
	u32 seccount=0;
	if(syear<1970||syear>2099)return 1;	   
	for(t=1970;t<syear;t++)	//��������ݵ��������
	{
		if(Is_Leap_Year(t))seccount+=31622400;//�����������
		else seccount+=31536000;			  //ƽ���������
	}
	smon-=1;
	for(t=0;t<smon;t++)	   //��ǰ���·ݵ����������
	{
		seccount+=(u32)mon_table[t]*86400;//�·����������
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//����2�·�����һ���������	   
	}
	seccount+=(u32)(sday-1)*86400;//��ǰ�����ڵ���������� 
	seccount+=(u32)hour*3600;//Сʱ������
    seccount+=(u32)min*60;	 //����������
	seccount+=sec;//�������Ӽ���ȥ
													    
	//����ʱ��
    RTC_WaitForLastTask(); 
	RTC_SetCounter(seccount);
    RTC_WaitForLastTask();
	return 0;	    
}
//�õ���ǰ��ʱ��
//����ֵ:0,�ɹ�;����:�������.
u8 RTC_Get(void)
{
	static u16 daycnt=0;
	u32 timecount=0; 
	u32 temp=0;
	u16 temp1=0;	  
	   
     
    timecount = RTC_GetCounter();			 

	temp=timecount/86400;   //�õ�����(��������Ӧ��)
	if(daycnt!=temp)//����һ����
	{	  
		daycnt=temp;
		temp1=1970;	//��1970�꿪ʼ
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//������
			{
				if(temp>=366)temp-=366;//�����������
				else {temp1++;break;}  
			}
			else temp-=365;	  //ƽ�� 
			temp1++;  
		}   
		timer.w_year=temp1;//�õ����
		temp1=0;
		while(temp>=28)//������һ����
		{
			if(Is_Leap_Year(timer.w_year)&&temp1==1)//�����ǲ�������/2�·�
			{
				if(temp>=29)temp-=29;//�����������
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//ƽ��
				else break;
			}
			temp1++;  
		}
		timer.w_month=temp1+1;//�õ��·�
		timer.w_date=temp+1;  //�õ����� 
	}
	temp=timecount%86400;     //�õ�������   	   
	timer.hour=temp/3600;     //Сʱ
	timer.min=(temp%3600)/60; //����	
	timer.sec=(temp%3600)%60; //����
	timer.week=RTC_Get_Week(timer.w_year,timer.w_month,timer.w_date);//��ȡ����   
	return 0;
}	 
//������������ڼ�
//��������:���빫�����ڵõ�����(ֻ����1901-2099��)
//������������������� 
//����ֵ�����ں�																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{	
	u16 temp2;
	u8 yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// ���Ϊ21����,�������100  
	if (yearH>19)yearL+=100;
	// ����������ֻ��1900��֮���  
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




//��ʱ��2 ������ʱͣ������
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&0X0001)//����ж�
	{	
		HSP1=!HSP1;
		HSP2=!HSP2;
	}
	
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	 
}


//��ʱ��5 10msһ��  ���ڼ����ٶȺͼӼ��ٿ���
void TIM5_IRQHandler(void)
{
	unsigned char x;
	u16 MotoSpeedActual_temp=0;
	if(TIM5->SR&0X0001)//����ж�
	{	
		Puls_100ms[TIM5_10ms]=Puls;
		TIM5_10ms++;
		Puls=0;
		
		//SET_AccelerateFinish��λ��2s����λSET_AccelerateFinish_1sdelay
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
		
		//���ٿ���
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
		//���ٿ���
		if(SET_DecelerateRun==1)
		{
			MotoSpeedTemp=(MotoSpeedSet/20!=0)? (MotoSpeedTemp-MotoSpeedSet/20) : (MotoSpeedTemp-1);
			if(MotoSpeedTemp<=6) ///�˴��޸�20170701
			{
				MotoSpeedTemp=6;
				SET_DecelerateRun=0;
				DOL &= ~(1<<0);
				DOL &= ~(1<<Work_LED);
				setdol();
			}			
		}
	}
	TIM5->SR&=~(1<<0);//����жϱ�־λ 	 
}






//Tim7 ÿ��Reamer1Advanceduanxu/10S��ʱ���ж�һ�Σ��ж��о��ǹر���ת�͸ף���ʱ0.5s����ǰ��
void TIM7_IRQHandler(void)
{
	if(TIM7->SR&0X0001)//����ж�
	{	

	}
	
	TIM7->SR&=~(1<<0);//����жϱ�־λ 	 
}


//TIM3 ��ʾ�ж� 100ms��ʾһλ
void TIM3_IRQHandler(void)
{
	int DisNum;
	char Num0,Num1,Num2,Num3;
	if(TIM3->SR&0X0001)//����ж�
	{
		TIM3_i++;
		if(TIM3_i>=14)
			TIM3_i=10;
		switch(DisplayContent)
		{
			case 0 : DisNum=(u16)MotoSpeedActual; break; //ʵ��ת��
			case 1 : DisNum=(u16)(MotoSpeedSet); break; //�趨ת��
			case 2 : DisNum=(u16)DisplayTime; break; //ʵʱʱ��
			case 3 : DisNum=(u16)ErrNO; break; //�������
			default : break;
		}
		if(TIM3_i==10)
		{
			Num3=DisNum/1000%10;
			GPIO_SetBits(GPIOA, GPIO_Pin_15);
			switch(DisplayContent)
			{
				case 0 : DOH=DOH=Seg7[10];break; //ʵ���ٶȲ���ʾ���λ
				case 1 : DOH=Seg7[11];break; //�����ٶ����λ��ʾS
				case 2 : DOH=Seg7[Num3]; break; //ʱ�����λ������ʾ
				case 3 : DOH=Seg7[12]; break;  //��������һλ E
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
				case 0 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //ʵ���ٶ�
				case 1 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //�趨�ٶ�
				case 2 : DOH=(Num2==0)? Seg7[10] : Seg7[Num2];break; //ʱ��
				case 3 : DOH=Seg7[13]; break;  //�������ڶ�λr
				default:break;
			}
			DOH=DisplayContent==2? (DOH & (~(1<<7))):(DOH);//��ʾʱ���ʱ���λ��ʾС����
			setdoh();
			GPIO_ResetBits(GPIOC, GPIO_Pin_11);
		}

		if(TIM3_i==12)
		{
			Num1=DisNum/10%10;
			GPIO_SetBits(GPIOC, GPIO_Pin_11);
			DOH=Seg7[Num1];
			DOH=DisplayContent<2? (DOH & (~(1<<7))):(DOH); //ʵ���ٶȺ��趨�ٶ���ʾС����
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
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	 
}


//�Զ��жϷ������
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
				DOL &=~(1<<0); //�ر��ŷ�
				DOL &= ~(1<<Work_LED);
				setdol();
			}
		}	
  	EXTI_ClearFlag(EXTI_Line4);			       //����жϱ�־�����룩
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
				DOL |=1<<0; //�����ŷ�
				setdol();
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			}
				
			if(MotoSpeedSet>=300)
			{
				MotoSpeedSet=300;
				//TIM2->ARR=(u16)(342860/(unsigned long)MotoSpeedSet)-1;
			}
				
		}	
  	EXTI_ClearFlag(EXTI_Line3);			       //����жϱ�־�����룩
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
  	EXTI_ClearFlag(EXTI_Line2);			       //����жϱ�־�����룩
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
  	EXTI_ClearFlag(EXTI_Line0);			       //����жϱ�־�����룩
   	EXTI_ClearITPendingBit(EXTI_Line0);
	 }	
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//����ж�
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
					DOL |=1<<0; //�����ŷ�
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
					DOL &=~(1<<0); //�ر��ŷ�
					DOL &= ~(1<<Work_LED);
					setdol();
				}
			}			
		}

	}
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	 
}

