#include "stm32f4xx_conf.h"
#include "hw_driver.h"
#include "usart.h"
#include "delay.h"
#include "iwdg.h"
#include "stmflash.h"
#include "string.h"
#include "spi2.h"
#include "socket.h"
#include "w5500.h"
#include "device.h"
#include "ult.h"
#include "sdio_sdcard.h"
#include "math.h"

#include <stdlib.h>  // 解决 malloc 和 free 的隐式声明警告


#include "core_cm4.h"  // Cortex-M4/M7 内核头文件
void DWT_Delay_Init(void)
{
    // 使能 DWT 计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;  // 清零计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // 使能 CYCCNT 计数
}
// uint32_t DWT_GetTickUS(void)
// {
//     return DWT->CYCCNT / (SystemCoreClock / 1000000);  // 转换为微秒
// }
// uint32_t DWT_GetTickMS(void)
// {
//     return DWT->CYCCNT / (SystemCoreClock / 1000);  // 转换为毫秒
// }

uint32_t DWT_GetTickUS(void)
{
	static uint32_t m_us;
	static uint32_t lastCYCCNT;
	const uint32_t cnt = DWT->CYCCNT;

	if(lastCYCCNT>cnt){
		m_us+=(4294967296 -lastCYCCNT + cnt)/(SystemCoreClock / 1000000);
		
	}else{
		m_us+=(cnt-lastCYCCNT)/(SystemCoreClock / 1000000);
	}
	lastCYCCNT = cnt;
    return m_us;  // 转换为微秒
}
uint32_t DWT_GetTickMS(void)
{
	static uint32_t m_ms;
	static uint32_t lastCYCCNT;
	const uint32_t cnt = DWT->CYCCNT ;

	if(lastCYCCNT>cnt){
		m_ms+=(4294967296-lastCYCCNT+cnt)/ (SystemCoreClock / 1000);
		
	}else{
		m_ms+=(cnt-lastCYCCNT)/ (SystemCoreClock / 1000);
	}
	lastCYCCNT = cnt;
    return m_ms;  // 转换为毫秒
}
// // 使用示例
// DWT_Delay_Init();  // 初始化 DWT
// uint32_t time_us = DWT_GetTickUS();  // 读取微秒级时间
// uint32_t time_ms = DWT_GetTickMS();  // 读取毫秒级时间

int time_mean;
typedef struct {
    float buffer[50];
    int capacity;
    int size;
    int head;
    int tail;
    float sum;
} MovingAverage;
void MovingAverage_Init(MovingAverage *ma) {
    ma->capacity = 50;
    ma->size = 0;
    ma->head = 0;
    ma->tail = 0;
    ma->sum = 0;
}
void MovingAverage_Add(MovingAverage *ma, float value) {
    if (ma->size < ma->capacity) {
        ma->buffer[ma->tail] = value;
        ma->sum += value;
        ma->size++;
    } else {
        ma->sum -= ma->buffer[ma->head];
        ma->buffer[ma->tail] = value;
        ma->sum += value;
        ma->head = (ma->head + 1) % ma->capacity;
    }
    ma->tail = (ma->tail + 1) % ma->capacity;
}
float MovingAverage_Get(MovingAverage *ma) {
    if (ma->size == 0) return 0;
    return ma->sum / ma->size;
}

double trans(double in_min, double in_max, double out_min, double out_max, double input) // 值变换函数
{
	return out_min + (out_max - out_min) * (input - in_min) / (in_max - in_min);
}

//函数声明//---------------------------------------------
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);
void TIM7_Int_Init(u16 Per,u16 Psc);
void TIM3_IRQHandler(void);	//20us	
void TIM5_IRQHandler(void);	//1ms
void TIM7_IRQHandler(void);	//10ms
void UDPService(void);


//变量//--------------------------------------------------
//TIM7
uint8_t tim7_val = 0;
uint8_t Tim_NetSend_flag_val = 0;
uint8_t Tim_10ms_flag = 0;
uint8_t Tim_100ms_flag = 0;
uint8_t Tim_500ms_flag = 0;
uint8_t Tim_1000ms_flag = 0;
uint8_t Tim_NetSend_flag = 0;

int led = 0;

extern const uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__((at(0x2001FFE0)));
extern uint16_t dac1_buffer[DAC_BUFFER_SIZE] __attribute__((at(0x2001FFA0)));  // DMA传输数据缓冲区 
extern uint16_t dac2_buffer[DAC_BUFFER_SIZE] __attribute__((at(0x2001FFC0)));  // DMA传输数据缓冲区
extern uint8_t Glb_DIState[DI_BUFFER_SIZE];
extern uint8_t Glb_DOState[DO_BUFFER_SIZE];

//-------------------------------------------------------
// 主程序
//-------------------------------------------------------
uint8_t sendbuff[1024]={0};
uint8_t sendip[4]={192,168,100,233};
uint16_t sendport =7000;

u16 len=0;
uint8_t tempIP[4];
uint16_t tempPort;
int UDP_FALG = 0;
int OUTPUT_FALG = 1;
uint8_t recvbuffer[2048] = {0};

int Amp = 1800;
int exAOoutput = 1800;
int AOoutput = 1800;

long long int Clock = 0;

int T = 20000;
double p = 0.4;
double p_init = 0.4;
uint32_t time_now = 0;
uint32_t ex_time = 0;
uint32_t dt = 0;

int int_xbar;
// double Kp = 0.25;
double Kp = 0.2;
double x_bar = 0;
// double Ki = 0.0001;
double Ki = 0.00001;
double integral = 0, maxIntegral=0.1;
double u = 0;



// ********************************

// int int_xbar;
// // double Kp = 0.25;
// double Kp = 0.2;
// double x_bar = 0;
// // double Ki = 0.0001;
// double Ki = 0.00001;
// // double Ki = 0;
// double integral = 0, maxIntegral=0.1;
// double u = 0;








void UDP_recvpackProce(void);
int DO_recvBuffer[8] = {0};
int AO_recvBuffer[2] = {0};
int STATE_recvBuffer[5] = {0};

void UDP_replypackProce(void);
void IO_control(void);

int UDP_time_last = 0;
int flag_10ms = 0;

// int A;


//状态机
#define STOP 0
#define WAIT 1
#define WORK 2
#define DEBUG 3
int STATAE = 0;
int SUB_STATAE = 0;

int RESET_FLAG;

uint32_t current_time = 0;
int high_freq_component = 0; // 100Hz正弦波，幅值50

int main(void) 
{
    SystemInit();   // 系统时钟配置（假设已配置为168MHz） 
	delay_init(168);
	DI_DO_Init();
	LED_RUN = LED_ON;
	LED_ERR = LED_ON;
	USART1_Init(115200);
    // 配置ADC  
    ADC_Config();  
    // 配置DAC  
    DAC_Config();  
    // 配置DMA  
    ADC_DMA_Config();  
    // 配置DMA  
    DAC_DMA_Config(); 
    // 配置NVIC  
    NVIC_Configuration();  	
	// 配置TIM6-DAC触发  
	TIM6_Config();
    // 生成1kHz三角波数据  
    //GenerateTriangleWave(); 	
    // 启动ADC1和ADC2的转换  
    ADC_SoftwareStartConv(ADC1);  
    // 启动DAC  
    DAC_Cmd(DAC_Channel_1, ENABLE);  
    DAC_Cmd(DAC_Channel_2, ENABLE);
	
	TIM7_Int_Init(100-1,8400-1);	
	W5500_Init();
	UDPService();
	//初始化SD卡
	// while(SD_Init()){}       
	// SD_Init();
	DWT_Delay_Init();  // 初始化 DWT
	//***************test***************
	Glb_DOState[0] = 0;
	Glb_DOState[1] = 0;
	Glb_DOState[2] = 0;
	Glb_DOState[3] = 0;
	Glb_DOState[4] = 0;
	Glb_DOState[5] = 0;
	Glb_DOState[6] = 0;
	Glb_DOState[7] = 0;

	//***************test***************

	MovingAverage ma;
    MovingAverage_Init(&ma);

	for(int i=0;i<1024;i++)
		sendbuff[i] = i;

	

	while (1) 
	{		
		// sendto(udp_socket,sendbuff,1024,sendip,sendport);
		// delay_ms(1);
		// delay_ms(50);
		// LED_RUN = LED_ON;
		// delay_ms(50);
		// LED_RUN = LED_OFF;

		if(flag_10ms)
		{
			UDPService();
			flag_10ms = 0;
		}
		// UDP接收+回复
		if(UDP_FALG==1)
		{
			UDP_recvpackProce();

			// 状态位
			STATAE = STATE_recvBuffer[2];

			// IO_control();
			UDP_replypackProce();
			sendto(udp_socket,sendbuff,27,tempIP,tempPort);		
			// sendto(udp_socket,recvtbuffer,11,tempIP,tempPort);		//回环发送
			memset(recvbuffer,0,sizeof(recvbuffer));						//清空数组
			UDP_FALG = 0;
		}


	




		// -----------

		
		switch (STATAE)
		{
		case STOP:
			for (int i = 0; i < 8;i++)
				Glb_DOState[i] = 0;
			Amp = 1850;
			OUTPUT_FALG = 0;
			SUB_STATAE = 0;
			/* code */
			break;

		

		case WAIT: // 分别调整至中位
			switch (SUB_STATAE)
			{
			case 0:
				Glb_DOState[0] = 1;
				Glb_DOState[1] = 0;
				Glb_DOState[2] = 1;

				// 生成高频抖动信号（例如：100Hz，幅值50）
				current_time = DWT_GetTickUS();
				high_freq_component = 10 * sin(2 * 3.1415926 * 200 * current_time / 1e6); // 100Hz正弦波，幅值50

				if (trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) > 0.2)
					dac2_buffer[0] = 1885 + high_freq_component;
					// dac2_buffer[0] = 2585 + high_freq_component;
				else if (trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) < -0.2)
					dac2_buffer[0] = 1815 + high_freq_component;
					// dac2_buffer[0] = 1185 + high_freq_component;
				else
					dac2_buffer[0] = 1850 + high_freq_component; // 中位值加上抖动

				if (trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) > -0.2 && 
					trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) < 0.2)
					SUB_STATAE = 1;
				break;

			case 1:
				Glb_DOState[0] = 0;
				Glb_DOState[1] = 1;
				Glb_DOState[2] = 1;

				// 生成高频抖动信号
				current_time = DWT_GetTickUS();
				high_freq_component = 10 * sin(2 * 3.1415926 * 200 * current_time / 1e6);

				if (trans(2170, 3970, -3.08, 3.06, adc_buffer[2]) > 0.2)
					dac2_buffer[0] = 1885 + high_freq_component;
				else if (trans(2170, 3970, -3.08, 3.06, adc_buffer[2]) < -0.2)
					dac2_buffer[0] = 1815 + high_freq_component;
				else
					dac2_buffer[0] = 1850 + high_freq_component; // 中位值加上抖动

				if (trans(2170, 3970, -3.08, 3.06, adc_buffer[2]) > -0.2 && 
					trans(2170, 3970, -3.08, 3.06, adc_buffer[2]) < 0.2)
					SUB_STATAE = 2;
				break;

			case 2:
				// 复位完成，可以进入下一个状态或停止抖动
				// 这里你可以选择停止抖动或保持
				Glb_DOState[0] = 0;
				Glb_DOState[1] = 0;
				Glb_DOState[2] = 0;

				if (trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) < -0.2 || 
					trans(2420, 1740, -3.4, 3.34, adc_buffer[1]) > 0.2)
					SUB_STATAE = 0;
				if (trans(2420, 1740, -3.4, 3.34, adc_buffer[2]) < -0.2 || 
					trans(2420, 1740, -3.4, 3.34, adc_buffer[2]) > 0.2)
					SUB_STATAE = 1;
				break;

			default:
				break;
			}
			
			break;



		case WORK:
			/* code */
			SUB_STATAE = 0;

			IO_control();
			
			// AO输出使能
			if(OUTPUT_FALG==1)
			{
				time_now = DWT_GetTickUS();  // 读取微秒级时间
				dt = time_now - ex_time;

				if(time_now-time_mean>=1e6/(40*recvbuffer[8]))
				{
					time_mean = time_now;
					MovingAverage_Add(&ma, adc_buffer[1]);
					int_xbar = MovingAverage_Get(&ma);

				}

				
				if(dt>=20)
				{
					// adc_buffer[1]//位移传感器1
					// adc_buffer[2]//位移传感器2

					x_bar=trans(0,4095,-5,5,int_xbar);


					integral+=x_bar;

					if(integral>100000)
						integral = 100000;
					if(integral<-100000)
						integral = -100000;

					//===============控制器===============
					u = Kp * x_bar + Ki * integral;
					p = p_init - u;
					if(p>0.8)
						p = 0.8;
					if(p<0.3)
						p = 0.3;

					//===============控制器===============
					AOoutput = (time_now % T <= p * T) ? Amp : 3700-Amp;
					// AOoutput = (time_now % T <= p * T) ? Amp : Amp-1800;
					if(AOoutput!=exAOoutput)
					{
						dac2_buffer[0] = AOoutput;
						exAOoutput=AOoutput;
					}
					ex_time = time_now;
				}

// *********************************************************************

			// AO输出使能
			// if(OUTPUT_FALG==1)
			// {
			// 	time_now = DWT_GetTickUS();  // 读取微秒级时间
			// 	dt = time_now - ex_time;

			// 	if(time_now-time_mean>=1e6/(40*recvbuffer[8]))
			// 	{
			// 		time_mean = time_now;
			// 		MovingAverage_Add(&ma, adc_buffer[2]);
			// 		int_xbar = MovingAverage_Get(&ma);
			// 	}

			// 	if(dt>=20)
			// 	{
			// 		// adc_buffer[1]//位移传感器1
			// 		// adc_buffer[2]//位移传感器2

			// 		x_bar=trans(2048,4095,5,-5,int_xbar);
			// 		integral+=x_bar;

			// 		if(integral>100000)
			// 			integral = 100000;
			// 		if(integral<-100000)
			// 			integral = -100000;

			// 		//===============控制器===============
			// 		u = Kp * x_bar + Ki * integral;
			// 		p = p_init - u;
			// 		if(p>0.8)
			// 			p = 0.8;
			// 		if(p<0.3)
			// 			p = 0.3;

			// 		//===============控制器===============
			// 		AOoutput = (time_now % T <= p * T) ? Amp : 3700-Amp;
			// 		// AOoutput = (time_now % T <= p * T) ? Amp : Amp-1800;
			// 		if(AOoutput!=exAOoutput)
			// 		{
			// 			dac2_buffer[0] = AOoutput;
			// 			exAOoutput=AOoutput;
			// 		}
			// 		ex_time = time_now;
			// 	}


			}

			break;


		default:
			break;
		}


		

		// u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt);	//读卡
		// u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt);	//写卡
		// SD_ReadDisk(SendbufA,((curSigNumber-1)*1200+SigBlockCnt),4); SigBlockCnt+=4;
		// ADC_SoftwareStartConv(ADC1);
        // 主循环  
    }    
}  
void IO_control(void)
{
	// DO
	Glb_DOState[0] = DO_recvBuffer[0];
	Glb_DOState[1] = DO_recvBuffer[1];
	Glb_DOState[2] = DO_recvBuffer[2];
	Glb_DOState[3] = DO_recvBuffer[3];
	Glb_DOState[4] = DO_recvBuffer[4];
	Glb_DOState[5] = DO_recvBuffer[5];
	Glb_DOState[6] = DO_recvBuffer[6];
	Glb_DOState[7] = DO_recvBuffer[7];

	// 两路AO
	Amp = AO_recvBuffer[0];			//AO0输出幅值
	// Amp = 1800 + A;
	if(OUTPUT_FALG==0)				//AO0恒值输出
		dac2_buffer[0] = Amp;

	// 状态量
	OUTPUT_FALG = STATE_recvBuffer[0];
	T = (1.0/STATE_recvBuffer[1])*1e6;

}


//网络/--------------------------------------------------------------------------------------------------
// ===================解析接收报文===================
void UDP_recvpackProce(void)
{
	// 八路DO recvbuffer[0]
	for(int i = 0; i < 8; i++) 
		DO_recvBuffer[i] = (recvbuffer[0] & (1 << i)) ? 1 : 0;

	// 一路AO    [5]
	AO_recvBuffer[0] = recvbuffer[1] * 256 + recvbuffer[2];
	// A = AO_recvBuffer - 2048;

	// 状态量   [6]~[11]
	STATE_recvBuffer[0] = recvbuffer[3];							//OUTPUT_FALG
	STATE_recvBuffer[1] = recvbuffer[4];							//f
	STATE_recvBuffer[2] = recvbuffer[5];							//STATE


	//memset(recvbuffer,0,sizeof(recvbuffer));						//清空接收缓存
}

// ===================构造发送报文===================
void UDP_replypackProce(void)
{
	//DO    [0]
    sendbuff[0]=0;//把DO八个字节凑成一个字节
    for(int j=0;j<8;j++)
        sendbuff[0]|=Glb_DOState[j]>0?1<<j:0x00;

	// 四路电压AI    [1]~[8]
	sendbuff[1] = adc_buffer[0] / 256;//AI0	AO0输出
	sendbuff[2] = adc_buffer[0] % 256;

	sendbuff[3] = adc_buffer[1] / 256;//AI1	激光位移传感器1
	sendbuff[4] = adc_buffer[1] % 256;

	sendbuff[5] = adc_buffer[2] / 256;//AI2	激光位移传感器2
	sendbuff[6] = adc_buffer[2] % 256;

	sendbuff[7] = adc_buffer[3] / 256;//AI3	阀芯位移
	sendbuff[8] = adc_buffer[3] % 256;

	// 三路电流AI    [9]~[14]
	sendbuff[9] = adc_buffer[5] / 256;//AI5	伺服阀压力传感器1
	sendbuff[10] = adc_buffer[5] % 256;

	sendbuff[11] = adc_buffer[6] / 256;//AI6	伺服阀压力传感器2
	sendbuff[12] = adc_buffer[6] % 256;

	sendbuff[13] = adc_buffer[7] / 256;//AI7	液压站压力传感器
	sendbuff[14] = adc_buffer[7] % 256;

	// 一路AO    [15]~[16]
	sendbuff[15] = Amp / 256;//AO0	伺服阀线圈
	sendbuff[16] = Amp % 256;

	

	// 状态量    [17]~[25]
	sendbuff[17] = OUTPUT_FALG;		//OUTPUT_FALG
	sendbuff[18] = STATE_recvBuffer[1];	//f
	sendbuff[19] = STATAE;
	sendbuff[20] = SUB_STATAE;

}

void UDPService(void)//报文指令处理，0 = socket0
{
	// u16 len=0;
	// uint8_t tempIP[4];
	// uint16_t tempPort;
	switch(getSn_SR(udp_socket))								// 获取socket0的状态
	{
		case SOCK_UDP:											// Socket处于初始化完成(打开)状态
			if(getSn_IR(udp_socket) & Sn_IR_RECV)				//接收中断
			{
				setSn_IR(udp_socket, Sn_IR_RECV);				// Sn_IR的RECV位置1
			}
			// 数据回环测试程序：数据从远程上位机发给W5500，W5500接收到数据后再回给远程上位机
			if((len=getSn_RX_RSR(udp_socket))>0)
			{ 	
				len -= 8;
				recvfrom(udp_socket,recvbuffer, 7, tempIP,&tempPort);	//udp W5500接收来自远程上位机的数据，并通过SPI发送给MCU
//报文处理------------------------------------------------------------------------------------------------------------------------------
				UDP_FALG = 1;
				//------------------------------------------------------------------------------------------------------------------------------				
				// sendto(udp_socket,netbuffer,7,tempIP,tempPort);		//回环发送
			}				
		break;
		case SOCK_CLOSED:										// Socket处于关闭状态
				socket(udp_socket,Sn_MR_UDP,7000,0);			// 打开Socket0，并配置为UDP模式，打开一个本地端口7000
		break;
	}
	
}

//网络/--------------------------------------------------------------------------------------------------
void EXTI9_5_IRQHandler(void)  
{  
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)  
    {  
		// UDPService();       				 //处理各类中断标志位
        EXTI_ClearITPendingBit(EXTI_Line7);  // 清除中断标志  		
    }  
} 

//定时器/--------------------------------------------------------------------------------------------------------
void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM5时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM5
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器5更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器5
}
void TIM5_IRQHandler(void)	//1ms
{
	// if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	// {
	// 	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
	// }
	Clock += 20;
}
void TIM7_Int_Init(u16 Per,u16 Psc)		
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM2时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = Per; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=Psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器2更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器2
}
void TIM7_IRQHandler(void)	//10ms
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		
		tim7_val++;
		flag_10ms = 1;					//10ms
		
		if(Tim_NetSend_flag_val>9)			//网络定时1s发送
			Tim_NetSend_flag=1;
			
		if((tim7_val%10)==9)				//100ms
		{
			Tim_100ms_flag = 1;
			if(Tim_NetSend_flag_val<=10)
				Tim_NetSend_flag_val++;			
		}	
		if((tim7_val%50)==49)				//500ms
		{		
			Tim_500ms_flag = 1;

		}	
		if(tim7_val>=100)					//1000ms
		{
			tim7_val=0;
			Tim_1000ms_flag = 1;

		}			
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);	
	}
}


 



















