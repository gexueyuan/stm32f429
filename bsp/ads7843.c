#ifndef TOUCH_H
#define TOUCH_H
#define SPI   0           //通过宏定义来选择SPI驱动，还是IO口模拟
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "math.h"
#include "TFT.h"
#define TCS_HIGH     GPIO_SetBits(GPIOB,GPIO_Pin_12)   // NSS Soft Mode 
#define TCS_LOW      GPIO_ResetBits(GPIOB,GPIO_Pin_12) // NSS Soft Mode
#define PEN          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) //INT state
#define TOUT         GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define TDIN_HIGH    GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define TDIN_LOW     GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define TCLK_HIGH    GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define TCLK_LOW     GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define TXMAX        4000         //根据设定的校准值，超出范围 数据无效
#define TYMAX        4000
#define TXMIN        100
#define TYMIN        100
#define SCREEN_W      240
#define SCREEN_H      320
#define ERROR_RANGE   100
#define KEY_UP        0x01 
#define KEY_DOWN      0x00
#define CHX      0xd0
#define CHY      0x90
#define EXTI_ENABLE		     EXTI->IMR|=0X0001          //开启线0中断
#define EXTI_DISABLE       EXTI->IMR&=~0X0001        //关闭线0中断         //ads7843芯片在第一个时钟的上升沿输入，第一个时钟的
typedef struct sldkf                         //下降沿输出，所以SPI要设置为1时钟，上升沿。在读取的时候，
{                                            //也是在1时钟，上升沿读取。但ads是在下降沿输出，所以，第一个值，是废值
	u16 x0,y0;     //原始坐标,即AD值         //所以左移3位
	u16 x,y;       //最终坐标，像素点值
	u8  flag;      //当前状态，其实就是判断中断否发生的标志
	float xfac,yfac,xoff,yoff; //偏移参数
}Hand;
Hand pence;
u16 TX,TY;
int ABS(int x)
{
	return x>0?x:-x;
}
void Touch_SPI_inti()  //SPI驱动ADS所用到的初始化
{
	SPI_InitTypeDef spi;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	EXTI_InitTypeDef exit;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	gpio.GPIO_Pin=GPIO_Pin_12;  //nss 推挽输出
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	gpio.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_15; // sck，mosi 复用推挽输出
	gpio.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&gpio);
	
	gpio.GPIO_Pin=GPIO_Pin_14;             //miso 浮空输入
	gpio.GPIO_Mode=GPIO_Mode_IPU;
	//	gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;//miso 配置为复用输出，或者浮空输入效果没差别，不解
	GPIO_Init(GPIOB,&gpio);	
	
	SPI_Cmd(SPI2,DISABLE);
	spi.SPI_Direction=SPI_Direction_2Lines_FullDuplex;  //全双工
	spi.SPI_Mode=SPI_Mode_Master;                       //主模式 
	spi.SPI_DataSize=SPI_DataSize_8b;                  //数据16位 
	spi.SPI_CPOL=SPI_CPOL_Low;                         //时钟空闲高电源
	spi.SPI_CPHA=SPI_CPHA_1Edge;                        //1个边沿捕捉
	spi.SPI_NSS=SPI_NSS_Soft;                           //NSS软件模式?
	spi.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_64;    //64分频，
	spi.SPI_FirstBit=SPI_FirstBit_MSB;                  //高位在前
	spi.SPI_CRCPolynomial=7;
	SPI_Init(SPI2,&spi);
	SPI_Cmd(SPI2,ENABLE);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0); //选择PB.0作为中断0的输入
	
	exit.EXTI_Line=EXTI_Line0;               //外部中断0
	exit.EXTI_Mode=EXTI_Mode_Interrupt;       //中断
	exit.EXTI_Trigger=EXTI_Trigger_Falling;   //下降沿触发
	exit.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exit);
	
	nvic.NVIC_IRQChannel=EXTI0_IRQChannel;  //nvic 中断配置
 	nvic.NVIC_IRQChannelPreemptionPriority=0;
	nvic.NVIC_IRQChannelSubPriority=2;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic);
}
u8 SPI2_Byte(u8 cmd)
{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	
	SPI_I2S_SendData(SPI2,cmd);
	
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	
	return SPI_I2S_ReceiveData(SPI2);
}
void Touch_IO_inti()  //IO 口模拟
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	EXTI_InitTypeDef exit;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB||RCC_APB2Periph_AFIO,ENABLE); //不能一起用，具体原因不知道
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	
	gpio.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15; //cs,clk,mosi
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	gpio.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_14;     //上拉输入 在spi里面已经使能了PB的时钟,int,miso 
	gpio.GPIO_Mode=GPIO_Mode_IPU;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0); //选择PB.0作为中断0的输入
	
	exit.EXTI_Line=EXTI_Line0;               //外部中断0
	exit.EXTI_Mode=EXTI_Mode_Interrupt;       //中断
	exit.EXTI_Trigger=EXTI_Trigger_Falling;   //下降沿触发
	exit.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exit);
	
	nvic.NVIC_IRQChannel=EXTI0_IRQChannel;  //nvic 中断配置
 	nvic.NVIC_IRQChannelPreemptionPriority=0;
	nvic.NVIC_IRQChannelSubPriority=2;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic);
}
#if SPI
void CMD_Write(u8 cmd)   //SPI 写
{
	SPI2_Byte(cmd);
}
u16 CMD_Read()          //SPi 读
{
	u16 ans=0,temp;
	temp=SPI2_Byte(0x00);
	ans=temp<<8;
	temp=SPI2_Byte(0x00);
	ans|=temp;
	ans>>=3;
	return ans&0x0fff;
}
#else 
void CMD_Write(u8 num)     //IO 模拟
{  
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TDIN_HIGH;  
		else TDIN_LOW;   
		num<<=1;    
		TCLK_LOW;//上升沿有效	   	 
		TCLK_HIGH;      
	} 			    
}

u16 CMD_Read()      //IO 模拟
{
	u16 i,ans;
	ans=0;
	for(i=0;i<12;i++)
	{
		ans<<=1;
		TCLK_HIGH;
		TCLK_LOW;
		if(TOUT)ans++;
	}
	return ans;
}
#endif
u8 Read_IO_ADS()  //读取一次
{
	TCS_LOW;
	
	CMD_Write(CHX);   
	#if SPI          //对SPI来说，采样的速率太快是边沿出现散点的原因
	SysTick_us(30); //对SPI来说，频率为64分频时，，适当的延时可以达到很好的效果，用的SPI驱动，所以不要用下降沿清除BUSY
	                //硬件来驱动，时序基本上不用考虑，唯一要考虑的是SPI的频率过高，可能导致采样不准
	               //所以，SPI初始化完成后的时序是不用考虑的，看ads的时序图，也可以看出这一点
	#else    //IO模拟必须要下降沿，原因不明,下降沿后，效果好很多
	         //如果没有，屏幕区域基本上集中在左上角1/4区域?
	SysTick_us(10);//经过研究时序图发现，此处给一个下降沿，是为了清除BUSY标志，同时启动传输
 	TCLK_HIGH;     //如果没有，那么在CMD_Read函数里面的读到的数的第12位会恒为0
	SysTick_us(10);//这样解释了，为什么X,Y的范围都会减半。在CMD_Read函数里面，在第一个下降沿过后，数据才开始传输
 	TCLK_LOW;     //所以，第一个下降沿的作用就是启动传输与清楚BUSY标志，所以在第一个下降沿得到的数肯定为0（因为刚刚开始传）
	SysTick_us(10);
	#endif
	TX=CMD_Read();
	
	CMD_Write(CHY);
	#if  SPI
	SysTick_us(30);
	#else         //<span style="background-color: rgb(255, 255, 255); ">IO模拟必须要下降沿，原因不明</span>

	
	SysTick_us(10);
 	TCLK_HIGH;
	SysTick_us(10);
 	TCLK_LOW;
	SysTick_us(10);
	#endif
	TY=CMD_Read();
	
	TCS_HIGH;
	
	if(TX>TXMAX||TY>TXMAX||TX<TXMIN||TY<TYMIN)
	{
		return 0;
	}
	return 1;
}
void Read_IO_XY(u16 *x,u16 *y)
{
	u16 xy[2][10],cnt,i,j,temp,t,XY[2];
	cnt=0;
	do                    //采集10个数，取中间的4个
	{
		if(Read_IO_ADS()) //采集10次合法数据
		{
			xy[0][cnt]=TX;
			xy[1][cnt]=TY;
			cnt++;
		}
	}while(PEN==0&&cnt<9);
	if(cnt<9)
	{
		return ;
	}
	for(t=0;t<2;t++)
	{
		for(i=0;i<cnt;i++)   //选择排序 
		{
			temp=i;
			for(j=i+1;j<cnt;j++)
			{
				if(xy[t][j]<xy[t][temp])
				{
					temp=j;
				}
			}
			if(temp!=i)
			{
				xy[t][i]^=xy[t][temp];
				xy[t][temp]^=xy[t][i];
				xy[t][i]^=xy[t][temp];
			}
		}
		XY[t]=(xy[t][3]+xy[t][4]+xy[t][5]+xy[t][6])/4;
	}
	*x=XY[0];
	*y=XY[1];
}
u8 GetXY(u16 *x,u16 *y)
{
	u16 x1,y1,x2,y2;
	Read_IO_XY(&x1,&y1);
	Read_IO_XY(&x2,&y2);
	if(ABS(x1-x2)>ERROR_RANGE||ABS(y1-y2)>ERROR_RANGE)//如果两次相差过大，去掉
	{
		return 0;
	}
	*x=(x1+x2)/2;
	*y=(y1+y2)/2;
	return 1;
}
u8 Read_TP()
{
	u8 t=0;
	EXTI_DISABLE;                //中断关闭
	pence.flag=KEY_UP;           //状态改变 
	GetXY(&pence.x0,&pence.y0);
	while(t<250&&PEN==0)
	{
		t++;
		SysTick_ms(10);
	}
	EXTI_ENABLE;
	if(t>=250)return 0;
	return 1;
}
void Touch_correct() //校准过后能得到一个大体上比较准确的值。但是还是会有些误差
{                   //我在这里得到的值，是根据实际情况，由此函数得到的值修改而来。修改的范围是很小的
       //经过校准得到的值是xfac=0.0771,xoff=-18.1920,yfac=0.1014,yoff=-2.8338
	   
                   //修改为 xfac=0.0671,xoff=-18.1920,yfac=0.0914,yoff=-18.8
	int x[4],y[4],cnt,temp1,temp2,ans;
	float d1,d2;
	TFT_Pant(BLUE);          //第一个校准点
	Draw_Touch_Point(20,20);  
	pence.flag=KEY_UP;          //清除标志
	cnt=0;
	while(1)
	{
		if(pence.flag==KEY_DOWN) //屏幕触碰 
		{
			if(Read_TP())      //读取AD值
			{
				x[cnt]=pence.x0;
				y[cnt]=pence.y0;
				cnt++;
			}
			switch(cnt)
			{
				case 1:
					TFT_Pant(BLUE);             //第2个校准点
					Draw_Touch_Point(220,20);
					break;
				case 2:
					TFT_Pant(BLUE);
					Draw_Touch_Point(20,300);//第3个?
				  break;
				case 3:
					TFT_Pant(BLUE);
					Draw_Touch_Point(220,300); //第4个
					break;
				case 4:                  //校准点完毕，进行合法性检查
					temp1=ABS(x[0]-x[1]);
					temp2=ABS(y[0]-y[1]);
					d1=sqrt(temp1*temp1+temp2*temp2); //1,2点的距离
				
				  temp1=ABS(x[2]-x[3]);             //3.4点的距离
					temp2=ABS(y[2]-y[3]);
					d2=sqrt(temp1*temp1+temp2*temp2);
					if(d1==0||d2==0||d1/d2>1.05||d1/d2<0.95) //距离差太多
					{
						cnt=0;
						TFT_Pant(BLACK);
						TFT_ShowString(35,100,"Adjust Failed(1)!!!");
						SysTick_ms(1000);
						TFT_Pant(BLUE);
						Draw_Touch_Point(20,20);
					//	continue;
						break;
					}
					
					temp1=ABS(x[0]-x[2]);
					temp2=ABS(y[0]-y[2]);
					d1=sqrt(temp1*temp1+temp2*temp2); //1,3点的距离	
					temp1=ABS(x[1]-x[3]);             //2.4点的距离
					temp2=ABS(y[1]-y[3]);
					d2=sqrt(temp1*temp1+temp2*temp2);					
					if(d1==0||d2==0||d1/d2>1.05||d1/d2<0.95) //距离差太多
					{
						cnt=0;
						TFT_Pant(BLACK);
						TFT_ShowString(35,100,"Adjust Failed(2)!!!");
						SysTick_ms(1000);
						TFT_Pant(BLUE);
						Draw_Touch_Point(20,20);
					//	continue;
						break;
					}
					
					temp1=ABS(x[0]-x[3]);
					temp2=ABS(y[0]-y[3]);
					d1=sqrt(temp1*temp1+temp2*temp2); //1,4点的距离	
					temp1=ABS(x[1]-x[2]);             //2.3点的距离
					temp2=ABS(y[1]-y[2]);
					d2=sqrt(temp1*temp1+temp2*temp2);					
					if(d1==0||d2==0||d1/d2>1.05||d1/d2<0.95) //距离差太多
					{
						cnt=0;
						TFT_Pant(BLACK);
						TFT_ShowString(35,100,"Adjust Failed(3)!!!");
						SysTick_ms(1000);
						Draw_Touch_Point(20,20);
					//	continue;
						break;
					}
					//参数检查完毕，手指触碰没有太大的误差
					pence.xfac=200.0/(x[1]-x[0]);               //解2元1次方程组
					pence.xoff=(240.0-pence.xfac*(x[1]+x[0]))/2;
					pence.yfac=280.0/(y[2]-y[0]);
					pence.yoff=(320.0-pence.yfac*(y[0]+y[2]))/2;
					TFT_Pant(BLUE);
					TFT_ShowString(35,100,"Touch Screen Adjust OK!");
					SysTick_ms(1000);
					TFT_Pant(WHITE);
					TFT_ShowNum(0,180,x[0]);
					TFT_ShowNum(0+CHARSIZE_W*5,180,y[0]);
					TFT_ShowNum(0,200,x[1]);
					TFT_ShowNum(0+CHARSIZE_W*5,200,y[1]);
					TFT_ShowNum(0,220,x[2]);
					TFT_ShowNum(0+CHARSIZE_W*5,220,y[2]);
					TFT_ShowNum(0,240,x[3]);
					TFT_ShowNum(0+CHARSIZE_W*5,240,y[3]);
					TFT_ShowFloat(100,100,pence.xfac);
					TFT_ShowFloat(100,120,pence.xoff);
					TFT_ShowFloat(100,140,pence.yfac);
					TFT_ShowFloat(100,160,pence.yoff);
					return ;
					default :break;
			}
		}
	}
}
void ConvertXY()
{
	float x,y;
	GetXY(&pence.x0,&pence.y0); //不能小于0
	x=pence.x0*pence.xfac+pence.xoff;
	y=pence.y0*pence.yfac+pence.yoff;
	pence.x=(u16)x;
	pence.y=(u16)y;
}
void Pence_inti()
{
	pence.flag=KEY_UP;
	pence.x=pence.y=0;
	pence.x0=pence.y0=0;
	pence.xfac=pence.xoff=0;
	pence.yfac=pence.yoff=0;
}
void Pence_adjust() //下面的是校准了的比较精确的参数
{
	pence.xfac=0.0671;
	pence.xoff=-18.1920;
	pence.yfac=0.0914;
	pence.yoff=-18.8;
}
void Touch_inti() //通过宏SPI，来选择硬件SPI驱动as,还是IO口模拟
{                //实际上来说，用SPI是没必要的，触摸屏对速度要求不高
	Pence_inti();  //如果用SPI，那么分频系数小了，或者在Read_IO_ADS函数中的延时
	#if SPI        //少了，会导致采集到的点过多，会出现散点的现象
	               //经过几次试验，发现64分频，延时30us，效果不错
      	Touch_SPI_inti();  //SPI
	#else
	      Touch_IO_inti();  //IO模拟
	#endif
	Pence_adjust();
//	Touch_correct();
}
void EXTI0_IRQHandler()
{
	static u32 count=0;
	if(EXTI_GetITStatus(EXTI_Line0)==SET)
	{
		count++;
		EXTI_ClearITPendingBit(EXTI_Line0);
		pence.flag=KEY_DOWN;    //屏幕触摸
		SysTick_ms(50);        //延时消抖
	}
}
#endif
	   
	   
	   
	   
	   
	   
	   
	   