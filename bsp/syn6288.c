/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : syn6288.c
 @brief  : syn6288  audio driver
 @author : gexueyuan
 @history:
           2015-8-6    gexueyuan    Created file
           ...
******************************************************************************/
#include <rthw.h>
#include <rtthread.h>
    
#include "stm32f4xx.h"
#include "board.h"
#include "usart.h"
#include "gpio.h"
#include <rtthread.h>
#include "finsh.h"


#define syn6288_DEVICE_NAME	"uart4"




void audio_io_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* Sound en enable*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

void play_test(void)
{

    uint8_t test[] = {0xFD,0x00,0x0B,0x01,0x00,0xD3,0xEE,0xD2,0xF4,0xCC,0xEC,0xCF,0xC2,0xC1};
    uint8_t serch[] = {0xFD,0x00,0x02,0x21,0xDE};
    
    rt_device_t dev;
    uint8_t tmp;

    dev = rt_device_find(syn6288_DEVICE_NAME);

    tmp = rt_device_write(dev,0,serch,sizeof(serch));

    if(tmp == sizeof(serch))
        rt_kprintf("play success!!\n");

    rt_kprintf("return %d\n",tmp);
}

FINSH_FUNCTION_EXPORT(play_test, sound test);

void play(char *txt)
{
    char tempbuff[205];
    uint8_t datalength;
    int i = 0;
    int j = 0;
    uint8_t xorcrc=0;
    rt_device_t dev;
    uint8_t tmp;
    RT_ASSERT(txt != NULL);
    
    i = 5;
    
    while(*txt != '\0'){
        
        tempbuff[i++] = *txt++;

    }
    datalength = i - 5;
    
    tempbuff[0] = 0xFD;

    tempbuff[1] = 0x00;

    tempbuff[2] = datalength + 3;
    
    tempbuff[3] = 0x01;
    
    tempbuff[4] = 0x00;

    for(j = 0;j < (datalength + 5);j++){
        xorcrc=xorcrc ^ tempbuff[j];      
    }
    tempbuff[datalength + 5] = xorcrc;
/*
    for(i = 0;i < (datalength + 6);i++){

        rt_kprintf("%X\n",tempbuff[i]);

    }
    */
    dev = rt_device_find(syn6288_DEVICE_NAME);
    rt_device_open(dev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);
    tmp = rt_device_write(dev,0,tempbuff,datalength + 6);
    
    rt_kprintf("return %d\n",tmp);
}


FINSH_FUNCTION_EXPORT(play, input:string);

void rt_audio_thread_entry(void * parameter)
{

	rt_device_t dev ;
    
	dev = rt_device_find(syn6288_DEVICE_NAME);
	rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
	
	while(1){
        //if(rt_device_read(dev, 0, &tmp, 1) == 1)
            //rt_kprintf("%d\n",tmp);
       }  


}


int audio_init(void)
{

  //  rt_thread_t tid;
    
    audio_io_init();

    /*
    tid = rt_thread_create("audio",
        rt_audio_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX-2, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);
*/
    return 0;

}

