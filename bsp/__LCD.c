#include <stm32f4xx.h> 
//#include "LCD.h"
#include "stm32f429i_discovery_lcd.h"
#define SPI_DELAY        1  

#define Hsync  5
#define HBP    5
#define HFP    5

#define Vsync  4
#define VBP    6
#define VFP    8

uint8_t LCD_ID[3] = {0, 0, 0};
//uint32_t LCD_PIXEL_WIDTH  = 480;
//uint32_t LCD_PIXEL_HEIGHT = 800;

#define TOUCH_CS  0
#define LCD_CS 1
#define NO_PICK 2

static void LCD_Delay(__IO uint32_t nCount)
{
    __IO uint32_t index = 0; 


    for(index = nCount; index != 0; index--)
    {
        ;
    }
}


//Sim SPI2
#define  Delay_us(__a__) LCD_Delay(__a__)

/*
#define  set_cs      GPIO_SetBits( GPIOG, GPIO_Pin_2 )
#define  reset_cs    GPIO_ResetBits( GPIOG, GPIO_Pin_2 )
*/
#define  set_cs      
#define  reset_cs    

//#define  set_cs     SetSPI2CS(NO_PICK)
//#define  reset_cs   SetSPI2CS(LCD_CS)

#define  set_clk    GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define  reset_clk  GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define  set_sdi    GPIO_SetBits(GPIOI, GPIO_Pin_3)
#define  reset_sdi  GPIO_ResetBits( GPIOI, GPIO_Pin_3)
#define  set_res    GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define  reset_res  GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define  getsdo     GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)
//end

/* Global variables to set the written text color */
//static uint16_t CurrentTextColor   = 0x0000;
//static uint16_t CurrentBackColor   = 0xFFFF;
/* Default LCD configuration with LCD Layer 1 */
static uint32_t CurrentFrameBuffer = LCD_FRAME_BUFFER;
static uint32_t CurrentLayer = LCD_BACKGROUND_LAYER;

const uint8_t rbHighLow[] = 
{
    0x0,0x10,0x8,0x18,0x4,0x14,0xc,0x1c,0x2,0x12,0xa,0x1a,0x6,0x16,0xe,0x1e,0x1,0x11,0x9,0x19,0x5,0x15,0xd,0x1d,0x3,0x13,0xb,0x1b,0x7,0x17,0xf,0x1f
};

const uint8_t  gHighLow[] = 
{
0x0,0x20,0x10,0x30,0x8,0x28,0x18,0x38,0x4,0x24,0x14,0x34,0xc,0x2c,0x1c,0x3c,0x2,
	0x22,0x12,0x32,0xa,0x2a,0x1a,0x3a,0x6,0x26,0x16,0x36,0xe,0x2e,0x1e,0x3e,0x1,0x21,
	0x11,0x31,0x9,0x29,0x19,0x39,0x5,0x25,0x15,0x35,0xd,0x2d,0x1d,0x3d,0x3,0x23,0x13,
	0x33,0xb,0x2b,0x1b,0x3b,0x7,0x27,0x17,0x37,0xf,0x2f,0x1f,0x3f,
};

void LCD_Init(void);
void SetSPI2CS( uint8_t cs );


//************??lcd??************//
void rest_lcd()
{
    set_res;
    Delay_us(20000);
    reset_res;
    Delay_us(20000);
    set_res;
    Delay_us(150000);
}

//***********??????*****************//
void send_cmd(uint8_t cmd)
{
  unsigned char i;
reset_cs;
 Delay_us(200);
reset_clk;
reset_sdi;
Delay_us(200);
set_clk;
 Delay_us(200);
for(i=0;i<8;i++)
    {
    reset_clk; 
  
      if (cmd&0x80)
       {
          set_sdi;
        }
       else
         {
         reset_sdi;
}
    Delay_us(200);
      set_clk;
Delay_us(200);
      cmd=cmd<<1;
    }
set_cs;
 Delay_us(20);
	set_sdi;
}

//**************??????**********************//
void send_date(uint8_t date)
{
  unsigned char i;
 reset_cs;
 Delay_us(200);
reset_clk;
set_sdi;
Delay_us(200);
set_clk;
 Delay_us(200);
for(i=0;i<8;i++)
    {
    reset_clk; 
  
      if (date&0x80)
       {
          set_sdi;
       }
       else
         {
         reset_sdi;
}
    Delay_us(200);
      set_clk;
Delay_us(200);
      date=date<<1;
    }
set_cs;

 Delay_us(20);
set_sdi;
 
}

uint8_t Read_PAs()
{
    int kk;
	  uint8_t rdValue;
	  reset_cs;
	  Delay_us(200);
	  rdValue = 0;
	  
	  for ( kk=7; kk>=0; kk-- )
	  {
	     reset_clk;
			 Delay_us(100);
			 
        
			 set_clk;
			 Delay_us(200);
			 if ( getsdo ) rdValue |= 0x01;
			
			 if ( kk > 0 ) rdValue = rdValue << 1;
	  }
		
		set_cs;
		
		Delay_us( 4000 );
		//set_sdi;
		return rdValue;
}

uint8_t   SPI_READ(uint8_t ID)
{
	uint8_t regV;

	
	send_cmd( ID );
	
	regV = Read_PAs();
	//regV = Read_PAs();
	return regV;	
}

void SPI_READ_ID()
{
    LCD_ID[0] = SPI_READ(0xda);
	  LCD_ID[1] = SPI_READ(0xdb);
	  LCD_ID[2] = SPI_READ(0xdc);
}

//*************??HX8369??*****************//
 void initial_hx8369()
 {


  rest_lcd(); //?????
  SPI_READ_ID();

  send_cmd(0xB9); //Set_EXTC 
	send_date(0xFF);	 
	send_date(0x83);	 
	send_date(0x69);				  
	 
	send_cmd(0xB1);  //Set Power	
	send_date(0x01);  
	send_date(0x00);  
	send_date(0x34);  
	send_date(0x06);  
	send_date(0x00);  
	send_date(0x0F);  
	send_date(0x0F);  
	send_date(0x2A);  
	send_date(0x32);  
	send_date(0x3F);  
	send_date(0x3F);  
	send_date(0x07);  
	send_date(0x23);  
	send_date(0x01);  
	send_date(0xE6);  
	send_date(0xE6);  
	send_date(0xE6);  
	send_date(0xE6);  
	send_date(0xE6); 
	 
	send_cmd(0xB2);  // SET Display  480x800 
	send_date(0x00);	
	send_date(0x2B); //2B	
	send_date(0x0A);	
	send_date(0x0A);	
	send_date(0x70);	
	send_date(0x00);	
	send_date(0xFF);	
	send_date(0x00);	
	send_date(0x00);	
	send_date(0x00);	
	send_date(0x00);	
	send_date(0x03);	
	send_date(0x03);	
	send_date(0x00);	
	send_date(0x01);	 
	
	send_cmd(0xB4);  // SET Display  480x800 
	send_date(0x00);	
	send_date(0x18);	
	send_date(0x80); 
	send_date(0x10);	
	send_date(0x01);	
	 
	send_cmd(0xB6);  // SET VCOM 
	send_date(0x2C);	
	send_date(0x2C);	  
	 
	send_cmd(0xD5);  //SET GIP 
	send_date(0x00);	
	send_date(0x05);	
	send_date(0x03);	
	send_date(0x00);	
	send_date(0x01);	
	send_date(0x09);	
	send_date(0x10);	
	send_date(0x80);	
	send_date(0x37);	
	send_date(0x37);	
	send_date(0x20);	
	send_date(0x31);	
	send_date(0x46);	
	send_date(0x8A);	
	send_date(0x57);	
	send_date(0x9B);	
	send_date(0x20);	
	send_date(0x31);	
	send_date(0x46);	
	send_date(0x8A);	
	send_date(0x57);	
	send_date(0x9B);	
	send_date(0x07);	
	send_date(0x0F);	
	send_date(0x02);	
	send_date(0x00); 
	
	send_cmd(0xE0);  //SET GAMMA 
	send_date(0x00);  
	send_date(0x08);  
	send_date(0x0D);  
	send_date(0x2D);  
	send_date(0x34);  
	send_date(0x3F);  
	send_date(0x19);  
	send_date(0x38);  
	send_date(0x09);  
	send_date(0x0E);  
	send_date(0x0E);  
	send_date(0x12);  
	send_date(0x14);  
	send_date(0x12);  
	send_date(0x14);  
	send_date(0x13);  
	send_date(0x19);  
	send_date(0x00);  
	send_date(0x08);  
	
	send_date(0x0D);  
	send_date(0x2D);  
	send_date(0x34);  
	send_date(0x3F);  
	send_date(0x19);  
	send_date(0x38);  
	send_date(0x09);  
	send_date(0x0E);  
	send_date(0x0E);  
	send_date(0x12);  
	send_date(0x14);  
	send_date(0x12);  
	send_date(0x14);  
	send_date(0x13);  
	send_date(0x19); 
	 
	send_cmd(0xC1); //set DGC 
	send_date(0x01); //enable DGC function 
	send_date(0x02); //SET R-GAMMA 
	send_date(0x08);  
	send_date(0x12);  
	send_date(0x1A);  
	send_date(0x22);  
	send_date(0x2A);  
	send_date(0x31);  
	send_date(0x36);  
	send_date(0x3F);  
	send_date(0x48);  
	send_date(0x51);  
	send_date(0x58);  
	send_date(0x60);  
	send_date(0x68);  
	send_date(0x70);  
	send_date(0x78);  
	send_date(0x80);  
	send_date(0x88);  
	send_date(0x90);  
	send_date(0x98);  
	send_date(0xA0);  
	send_date(0xA7);  
	send_date(0xAF);  
	send_date(0xB6);  
	send_date(0xBE);  
	send_date(0xC7);  
	send_date(0xCE);  
	send_date(0xD6);  
	send_date(0xDE);  
	send_date(0xE6);  
	send_date(0xEF);  
	send_date(0xF5);  
	send_date(0xFB);  
	send_date(0xFC); 
	send_date(0xFE);  
	send_date(0x8C);  
	send_date(0xA4);  
	send_date(0x19);  
	send_date(0xEC);  
	send_date(0x1B);  
	send_date(0x4C);  
	
	send_date(0x40);	 
	send_date(0x02); //SET G-Gamma 
	send_date(0x08);  
	send_date(0x12);  
	send_date(0x1A);  
	send_date(0x22);  
	send_date(0x2A);  
	send_date(0x31);  
	send_date(0x36);  
	send_date(0x3F);  
	send_date(0x48);  
	send_date(0x51);  
	send_date(0x58);  
	send_date(0x60);  
	send_date(0x68);  
	send_date(0x70);  
	send_date(0x78);  
	send_date(0x80);  
	send_date(0x88);  
	send_date(0x90);  
	send_date(0x98);  
	send_date(0xA0);  
	send_date(0xA7);  
	send_date(0xAF);  
	send_date(0xB6);  
	send_date(0xBE);  
	send_date(0xC7);  
	send_date(0xCE);  
	send_date(0xD6);  
	send_date(0xDE);  
	send_date(0xE6);  
	send_date(0xEF);  
	send_date(0xF5);  
	send_date(0xFB);  
	send_date(0xFC); 
	send_date(0xFE);  
	send_date(0x8C);  
	send_date(0xA4);  
	send_date(0x19);  
	send_date(0xEC);  
	send_date(0x1B);  
	send_date(0x4C);  
	send_date(0x40);  
	send_date(0x02); //SET B-Gamma 
	
	send_date(0x08);  
	send_date(0x12);  
	send_date(0x1A);  
	send_date(0x22);  
	send_date(0x2A);  
	send_date(0x31);  
	send_date(0x36);  
	send_date(0x3F);  
	send_date(0x48);  
	send_date(0x51);  
	send_date(0x58);  
	send_date(0x60);  
	send_date(0x68);  
	send_date(0x70);  
	send_date(0x78);  
	send_date(0x80);  
	send_date(0x88);  
	send_date(0x90);  
	send_date(0x98);  
	send_date(0xA0);  
	send_date(0xA7);  
	send_date(0xAF);  
	send_date(0xB6);  
	send_date(0xBE);  
	send_date(0xC7);  
	send_date(0xCE);  
	send_date(0xD6);  
	send_date(0xDE);  
	send_date(0xE6);  
	send_date(0xEF);  
	send_date(0xF5);  
	send_date(0xFB);  
	send_date(0xFC); 
	send_date(0xFE);  
	send_date(0x8C);  
	send_date(0xA4);  
	send_date(0x19);  
	send_date(0xEC);  
	send_date(0x1B);  
	send_date(0x4C);  
	send_date(0x40); 
	 
	send_cmd(0x3A);  //Set RGB16  
	send_date(0x55);	// 
	 
	send_cmd(0x11);  //Sleep Out 
	Delay_us(150000);   
	 
	send_cmd(0x29);  //Display On  
   
}
 


/**
  * @brief  Initializes the LCD layers.
  * @param  None
  * @retval None
  */
void LCD_LayerInit(void)
{ 

    LTDC_Layer_InitTypeDef LTDC_Layer_InitStruct;
  

 
    /* Windowing configuration */
    /* Horizontal start = horizontal synchronization + Horizontal back porch = 43 
    Vertical start   = vertical synchronization + vertical back porch     = 12
    Horizontal stop = Horizontal start + LCD_PIXEL_WIDTH -1 
    Vertical stop   = Vertical start + LCD_PIXEL_HEIGHT -1        
    */      
    LTDC_Layer_InitStruct.LTDC_HorizontalStart = Hsync + HBP ;
    LTDC_Layer_InitStruct.LTDC_HorizontalStop = (LCD_PIXEL_WIDTH + Hsync + HBP - 1 ); 
    LTDC_Layer_InitStruct.LTDC_VerticalStart = Vsync + VBP;
    LTDC_Layer_InitStruct.LTDC_VerticalStop = (LCD_PIXEL_HEIGHT + Vsync + VBP - 1 );
    
    /* Pixel Format configuration*/
    LTDC_Layer_InitStruct.LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
    /* Alpha constant (255 totally opaque) */
    LTDC_Layer_InitStruct.LTDC_ConstantAlpha = 255; 
    /* Default Color configuration (configure A,R,G,B component values) */          
    LTDC_Layer_InitStruct.LTDC_DefaultColorBlue =0;        
    LTDC_Layer_InitStruct.LTDC_DefaultColorGreen = 0;       
    LTDC_Layer_InitStruct.LTDC_DefaultColorRed = 0;         
    LTDC_Layer_InitStruct.LTDC_DefaultColorAlpha = 0;
    /* Configure blending factors */       
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;    
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor1_PAxCA;
    
    /* the length of one line of pixels in bytes + 3 then :
    Line Lenth = Active high width x number of bytes per pixel + 3 
    Active high width         = LCD_PIXEL_WIDTH 
    number of bytes per pixel = 2    (pixel_format : RGB565) 
    */
    LTDC_Layer_InitStruct.LTDC_CFBLineLength = ((LCD_PIXEL_WIDTH * 2) + 3);
    /* the pitch is the increment from the start of one line of pixels to the 
    start of the next line in bytes, then :
    Pitch = Active high width x number of bytes per pixel     
    */ 
    LTDC_Layer_InitStruct.LTDC_CFBPitch = (LCD_PIXEL_WIDTH * 2);
    
    /* Configure the number of lines */  
    LTDC_Layer_InitStruct.LTDC_CFBLineNumber = LCD_PIXEL_HEIGHT;
    
    /* Start Address configuration : the LCD Frame buffer is defined on SDRAM */    
    LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER;
    
    LTDC_LayerInit(LTDC_Layer1, &LTDC_Layer_InitStruct);
    
    /* Configure Layer2 */
    /* Start Address configuration : the LCD Frame buffer is defined on SDRAM w/ Offset */     
    LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER + BUFFER_OFFSET;
    
    /* Configure blending factors */   
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;    
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;  
    //LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;    
    //LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor1_PAxCA; 
		
    LTDC_LayerInit(LTDC_Layer2, &LTDC_Layer_InitStruct);
    
		
  
  LTDC_ReloadConfig(LTDC_IMReload);
  
  /* Enable foreground & background Layers */
  LTDC_LayerCmd(LTDC_Layer1, ENABLE);
  //LTDC_LayerCmd(LTDC_Layer2, ENABLE);
  LTDC_ReloadConfig(LTDC_IMReload);
  //LTDC_ReloadConfig(LTDC_IMReload);
	
	//LTDC_DitherCmd(ENABLE);
  //LCD_SetFont(&LCD_DEFAULT_FONT);
}

/**
  * @brief  Sets the LCD Layer.
  * @param  Layerx: specifies the Layer foreground or background.
  * @retval None
  */
void LCD_SetLayer(uint32_t Layerx)
{
  if (Layerx == LCD_BACKGROUND_LAYER)
  {
    CurrentFrameBuffer = LCD_FRAME_BUFFER; 
    CurrentLayer = LCD_BACKGROUND_LAYER;
  }
  else
  {
    CurrentFrameBuffer = LCD_FRAME_BUFFER + BUFFER_OFFSET;
    CurrentLayer = LCD_FOREGROUND_LAYER;
  }
}  

/**
  * @brief  Configure the transparency.
  * @param  transparency: specifies the transparency, 
  *         This parameter must range from 0x00 to 0xFF.
  * @retval None
  */
void LCD_SetTransparency(uint8_t transparency)
{
  if (CurrentLayer == LCD_BACKGROUND_LAYER)
  {
    LTDC_LayerAlpha(LTDC_Layer1, transparency);
  }
  else
  {     
    LTDC_LayerAlpha(LTDC_Layer2, transparency);
  }
  LTDC_ReloadConfig(LTDC_IMReload);
}

void LCD_Config(void)
{
     /* Initialize the LCD */
	LCD_Init();
	
	
	LCD_LayerInit();
	
	/* Set LCD background layer */
	LCD_SetLayer(LCD_BACKGROUND_LAYER);
	
	/* Set LCD transparency */
	LCD_SetTransparency(255);

	/* Set LCD foreground layer */
	LCD_SetLayer(LCD_FOREGROUND_LAYER);
	LCD_SetTransparency(255);
	/* LTDC reload configuration */  
	LTDC_ReloadConfig(LTDC_IMReload);
	
	
	/* Enable the LTDC */
	LTDC_Cmd(ENABLE);
	
}


void InitLCDBackRight()
{
	    GPIO_InitTypeDef GPIO_InitStructure;
	
	    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOI, ENABLE);
    	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOI, &GPIO_InitStructure);
}

  
/**
  * @brief GPIO configuration for LTDC.
  * @param  None
  * @retval None
  */
static void LCD_AF_GPIOConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable GPIOI, GPIOJ, GPIOK AHB Clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG | \
                         RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOA | \
	                       RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD ,  ENABLE);

/* GPIOs Configuration */
/*
 +------------------------+-----------------------+----------------------------+
 +                       LCD pins assignment                                   +
 +------------------------+-----------------------+----------------------------+
 |  LCD_TFT R0 <-> PH.02  |  LCD_TFT G0 <-> PE.05 |  LCD_TFT B0 <-> PE.04      |
 |  LCD_TFT R1 <-> PH.03  |  LCD_TFT G1 <-> PE.06 |  LCD_TFT B1 <-> PG.12      |
 |  LCD_TFT R2 <-> PH.08  |  LCD_TFT G2 <-> PH.13 |  LCD_TFT B2 <-> PD.06      |
 |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
 |  LCD_TFT R4 <-> PH.10  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PI.04      |
 |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
 |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
 |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PI.02 |  LCD_TFT B7 <-> PB.09      |
 -------------------------------------------------------------------------------
          |  LCD_TFT HSYNC <-> PI.10  | LCDTFT VSYNC <->  PI.09 |
          |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
           -----------------------------------------------------
	LCD_RESET<-> PB.06
*/


	
 /* GPIOI configuration */
 /*
  //GPIO E
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStruct);

	
*/


  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	//GPIO F
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	
	//GPIO G
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource6, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource7, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, 0x09);//库有bug,应该复用为G3
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_LTDC);
	//GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_7 ;
  GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	//GPIO H
	//GPIO_PinAFConfig(GPIOH, GPIO_PinSource2, GPIO_AF_LTDC);
	//GPIO_PinAFConfig(GPIOH, GPIO_PinSource3, GPIO_AF_LTDC);
	//GPIO_PinAFConfig(GPIOH, GPIO_PinSource8, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource13, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_13 ;
  GPIO_Init(GPIOH, &GPIO_InitStruct);
	
  //GPIO I
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource4, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource9, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource10, GPIO_AF_LTDC);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_9 | GPIO_Pin_10;                             
  GPIO_Init(GPIOI, &GPIO_InitStruct); 
	
	//GPIOA
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_LTDC);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_12;                        
  GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
	//GPIOB


  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LTDC);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LTDC);
	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LTDC);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LTDC);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, 9);//库有问题应该是9
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, 9);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10 | GPIO_Pin_11;
	///GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10 | GPIO_Pin_11;
                             
  GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	/*
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct); 
	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
	*/
	
	//while ( 1 ) GPIO_ToggleBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1);
	
	//GPIOC
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LTDC);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
                             

  GPIO_Init(GPIOC, &GPIO_InitStruct); 
	
	//GPIOD
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_LTDC);
  //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
                             

  //GPIO_Init(GPIOD, &GPIO_InitStruct); 
	
	

#if 0  
	 //GPIO E
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	//GPIO F
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	//GPIO G
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource6, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource7, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_LTDC);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	//GPIO H
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource2, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource3, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource8, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource13, GPIO_AF_LTDC);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_13 ;
	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIO_InitStruct);
	
  //GPIO I
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource4, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource9, GPIO_AF_LTDC);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource10, GPIO_AF_LTDC);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_9 | GPIO_Pin_10;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIO_InitStruct); 
	
	//GPIOA
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_LTDC);


  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_12;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
	//GPIOB
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LTDC);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10 | GPIO_Pin_11;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	//GPIOC
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LTDC);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct); 
	
	//GPIOD
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_LTDC);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStruct); 
	#endif
	
	
} 

void LCD_Sim_SPI2_Init()
{
/* GPIOs Configuration */
/*
 +------------------------+-----------------------+----------------------------+
 
          |  SPI2_CLK    <-> PB.13     | SPI2_MISO  <->  PC.02 |
          |  SPI2_MOSI   <-> PI.03     | SPI2_CS0   <->  PG.02 LCD_CS| SPI2_CS1 PG.14 TOUCH_CS
	        |  LCD_Reset   <-> pB.06
           -----------------------------------------------------
*/
	//SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//GPIO PINS Config
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOG, ENABLE );
	
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	*/
	
	//spi2 clk,LCD_Reset 设置为输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	
	//spi2 miso 设置为输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//spi2 mosi 输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	
	// spi2 cs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	//spi2 CS置高
	GPIO_SetBits( GPIOG, GPIO_Pin_2|GPIO_Pin_14 );
	/*
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_Cmd(SPI2, DISABLE);  
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init( SPI2, &SPI_InitStructure );
	SPI_Cmd(SPI2, ENABLE);
	*/
	
}


//触摸屏或者LCD的SPI2片选
void SetSPI2CS( uint8_t cs )
{

	if ( cs == TOUCH_CS )
	{
		  GPIO_SetBits( GPIOG, GPIO_Pin_2 );
	    GPIO_ResetBits(GPIOG, GPIO_Pin_14);
		  
	}
	else if (cs == LCD_CS  )
	{
	    GPIO_SetBits( GPIOG, GPIO_Pin_14 );
	    GPIO_ResetBits( GPIOG, GPIO_Pin_2);
	}
	else
	{
	     GPIO_SetBits( GPIOG, GPIO_Pin_14 );
	     GPIO_SetBits( GPIOG, GPIO_Pin_2);
	}
	
	
	
}


void LCD_Init(void)
{
    LTDC_InitTypeDef       LTDC_InitStruct;
    LTDC_Layer_TypeDef     LTDC_Layerx;
	  
	  InitLCDBackRight();
	  //LCD_BACKRIGHT_ON();
	
	
	  /* Configure the LCD Control pins */
    LCD_AF_GPIOConfig();
	
	    /* Enable the LTDC Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, ENABLE);
  
    /* Enable the DMA2D Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE); 
	
	 
	  
	  LCD_Sim_SPI2_Init();
	  //GPIO_ResetBits( GPIOG, GPIO_Pin_2 );
	  SetSPI2CS( LCD_CS );
    initial_hx8369(); 
	  SetSPI2CS( NO_PICK );
	  
	  
  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  LTDC_InitStruct.LTDC_HSPolarity = LTDC_HSPolarity_AL;     
  /* Initialize the vertical synchronization polarity as active low */  
  LTDC_InitStruct.LTDC_VSPolarity = LTDC_VSPolarity_AL;     
  /* Initialize the data enable polarity as active low */
  LTDC_InitStruct.LTDC_DEPolarity = LTDC_DEPolarity_AL;     
  /* Initialize the pixel clock polarity as input pixel clock */ 
  LTDC_InitStruct.LTDC_PCPolarity = LTDC_PCPolarity_IPC;
	
  
  /* Configure R,G,B component values for LCD background color */                   
  LTDC_InitStruct.LTDC_BackgroundRedValue = 0;            
  LTDC_InitStruct.LTDC_BackgroundGreenValue = 0;          
  LTDC_InitStruct.LTDC_BackgroundBlueValue = 0;  
	
	// if(CurrentLcd == USE_LCD_AM640480)
    //LCD_PIXEL_WIDTH  = 480; 
    //LCD_PIXEL_HEIGHT = 800;
    
	RCC_PLLSAIConfig(60, 7,2 );
  RCC_LTDCCLKDivConfig(RCC_PLLSAIDivR_Div2);
    
    /* Enable PLLSAI Clock */
    RCC_PLLSAICmd(ENABLE);
    /* Wait for PLLSAI activation */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLSAIRDY) == RESET)
    {
    }
    
      /* Timing configuration */
   /* Horizontal synchronization width = Hsync - 1 */     
   LTDC_InitStruct.LTDC_HorizontalSync = Hsync-1;
   /* Vertical synchronization height = Vsync - 1 */
   LTDC_InitStruct.LTDC_VerticalSync = Vsync - 1;
   /* Accumulated horizontal back porch = Hsync + HBP - 1 */
   LTDC_InitStruct.LTDC_AccumulatedHBP = Hsync + HBP - 1; 
   /* Accumulated vertical back porch = Vsync + VBP - 1 */
   LTDC_InitStruct.LTDC_AccumulatedVBP = Vsync + VBP - 1;  
   /* Accumulated active width = Hsync + HBP + Active Width - 1 */  
   LTDC_InitStruct.LTDC_AccumulatedActiveW = Hsync + HBP  + LCD_PIXEL_WIDTH - 1;
   /* Accumulated active height = Vsync + VBP + Active Heigh - 1 */
   LTDC_InitStruct.LTDC_AccumulatedActiveH = Vsync + VBP + LCD_PIXEL_HEIGHT - 1;
   /* Total width = Hsync + HBP + Active Width + HFP - 1 */
   LTDC_InitStruct.LTDC_TotalWidth = Hsync + HBP + LCD_PIXEL_WIDTH + HFP - 1; 
   /* Total height = Vsync + VBP + Active Heigh + VFP - 1 */
   LTDC_InitStruct.LTDC_TotalHeigh = Vsync + VBP + LCD_PIXEL_HEIGHT + VFP - 1;

		
    LTDC_Init(&LTDC_InitStruct);
		
	
	  
}


/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height,uint16_t color)
{
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  
  uint32_t  Xaddress = 0; 
  uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
 
  Red_Value = (0xF800 & color) >> 11;
  Blue_Value = 0x001F & color;
  Green_Value = (0x07E0 & color) >> 5;
  
  //Xaddress = CurrentFrameBuffer + 2*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  Xaddress = LCD_FRAME_BUFFER +  2*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  /* configure DMA2D */
  DMA2D_DeInit();
  DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;       
  DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;      
  DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;      
  DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;     
  DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;                
  DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;                  
  DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Xaddress;                
  DMA2D_InitStruct.DMA2D_OutputOffset = (LCD_PIXEL_WIDTH - Width);                
  DMA2D_InitStruct.DMA2D_NumberOfLine = Height;            
  DMA2D_InitStruct.DMA2D_PixelPerLine = Width;
  DMA2D_Init(&DMA2D_InitStruct); 
  
  /* Start Transfer */ 
  DMA2D_StartTransfer();
  
  /* Wait for CTC Flag activation */
  while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
  {
  } 

  //LCD_SetTextColor(CurrentTextColor);
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect888(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint32_t color)
{
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  
  uint32_t  Xaddress = 0; 
  uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
 
	/*
  Red_Value = (0xF800 & color) >> 11;
  Blue_Value = 0x001F & color;
  Green_Value = (0x07E0 & color) >> 5;
 */ 
	
  Red_Value = color>>16;
  Blue_Value = color & 0xff;
  Green_Value = (color>>8)&0xff;
	
  //Xaddress = LCD_FRAME_BUFFER+BUFFER_OFFSET*1 + 2*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  
  /* configure DMA2D */
  //Xaddress = LCD_FRAME_BUFFER+BUFFER_OFFSET + 3*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  Xaddress = LCD_FRAME_BUFFER+BUFFER_OFFSET + 3*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  /* configure DMA2D */
  DMA2D_DeInit();
  DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;       
  DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB888;      
  DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;      
  DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;     
  DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;                
  DMA2D_InitStruct.DMA2D_OutputAlpha = 0xf;                  
  DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Xaddress;                
  DMA2D_InitStruct.DMA2D_OutputOffset = (LCD_PIXEL_WIDTH - Width);                
  DMA2D_InitStruct.DMA2D_NumberOfLine = Height;            
  DMA2D_InitStruct.DMA2D_PixelPerLine = Width;
  DMA2D_Init(&DMA2D_InitStruct); 
  
  /* Start Transfer */ 
  DMA2D_StartTransfer();
  
  /* Wait for CTC Flag activation */
  while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
  {
  } 

  //LCD_SetTextColor(CurrentTextColor);
}


void MyLCD_DrawFullRect888(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint32_t color)
{
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  
  __IO uint8_t*  Xaddress = 0; 
  uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
	uint16_t i, j;
 
	/*
  Red_Value = (0xF800 & color) >> 11;
  Blue_Value = 0x001F & color;
  Green_Value = (0x07E0 & color) >> 5;
 */ 
	
  Red_Value = color>>16;
  Blue_Value = color & 0xff;
  Green_Value = (color>>8)&0xff;
	
  //Xaddress = LCD_FRAME_BUFFER+BUFFER_OFFSET*1 + 2*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  
  /* configure DMA2D */
  //Xaddress = LCD_FRAME_BUFFER+BUFFER_OFFSET + 3*(LCD_PIXEL_WIDTH*Ypos + Xpos);
  
  for ( i=0; i<Height; i++ )
	  for ( j=0; j<Width; j++ )
		{
		    Xaddress = (__IO uint8_t*)(LCD_FRAME_BUFFER+BUFFER_OFFSET + 3*(LCD_PIXEL_WIDTH*(i+Ypos) + j+Xpos));
			  Xaddress[0] = Blue_Value;
			  Xaddress[1] = Green_Value;
			  Xaddress[2] = Red_Value;
		}
  

  //LCD_SetTextColor(CurrentTextColor);
}

//end
