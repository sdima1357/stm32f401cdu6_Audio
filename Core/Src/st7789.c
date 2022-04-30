/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "st7789.h"
#include "fonts.h"
#include "main.h"

//https://create.arduino.cc/projecthub/calogerus/arduino-uno-2-4-tft-lcd-display-shield-touch-panel-ili9341-576b1b

//#define SPIH  hspi3
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi1;

volatile SPI_HandleTypeDef SPIH;
//#define SPIH hspi3

//#define LCD_DC_0   LCD_CMD_GPIO_Port->BSRR = LCD_CMD_Pin<<16;
//#define LCD_DC_1   LCD_CMD_GPIO_Port->BSRR = LCD_CMD_Pin;

volatile GPIO_TypeDef * Cmd_Port;
volatile uint32_t  Cmd_Pin;
//#define SPI_BSY    {while(SPIH.State!=HAL_SPI_STATE_READY){};while((SPIH.Instance->SR) &SPI_FLAG_BSY);while(((SPIH.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE){};}
//#define SPI_BSY  {while(SPIH.State!=HAL_SPI_STATE_READY){};while(((SPIH.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE){};}
#define SPI_BSY    {}
#define LCD_DC_0   {SPI_BSY;Cmd_Port->BSRR = Cmd_Pin<<16;}
#define LCD_DC_1   {SPI_BSY;Cmd_Port->BSRR = Cmd_Pin;}


void setSPI(SPI_HandleTypeDef spi_inp, GPIO_TypeDef *cmd_port,uint32_t cmd_pin)
{
	SPIH = spi_inp;
	Cmd_Port = cmd_port;
	Cmd_Pin = cmd_pin;
}
int oldNum  = -1;

void setLCD(int num)
{
	if(oldNum==num)
	{
	}
	else
	{
		if(oldNum!=-1)
		{
		//	writecommand(ST7789_NOP);
		//	writedata(0);
		}
		else
		{
			//setSPI(hspi1,LCD_CMD_GPIO_Port,LCD_CMD_Pin);
			//LCD_DC_1;
			//setSPI(hspi1,LCD1_CMD_GPIO_Port,LCD1_CMD_Pin);
			//LCD_DC_1;
		}
		if(num==0)
		{
			setSPI(hspi3,LCD_CMD_GPIO_Port,LCD_CMD_Pin);
			//setSPI(hspi1,LCD_CMD_GPIO_Port,LCD_CMD_Pin);
		}
		else
		{
			setSPI(hspi1,LCD1_CMD_GPIO_Port,LCD1_CMD_Pin);
		}
		oldNum = num;
	}
}


#define LCD_RES_0   HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin,GPIO_PIN_RESET);
#define LCD_RES_1   HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin,GPIO_PIN_SET);



#define LCD_BL_0   HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin,GPIO_PIN_RESET);
#define LCD_BL_1   HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin,GPIO_PIN_SET);

//#define LCD_CLK_0    CLK_GPIO_Port->BSRR = CLK_Pin<<16;
//#define LCD_CLK_1    CLK_GPIO_Port->BSRR = CLK_Pin;


#define SpixTimeout 1000
#define SPIx_WriteF(Value)  { while(((SPIH.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE){} *((__IO uint8_t*)&SPIH.Instance->DR) = Value; }
/*
void SPIx_WriteF(uint8_t  Value)
{
	    //HAL_SPI_Transmit(&hspi2,&Value,1,SpixTimeout);
	    while(((SPIH.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
	    {
	    }
	    *((__IO uint8_t*)&SPIH.Instance->DR) = Value;
}
*/

void spiwrite(uint8_t c)
{
	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	uint8_t dummy ;
	HAL_SPI_TransmitReceive(&SPIH, (uint8_t*) &c, &dummy,1 ,SpixTimeout);
	//while(SPIH.State!=HAL_SPI_STATE_READY){};
}

void    Delay(int num)
{
	HAL_Delay(num);
}

void setBK_imp(int perc);

int iBacklightPercent = 0;
void setBacklight(int percent)
{
	if(percent>100)percent = 100;
	if(percent<10)percent = 10;
	iBacklightPercent = percent;
	setBK_imp(iBacklightPercent);
}
int getBacklight()
{
	return iBacklightPercent;
}

void setBacklightON()
{

	setBK_imp(iBacklightPercent);
}

void setBacklightOff()
{
	setBK_imp(0);
}
void color_convert(uint16_t color,uint8_t* result)
{
	//~ #define RED             0xF800
	//~ #define BLUE            0x001F
	//~ #define GREEN           0x07E0
	result[2]=  ((color&0x1f)		<<(1+2))<<1;//|0x80;    //5 bit BLUE
	result[1]=  (((color>>5)&0x3f) <<(0+1))<<1;//|0x80;    //6 bit GREEN
	result[0]=  (((color>>11)&0x1f)<<(1+2))<<1;//|0x80;    //5 bit  //RED
}

uint16_t color_convertRGB_to16(uint8_t * adress)
{
	return ((adress[0]>>3)<<11)|((adress[1]>>2)<<5)|(adress[2]>>3);
}
uint16_t color_convertRGB_to16d(uint8_t R,uint8_t G,uint8_t B)
{
	return ((R>>3)<<11)|((G>>2)<<5)|(B>>3);
}

static uint16_t screen_width  = ILI9341_LCD_PIXEL_WIDTH;
static uint16_t screen_height = ILI9341_LCD_PIXEL_HEIGHT;
void LCD_setRotation(uint8_t rotation)
{
	setRotation(rotation);
}
int LCD_getWidth() {
    return screen_width;
}

int LCD_getHeight() {
    return screen_height;
}
static const uint8_t
  cmd_240x240[] = {                 		// Initialization commands for 7789 screens
    10-1,                       				// 9 commands in list:
    //ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
//      150,                     				// 150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
	  10,                    				// 255 = 500 ms delay
    ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
      0x55,                   				// 16-bit color
      10,                     				// 10 ms delay
    ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
      0x00,                   				// Row addr/col addr, bottom to top refresh
    ST7789_CASET  , 4,  					// 5: Column addr set, 4 args, no delay:
      0x00, ST7789_240x240_XSTART,          // XSTART = 0
	  (240+ST7789_240x240_XSTART) >> 8,
	  (240+ST7789_240x240_XSTART) & 0xFF,   // XEND = 240
    ST7789_RASET  , 4,  					// 6: Row addr set, 4 args, no delay:
      0x00, ST7789_240x240_YSTART,          // YSTART = 0
      (240+ST7789_240x240_YSTART) >> 8,
	  (240+ST7789_240x240_YSTART) & 0xFF,	// YEND = 240
    ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
      10,
    ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
      10,                     				// 10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
    100 };                  				// 255 = 500 ms delay
 uint16_t swapcolor(uint16_t x) {
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

 void writecommand(uint8_t c) {
	LCD_DC_0;
	spiwrite(c);
}

void writedata(uint8_t c)
{
	LCD_DC_1;
	spiwrite(c);
}
uint8_t  _xstart;
uint8_t  _ystart;

uint8_t pgm_read_byte(uint8_t *addr)
{
	return *addr;
}
void displayInit(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;
  //<-----------------------------------------------------------------------------------------
  //LCD_DC_1;
  //<-----------------------------------------------------------------------------------------

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      Delay(ms);
    }
  }
}
void LCD_reset()
{
	LCD_BL_1;
	LCD_RES_1;
    Delay(5);
	LCD_RES_0;
    Delay(15);
	LCD_RES_1;
    Delay(150);
}

void LCD_init()
{
    displayInit(cmd_240x240);
    setRotation(2);
}
void setRotation(uint8_t m) {

  writecommand(ST7789_MADCTL);
  int rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case PORTRAIT:
     writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);

     _xstart = 0;
     _ystart = 320-240;//_rowstart;
     break;
   case LANDSCAPE:
     writedata(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = 0;
     _xstart = 320-240;//_rowstart;
     break;
  case PORTRAIT_FLIP:
     writedata(ST7789_MADCTL_RGB);

     _xstart = 0;
     _ystart = 0;
     break;

   case LANDSCAPE_FLIP:
     writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = 0;
     _xstart = 0;
     break;
  }
}
#define READ_WSPI(x) {while(!((SPIH.Instance->SR) & SPI_FLAG_RXNE));x= *((__IO uint8_t*)&SPIH.Instance->DR);}

void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  uint8_t dummy;
  dummy = *((__IO uint8_t*)&SPIH.Instance->DR); // clear read flag
  LCD_DC_0;
  SPIx_WriteF(ST7789_CASET);
  READ_WSPI(dummy);

  LCD_DC_1;
  SPIx_WriteF(x_start >> 8);
  READ_WSPI(dummy);
  SPIx_WriteF(x_start &0xff);
  READ_WSPI(dummy);
  SPIx_WriteF(x_end >> 8);
  READ_WSPI(dummy);
  SPIx_WriteF(x_end &0xff);
  READ_WSPI(dummy);

  LCD_DC_0;
  SPIx_WriteF(ST7789_RASET);
  READ_WSPI(dummy);

  LCD_DC_1;
  SPIx_WriteF(y_start >> 8);
  READ_WSPI(dummy);
  SPIx_WriteF(y_start &0xff);
  READ_WSPI(dummy);
  SPIx_WriteF(y_end >> 8);
  READ_WSPI(dummy);
  SPIx_WriteF(y_end &0xff);
  READ_WSPI(dummy);

  LCD_DC_0;
  SPIx_WriteF(ST7789_RAMWR);
  READ_WSPI(dummy);
  LCD_DC_1;
}







void setAddrWindowA(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  uint8_t b4[4];

  writecommand(ST7789_CASET); // Column addr set
  b4[0] = x_start >> 8;
  b4[1] = x_start & 0xff;
  b4[2] = x_end >> 8;
  b4[3] = x_end & 0xff;
  LCD_DC_1;
  HAL_SPI_Transmit(&SPIH, (uint8_t*) &b4, 4 ,SpixTimeout);

  writecommand(ST7789_RASET); // Row addr set
  b4[0] = y_start >> 8;
  b4[1] = y_start & 0xff;
  b4[2] = y_end >> 8;
  b4[3] = y_end & 0xff;
  LCD_DC_1;
  HAL_SPI_Transmit(&SPIH, (uint8_t*) &b4, 4 ,SpixTimeout);

  writecommand(ST7789_RAMWR); // write to RAM
  LCD_DC_1;

}
#if 0
//uint8_t  block_send[128];
void LCD_sendLineRect(uint16_t y1,uint8_t * data)
{
	int x1 = 0;
	int w  = ILI9341_LCD_PIXEL_WIDTH;
	int h = 1;
	while(SPIH.State!=HAL_SPI_STATE_READY){};
	setAddrWindow(x1, y1,x1+w-1,y1+h-1);
	//LCD_DC_1;
	HAL_SPI_Transmit_DMA(&SPIH,data, ILI9341_LCD_PIXEL_WIDTH*2);
	//HAL_SPI_Transmit(&SPIH,data, ILI9341_LCD_PIXEL_WIDTH*2,SpixTimeout);
}

void LCD_Write8x8line16(uint16_t x1,uint16_t y1,uint16_t * data)
{
	int w  = 8;
	int h  = 8;
	while(SPIH.State!=HAL_SPI_STATE_READY){};
	setAddrWindow(x1, y1,x1+w-1,y1+h-1);
	//LCD_DC_1;
	HAL_SPI_Transmit_DMA(&SPIH,data, 8*8*2);
}
#endif

void LCD_fillRectData(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint8_t* color)
{
	int dw = w*h;
	setAddrWindow(x1, y1,x1+w-1,y1+h-1);
	int k;
	//LCD_DC_1;
	for(k=0;k<dw;k++)
	{
		SPIx_WriteF(color[k*2+1]);
		SPIx_WriteF(color[k*2+0]);
	}
}

//uint8_t block_send[128];

void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
	int dw = w*h;
	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	setAddrWindow(x1, y1,x1+w-1,y1+h-1);
	int k;
	uint8_t  Data0 = (color>>8u)&0xff;
	uint8_t  Data1 = (color)&0xff;

	//LCD_DC_1;
#if 0
	if(dw >= 64)
	{
		for(int k=0;k<64;k++)
		{
			block_send[2*k] = Data0;
			block_send[2*k+1] = Data1;
		}
		while(dw>=64)
		{
			HAL_SPI_Transmit_DMA(&SPIH,block_send, 128);
			while(SPIH.State!=HAL_SPI_STATE_READY){};
			dw-=64;
		}
		if(dw)
		{
			HAL_SPI_Transmit_DMA(&SPIH,block_send, dw*2);
			while(SPIH.State!=HAL_SPI_STATE_READY){};
		}

	}
	else
#endif
	{
		for(k=0;k<dw;k++)
		{
			SPIx_WriteF(Data0);
			SPIx_WriteF(Data1);
			//FMC_BANK1_WriteData(Data0);
			//FMC_BANK1_WriteData(Data1);
		}
	}
//	while(((hspi2.Instance->SR) & SPI_FLAG_BSY) != RESET)
//	{
//	}

}

void LCD_Draw_Line(int x0, int y0, int x1, int y1,uint16_t color) {

	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	int k;
	uint8_t  Data0 = (color>>8u)&0xff;
	uint8_t  Data1 = (color)&0xff;

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
		setAddrWindow(x0, y0,x0,y0);
		SPIx_WriteF(Data0);
		SPIx_WriteF(Data1);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}
void LCD_Draw_LineNX(int x0, int y0, int x1, int y1,int N,uint8_t* color) {

	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	int k;

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
		setAddrWindow(x0, y0,x0+N-1,y0);
		for(int k=0;k<N;k++)
		{
		SPIx_WriteF(color[k*2+1]);
		SPIx_WriteF(color[k*2]);
		}
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}
void LCD_Draw_LinePNX(int x0, int y0, int x1, int y1,int Width,int N,uint8_t* color) {

	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	int k;

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
		setAddrWindow(x0, y0,x0+N-1,y0);
		for(int k=0;k<N;k++)
		{
			int ind = (y0*Width+x0+k)*2;
			SPIx_WriteF(color[ind+1]);
			SPIx_WriteF(color[ind]);
		}
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}


void LCD_Draw_LinePNXShadow(int x0, int y0, int x1, int y1,int Width,int N,uint8_t* color) {

	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	int k;
  uint16_t* color16 = (uint16_t*)color;
  uint16_t  result_color;
  uint8_t*  pbyet = (uint8_t*)&result_color;

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
	  //while(SPIH.State!=HAL_SPI_STATE_READY){};
		setAddrWindow(x0, y0,x0+N-1,y0);
		//LCD_DC_1;
		for(int k=0;k<N;k++)
		{
			int ind = (y0*Width+x0+k);

			uint16_t clr = color16[ind];
#if 1
			result_color =  (( 7*((clr>>11)&0x1f)>>3)<<11)|
					        ( (7*((clr>>5)&0x3f)>>3)<<5)|
							  (7*(clr&0x1f)>>3 );
#else
			result_color =  ((   ((clr>>11)&0x1f)>>1)<<11)|
					        ( (  ((clr>>5)&0x3f)>>1)<<5)|
							  (  (clr&0x1f)>>1 );
#endif
			SPIx_WriteF(pbyet[1]);
			SPIx_WriteF(pbyet[0]);
		}
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}
void LCD_Draw_LineP(int x0, int y0, int x1, int y1,uint8_t* color) {

	//while(SPIH.State!=HAL_SPI_STATE_READY){};
	int k;

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
	  //while(SPIH.State!=HAL_SPI_STATE_READY){};
		setAddrWindow(x0, y0,x0,y0);
		int ind = (y0*240+x0)*2;
		//LCD_DC_1;
		SPIx_WriteF(color[ind+1]);
		SPIx_WriteF(color[ind]);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}

#include "fonts.h"
void LCD_Draw_Char2(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX,uint16_t SizeY, uint16_t Background_Colour)
{
	//~ flagReinit = 1;
	uint8_t 	function_char;
    int16_t 	i,j;

	function_char = Character;

	// Draw pixels
	LCD_fillRect(X, Y, CHAR_WIDTH*SizeX, CHAR_HEIGHT*SizeY, Background_Colour);

    if (function_char <= 0x20)
	{

	}
	else
	{
		function_char -= 0x20;
		int rw = (CHAR_WIDTH+7)/8;
		for (j=0; j<FONT_CURR.Height; j++)
		{
			uint8_t* pnt =  FONT_CURR.table+j*rw+function_char*FONT_CURR.Height*rw;
			for (i=0; i<FONT_CURR.Width; i++)
			{
				uint8_t bt = pnt[i/8];
				if(bt&(1<<(7-(i&7))))
				{
					LCD_fillRect(X+(i*SizeX), Y+(j*SizeY), SizeX,SizeY, Colour);
				}
			}
		}
	}
}

void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	LCD_Draw_Char2(Character,X,Y,Colour,Size,Size,Background_Colour);
}
/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void  LCD_Draw_Text(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text) {
        LCD_Draw_Char(*Text, X, Y, Colour, Size, Background_Colour);
        X += CHAR_WIDTH*Size;
	Text++;
    }
}
void  LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour)
{
    while (*Text) {
        LCD_Draw_Char2(*Text, X, Y, Colour, SizeX,SizeY, Background_Colour);
        X += CHAR_WIDTH*SizeX;
	Text++;
    }
}
