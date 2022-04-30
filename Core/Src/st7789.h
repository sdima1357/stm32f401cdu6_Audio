/**
  ******************************************************************************
  * @file    ili9341.h
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file contains all the functions prototypes for the ili9341.c
  *          driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST7789_H
#define __ST7789_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include <stdint.h>
#define BLACK           0x0000      /*   0,   0,   0 */
#define NAVY            0x000F      /*   0,   0, 128 */
#define DGREEN          0x03E0      /*   0, 128,   0 */
#define DCYAN           0x03EF      /*   0, 128, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define LGRAY           0xC618      /* 192, 192, 192 */
#define DGRAY           0x7BEF      /* 128, 128, 128 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0xFFE0      /* 255, 255,   0 */
#define WHITE           0xFFFF      /* 255, 255, 255 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */
#define BROWN                 0XBC40
#define BRRED                 0XFC07
#define LG                    ((0xd7>>3)<<11)|((0xd0>>2)<<5)|(0xd7>>3)
#define ST7789_TFTWIDTH_240 	240
#define ST7789_TFTHEIGHT_240 	240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0

#define ST_CMD_DELAY   0x80    // special signifier for command lists

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

/* Includes ------------------------------------------------------------------*/

#define  ILI9341_LCD_PIXEL_WIDTH    ((uint16_t)240)
#define  ILI9341_LCD_PIXEL_HEIGHT   ((uint16_t)240)


void LCD_init();
uint32_t LCD_ReadID();
void LCD_setRotation(uint8_t rotation);
void LCD_sendLineRect(uint16_t y1,uint8_t * data);
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color);
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color);
void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void LCD_Draw_Text(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour);
void LCD_Draw_LineP(int x0, int y0, int x1, int y1,uint8_t* color) ;
void LCD_Draw_Line(int x0, int y0, int x1, int y1,uint16_t color) ;
void LCD_reset();
void setLCD(int num);
uint16_t color_convertRGB_to16(uint8_t * adress);
uint16_t color_convertRGB_to16d(uint8_t R,uint8_t G,uint8_t B);
void LCD_Draw_LinePNX(int x0, int y0, int x1, int y1,int Width,int N,uint8_t* color);


void setBacklight(int percent);
int  getBacklight();

/* Level 1 Commands */

#define PORTRAIT	0
#define LANDSCAPE	1
#define PORTRAIT_FLIP	2
#define LANDSCAPE_FLIP	3

//#define COLOR_3BYTES 1

/* LCD IO functions */
//~ void     LCD_IO_Init(void);
//~ void     LCD_IO_WriteData(uint16_t RegValue);
//~ void     LCD_IO_WriteReg(uint8_t Reg);
//~ uint16_t LCD_IO_ReadData(uint16_t RegValue);
void     LCD_Delay (uint32_t delay);
      
#ifdef __cplusplus
}
#endif

#endif /* __ILI9341_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
