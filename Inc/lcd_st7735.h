/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD2004_H__
#define __LCD2004_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// void LCD_Initialize (void);
// void LCD_displayL(uint8_t x,uint8_t y,uint8_t *s);
//

/* USER CODE BEGIN Prototypes */
#define WHITE 0xFFFF
#define BLACK 0x0000
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define MAGENTA 0xF81F
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

#define GRAY0 0xEF7D
#define GRAY1 0x8410
#define GRAY2 0x4208

#define ST7735_LCD_PIXEL_WIDTH	160
#define ST7735_LCD_PIXEL_HEIGHT	80

/**
 * @brief  ST7735 Registers
 */
#define LCD_REG_0 0x00   /* No Operation: NOP */
#define LCD_REG_1 0x01   /* Software reset: SWRESET */
#define LCD_REG_4 0x04   /* Read Display ID: RDDID */
#define LCD_REG_9 0x09   /* Read Display Statu: RDDST */
#define LCD_REG_10 0x0A  /* Read Display Power: RDDPM */
#define LCD_REG_11 0x0B  /* Read Display: RDDMADCTL */
#define LCD_REG_12 0x0C  /* Read Display Pixel: RDDCOLMOD */
#define LCD_REG_13 0x0D  /* Read Display Image: RDDIM */
#define LCD_REG_14 0x0E  /* Read Display Signal: RDDSM */
#define LCD_REG_16 0x10  /* Sleep in & booster off: SLPIN */
#define LCD_REG_17 0x11  /* Sleep out & booster on: SLPOUT */
#define LCD_REG_18 0x12  /* Partial mode on: PTLON */
#define LCD_REG_19 0x13  /* Partial off (Normal): NORON */
#define LCD_REG_32 0x20  /* Display inversion off: INVOFF */
#define LCD_REG_33 0x21  /* Display inversion on: INVON */
#define LCD_REG_38 0x26  /* Gamma curve select: GAMSET */
#define LCD_REG_40 0x28  /* Display off: DISPOFF */
#define LCD_REG_41 0x29  /* Display on: DISPON */
#define LCD_REG_42 0x2A  /* Column address set: CASET */
#define LCD_REG_43 0x2B  /* Row address set: RASET */
#define LCD_REG_44 0x2C  /* Memory write: RAMWR */
#define LCD_REG_45 0x2D  /* LUT for 4k,65k,262k color: RGBSET */
#define LCD_REG_46 0x2E  /* Memory read: RAMRD*/
#define LCD_REG_48 0x30  /* Partial start/end address set: PTLAR */
#define LCD_REG_52 0x34  /* Tearing effect line off: TEOFF */
#define LCD_REG_53 0x35  /* Tearing effect mode set & on: TEON */
#define LCD_REG_54 0x36  /* Memory data access control: MADCTL */
#define LCD_REG_56 0x38  /* Idle mode off: IDMOFF */
#define LCD_REG_57 0x39  /* Idle mode on: IDMON */
#define LCD_REG_58 0x3A  /* Interface pixel format: COLMOD */
#define LCD_REG_177 0xB1 /* In normal mode (Full colors): FRMCTR1 */
#define LCD_REG_178 0xB2 /* In Idle mode (8-colors): FRMCTR2 */
#define LCD_REG_179 0xB3 /* In partial mode + Full colors: FRMCTR3 */
#define LCD_REG_180 0xB4 /* Display inversion control: INVCTR */
#define LCD_REG_192 0xC0 /* Power control setting: PWCTR1 */
#define LCD_REG_193 0xC1 /* Power control setting: PWCTR2 */
#define LCD_REG_194 0xC2 /* In normal mode (Full colors): PWCTR3 */
#define LCD_REG_195 0xC3 /* In Idle mode (8-colors): PWCTR4 */
#define LCD_REG_196 0xC4 /* In partial mode + Full colors: PWCTR5 */
#define LCD_REG_197 0xC5 /* VCOM control 1: VMCTR1 */
#define LCD_REG_199 0xC7 /* Set VCOM offset control: VMOFCTR */
#define LCD_REG_209 0xD1 /* Set LCM version code: WRID2 */
#define LCD_REG_210 0xD2 /* Customer Project code: WRID3 */
#define LCD_REG_217 0xD9 /* NVM control status: NVCTR1 */
#define LCD_REG_218 0xDA /* Read ID1: RDID1 */
#define LCD_REG_219 0xDB /* Read ID2: RDID2 */
#define LCD_REG_220 0xDC /* Read ID3: RDID3 */
#define LCD_REG_222 0xDE /* NVM Read Command: NVCTR2 */
#define LCD_REG_223 0xDF /* NVM Write Command: NVCTR3 */
#define LCD_REG_224 0xE0 /* Set Gamma adjustment (+ polarity): GAMCTRP1 */
#define LCD_REG_225 0xE1 /* Set Gamma adjustment (- polarity): GAMCTRN1 */

void     st7735_Init(void);
uint16_t st7735_ReadID(void);

void     st7735_DisplayOn(void);
void     st7735_DisplayOff(void);
void     st7735_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     st7735_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
void     st7735_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue);
uint8_t  st7735_ReadReg(uint8_t LCDReg);

void     st7735_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     st7735_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     st7735_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);

uint16_t st7735_GetLcdPixelWidth(void);
uint16_t st7735_GetLcdPixelHeight(void);
void     st7735_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);

void     LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void     LCD_IO_WriteReg(uint8_t Reg);
void     LCD_Delay(uint32_t delay);

void Lcd_WriteIndex(uint8_t Index);
void Lcd_WriteData(uint8_t Data);
void Lcd_WriteReg(uint8_t Index, uint8_t Data);
uint16_t Lcd_ReadReg(uint8_t LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(uint16_t Color);
void Lcd_SetXY(uint16_t x, uint16_t y);
void Gui_DrawPoint(uint16_t x, uint16_t y, uint16_t Data);
unsigned int Lcd_ReadPoint(uint16_t x, uint16_t y);
void Lcd_SetRegion(uint16_t x_start, uint16_t y_start, uint16_t x_end,
                   uint16_t y_end);
void LCD_WriteData_16Bit(uint16_t Data);

void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc,
                        uint8_t *s);
void Gui_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t Color);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __LCD2004_H__ */
