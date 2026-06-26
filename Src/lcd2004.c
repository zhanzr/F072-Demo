#include "main.h"
#include "lcd2004.h"

static inline void SimpleDelay(uint32_t d)
{
	uint32_t t = d * 3;
	while(--t)
	{
		__NOP();
	}
}

static inline void RS_H(void)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
}

static inline void RS_L(void)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
}

static inline void E_H(void)
{
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
}

static inline void E_L(void)
{
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
}

static inline void D4_H(void)
{
	HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_SET);
}

static inline void D4_L(void)
{
	HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_RESET);
}

static inline void D5_H(void)
{
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_SET);
}

static inline void D5_L(void)
{
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_RESET);
}

static inline void D6_H(void)
{
	HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, GPIO_PIN_SET);
}

static inline void D6_L(void)
{
	HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, GPIO_PIN_RESET);
}

static inline void D7_H(void)
{
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_SET);
}

static inline void D7_L(void)
{
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_RESET);
}

static inline void DB4_Wr(uint8_t dat) {
	(0==(dat&0x08))?D7_L():D7_H();
	(0==(dat&0x04))?D6_L():D6_H();
	(0==(dat&0x02))?D5_L():D5_H();
	(0==(dat&0x01))?D4_L():D4_H();
}

void LCD_WrCmd_4 (uint8_t cmd)
{
	SimpleDelay(WAIT_AVAIL_DLY);

	RS_L();
	SimpleDelay(10);
//	RW_L();
	DB4_Wr(cmd >> 4);
	SimpleDelay(10);
	E_H();
	SimpleDelay(10);
	E_L();
	SimpleDelay(10);
	DB4_Wr(cmd & 0x0F);
	SimpleDelay(10);
	E_H();
	SimpleDelay(10);
	E_L();
}

void LCD_WrDat_4 (uint8_t dat)
{
	SimpleDelay(WAIT_AVAIL_DLY);

	RS_H();
	SimpleDelay(10);
//	RW_L();
	DB4_Wr(dat >> 4);
	SimpleDelay(10);
	E_H();
	SimpleDelay(10);
	E_L();
	SimpleDelay(10);
	DB4_Wr(dat & 0x0F);
	SimpleDelay(10);
	E_H();
	SimpleDelay(10);
	E_L();
}

void LCD_SetPos (uint8_t x, uint8_t y)
{
	const static uint8_t pos_tab[] = {0x80, 0xc0, 0x94, 0xd4};

	LCD_WrCmd_4(pos_tab[x] + y);
}

void LCD_Initialize (void) {
	// 4-bit mode
	LCD_WrCmd_4(0x33);
	LCD_WrCmd_4(0x32);
	LCD_WrCmd_4(FUNCTION_SET | OPT_N);

	LCD_WrCmd_4(CLEAR_DISPLAY);
	LCD_WrCmd_4(DISPLAY_ON_OFF_CONTROL | OPT_D);
	LCD_WrCmd_4(ENTRY_MODE_SET | OPT_INC);
}

void LCD_displayL(uint8_t l,uint8_t hori,uint8_t *s)
{
	LCD_SetPos(l, hori);
	while(*s) {
		LCD_WrDat_4(*s);
		s++;
	}
}
