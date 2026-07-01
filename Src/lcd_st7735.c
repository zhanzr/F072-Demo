#include "lcd_st7735.h"
#include "main.h"

static uint16_t ArrayRGB[320];

static inline void SimpleDelay(uint32_t d) {
	uint32_t t = d * 3;
	while (--t) {
		__NOP();
	}
}

static inline void Lcd_Delay(uint32_t d) {
	volatile uint32_t c = HAL_GetTick() + d;
	while (c > HAL_GetTick()) {
		__NOP();
	}
}

static inline void DC_H(void) {
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
}

static inline void DC_L(void) {
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
}

static inline void CS_H(void) {
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
}

static inline void CS_L(void) {
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
}

static inline void RST_H(void) {
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_SET);
}

static inline void RST_L(void) {
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
}

static inline void SCL_H(void) {
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_SET);
}

static inline void SCL_L(void) {
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, GPIO_PIN_RESET);
}

static inline void MOSI_H(void) {
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_SET);
}

static inline void MOSI_L(void) {
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_RESET);
}

void softspi_write_8bit(uint8_t dat) {
	for (uint8_t i = 0; i < 8; i++) {
		if ((dat & 0x80) != 0) {
			MOSI_H();
		} else {
			MOSI_L();
		}

		dat <<= 1;

		SCL_L();
		SCL_H();
	}
}

#define lcd_write_8bit softspi_write_8bit

void LCD_IO_WriteReg(uint8_t i) {
	CS_L();
	DC_L();

	lcd_write_8bit(i);
	CS_H();
}

void LCD_WR_DATA(uint8_t i) {
	CS_L();
	DC_H();

	lcd_write_8bit(i);
	CS_H();
}

void LCD_RESET(void) {
	RST_H();
	Lcd_Delay(1);
	RST_L();
	Lcd_Delay(100);
	RST_H();
	Lcd_Delay(20);
}

void LCD_WriteData_16Bit(uint16_t Data) {
	CS_L();

	DC_H();
	lcd_write_8bit(Data >> 8);
	lcd_write_8bit(Data);

	CS_H();
}

void Lcd_WriteReg(uint8_t Index, uint8_t Data) {
	LCD_IO_WriteReg(Index);
	LCD_WR_DATA(Data);
}

void LCD_IO_WriteMultipleData(uint8_t *buf, uint32_t len) {
	CS_L();
	DC_H();
	for (uint32_t i = 0; i < len; ++i) {
		lcd_write_8bit(*(buf + i));
	}
	CS_H();
}

void Lcd_SetRegion(uint16_t x_start, uint16_t y_start, uint16_t x_end,
		uint16_t y_end) {
	LCD_IO_WriteReg(0x2a);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(x_start + 1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(x_end + 1);

	LCD_IO_WriteReg(0x2b);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(y_start + 0x1A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(y_end + 0x1A);
	LCD_IO_WriteReg(0x2c);
}

void Lcd_SetXY(uint16_t x, uint16_t y) {
	Lcd_SetRegion(x, y, x, y);
}

void Gui_DrawPoint(uint16_t x, uint16_t y, uint16_t Data) {
	Lcd_SetRegion(x, y, x + 1, y + 1);
	LCD_WriteData_16Bit(Data);
}

void Lcd_Clear(uint16_t Color) {
	Lcd_SetRegion(0, 0, ST7735_LCD_PIXEL_WIDTH - 1, ST7735_LCD_PIXEL_HEIGHT - 1);
	LCD_IO_WriteReg(0x2C);
	for (uint16_t i = 0; i < ST7735_LCD_PIXEL_WIDTH; i++)
		for (uint16_t m = 0; m < ST7735_LCD_PIXEL_HEIGHT; m++) {
			LCD_WriteData_16Bit(Color);
		}
}

void st7735_Init(void) {
	uint8_t data = 0;

	/* Initialize ST7735 low level bus layer -----------------------------------*/
	LCD_RESET();

	/* Out of sleep mode, 0 args, no delay */
	st7735_WriteReg(LCD_REG_17, 0x00);
	/* Frame rate ctrl - normal mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)*/
	LCD_IO_WriteReg(LCD_REG_177);
	data = 0x01;
	LCD_IO_WriteMultipleData(&data, 1);
	data = 0x2C;
	LCD_IO_WriteMultipleData(&data, 1);
	data = 0x2D;
	LCD_IO_WriteMultipleData(&data, 1);
	/* Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D) */
	st7735_WriteReg(LCD_REG_178, 0x01);
	st7735_WriteReg(LCD_REG_178, 0x2C);
	st7735_WriteReg(LCD_REG_178, 0x2D);
	/* Frame rate ctrl - partial mode, 6 args: Dot inversion mode, Line inversion mode */
	st7735_WriteReg(LCD_REG_179, 0x01);
	st7735_WriteReg(LCD_REG_179, 0x2C);
	st7735_WriteReg(LCD_REG_179, 0x2D);
	st7735_WriteReg(LCD_REG_179, 0x01);
	st7735_WriteReg(LCD_REG_179, 0x2C);
	st7735_WriteReg(LCD_REG_179, 0x2D);
	/* Display inversion ctrl, 1 arg, no delay: No inversion */
	st7735_WriteReg(LCD_REG_180, 0x07);
	/* Power control, 3 args, no delay: -4.6V , AUTO mode */
	st7735_WriteReg(LCD_REG_192, 0xA2);
	st7735_WriteReg(LCD_REG_192, 0x02);
	st7735_WriteReg(LCD_REG_192, 0x84);
	/* Power control, 1 arg, no delay: VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD */
	st7735_WriteReg(LCD_REG_193, 0xC5);
	/* Power control, 2 args, no delay: Opamp current small, Boost frequency */
	st7735_WriteReg(LCD_REG_194, 0x0A);
	st7735_WriteReg(LCD_REG_194, 0x00);
	/* Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low */
	st7735_WriteReg(LCD_REG_195, 0x8A);
	st7735_WriteReg(LCD_REG_195, 0x2A);
	/* Power control, 2 args, no delay */
	st7735_WriteReg(LCD_REG_196, 0x8A);
	st7735_WriteReg(LCD_REG_196, 0xEE);
	/* Power control, 1 arg, no delay */
	st7735_WriteReg(LCD_REG_197, 0x0E);
	/* Don't invert display, no args, no delay */
	LCD_IO_WriteReg(LCD_REG_32);
	/* Set color mode, 1 arg, no delay: 16-bit color */
	st7735_WriteReg(LCD_REG_58, 0x05);
	/* Column addr set, 4 args, no delay: XSTART XEND */
	LCD_IO_WriteReg(LCD_REG_42);
	data = 0x00;
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteMultipleData(&data, 1);
	data = ST7735_LCD_PIXEL_WIDTH - 1;
	LCD_IO_WriteMultipleData(&data, 1);
	/* Row addr set, 4 args, no delay: YSTART YEND */
	LCD_IO_WriteReg(LCD_REG_43);
	data = 0x00;
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteMultipleData(&data, 1);
	data = ST7735_LCD_PIXEL_HEIGHT - 1;
	LCD_IO_WriteMultipleData(&data, 1);
	/* Magical unicorn dust, 16 args, no delay */
	st7735_WriteReg(LCD_REG_224, 0x02);
	st7735_WriteReg(LCD_REG_224, 0x1c);
	st7735_WriteReg(LCD_REG_224, 0x07);
	st7735_WriteReg(LCD_REG_224, 0x12);
	st7735_WriteReg(LCD_REG_224, 0x37);
	st7735_WriteReg(LCD_REG_224, 0x32);
	st7735_WriteReg(LCD_REG_224, 0x29);
	st7735_WriteReg(LCD_REG_224, 0x2d);
	st7735_WriteReg(LCD_REG_224, 0x29);
	st7735_WriteReg(LCD_REG_224, 0x25);
	st7735_WriteReg(LCD_REG_224, 0x2B);
	st7735_WriteReg(LCD_REG_224, 0x39);
	st7735_WriteReg(LCD_REG_224, 0x00);
	st7735_WriteReg(LCD_REG_224, 0x01);
	st7735_WriteReg(LCD_REG_224, 0x03);
	st7735_WriteReg(LCD_REG_224, 0x10);
	/* Sparkles and rainbows, 16 args, no delay */
	st7735_WriteReg(LCD_REG_225, 0x03);
	st7735_WriteReg(LCD_REG_225, 0x1d);
	st7735_WriteReg(LCD_REG_225, 0x07);
	st7735_WriteReg(LCD_REG_225, 0x06);
	st7735_WriteReg(LCD_REG_225, 0x2E);
	st7735_WriteReg(LCD_REG_225, 0x2C);
	st7735_WriteReg(LCD_REG_225, 0x29);
	st7735_WriteReg(LCD_REG_225, 0x2D);
	st7735_WriteReg(LCD_REG_225, 0x2E);
	st7735_WriteReg(LCD_REG_225, 0x2E);
	st7735_WriteReg(LCD_REG_225, 0x37);
	st7735_WriteReg(LCD_REG_225, 0x3F);
	st7735_WriteReg(LCD_REG_225, 0x00);
	st7735_WriteReg(LCD_REG_225, 0x00);
	st7735_WriteReg(LCD_REG_225, 0x02);
	st7735_WriteReg(LCD_REG_225, 0x10);
	/* Normal display on, no args, no delay */
	st7735_WriteReg(LCD_REG_19, 0x00);
	/* Main screen turn on, no delay */
	st7735_WriteReg(LCD_REG_41, 0x00);
	/* Memory access control */
	st7735_WriteReg(LCD_REG_54, 0xA8);
}

/**
 * @brief  Enables the Display.
 * @param  None
 * @retval None
 */
void st7735_DisplayOn(void) {
	uint8_t data = 0;
	LCD_IO_WriteReg(LCD_REG_19);
	Lcd_Delay(10);
	LCD_IO_WriteReg(LCD_REG_41);
	Lcd_Delay(10);
	LCD_IO_WriteReg(LCD_REG_54);
	data = 0xC0;
	LCD_IO_WriteMultipleData(&data, 1);
}

/**
 * @brief  Disables the Display.
 * @param  None
 * @retval None
 */
void st7735_DisplayOff(void) {
	uint8_t data = 0;
	LCD_IO_WriteReg(LCD_REG_19);
	Lcd_Delay(10);
	LCD_IO_WriteReg(LCD_REG_40);
	Lcd_Delay(10);
	LCD_IO_WriteReg(LCD_REG_54);
	data = 0xC0;
	LCD_IO_WriteMultipleData(&data, 1);
}

/**
 * @brief  Sets Cursor position.
 * @param  Xpos: specifies the X position.
 * @param  Ypos: specifies the Y position.
 * @retval None
 */
void st7735_SetCursor(uint16_t Xpos, uint16_t Ypos) {
	uint8_t data = 0;
	LCD_IO_WriteReg(LCD_REG_42);
	data = (Xpos) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Xpos) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteReg(LCD_REG_43);
	data = (Ypos) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Ypos) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
	LCD_IO_WriteReg(LCD_REG_44);
}

/**
 * @brief  Writes pixel.
 * @param  Xpos: specifies the X position.
 * @param  Ypos: specifies the Y position.
 * @param  RGBCode: the RGB pixel color
 * @retval None
 */
void st7735_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode) {
	uint8_t data = 0;
	if ((Xpos >= ST7735_LCD_PIXEL_WIDTH) || (Ypos >= ST7735_LCD_PIXEL_HEIGHT)) {
		return;
	}

	/* Set Cursor */
	st7735_SetCursor(Xpos, Ypos);

	data = RGBCode >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = RGBCode;
	LCD_IO_WriteMultipleData(&data, 1);
}

/**
 * @brief  Writes to the selected LCD register.
 * @param  LCDReg: Address of the selected register.
 * @param  LCDRegValue: value to write to the selected register.
 * @retval None
 */
void st7735_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue) {
	LCD_IO_WriteReg(LCDReg);
	LCD_IO_WriteMultipleData(&LCDRegValue, 1);
}

/**
 * @brief  Sets a display window
 * @param  Xpos:   specifies the X bottom left position.
 * @param  Ypos:   specifies the Y bottom left position.
 * @param  Height: display window height.
 * @param  Width:  display window width.
 * @retval None
 */
void st7735_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width,
		uint16_t Height) {
	uint8_t data = 0;
	/* Column addr set, 4 args, no delay: XSTART = Xpos, XEND = (Xpos + Width - 1) */
	LCD_IO_WriteReg(LCD_REG_42);
	data = (Xpos) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Xpos) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Xpos + Width - 1) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Xpos + Width - 1) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
	/* Row addr set, 4 args, no delay: YSTART = Ypos, YEND = (Ypos + Height - 1) */
	LCD_IO_WriteReg(LCD_REG_43);
	data = (Ypos) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Ypos) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Ypos + Height - 1) >> 8;
	LCD_IO_WriteMultipleData(&data, 1);
	data = (Ypos + Height - 1) & 0xFF;
	LCD_IO_WriteMultipleData(&data, 1);
}

/**
 * @brief  Draws horizontal line.
 * @param  RGBCode: Specifies the RGB color
 * @param  Xpos: specifies the X position.
 * @param  Ypos: specifies the Y position.
 * @param  Length: specifies the line length.
 * @retval None
 */
void st7735_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos,
		uint16_t Length) {
	uint8_t counter = 0;

	if (Xpos + Length > ST7735_LCD_PIXEL_WIDTH)
		return;

	/* Set Cursor */
	st7735_SetCursor(Xpos, Ypos);

	for (counter = 0; counter < Length; counter++) {
		ArrayRGB[counter] = RGBCode;
	}
	LCD_IO_WriteMultipleData((uint8_t*) &ArrayRGB[0], Length * 2);
}

/**
 * @brief  Draws vertical line.
 * @param  RGBCode: Specifies the RGB color
 * @param  Xpos: specifies the X position.
 * @param  Ypos: specifies the Y position.
 * @param  Length: specifies the line length.
 * @retval None
 */
void st7735_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos,
		uint16_t Length) {
	uint8_t counter = 0;

	if (Ypos + Length > ST7735_LCD_PIXEL_HEIGHT)
		return;
	for (counter = 0; counter < Length; counter++) {
		st7735_WritePixel(Xpos, Ypos + counter, RGBCode);
	}
}

/**
 * @brief  Gets the LCD pixel Width.
 * @param  None
 * @retval The Lcd Pixel Width
 */
uint16_t st7735_GetLcdPixelWidth(void) {
	return ST7735_LCD_PIXEL_WIDTH;
}

/**
 * @brief  Gets the LCD pixel Height.
 * @param  None
 * @retval The Lcd Pixel Height
 */
uint16_t st7735_GetLcdPixelHeight(void) {
	return ST7735_LCD_PIXEL_HEIGHT;
}

/**
 * @brief  Displays a bitmap picture loaded in the internal Flash.
 * @param  BmpAddress: Bmp picture address in the internal Flash.
 * @retval None
 */
void st7735_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp) {
	uint32_t index = 0, size = 0;

	/* Read bitmap size */
	size = *(volatile uint16_t*) (pbmp + 2);
	size |= (*(volatile uint16_t*) (pbmp + 4)) << 16;
	/* Get bitmap data address offset */
	index = *(volatile uint16_t*) (pbmp + 10);
	index |= (*(volatile uint16_t*) (pbmp + 12)) << 16;
	size = (size - index) / 2;
	pbmp += index;

	/* Set GRAM write direction and BGR = 0 */
	/* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
	st7735_WriteReg(LCD_REG_54, 0x40);

	/* Set Cursor */
	st7735_SetCursor(Xpos, Ypos);

	LCD_IO_WriteMultipleData((uint8_t*) pbmp, size * 2);

	/* Set GRAM write direction and BGR = 0 */
	/* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
	st7735_WriteReg(LCD_REG_54, 0xC0);
}
