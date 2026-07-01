#include "ascii_font.h"
#include "lcd_st7735.h"
#include "main.h"

void Gui_Circle(uint16_t X, uint16_t Y, uint16_t R, uint16_t fc) {
	uint16_t a, b;
	int c;
	a = 0;
	b = R;
	c = 3 - 2 * R;
	while (a < b) {
		Gui_DrawPoint(X + a, Y + b, fc); //        7
		Gui_DrawPoint(X - a, Y + b, fc); //        6
		Gui_DrawPoint(X + a, Y - b, fc); //        2
		Gui_DrawPoint(X - a, Y - b, fc); //        3
		Gui_DrawPoint(X + b, Y + a, fc); //        8
		Gui_DrawPoint(X - b, Y + a, fc); //        5
		Gui_DrawPoint(X + b, Y - a, fc); //        1
		Gui_DrawPoint(X - b, Y - a, fc); //        4

		if (c < 0)
			c = c + 4 * a + 6;
		else {
			c = c + 4 * (a - b) + 10;
			b -= 1;
		}
		a += 1;
	}
	if (a == b) {
		Gui_DrawPoint(X + a, Y + b, fc);
		Gui_DrawPoint(X + a, Y + b, fc);
		Gui_DrawPoint(X + a, Y - b, fc);
		Gui_DrawPoint(X - a, Y - b, fc);
		Gui_DrawPoint(X + b, Y + a, fc);
		Gui_DrawPoint(X - b, Y + a, fc);
		Gui_DrawPoint(X + b, Y - a, fc);
		Gui_DrawPoint(X - b, Y - a, fc);
	}
}

void Gui_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t Color) {
	int dx,  // difference in x's
			dy,  // difference in y's
			dx2, // dx,dy * 2
			dy2, x_inc, // amount in pixel space to move during drawing
			y_inc, // amount in pixel space to move during drawing
			error, // the discriminant i.e. error i.e. decision variable
			index; // used for looping

	Lcd_SetXY(x0, y0);
	dx = x1 - x0; // ¼ÆËãx¾àÀë
	dy = y1 - y0; // ¼ÆËãy¾àÀë

	if (dx >= 0) {
		x_inc = 1;
	} else {
		x_inc = -1;
		dx = -dx;
	}

	if (dy >= 0) {
		y_inc = 1;
	} else {
		y_inc = -1;
		dy = -dy;
	}

	dx2 = dx << 1;
	dy2 = dy << 1;

	if (dx > dy) // x¾àÀë´óÓÚy¾àÀë£¬ÄÇÃ´Ã¿¸öxÖáÉÏÖ»ÓÐÒ»¸öµã£¬Ã¿¸öyÖáÉÏÓÐÈô¸É¸öµã
			{ // ÇÒÏßµÄµãÊýµÈÓÚx¾àÀë£¬ÒÔxÖáµÝÔö»­µã
			  // initialize error term
		error = dy2 - dx;

		// draw the line
		for (index = 0; index <= dx; index++) // Òª»­µÄµãÊý²»»á³¬¹ýx¾àÀë
				{
			// »­µã
			Gui_DrawPoint(x0, y0, Color);

			// test if error has overflowed
			if (error >= 0) // ÊÇ·ñÐèÒªÔö¼Óy×ø±êÖµ
					{
				error -= dx2;

				// move to next line
				y0 += y_inc; // Ôö¼Óy×ø±êÖµ
			} // end if error overflowed

			// adjust the error term
			error += dy2;

			// move to the next pixel
			x0 += x_inc; // x×ø±êÖµÃ¿´Î»­µãºó¶¼µÝÔö1
		} // end for
	} // end if |slope| <= 1
	else {
		// initialize error term
		error = dx2 - dy;

		// draw the line
		for (index = 0; index <= dy; index++) {
			// set the pixel
			Gui_DrawPoint(x0, y0, Color);

			// test if error overflowed
			if (error >= 0) {
				error -= dy2;

				// move to next line
				x0 += x_inc;
			} // end if error overflowed

			// adjust the error term
			error += dx2;

			// move to the next pixel
			y0 += y_inc;
		} // end for
	} // end else |slope| > 1
}

void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bc) {
	Gui_DrawLine(x, y, x + w, y, 0xEF7D);
	Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
	Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
	Gui_DrawLine(x, y, x, y + h, 0xEF7D);
	Gui_DrawLine(x + 1, y + 1, x + 1 + w - 2, y + 1 + h - 2, bc);
}

void Gui_box2(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mode) {
	if (mode == 0) {
		Gui_DrawLine(x, y, x + w, y, 0xEF7D);
		Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
		Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
		Gui_DrawLine(x, y, x, y + h, 0xEF7D);
	}
	if (mode == 1) {
		Gui_DrawLine(x, y, x + w, y, 0x2965);
		Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xEF7D);
		Gui_DrawLine(x, y + h, x + w, y + h, 0xEF7D);
		Gui_DrawLine(x, y, x, y + h, 0x2965);
	}
	if (mode == 2) {
		Gui_DrawLine(x, y, x + w, y, 0xffff);
		Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xffff);
		Gui_DrawLine(x, y + h, x + w, y + h, 0xffff);
		Gui_DrawLine(x, y, x, y + h, 0xffff);
	}
}

void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc,
		uint8_t *s) {
	uint8_t i, j;
	uint16_t k, x0;
	x0 = x;

	while (*s) {
		if ((*s) < 128) {
			k = *s;
			if (k == 13) {
				x = x0;
				y += 16;
			} else {
				if (k > 32)
					k -= 32;
				else
					k = 0;

				for (i = 0; i < 16; i++)
					for (j = 0; j < 8; j++) {
						if (asc16[k * 16 + i] & (0x80 >> j))
							Gui_DrawPoint(x + j, y + i, fc);
						else {
							if (fc != bc)
								Gui_DrawPoint(x + j, y + i, bc);
						}
					}
				x += 8;
			}
			s++;
		}

		else {
			// non ascii NOT SUPPORTED
		}
	}
}

void Gui_DrawFont_char(uint16_t x, uint16_t y, uint8_t s) {
	uint8_t i, j;

	if (s > 32)
		s -= 32;
	else
		s = 0;

	for (i = 0; i < 16; i++)
		for (j = 0; j < 8; j++) {
			if (asc16[s * 16 + i] & (0x80 >> j))
				Gui_DrawPoint(x + j, y + i, RED);
			else {
				Gui_DrawPoint(x + j, y + i, GRAY0);
			}
		}
}

int oled_pow(uint8_t m, uint8_t n) {
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

void LCD_ShowNum(uint8_t x, uint8_t y, uint16_t num, uint8_t len) {
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++) {
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1)) {
			if (temp == 0) {
				Gui_DrawFont_char(x + 8 * t, y, ' ');
				continue;
			} else
				enshow = 1;
		}
		Gui_DrawFont_char(x + 8 * t, y, temp + '0');
	}
}

void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc,
		uint8_t *s) {
	uint8_t i, j;
	uint16_t k;

	while (*s) {
		if (*s < 0x80) {
			k = *s;
			if (k > 32)
				k -= 32;
			else
				k = 0;

			for (i = 0; i < 16; i++)
				for (j = 0; j < 8; j++) {
					if (asc16[k * 16 + i] & (0x80 >> j))
						Gui_DrawPoint(x + j, y + i, fc);
					else {
						if (fc != bc)
							Gui_DrawPoint(x + j, y + i, bc);
					}
				}
			s++;
			x += 8;
		} else {
			// non ascii NOT SUPPORTED
		}
	}
}

void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc,
		uint16_t num) {
	uint8_t i, j, k, c;

	for (i = 0; i < 32; i++) {
		for (j = 0; j < 4; j++) {
			c = *(sz32 + num * 32 * 4 + i * 4 + j);
			for (k = 0; k < 8; k++) {

				if (c & (0x80 >> k))
					Gui_DrawPoint(x + j * 8 + k, y + i, fc);
				else {
					if (fc != bc)
						Gui_DrawPoint(x + j * 8 + k, y + i, bc);
				}
			}
		}
	}
}
