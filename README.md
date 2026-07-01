Demo for F072RTB6 Discovery Board + Uart + ADC + PWM + ST7735S.

0.96' 160x80 ST7735S LCD module.
Backlight(Could be left unconnected, if don't need to control the intensity of backlight, it is default on):
BL	<->	PA15

CS	<->	PB8
DC	<->	PA1
RST	<->	PB9

MOSI	<->	PB7
SCLK	<->	PB5

3V3
GND

This is a pure soft spi flavour for easy testing and porting.