/*
 * display.c
 *
 *  Created on: Feb 8, 2024
 *      Author: aivs
 */

#include "display.h"
#include "printf.h"

#define X_PARAMETER_CENTER_1_OF_4 35
#define X_PARAMETER_CENTER_2_OF_4 97
#define X_PARAMETER_CENTER_3_OF_4 159
#define X_PARAMETER_CENTER_4_OF_4 221

#define Y_PARAMATER_VALUE 40

/* Private variables ---------------------------------------------------------*/
int16_t stored_values[GET_SIZE(params)] = {0};
/* Private function prototypes -----------------------------------------------*/
uint8_t getX(uint8_t index, uint8_t pixel_width_string);

/* Private user code ---------------------------------------------------------*/
void display_UpdateElement(u8g2_t *u8g2, uint8_t index, int16_t value)
{
	stored_values[index] = value;

	u8g2_ClearBuffer(u8g2);
	u8g2_SetDrawColor(u8g2, 1); // White font color
	u8g2_SetFont(u8g2, u8g2_font_profont22_tf);

	/* Draw custom design here ----------------------------------------------*/
	u8g2_DrawFrame(u8g2, 0, 0, 256, 64);
	/*-----------------------------------------------------------------------*/

	// Draw all stored values
	for (uint8_t i = 0; i < GET_SIZE(stored_values); i++) {
		char str[10];
		sprintf(str, "%i", stored_values[i]);
		uint8_t pixel_width_string = u8g2_GetUTF8Width(u8g2, str);
		uint8_t x = getX(i, pixel_width_string);
		u8g2_DrawUTF8(u8g2, x, Y_PARAMATER_VALUE, str);
	}

	u8g2_SendBuffer(u8g2);
}

uint8_t getX(uint8_t index, uint8_t pixel_width_string)
{
	uint8_t x = 0;

	switch(index)
	{
	case 0: x = X_PARAMETER_CENTER_1_OF_4 - pixel_width_string/2; break;
	case 1: x = X_PARAMETER_CENTER_2_OF_4 - pixel_width_string/2; break;
	case 2: x = X_PARAMETER_CENTER_3_OF_4 - pixel_width_string/2; break;
	case 3: x = X_PARAMETER_CENTER_4_OF_4 - pixel_width_string/2; break;
	}

	return x;
}
