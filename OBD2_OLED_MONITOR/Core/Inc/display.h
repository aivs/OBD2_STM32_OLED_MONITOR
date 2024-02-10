/*
 * display.h
 *
 *  Created on: Feb 8, 2024
 *      Author: aivs
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

/* Includes ------------------------------------------------------------------*/
#include "u8g2.h"
#include "global_vars.h"

void display_UpdateElement(u8g2_t *u8g2, uint8_t index, int16_t value);


#endif /* INC_DISPLAY_H_ */
