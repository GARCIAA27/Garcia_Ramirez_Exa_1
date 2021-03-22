/*
 * Select_Color.h
 *
 *  Created on: 22 mar. 2021
 *      Author: Alexis
 */

#ifndef SELECT_COLOR_H_
#define SELECT_COLOR_H_

#include "Bits.h"
#include "GPIO.h"
#include "NVIC.h"
#include "PIT.h"
#include <stdint.h>

void select_init();
void SM_init();
void white_blink();

#endif /* SELECT_COLOR_H_ */
