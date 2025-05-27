/*
 * indicator_control.h
 *
 *  Created on: 2023. 11. 12.
 *      Author: pc
 */

#ifndef SOURCE_MODULE_INDICATOR_CONTROL_INDICATOR_CONTROL_H_
#define SOURCE_MODULE_INDICATOR_CONTROL_INDICATOR_CONTROL_H_

#include "main.h"





//void INDC_left_indicator_on(uint8_t onoff);
//void INDC_right_indicator_on(uint8_t onoff);
void INDC_left_indicator_on(uint8_t onoff, uint16_t head_ttc);
void INDC_right_indicator_on(uint8_t onoff, uint16_t head_ttc);
void INDC_left_indicator_blink(uint8_t onoff);
void INDC_right_indicator_blink(uint8_t onoff);
void INDC_indicator_check(uint8_t onoff);
void INDC_indicator_control_initialize(void);


#endif /* SOURCE_MODULE_INDICATOR_CONTROL_INDICATOR_CONTROL_H_ */
