/*
 * radar_communication.h
 *
 *  Created on: 2023. 10. 28.
 *      Author: grantLim
 */


#ifndef RADAR_FUNCTION_H_
#define RADAR_FUNCTION_H_

#include "stdbool.h"
#include "radar_communication_parameter.h"
#include "radar_communication_setting.h"
#include "radar_communication_errorcode.h"
#include "FreeRTOS.h"
#include "semphr.h"
/* 1ch Data Buffer Size : 1024*3-16, Frame Size : 3056 x 8 = 24448*/
/* DMA Buffer max Size 512*/

/* Define a Structure with bit field */
typedef	struct {
	uint8_t ping_recieve_done; //uart
	uint8_t radar_ready_flag;
	uint8_t ch_compare_hex;
	uint16_t radar_check_cnt;
	uint8_t fault;
} radar_values_t;

typedef union
{
	uint8_t R;
	struct
	{
		bool CH1 : 1;  // 1-bit field
		bool CH2 : 1;  // 1-bit field
		bool CH3 : 1;  // 1-bit field
		bool CH4 : 1;  // 1-bit field
		bool CH5 : 1;  // 1-bit field
		bool CH6 : 1;  // 1-bit field
		bool CH7 : 1;  // 1-bit field
		bool CH8 : 1;  // 1-bit field
	}A;
}ping_flags_t;

typedef union
{
	uint8_t R;
	struct
	{
		bool CH1 : 1;  // 1-bit field
		bool CH2 : 1;  // 1-bit field
		bool CH3 : 1;  // 1-bit field
		bool CH4 : 1;  // 1-bit field
		bool CH5 : 1;  // 1-bit field
		bool CH6 : 1;  // 1-bit field
		bool CH7 : 1;  // 1-bit field
		bool CH8 : 1;  // 1-bit field
	}A;
}check_flags_t;


/* Check Radar DTC */
typedef	struct {
	uint8_t left_BSD_status;
	uint8_t left_BSIS_B_status;
	uint8_t left_BSIS_F_status;
	uint8_t right_BSD_status;
	uint8_t right_BSIS_B_status;
	uint8_t right_BSIS_F_status;
	uint8_t current_radar_num;
	uint32_t radar_DM1_req;
} MRADAR_t;


extern uint8_t g_frame_buffer[RADAR_NUM][FRAME_BUF_SIZE];

extern SemaphoreHandle_t radar_to_algorithm_xSemaphore;
void radar_status_initialize(void);
extern void rc_radar_initialize(void);
extern void rc_channel_rxd_pin_check(void);
extern void rc_radar_rx_done_check(void);

extern radar_values_t *get_radar_value(void);
extern MRADAR_t *get_radar_status(void);
extern MRADAR_t get_radar_info(void);

extern uint32_t g_frame_incfail_cnt[6];
extern uint8_t (*get_radar_data(void))[FRAME_BUF_SIZE];
#endif /* RADAR_FUNCTION_H_ */
