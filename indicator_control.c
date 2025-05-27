/************************************************************************************
 * 	Headers																			*
 ************************************************************************************/
#include "indicator_control.h"
#include "indicator_control_setting.h"
#include "indicator_control_parameter.h"
#include "pit_lld.h"
#include "pit_lld_cfg.h"
#include "board.h"
#include "siul_lld.h"

//#include "vehicle_communication.h"

/************************************************************************************
 * 	Global Variables																*
 ************************************************************************************/


/************************************************************************************
 * 	Declare Local Functions															*
 ************************************************************************************/
uint8_t 	g_flag_left_blinking;
uint8_t 	g_flag_right_blinking;
uint8_t 	g_left_blink_count;
uint8_t 	g_right_blink_count;
uint32_t 	g_left_timer_count;
uint32_t 	g_right_timer_count;
uint8_t 	g_left_off_flag;
uint8_t 	g_right_off_flag;
uint16_t 	g_left_off_timer_count;
uint16_t 	g_right_off_timer_count;

#define		max_off_timer_count  30
#define 	min_off_timer_count  10
uint16_t 	g_vehicle_length = 10;
uint16_t	g_relative_left_off_timer_count = max_off_timer_count;
uint16_t	g_relative_right_off_timer_count = max_off_timer_count;
/************************************************************************************
 * 	 Functions																		*
 ************************************************************************************/

// relative_speed --> head_ttc
void INDC_left_indicator_on(uint8_t onoff, uint16_t head_ttc)
{
	if(g_flag_left_blinking == 0)
	{
		if(onoff == 1)
		{
			g_left_off_flag = 0;
			g_left_off_timer_count = 0;
			if(head_ttc != 200 && head_ttc < g_relative_left_off_timer_count)
			{
				if(head_ttc < min_off_timer_count) g_relative_left_off_timer_count = min_off_timer_count;
				else g_relative_left_off_timer_count = head_ttc;
			}
			pal_lld_setpad(PORT_LSW_INDL_EN, LSW_INDL_EN);
		}
		if(onoff == 0)
		{
			g_left_off_flag = 1;
			//pal_lld_clearpad(PORT_LSW_INDL_EN, LSW_INDL_EN);
		}
	}
	else{}
	return;
}

void INDC_right_indicator_on(uint8_t onoff, uint16_t head_ttc)
{
	if(g_flag_right_blinking == 0)
	{
		if(onoff == 1)
		{
			g_right_off_flag = 0;
			g_right_off_timer_count = 0;
			if(head_ttc != 200 && head_ttc < g_relative_right_off_timer_count)
			{
				if(head_ttc < min_off_timer_count) g_relative_right_off_timer_count = min_off_timer_count;
				else g_relative_right_off_timer_count = head_ttc;
			}
			pal_lld_setpad(PORT_LSW_INDR_EN, LSW_INDR_EN);
		}
		if(onoff == 0)
		{
			g_right_off_flag = 1;
			//pal_lld_clearpad(PORT_LSW_INDR_EN, LSW_INDR_EN);
		}
	}
	else{}
	return;
}

void INDC_left_indicator_blink(uint8_t onoff)
{
	if(onoff == 1)
	{
		if(g_flag_left_blinking == 0)
		{
			g_flag_left_blinking = 1;
			g_left_blink_count = 0;
			g_left_timer_count = 0;
			pit_lld_channel_start(&PITD1,INDC_LEFT_TIMER_CHANNEL);
		}
		else{}
	}
	else{}
	return;
}

void INDC_right_indicator_blink(uint8_t onoff)
{
	if(onoff == 1)
	{
		if(g_flag_right_blinking == 0)
		{
			g_flag_right_blinking = 1;
			g_right_blink_count = 0;
			g_right_timer_count = 0;
			pit_lld_channel_start(&PITD1,INDC_RIGHT_TIMER_CHANNEL);
		}
		else{}
	}
	else{}
	return;
}


void INDC_indicator_control_initialize(void)
{
	INDC_left_indicator_blink(1);
	INDC_right_indicator_blink(1);

	g_left_off_flag = 0;
	g_right_off_flag = 0;
	g_left_off_timer_count = 0;
	g_right_off_timer_count = 0;
	g_vehicle_length = 0; 			// get vehicle length need
	pit_lld_channel_start(&PITD1,INDC_OFF_TIMER_CHANNEL);
	return;
}


void IRQ_timer_channel1_callback(void)
{
	if(g_left_timer_count++ > INDC_BLINK_TIMER_COUNT_10MS)
	{
		if (g_left_blink_count < INDC_BLINK_COUNT*2)
		{
			pal_lld_togglepad(PORT_LSW_INDL_EN, LSW_INDL_EN);
			g_left_blink_count++;
		}
		else
		{
			pit_lld_channel_stop(&PITD1,INDC_LEFT_TIMER_CHANNEL);
			pal_lld_clearpad(PORT_LSW_INDL_EN, LSW_INDL_EN);
			g_flag_left_blinking = 0;
		}
		g_left_timer_count = 0;
	}
	else{}
}

void IRQ_timer_channel2_callback(void)
{
	if(g_right_timer_count++ > INDC_BLINK_TIMER_COUNT_10MS)
	{
		if (g_right_blink_count < INDC_BLINK_COUNT*2)
		{
			pal_lld_togglepad(PORT_LSW_INDR_EN, LSW_INDR_EN);
			g_right_blink_count++;
		}
		else
		{
			pit_lld_channel_stop(&PITD1,INDC_RIGHT_TIMER_CHANNEL);
			pal_lld_clearpad(PORT_LSW_INDR_EN, LSW_INDR_EN);
			g_flag_right_blinking = 0;
		}
		g_right_timer_count = 0;
	}
	else{}
}

void IRQ_timer_channel4_callback(void)
{
	if(g_left_off_flag == 1)
	{
		if(g_left_off_timer_count++ > g_relative_left_off_timer_count)
		{
			pal_lld_clearpad(PORT_LSW_INDL_EN, LSW_INDL_EN);
			g_left_off_timer_count = 0;
			g_left_off_flag = 0;
			g_relative_left_off_timer_count = max_off_timer_count;
		}
		else{}
	}
	else{}

	if(g_right_off_flag == 1)
	{
		if(g_right_off_timer_count++ > g_relative_right_off_timer_count)
		{
			pal_lld_clearpad(PORT_LSW_INDR_EN, LSW_INDR_EN);
			g_right_off_timer_count = 0;
			g_right_off_flag = 0;
			g_relative_right_off_timer_count = max_off_timer_count;
		}
		else{}
	}
	else{}
}
//void IRQ_timer_channel4_callback(void)
//{
//
//	if(g_left_off_flag == 1)
//	{
//		if(g_left_off_timer_count++ > INDC_OFF_TIMER_COUNT_2S)
//		{
//			pal_lld_clearpad(PORT_LSW_INDL_EN, LSW_INDL_EN);
//			g_left_off_timer_count = 0;
//			g_left_off_flag = 0;
//		}
//		else{}
//	}
//	else{}
//
//	if(g_right_off_flag == 1)
//	{
//		if(g_right_off_timer_count++ > INDC_OFF_TIMER_COUNT_2S)
//		{
//			pal_lld_clearpad(PORT_LSW_INDR_EN, LSW_INDR_EN);
//			g_right_off_timer_count = 0;
//			g_right_off_flag = 0;
//		}
//		else{}
//	}
//	else{}
//}
/************************************************************************************
 * 	Local Functions																	*
 ************************************************************************************/







