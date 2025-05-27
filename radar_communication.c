/*
 * radar_communication.c
 *
 *  Created on: 2023. 8. 18.
 *      Author: grantLim
 */
#include "radar_communication.h"
#include "radar_communication_setting.h"
#include "vehicle_communication.h"
#include "diagnosis_errorcode.h"
#include "algorithm_setting.h"
#include "algorithm_parameter.h"

#include "string.h"
#include "serial_lld_cfg.h"
#include "siul_lld.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "pit_lld.h"
#include "pit_lld_cfg.h"
#include "spi_lld_cfg.h"
#include "spi_lld.h"
#include "pit_lld.h"
#include "pit_lld_cfg.h"

ping_flags_t g_pingflag;
check_flags_t g_checkflag;
radar_values_t radar_value;
MRADAR_t radar_info;

uint8_t g_frame_buffer[RADAR_NUM][FRAME_BUF_SIZE];
uint8_t g_radar_tx_buffer[RADAR_TX_BUF_SIZE] = {0, };

#ifdef CAN_TEST
uint8_t g_before_number[] = {0,0,0,0,0,0};
uint8_t g_curr_number[] = {0,0,0,0,0,0};
uint32_t g_frame_incfail_cnt[] = {0,0,0,0,0,0};
#endif

CANRxFrame g_rdBuf; // 74 * 15 = 2,368 Byte

uint8_t (*get_radar_data())[FRAME_BUF_SIZE]
{
	return g_frame_buffer;
}
radar_values_t *get_radar_value(void)
{
	return (&radar_value);
}
MRADAR_t *get_radar_status(void)
{
	return (&radar_info);
}

// DM1 사이클에 맞춰 레이더 상태 Update, 이 사이클 주기(1000ms) 안에 한번이라도 레이더 통신이 된다면 해당 레이더는 동작 중인 것으로 판단.
MRADAR_t get_radar_info(void)
{

	radar_info.radar_DM1_req = 0;
#ifdef SYSTEM_MODE_SGAS
	radar_info.current_radar_num = 6;

	if (radar_info.left_BSD_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_LETF_BSD_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.right_BSD_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_RIGHT_BSD_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.left_BSIS_B_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_LEFT_BSIS_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.left_BSIS_F_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_LEFT_BSIS_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.right_BSIS_B_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_RIGHT_BSIS_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.right_BSIS_F_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_RIGHT_BSIS_COMM_ERR);
		radar_info.current_radar_num--;
	}
#endif
#ifdef SYSTEM_MODE_BSD
	radar_info.current_radar_num = 2;

	if (radar_info.left_BSD_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_LETF_BSD_COMM_ERR);
		radar_info.current_radar_num--;
	}
	if (radar_info.right_BSD_status != RADAR_STATUS_RUN)
	{
		radar_info.radar_DM1_req |= (1<<DM1_RIGHT_BSD_COMM_ERR);
		radar_info.current_radar_num--;
	}
#endif
	radar_status_initialize();
	return radar_info;
}

void radar_status_initialize(void)
{
	radar_info.left_BSD_status = 0;
	radar_info.left_BSIS_B_status = 0;
	radar_info.left_BSIS_F_status = 0;
	radar_info.right_BSD_status = 0;
	radar_info.right_BSIS_B_status = 0;
	radar_info.right_BSIS_F_status = 0;
}

void rc_radar_initialize(void)
{//PDK_ECU
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	g_pingflag.R = false;
	g_checkflag.R = false;
	p_radar_value->ping_recieve_done = false;
	p_radar_value->radar_ready_flag = false;
	p_radar_value->radar_check_cnt = false;
	p_radar_value->ch_compare_hex = false;
	p_radar_value->fault = false;

	memset(&radar_info,0, sizeof(radar_info));
}

void Lcanfdconf_Lrxreceive(uint32_t msgbuf, CANRxFrame crfp)
{
//
////	// for test
////	uint8_t g_test_tx_buffer[32] = {0, };
////	uint32_t t_id = 0x600;
////	g_test_tx_buffer[0] = 0xFF;
////	g_test_tx_buffer[1] = 0xFF;
////	g_test_tx_buffer[2] = 0xFF;
////	g_test_tx_buffer[3] = 0xFF;
////	g_test_tx_buffer[4] = 0xFF;
////	g_test_tx_buffer[5] = 0xFF;
////	g_test_tx_buffer[6] = 0xFF;
////	g_test_tx_buffer[7] = 0xFF;
////	vc_tx_test_packet_t(t_id, g_test_tx_buffer);
//
//	radar_values_t *p_radar_value;
//	p_radar_value = get_radar_value();
//#ifdef CAN_TEST
//	uint32_t framenumber_temp;
//#endif
//
//    if (crfp.ID == BSD)
//    {
//    	radar_info.left_BSD_status = RADAR_STATUS_RUN;
//    	memcpy((void *)&g_frame_buffer[BSD_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//    	if(p_radar_value->radar_ready_flag == false)
//		{
//    		g_checkflag.A.CH3 = true;
//		}
//    	else
//    	{
//    		g_pingflag.A.CH3 = true;
//    	}
//#ifdef CAN_TEST
//    	memcpy((void *)&framenumber_temp, (void *)&crfp.data8[3], 4);
//    	g_curr_number[BSD_L] = swap4(framenumber_temp);
//    	if((g_before_number[BSD_L]+1) != g_curr_number[BSD_L])
//    	{
//    		g_frame_incfail_cnt[BSD_L]++;
//    	}
//    	g_before_number[BSD_L] = g_curr_number[BSD_L];
//#endif
//    }
//    if (crfp.ID == BSD_D1)
//    {
//    	memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//    }
//    if (crfp.ID == BSD_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == BSD_D3)
//   	{
//   		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//   	}
//    if (crfp.ID == BSD_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == BSD_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == BSD_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == BSD_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSD_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSD_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSD_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	if (crfp.ID == BSIS_B)
//	{
//		radar_info.left_BSIS_B_status = RADAR_STATUS_RUN;
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		if(p_radar_value->radar_ready_flag == false)
//		{
//			g_checkflag.A.CH2 = true;
//		}
//		else
//		{
//			g_pingflag.A.CH2 = true;
//		}
//	}
//	if (crfp.ID == BSIS_B_D1)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D3)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_B_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//	///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//	if (crfp.ID == BSIS_F)
//	{
//		radar_info.left_BSIS_F_status = RADAR_STATUS_RUN;
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		if(p_radar_value->radar_ready_flag == false)
//		{
//			g_checkflag.A.CH1 = true;
//		}
//		else
//		{
//			g_pingflag.A.CH1 = true;
//		}
//	}
//	if (crfp.ID == BSIS_F_D1)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D3)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == BSIS_F_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//
//	///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	////////////////////////////////////////////////////////
//	//////////// Radar One Cable Use Protocol //////////////
//	////////////////////////////////////////////////////////
//
//    if (crfp.ID == R_BSD)
//    {
//    	radar_info.right_BSD_status = RADAR_STATUS_RUN;
//    	memcpy((void *)&g_frame_buffer[BSD_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
//    	if(p_radar_value->radar_ready_flag == false)
//		{
//    		g_checkflag.A.CH3 = true;
//		}
//    	else
//    	{
//    		g_pingflag.A.CH3 = true;
//    	}
//#ifdef CAN_TEST
//    	memcpy((void *)&framenumber_temp, (void *)&crfp.data8[3], 4);
//    	g_curr_number[BSD_L] = swap4(framenumber_temp);
//    	if((g_before_number[BSD_L]+1) != g_curr_number[BSD_L])
//    	{
//    		g_frame_incfail_cnt[BSD_L]++;
//    	}
//    	g_before_number[BSD_L] = g_curr_number[BSD_L];
//#endif
//    }
//    if (crfp.ID == R_BSD_D1)
//    {
//    	memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//    }
//    if (crfp.ID == R_BSD_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == R_BSD_D3)
//   	{
//   		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//   	}
//    if (crfp.ID == R_BSD_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == R_BSD_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == R_BSD_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//    if (crfp.ID == R_BSD_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSD_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSD_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSD_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	if (crfp.ID == R_BSIS_B)
//	{
//		radar_info.right_BSIS_B_status = RADAR_STATUS_RUN;
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		if(p_radar_value->radar_ready_flag == false)
//		{
//			g_checkflag.A.CH2 = true;
//		}
//		else
//		{
//			g_pingflag.A.CH2 = true;
//		}
//	}
//	if (crfp.ID == R_BSIS_B_D1)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D3)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_B_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//	///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//	if (crfp.ID == R_BSIS_F)
//	{
//		radar_info.right_BSIS_F_status = RADAR_STATUS_RUN;
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		if(p_radar_value->radar_ready_flag == false)
//		{
//			g_checkflag.A.CH1 = true;
//		}
//		else
//		{
//			g_pingflag.A.CH1 = true;
//		}
//	}
//	if (crfp.ID == R_BSIS_F_D1)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D2)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D3)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D4)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D5)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D6)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D7)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D8)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D9)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//	if (crfp.ID == R_BSIS_F_D10)
//	{
//		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
//	}
//
//
//	///////////////////////////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////
//	//////////// Radar FW Update Protocol //////////////
//	////////////////////////////////////////////////////////
//
//	// g_radar_fw_update_flag change is for test.
//	if (crfp.ID == FW_Update_BSIS_F)
//	{
//		g_radar_fw_update_flag = 1;
//		memcpy((void *)&g_frame_buffer[BSIS_F_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		p_radar_value->radar_ready_flag = true;
//	}
//	if (crfp.ID == FW_Update_BSIS_R)
//	{
//		g_radar_fw_update_flag = 1;
//		memcpy((void *)&g_frame_buffer[BSIS_B_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		p_radar_value->radar_ready_flag = true;
//	}
//	if (crfp.ID == FW_Update_BSD)
//	{
//		g_radar_fw_update_flag = 1;
//		memcpy((void *)&g_frame_buffer[BSD_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
//		p_radar_value->radar_ready_flag = true;
//	}
//  (void)msgbuf;
}


void Rcanfdconf_Rrxreceive(uint32_t msgbuf, CANRxFrame crfp)
{
//	// for test
//	uint8_t g_test_tx_buffer[32] = {0, };
//	uint32_t t_id = 0x800;
//	g_test_tx_buffer[0] = (uint8_t)(crfp.ID >>24);
//	g_test_tx_buffer[1] = (uint8_t)(crfp.ID >>16);
//	g_test_tx_buffer[2] = (uint8_t)(crfp.ID >>8);
//	g_test_tx_buffer[3] = (uint8_t)(crfp.ID);
//	g_test_tx_buffer[4] = crfp.data8[0];
//	g_test_tx_buffer[5] = crfp.data8[1];
//	g_test_tx_buffer[6] = crfp.data8[2];
//	g_test_tx_buffer[7] = crfp.data8[3];
//	vc_tx_test_packet_t(t_id, g_test_tx_buffer);
//
//	t_id = 0x801;
//	g_test_tx_buffer[0] = crfp.data8[4];
//	g_test_tx_buffer[1] = crfp.data8[5];
//	g_test_tx_buffer[2] = crfp.data8[6];
//	g_test_tx_buffer[3] = crfp.data8[7];
//	g_test_tx_buffer[4] = crfp.data8[8];
//	g_test_tx_buffer[5] = crfp.data8[9];
//	g_test_tx_buffer[6] = crfp.data8[10];
//	g_test_tx_buffer[7] = crfp.data8[11];
//	vc_tx_test_packet_t(t_id, g_test_tx_buffer);

	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();
#ifdef CAN_TEST
	uint32_t framenumber_temp;
#endif

    if (crfp.ID == BSD)
    {
    	radar_info.left_BSD_status = RADAR_STATUS_RUN;
    	memcpy((void *)&g_frame_buffer[BSD_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
    	if(p_radar_value->radar_ready_flag == false)
		{
    		g_checkflag.A.CH3 = true;
		}
    	else
    	{
    		g_pingflag.A.CH3 = true;
    	}
#ifdef CAN_TEST
    	memcpy((void *)&framenumber_temp, (void *)&crfp.data8[3], 4);
    	g_curr_number[BSD_L] = swap4(framenumber_temp);
    	if((g_before_number[BSD_L]+1) != g_curr_number[BSD_L])
    	{
    		g_frame_incfail_cnt[BSD_L]++;
    	}
    	g_before_number[BSD_L] = g_curr_number[BSD_L];
#endif
    }
    if (crfp.ID == BSD_D1)
    {
    	memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
    }
    if (crfp.ID == BSD_D2)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == BSD_D3)
   	{
   		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
   	}
    if (crfp.ID == BSD_D4)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == BSD_D5)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == BSD_D6)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == BSD_D7)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSD_D8)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSD_D9)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSD_D10)
	{
		memcpy((void *)&g_frame_buffer[BSD_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////

    if (crfp.ID == R_BSD)
    {
    	radar_info.right_BSD_status = RADAR_STATUS_RUN;
    	memcpy((void *)&g_frame_buffer[BSD_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
    	if(p_radar_value->radar_ready_flag == false)
		{
    		g_checkflag.A.CH3 = true;
		}
    	else
    	{
    		g_pingflag.A.CH3 = true;
    	}
#ifdef CAN_TEST
    	memcpy((void *)&framenumber_temp, (void *)&crfp.data8[3], 4);
    	g_curr_number[BSD_L] = swap4(framenumber_temp);
    	if((g_before_number[BSD_L]+1) != g_curr_number[BSD_L])
    	{
    		g_frame_incfail_cnt[BSD_L]++;
    	}
    	g_before_number[BSD_L] = g_curr_number[BSD_L];
#endif
    }
    if (crfp.ID == R_BSD_D1)
    {
    	memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
    }
    if (crfp.ID == R_BSD_D2)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == R_BSD_D3)
   	{
   		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
   	}
    if (crfp.ID == R_BSD_D4)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == R_BSD_D5)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == R_BSD_D6)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
    if (crfp.ID == R_BSD_D7)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSD_D8)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSD_D9)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSD_D10)
	{
		memcpy((void *)&g_frame_buffer[BSD_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////
	//////////// Radar FW Update Protocol //////////////
	////////////////////////////////////////////////////////

	// g_radar_fw_update_flag change is for test.
	if (crfp.ID == FW_Update_BSIS_F)
	{
//		g_radar_fw_update_flag = 1;
		memcpy((void *)&g_frame_buffer[BSIS_F_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
		p_radar_value->radar_ready_flag = true;
	}
	if (crfp.ID == FW_Update_BSIS_R)
	{
//		g_radar_fw_update_flag = 1;
		memcpy((void *)&g_frame_buffer[BSIS_B_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
		p_radar_value->radar_ready_flag = true;
	}
	if (crfp.ID == FW_Update_BSD)
	{
//		g_radar_fw_update_flag = 1;
		memcpy((void *)&g_frame_buffer[BSD_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
		p_radar_value->radar_ready_flag = true;
	}







#ifdef SYSTEM_MODE_SGAS
	////////////////////////////////////////////////////////////////////////////////////////////////////


	if (crfp.ID == BSIS_B)
	{
		radar_info.left_BSIS_B_status = RADAR_STATUS_RUN;
		memcpy((void *)&g_frame_buffer[BSIS_B_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
		if(p_radar_value->radar_ready_flag == false)
		{
			g_checkflag.A.CH2 = true;
		}
		else
		{
			g_pingflag.A.CH2 = true;
		}
	}
	if (crfp.ID == BSIS_B_D1)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D2)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D3)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D4)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D5)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D6)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D7)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D8)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D9)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_B_D10)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////


	if (crfp.ID == BSIS_F)
	{
		radar_info.left_BSIS_F_status = RADAR_STATUS_RUN;
		memcpy((void *)&g_frame_buffer[BSIS_F_L][0], (void *)&crfp.data8[0], HEADER_SIZE);
		if(p_radar_value->radar_ready_flag == false)
		{
			g_checkflag.A.CH1 = true;
		}
		else
		{
			g_pingflag.A.CH1 = true;
		}
	}
	if (crfp.ID == BSIS_F_D1)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D2)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D3)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D4)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D5)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D6)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D7)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D8)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D9)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == BSIS_F_D10)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_L][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////


	if (crfp.ID == R_BSIS_B)
	{
		radar_info.right_BSIS_B_status = RADAR_STATUS_RUN;
		memcpy((void *)&g_frame_buffer[BSIS_B_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
		if(p_radar_value->radar_ready_flag == false)
		{
			g_checkflag.A.CH2 = true;
		}
		else
		{
			g_pingflag.A.CH2 = true;
		}
	}
	if (crfp.ID == R_BSIS_B_D1)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D2)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D3)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D4)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D5)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D6)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D7)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D8)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D9)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_B_D10)
	{
		memcpy((void *)&g_frame_buffer[BSIS_B_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////


	if (crfp.ID == R_BSIS_F)
	{
		radar_info.right_BSIS_F_status = RADAR_STATUS_RUN;
		memcpy((void *)&g_frame_buffer[BSIS_F_R][0], (void *)&crfp.data8[0], HEADER_SIZE);
		if(p_radar_value->radar_ready_flag == false)
		{
			g_checkflag.A.CH1 = true;
		}
		else
		{
			g_pingflag.A.CH1 = true;
		}
	}
	if (crfp.ID == R_BSIS_F_D1)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D2)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+TRACK_SIZE], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D3)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*2)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D4)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*3)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D5)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*4)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D6)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*5)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D7)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*6)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D8)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*7)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D9)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*8)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
	if (crfp.ID == R_BSIS_F_D10)
	{
		memcpy((void *)&g_frame_buffer[BSIS_F_R][TRACK_ADDRESS+(TRACK_SIZE*9)], (void *)&crfp.data8[0], TRACK_SIZE);
	}
#endif //SYSTEM_MODE_SGAS


  (void)msgbuf;
}


void rc_get_radar_buffer(void)
{
	vehicle_info_t *p_vehicle_info;
	p_vehicle_info = get_vehicle_info();

	g_radar_tx_buffer[0] = 0x20;	//magic
	g_radar_tx_buffer[1] = 0x50;	//magic
	g_radar_tx_buffer[2] = 0x46;	//magic
	g_radar_tx_buffer[3] = 0x97;	//magic
	g_radar_tx_buffer[4] = 0x00; // Mode Mode
	g_radar_tx_buffer[5] = 0x00; // Radar Number
	g_radar_tx_buffer[6] = 0xf0; // Warning indicate
	g_radar_tx_buffer[7] = (uint8_t)p_vehicle_info->speed; // Ego Speed
	g_radar_tx_buffer[8] = p_vehicle_info->ign; // IGN
	g_radar_tx_buffer[9] = p_vehicle_info->gear; // Gear
	g_radar_tx_buffer[10] = (uint8_t)p_vehicle_info->speed; // Speed
	g_radar_tx_buffer[11] = (uint8_t)(((p_vehicle_info->left_turn_signal & 0x1) << 4) | (p_vehicle_info->right_turn_signal & 0x1)); // Turn Signal L: 4bit R:0bit
	g_radar_tx_buffer[12] = p_vehicle_info->hazard; // Hazard
	g_radar_tx_buffer[13] |= (p_vehicle_info->driver_doors_switch & 0x1) << 4; // Door (DRV:4bit, AST:0bit)
	g_radar_tx_buffer[13] |= (p_vehicle_info->assist_doors_switch & 0x1); // Door (DRV:4bit, AST:0bit)
	g_radar_tx_buffer[14] = (p_vehicle_info->wheel & 0xFF00)>>8;	//Steer MSB
	g_radar_tx_buffer[15] = (p_vehicle_info->wheel & 0xFF);	//Steer LSB

	// RADAR Mode
	// No Mode 	0x00
	// BCW  	0x01
	// RCCW 	0x02
	if(p_vehicle_info->gear == CAN_GEAR_D)
	{
		g_radar_tx_buffer[4] = 0x01;
	}
	else if(p_vehicle_info->gear == CAN_GEAR_R)
	{
		g_radar_tx_buffer[4] = 0x02;
	}

}

void rc_give_radar_infomation(void)
{
	rc_get_radar_buffer();
	vc_tx_bsd_packet_t(g_radar_tx_buffer);
}

void rc_channel_rxd_pin_check(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();
	if(p_radar_value->radar_ready_flag == true)
	{
		if(g_pingflag.R == p_radar_value->ch_compare_hex)
		{
			g_pingflag.R = 0x00;
			p_radar_value->ping_recieve_done = true;
		}
		else{}
	}
}

void rc_radar_rx_done_check(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	if(g_radar_fw_update_flag != 1)
	{
		if(p_radar_value->radar_ready_flag == false)
		{
			if(p_radar_value->radar_check_cnt >= CH_SET_WATING_TIME)	//radar ch auto set : 250ms
			{
	#if AUTOCONFIG		/*Auto Set*/
				p_radar_value->ch_compare_hex = g_checkflag.R;
	#else				/*parameter Load*/
				if(BSD_REAR == 1)			p_radar_value->radar_ch_mask |= 0x30;
				else if(BSD_MIDDLE == 1)	p_radar_value->radar_ch_mask |= 0xCC;
				else						p_radar_value->fault = 1;
	#endif
				p_radar_value->radar_ready_flag = true;
				pit_lld_channel_start(&PITD1,RADAR_ALG_TIMER_CHANNEL);
			}
			else
			{
				p_radar_value->radar_check_cnt++;
			}
		}

		if(p_radar_value->ping_recieve_done == true)
		{
			p_radar_value->ping_recieve_done = false;
			rc_give_radar_infomation();
			xSemaphoreGive(radar_to_algorithm_xSemaphore);
		}
	}
	else
	{
		if (p_radar_value->radar_ready_flag == true)
		{
			p_radar_value->radar_ready_flag = false;
			xSemaphoreGive(radar_to_algorithm_xSemaphore);
		}
	}

}

void IRQ_timer_channel3_callback(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();
	//pal_lld_togglepad(PORT_LED_B_STATE, LED_B_STATE);
	p_radar_value->ping_recieve_done = true;
}
