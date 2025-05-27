#include "components.h"
#include "diagnosis.h"
#include "diagnosis_parameter.h"
#include "radar_communication.h"
#include "vehicle_communication.h"
#include "pas_communication.h"
#include "algorithm_setting.h"

#include "saradc_lld.h"
#include "saradc_lld_cfg.h"
#include "spi_lld_cfg.h"

//// pin control
//#include "siul_lld.h"
//#include "adc

/* set channels to convert */
uint8_t anp[NUMOFCHANNELS] = {24U,25U,26U,27U,39U,40U,41U};
uint16_t value[NUMOFCHANNELS] = {0U,0U,0U,0U,0U,0U,0U};
SARADCDriver* saradc_driver;
diag_mmwave_t dg_mmwave;
uint8_t g_reset_timer;
uint8_t g_radar_frameNo[8];

uint8_t g_radar_system_check_flag = 0;
uint8_t g_sgas_diag_state = SGAS_DIAG_LOADING;
uint32_t g_radar_system_DM1 = 0;
uint8_t diag_data[8] = {0xC3, 0xFF, 0, 0, 0, 0, 0xFF, 0xFF};


// SPN Code, 0x1388 == 5000
DM1_t DM1_Code[DM1_ERROR_MAX]=
{
		// PAS DTC Table
		{0x1388,0x9}, //DM1_LIN_COMM_ERR, PAS_MASTER_ERROR
		{0x138A,0xD}, //DM1_SENSOR_CAL_DATA_EMPTY
		{0x138D,0x5}, //DM1_SENSOR_OPEN_S1
		{0x138E,0x6}, //DM1_SENSOR_SHORT_S1
		{0x1390,0x5}, //DM1_SENSOR_OPEN_S2
		{0x1391,0x6}, //DM1_SENSOR_SHORT_S2
		{0x1393,0x5}, //DM1_SENSOR_OPEN_S3
		{0x1394,0x6}, //DM1_SENSOR_SHORT_S3

		// RADAR DTC Table
		{0x13BA,0x6}, //DM1_ECU_POWER_ERR
		{0x13BB,0x2}, //DM1_VCAN_COMM_ERR
		{0x13BC,0x6}, //DM1_LEFT_INDICATOR_ERR
		{0x13BD,0x6}, //DM1_RIGHT_INDICATOR_ERR
		{0x13BE,0x6}, //DM1_LEFT_BSD_POWER_ERR
		{0x13BE,0x9}, //DM1_LEFT_BSD_COMM_ERR
		{0x13BE,0x2}, //DM1_LEFT_BSD_LOC_ERR
		{0x13BF,0x6}, //DM1_RIGHT_BSD_POWER_ERR
		{0x13BF,0x9}, //DM1_RIGHT_BSD_COMM_ERR
		{0x13BF,0x2}, //DM1_RIGHT_BSD_LOC_ERR

#ifdef SYSTEM_MODE_SGAS
		{0x13C0,0x6}, //DM1_LEFT_BSIS_POWER_ERR
		{0x13C0,0x9}, //DM1_LEFT_BSIS_COMM_ERR
		{0x13C0,0x2}, //DM1_LEFT_BSIS_LOC_ERR

		{0x13C1,0x6}, //DM1_RIGHT_BSIS_POWER_ERR
		{0x13C1,0x9}, //DM1_RIGHT_BSIS_COMM_ERR
		{0x13C1,0x2}  //DM1_RIGHT_BSIS_LOC_ERR
#endif
};




uint8_t get_diag_state(void)
{
	uint8_t state = g_sgas_diag_state;
	return state;
}

void diag_initialize(void)
{
	dg_mmwave.loadsw_off = 0;
	dg_mmwave.loadsw_on = 0;
	dg_mmwave.sync_flag = 0;
	dg_mmwave.sync_reset_flag = 0;
	dg_mmwave.loadsw_off_flag = 0;
	dg_mmwave.reset_flag = 0;
	dg_mmwave.sync_ch_select = 0;
	g_reset_timer = 0;
}

/* Watchdog callback function.*/
void diag_callback_watchDog(SWTDriver *swtp)
{
    swt_lld_keep_alive(swtp);
}

void cli_reboot(void)
{
    printf("Reset...\r\n");    
    vTaskDelay(RESET_DELAY_TIME);
    MC_ME.MCTL.R = SPC5_ME_MCTL_MODE(0) | SPC5_ME_MCTL_KEY;
    MC_ME.MCTL.R = SPC5_ME_MCTL_MODE(0) | SPC5_ME_MCTL_KEY_INV;
}

/* conversion callback */
void saradcconf_conv_cb(SARADCDriver *saradcp)
{
    uint8_t i;
    /* Read converted channels */
    for (i = 0; i < NUMOFCHANNELS; i++)
    {
        value[i] = saradc_lld_readchannel(saradcp, anp[i]);    
    }
    saradc_lld_stop_conversion(saradcp);
    //pal_lld_togglepad(PORT_LED_G_STATE, LED_G_STATE);
}

void diag_init_initialize(void)
{
    saradc_driver = &SARADC12DSV;
    saradc_lld_start(saradc_driver, &saradc_config_saradcconf);
    //saradc_lld_start_conversion(saradc_driver);           
} 

void diag_read_of_adc_e_fuse(void)
{
    saradc_lld_start_conversion(saradc_driver);           
    //pal_lld_togglepad(PORT_LED_R_STATE, LED_R_STATE);      
}

void diag_mmwave_sync(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	if(dg_mmwave.sync_flag == 1)
	{
		if(dg_mmwave.loadsw_off_flag == 1)
		{
			if(g_reset_timer++ >= 4)
			{
				dg_mmwave.loadsw_off_flag = 0;
				dg_mmwave.loadsw_on = 1;
			}
			else{}
		}
		else dg_mmwave.loadsw_off = 1;
	}
	else{}

	if(dg_mmwave.loadsw_off == 1)
	{
		dg_mmwave.loadsw_off = 0;
		dg_mmwave.loadsw_off_flag = 1;
		pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
		rc_radar_initialize();
	}
	else if(dg_mmwave.loadsw_on == 1)
	{
		dg_mmwave.loadsw_on = 0;
		dg_mmwave.sync_flag = 0;
		g_reset_timer = 0;
		p_radar_value->radar_check_cnt = 0;
		//p_ping_data->R = 0x00;
		//p_pong_data->R = 0x00;
		pal_lld_setpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_setpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_setpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		pal_lld_setpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	else{}

	if(dg_mmwave.reset_flag == 1)	//Not use
	{
        pal_lld_writepad(PORT_RST_WR_M, RST_WR_M, mmWAVE_RESET_LOW);   //Set Reset Pin to low
        vTaskDelay(DELAY_FOR_RESET);
        pal_lld_writepad(PORT_RST_WR_M, RST_WR_M, mmWAVE_RESET_HIGH);   //Set Reset Pin to High
	}
	else{}
}

void diag_radar_reset(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	//Set Radar EN Pin to low
	pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
	pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
	pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
	pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	rc_radar_initialize();

	vTaskDelay(DELAY_FOR_RESET);

	// Set Radar EN Pin to High
	p_radar_value->radar_check_cnt = 0;
	pal_lld_setpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
	pal_lld_setpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
	pal_lld_setpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
	pal_lld_setpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
}

void diag_radar_off(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	//Set Radar EN Pin to low
	pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
	pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
	pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
	pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	rc_radar_initialize();
	vTaskDelay(DELAY_FOR_RESET);
}

void diag_radar_off_non_update(uint8_t radar_param)
{
//	uint8_t radar_param = radar_type - RADAR_DIFF_ARR_NUM;

	//Set Radar EN Pin to low
	if (radar_param == BSIS_F_L)
	{
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	else if (radar_param == BSIS_B_L)
	{
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	else if (radar_param == BSD_L)
	{
		pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	else if (radar_param == BSD_R)
	{
		pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
	}
	else if (radar_param == BSIS_B_R)
	{
		pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	else if (radar_param == BSIS_F_R)
	{
		pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
	}
	rc_radar_initialize();
}



void dg_radar_fault(void)
{
	radar_values_t *p_radar_value;
	p_radar_value = get_radar_value();

	/*Radar Frame Sync - Reset from Frame Number 2 00 00 over : 3h 41m*/

//	g_radar_frameNo[BSD_L] = g_frame_buffer[BSD_L][5];
//	g_radar_frameNo[BSIS_F_L] = g_frame_buffer[BSIS_F_L][5];
//	g_radar_frameNo[BSIS_B_L] = g_frame_buffer[BSIS_B_L][5];
//	g_radar_frameNo[BSD_R] = g_frame_buffer[BSD_R][5];
//	g_radar_frameNo[BSIS_B_R] = g_frame_buffer[BSIS_B_R][5];
//	g_radar_frameNo[BSIS_F_R] = g_frame_buffer[BSIS_F_R][5];
//	g_radar_frameNo[BSD_L] = g_frame_buffer[BSD_L][5];
//	g_radar_frameNo[BSD_L] = g_frame_buffer[BSD_L][5];


	if(dg_mmwave.sync_flag != 1)
	{
		g_radar_frameNo[dg_mmwave.sync_ch_select] = g_frame_buffer[dg_mmwave.sync_ch_select][4];
		if(g_radar_frameNo[dg_mmwave.sync_ch_select] == 2)
		{
			dg_mmwave.sync_reset_flag++;
		}
		dg_mmwave.sync_ch_select++;

		if(dg_mmwave.sync_ch_select >= mmWAVE_MAX)	dg_mmwave.sync_ch_select = 0;
	}

	if((dg_mmwave.sync_reset_flag >= 1) || (p_radar_value->fault == 1))
	{
		dg_mmwave.sync_reset_flag = 0;
		p_radar_value->fault = 0;
		dg_mmwave.sync_flag = 1;	//load switch reset 500ms counter
	}
	else{}

	/*Radar Frame_no Check*/

}

////////////////////////////////////////////////////////////////////////////////
//////////////  PAS DM1 참조
////////////////////////////////////////////////////////////////////////////////

uint8_t *diag_dm1_default(void)
{
	diag_data[0] = 0xC3;
	diag_data[1] = 0xFF;
	diag_data[2] = 0;
	diag_data[3] = 0;
	diag_data[4] = 0;
	diag_data[5] = 0;
	diag_data[6] = 0xFF;
	diag_data[7] = 0xFF;
    return &diag_data[0];
}

uint8_t *CAN_SEND_DM1(uint32_t DTC)
{
	uint32_t SPN;
	uint8_t FMI;

	SPN=DM1_Code[DTC].SPN;
	FMI=DM1_Code[DTC].FMI;

	if(SPN==0x1388 || SPN==0x1389)
	{
		diag_data[0]=0xD3; //red xx0100xx
	}
	else
	{
		diag_data[0]=0xC7; //amber xx0001xx
	}

	diag_data[1]=0xFF;
	diag_data[2]=(uint8_t)(SPN & 0xFF);
	diag_data[3]=(uint8_t)((SPN >> 8)& 0xFF);
	diag_data[4]=(uint8_t)(((SPN >> 16)& 0x7) << 5) | ( FMI & 0x1F);
	diag_data[5]=1; //OC
	diag_data[6]=0xFF;
	diag_data[7]=0xFF;
    return &diag_data[0];
}

void CAN_SEND_BAM(uint32_t DTC_req,uint8_t count)
{
	uint16_t total_byte;
	uint8_t total_packet,i,packet_count,poffset;
	uint8_t Send_data[DM1_ERROR_MAX*4+10];
	uint32_t SPN;
	uint8_t FMI;

	total_byte=count*4+2; // 4bye DTC(SPN+FMI+OC)*n + 2byte lamp state
	total_packet= total_byte/7;

	if(total_byte%7)
		total_packet+=1;

	packet_count=0;

	memset(Send_data,0xFF,DM1_ERROR_MAX*4+10);
	Send_data[0]=0xC7; //amber xx0001xx
	Send_data[1]=0xFF;
	poffset=2;

	for(i=0;i<DM1_ERROR_MAX;i++)
	{
		if(DTC_req & (1<<i))
		{
			SPN=DM1_Code[i].SPN;
			FMI=DM1_Code[i].FMI;

			if(SPN==0x1388 || SPN==0x1389)
			{
				Send_data[0]=0xD3; //red xx0100xx
			}

			Send_data[poffset]=(uint8_t)(SPN & 0xFF);
			poffset++;
			Send_data[poffset]=(uint8_t)((SPN >> 8)& 0xFF);
			poffset++;
			Send_data[poffset]=(uint8_t)(((SPN >> 16)& 0x7) << 5) | ( FMI & 0x1F);
			poffset++;
			Send_data[poffset]=1;
			poffset++;
			packet_count++;

		}
	}
	if(packet_count != count)
	{
		printf("DTC count is diffrent \n\r");
		return;
	}

//TPCM_BAM
	diag_data[0]=0x20; //control byte BAM:0x20
	diag_data[1]=(uint8_t)(total_byte & 0xFF); //total message size BAM Low
	diag_data[2]=(uint8_t)((total_byte>>8) & 0xFF); //total message size BAM High
	diag_data[3]=total_packet; // Total Number of packets(BAM)
	diag_data[4]=0xFF; //Maximum Number of Packets
	diag_data[5]=0xCA; //PGN Least byte
	diag_data[6]=0xFE; //PGN Middle byte
	diag_data[7]=0x00; //PGN Highest byte

	// CanTx_Buf[index].ExtId=0x18ECFF7A;
	vc_tx_dtc_packet_t(3, &diag_data[0]);

	// send packet

//TPDT
	packet_count=1;
	poffset=0;
	do
	{
		diag_data[0]=packet_count;
		for(i=1;i<8;i++)
		{
			diag_data[i]=Send_data[poffset];
			poffset++;
		}
		packet_count++;

		// send packet
//		CanTx_Buf[index].ExtId=0x18EBFF7A;
		vc_tx_dtc_packet_t(2, &diag_data[0]);

	}while(packet_count-1<total_packet);
}

// System check
// 레이더를 순차적으로 하나씩 켜보면서 확인.
uint32_t diag_system_check(uint32_t DM1_current, uint8_t current_radar_num)
{
	/////////////////////////////////////////////////////////
	// Radar System Check
	if(g_radar_system_check_flag == 0)
	{
		// Turn off all RADAR and Turn on BSD-L Radar
		diag_radar_off();
		g_radar_system_check_flag++;
	}
	else if(g_radar_system_check_flag == 1)
	{
		// Turn off all RADAR and Turn on BSD-L Radar
		pal_lld_setpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);
		g_radar_system_check_flag++;
	}
	else if(g_radar_system_check_flag == 2)
	{
		// BSD-L Radar Status Check
		if(DM1_current & (1<<DM1_LETF_BSD_COMM_ERR))
		{
			// Check other radar is connected
			if (current_radar_num == 1)
			{
				// other radar is connected
				g_radar_system_DM1 |= (1<<DM1_LEFT_BSD_LOCATION_ERR);
			}
			else
			{
				// radar is not connected
				g_radar_system_DM1 |= (1<<DM1_LEFT_BSD_POWER_ERR);
			}
		}

		// Turn off all RADAR and Turn on BSD-R Radar
		diag_radar_off();
		pal_lld_setpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);
		g_radar_system_check_flag++;
	}
	else if(g_radar_system_check_flag == 3)
	{
		// BSD-R Radar Status Check
		if(DM1_current & (1<<DM1_RIGHT_BSD_COMM_ERR))
		{
			// Check other radar is connected
			if (current_radar_num == 1)
			{
				// other radar is connected
				g_radar_system_DM1 |= (1<<DM1_RIGHT_BSD_LOCATION_ERR);
			}
			else
			{
				// radar is not connected
				g_radar_system_DM1 |= (1<<DM1_RIGHT_BSD_POWER_ERR);
			}
		}
		diag_radar_reset();
		g_radar_system_check_flag = 0xff;
#ifdef SYSTEM_MODE_SGAS
	// Turn off all RADAR and Turn on BSIS-L Radar
	diag_radar_off();
	pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);
	g_radar_system_check_flag = 3;

#endif
	}


#ifdef SYSTEM_MODE_SGAS
	else if(g_radar_system_check_flag == 3)
	{
		// BSIS-L Radar Status Check
		if(DM1_current & (1<<DM1_LEFT_BSIS_COMM_ERR))
		{
			// 현재 위치에 설치한 것이 다른 레이더 인지 확인.
			if (current_radar_num == 2)
			{
				// other radar is connected
				g_radar_system_DM1 |= (1<<DM1_LEFT_BSIS_LOCATION_ERR);
			}
			else
			{
				// radar is not connected
				g_radar_system_DM1 |= (1<<DM1_LEFT_BSIS_POWER_ERR);
			}
		}

		// Turn off all RADAR and Turn on BSIS-R Radar
		diag_radar_off();
		pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);
		g_radar_system_check_flag++;
	}
	else if(g_radar_system_check_flag == 4)
	{
		// BSIS-L Radar Status Check
		if(DM1_current & (1<<DM1_RIGHT_BSIS_COMM_ERR))
		{
			// 현재 위치에 설치한 것이 다른 레이더 인지 확인.
			if (current_radar_num == 2)
			{
				// other radar is connected
				g_radar_system_DM1 |= (1<<DM1_RIGHT_BSIS_LOCATION_ERR);
			}
			else
			{
				// radar is not connected
				g_radar_system_DM1 |= (1<<DM1_RIGHT_BSIS_POWER_ERR);
			}
		}
		diag_radar_reset();
		g_radar_system_check_flag = 0xff;
	}

#endif

	// return g_radar_system_DM1
	// turn on all radars
	else if(g_radar_system_check_flag == 0xff)
	{
		DM1_current |= g_radar_system_DM1;
		return DM1_current;
	}
	return 0;
}


void diag_communication(void)
{
	uint8_t i;
	uint8_t count_DM1;
	uint32_t DM1_current;
	uint8_t vehicle_info = get_vehicle_diag();
    MPAS_t *pPas_info = get_pas_info();
    MRADAR_t radar_info = get_radar_info();

	count_DM1=0;
	DM1_current = pPas_info->DM1_req;
	DM1_current |= radar_info.radar_DM1_req;

	if(vehicle_info == 0)
	{
		DM1_current |= (1<<DM1_VCAN_COMM_ERR);
	}

    ///////////////// Add ecu system standby check function need
	DM1_current = diag_system_check(DM1_current, radar_info.current_radar_num);
	if(DM1_current != 0)
	{
		for(i=0;i<DM1_ERROR_MAX;i++)
		{
			if(DM1_current & (1<<i))
			{
				count_DM1++;
			}
		}

//		// for test
//		uint8_t g_test_tx_buffer[32] = {0, };
//		uint32_t t_id = 0x84;
//		g_test_tx_buffer[0] = (uint8_t)(DM1_current >> 24);
//		g_test_tx_buffer[1] = (uint8_t)(DM1_current >> 16);
//		g_test_tx_buffer[2] = (uint8_t)(DM1_current >> 8);
//		g_test_tx_buffer[3] = (uint8_t)(DM1_current);
//		g_test_tx_buffer[4] = count_DM1;
//		g_test_tx_buffer[5] = 0xFF;
//		g_test_tx_buffer[6] = 0xFF;
//		g_test_tx_buffer[7] = 0xFF;
//		vc_tx_test_packet_t(t_id, g_test_tx_buffer);


		if(count_DM1>0)
		{
			g_sgas_diag_state = SGAS_DIAG_SYSTEM_ERROR;
			if(count_DM1==1)
			{
				for(i=0;i<DM1_ERROR_MAX;i++)
				{
					if(DM1_current & (1<<i))
					{
						// SEND_DM1
						vc_tx_dtc_packet_t(1, CAN_SEND_DM1(i));
						break;
					}
				}
			}
			else
			{
				// Send TPDT, BAM
				CAN_SEND_BAM(DM1_current,count_DM1);
			}
		}
		else
		{
			g_sgas_diag_state = SGAS_DIAG_SYSTEM_GOOD;
			// SEND_DM1_default
			vc_tx_dtc_packet_t(1, diag_dm1_default());
		}

		#if 0//def DM1_ON_REQUEST
			pas_info.DM1_req=0;
			pas_info.Send_DTC=0; //ksnoh 20210225

		#endif
	}
}






