#include "components.h"
#include <FreeRTOS.h>
#include <task.h>
#include "can_lld.h"
#include "can_lld_cfg.h"
#include "PDK_RTOS_manager.h"
#include "vehicle_communication.h"
#include "saradc_lld_cfg.h"
#include "warning.h"
#include "diagnosis.h"
#include <string.h>

#include "pas_communication.h"
#include "radar_communication.h"
#include "diagnosis_parameter.h"
#include "flash_control.h"

// ECU FW Version
#undef RL_Isolation_Version
#define Port_Integration_Version

/*The vehicle_info_t structure variable holds vehicle related information to share other task. 
This is obatained through the'polling_receive_vehicle_info' function on can communication*/	

// System Global Parameters Initialize
uint8_t g_xbf_active_flag = 0; 			// XBR
uint8_t g_radar_fw_update_flag = 0;		// FW Update

// Get SGAS Switch Flag
// Default 0, On 1, Off 2
uint32_t g_sgas_avn_switch_flag = 0;		// AVN_CAN Data Switch

// Global Parameters Initialize
vehicle_info_t g_vehicle_info;
gear_t g_gear;
uint8_t g_broad_delay;
CANRxFrame rxBuf[MAX_CAN_BUFFER_NUMBER];

// FW Update Parameters
uint8_t g_radar_id = 0x05;
uint32_t g_radar_fw_data_uds_packet_counter = 0;
uint16_t g_radar_fw_data_sbl_packet_counter = 0;
uint8_t g_radar_image_tx_buffer[RADAR_TX_IMAGE_BUF_NUM][MAX_RADAR_IMAGE_BUF_SIZE];
uint8_t g_radar_image_tx_buf_num = 0;
uint8_t g_radar_image_tx_use_num = 0;
uint8_t g_radar_iamge_buffer[MAX_RADAR_IMAGE_BUF_SIZE] = {0, };
uint8_t g_radar_image_buffer_counter = 0;

// CWI1
uint8_t tx_CWI1_data[8];



/*The can_desc variable is initialized with a list of communication protocol information
 * This information is utilized to parse catask_command_interface_handle packets for each protocol.
 */	
/*The can_desc variable is initialized with a list of communication protocol information
 * This information is utilized to parse can packets for each protocol.
 */	

// for Test, HKT
uint8_t g_test_tx_buffer[32] = {0, };
uint32_t t_id = 0x700;

can_desc_t can_desc =
{
    4, /*total: The number of total protocl*/ 
    2, /*select: The protocol in currently using */
    {/*each protocol structure*/
        {/*ZIG TEST Protocol*/
			"zig",             /*Manufacturer*/ 
            "test",            /*Model*/ 
            2023,              /*Production year*/
            {500,5000},        /*baud rate : classic can, can fd*/
            {800,750},           /*Sample Point : classic can, can fd*/
            {
              /*{CAN ID , FLAG Number, Start Bit, Length}*/
//              {0x34, FLAG_IGN, 24,12},            /*0.IGN */
//				{0x36, FLAG_IGN, 24,12},
				{0x34, FLAG_FW_UPDATE_REQUSET, 4,12},
				{0x36, FLAG_FW_TRANSMIT, 4,12},
//                {0x201, FLAG_GEAR, 32,16},           /*1.GEAR*/
                {0x316, FLAG_SPEED, 16,16},          /*2.Speed*/
                {0x218A006, FLAG_HAZARD, 8,8},       /*3.Hazard*/
                {0x002, FLAG_WHEEL, 40,8},           /*4.Angle*/
                {0x423, FLAG_TURN_RIGHT, 8,8},       /*5.Turn Right*/
                {0x423, FLAG_TURN_LEFT, 0,8},        /*6.Turn Left*/
                {0x18FEBF0B ,FLAG_BRAKE, 8,8},       /*7.brake*/
				{0x218A006, FLAG_ASSIT_DOOR, 16,8},  /*8.Driver door switch*/
                {0x4394000, FLAG_DRIVER_DOOR, 0,8},  /*9.Assist door switch*/
                {0x18FEF100, FLAG_RPM, 16,8},        /*10.RPM*/

            }
        },
        {/*Santafe Protocol*/
			"hyudai","santafe",2023,{500,5000},{800,750},
            {
//                {0x02B0, FLAG_IGN, 24,1},           /*0.IGN */
//                {0x0367, FLAG_GEAR, 32,4},          /*1.GEAR*/
                {0x0366, FLAG_SPEED, 40,8},         /*2.Speed*/
                {0x0541, FLAG_HAZARD, 33,2},        /*3.Hazard*/
				{0x02B0, FLAG_WHEEL, 0,16},			/*4.Angle*/   
                {0x0541, FLAG_TURN_RIGHT, 62,1},    /*5.Turn Right*/
                {0x0541, FLAG_TURN_LEFT, 19,1},     /*6.Turn Left*/
                {0x0367, FLAG_BRAKE, 32,1},          /*7.brake*/
				{0x0541, FLAG_ASSIT_DOOR, 8,1},     /*8.Driver door switch*/
                {0x0541, FLAG_DRIVER_DOOR, 35,1},   /*9.Assist door switch*/
                {0x0000, FLAG_RPM, 0,0},            /*10.RPM*/ 
            }
        },
        {/*KUXEN Protocol
        *	Current TDCV Use Protocol
        */
			"tata","kuxen",2023,{500,5000},{875,750},
            {
				{0x34, FLAG_FW_UPDATE_REQUSET, 4,12},
				{0x36, FLAG_FW_TRANSMIT, 4,12},
                {0x18EF4A1F, FLAG_IGN, 5,2},        	/*0.IGN  SMK1*/
                {0x18F00503, FLAG_GEAR, 24,8},      	/*1.GEAR ETC2*/
                {0x18FEBF0B, FLAG_SPEED, 8,8},      	/*2.Speed EBC2*/
                {0xCFDCC27, FLAG_HAZARD, 12,1},     	/*3.Hazard OEL*/
				{0x18F0090B, FLAG_WHEEL, 0,16},			/*4.Angle ETC3*/
                {0xCFDCC27, FLAG_TURN_RIGHT, 9,1},  	/*5.Turn Right OEL*/
                {0xCFDCC27, FLAG_TURN_LEFT, 8,1},   	/*6.Turn Left OEL*/
                {0x0000, FLAG_BRAKE, 0,0},          /*7.brake*/
				{0x0000, FLAG_ASSIT_DOOR, 0,1},     /*8.Driver door switch*/
                {0x0000, FLAG_DRIVER_DOOR, 0,1},    /*9.Assist door switch*/
                {0x0000, FLAG_RPM, 0,0},            /*10.RPM*/ 
            }
        },
		{
			"kia","bongo3",2016,{500,5000},{800,750},
			{
//				{0x0260, FLAG_IGN, 29, 1},				/*0.IGN  CGW1*/
//				{0x043F, FLAG_GEAR, 0, 4},			/*1.GEAR EV_PC5*/
				{0x00A0, FLAG_SPEED,32,8},			/*2.Speed CLU15*/
				{0x02B0, FLAG_WHEEL, 0,16},			/*4.Angle*/
				{0x0000, FLAG_HAZARD, 33, 2},			/*3.Hazard CGW1*/
				{0x0000, FLAG_TURN_LEFT, 19, 2},		/*4.Turn Left CGW1*/
				{0x0000, FLAG_TURN_RIGHT, 62, 2},		/*5.Turn Right CGW1*/
				{0x0000, FLAG_ASSIT_DOOR, 35, 1},     /*6.Driver door switch CGW1*/
				{0x0000, FLAG_DRIVER_DOOR, 8, 2},     /*7.Assist door switch CGW1*/
				{0x0000, FLAG_BRAKE, 0,0},          /*8.brake*/
				{0x0000, FLAG_RPM, 0,0},            /*9.RPM*/
			}
		},
        // {
		// 	"tata","unknown",2022,1000,80,
		// 	{
		// 		{0x814C035,0x2,4,2}, 	  // VEHICLE_DART
		// 		{0x690,0x2,4,2},	 	  // VEHICLE_CARNIBAL
		// 		{0x18ffb11e,0x2,4,2},	  // VEHICLE_G420 
		// 		{0x3b3,0x2,4,2},     	  // VEHICLE_F150
		// 		{0x531,0x2,4,2},     	  // VEHICLE_VW
		// 		{0x006,0x2,4,2},     	  // VEHICLE_G_CHEROKEE
		// 		{0x10FF0121,0x2,4,2},     // VEHICLE_STRALIS
		// 		{0x02214000,0x2,4,2},     // VEHICLE_DUCAT
		// 		{0x10042040,0x2,4,2},     // VEHICLE_SAVANA
		// 		{0x0CFDCC27,0x2,4,2}      // VEHICLE_TATA
		// 	}
		// },
    }
};


can_list_t *get_can_list(void)
{
	can_list_t *pCan;
	pCan = &can_desc.can_list[can_desc.select];
	return (pCan);
}

can_desc_t *get_can_desc(void)
{
	can_desc_t *pCan;
	pCan = &can_desc;
	return (pCan);
}

vehicle_info_t *get_vehicle_info(void)
{   
    vehicle_info_t *pVehicle;
    pVehicle = &g_vehicle_info;
    return (pVehicle);
}

uint8_t get_vehicle_diag(void)
{
	uint8_t vehicle_comm_checker = g_gear.vehicle_comm_checker;
	g_gear.vehicle_comm_checker = 0;
	return vehicle_comm_checker;
}

void vc_can_initialize(void)
{
	can_desc_t *pCan;
    pCan = get_can_desc();
    g_radar_fw_update_flag = 0;
    g_gear.vehicle_comm_checker = 0;

    if(pCan->select == SANTAFE)
    {
    	g_gear.parking_number = 0;
    	g_gear.drive_number = 5;
    	g_gear.netural_number = 6;
    	g_gear.reverse_number = 7;
    }
    else if(pCan->select == KUXEN)
    {
    	g_gear.netural_number = 125;
    	g_gear.parking_number = 251;
    }
    else
    {
    	g_gear.parking_number = 0;
    	g_gear.drive_number = 1;
    	g_gear.netural_number = 0;
    	g_gear.reverse_number = 7;
    }
    g_broad_delay = 0;

    // Get SGAS Switch Flag
    // Default EEPROM Data is 0, if return data is 0, switch_flag set On Signal
    g_sgas_avn_switch_flag = get_SGAS_switch_flag();
    // for Test  Default Data Set to On Switch
    if(g_sgas_avn_switch_flag == 0) g_sgas_avn_switch_flag = 1;
}

uint16_t vc_can_lld_read_buf(CANDriver *canp, CANRxFrame *crfp)
{
	uint32_t msgbuf = 0;
	uint32_t count = 0;
	uint16_t num = 0;

    if(canp->mcan->IR.B.DRX == 1) 
    {
        for (count = 0; count < canp->config->num_of_rx_buffers; count++)
        {
            if (count < 32)
            {
                if (((canp->mcan->NDAT1.R) & (1UL << count)) != 0U)
                {
                    msgbuf = count;
                    can_lld_readBuffer(canp, CAN_DEDICATED_RXBUFFER, msgbuf, &crfp[num++]);
                }
                else{}
            } 
            else 
            {
                if (((canp->mcan->NDAT2.R) & (1UL << (count-32))) != 0U)
                {
                    msgbuf = count;
                    can_lld_readBuffer(canp, CAN_DEDICATED_RXBUFFER, msgbuf, &crfp[num++]);
                }
                else{}
            }

            if (msgbuf < 32) 
            {
                canp->mcan->NDAT1.R = (1UL << msgbuf);
            } 
            else 
            {
                canp->mcan->NDAT2.R = (1UL << (msgbuf-32));
            }
            msgbuf = 0;

            if(num > MAX_CAN_BUFFER_NUMBER -1)
            {
                num = 0;
            }
            else{}
        }
        /* read message */
        /* clear flag */
        canp->mcan->IR.B.DRX = 1;
    }
    else{}

    return num;
}

void vc_polling_get_vehicle_infomation(void)
{
    uint16_t tot_num=NULL; 
    uint8_t item_idx=NULL;
    uint8_t check_flag = FALSE;
    static uint16_t temp_vehicle_info[TOTAL_CAN_INDEX];
    static uint8_t temp_gear = 0;
    static uint8_t temp_TurnSig = 1;
//    CANRxFrame rxBuf[MAX_CAN_BUFFER_NUMBER]; // 74 * 15 = 2,368 Byte

    can_desc_t *pCan;
    pCan = get_can_desc(); //this function is used to retrieve the address of the can structure that is currently in use.

    tot_num = vc_can_lld_read_buf(&CAND1, rxBuf); //this function is used to retrieve the data from dedicated buffers having actived bit in New Data Register of CAN.
    for(int i=0; i<tot_num; i++)
    {

//    	////////////// For Test ///////////////////////////
//    	t_id = 0x700;
//    	g_test_tx_buffer[0] = rxBuf[i].data8[0];	//magic
//    	g_test_tx_buffer[1] = rxBuf[i].data8[1];	//magic
//    	g_test_tx_buffer[2] = rxBuf[i].data8[2];
//    	g_test_tx_buffer[3] = rxBuf[i].data8[3];
//    	g_test_tx_buffer[4] = (uint8_t)(rxBuf[i].SID >> 24); //
//    	g_test_tx_buffer[5] = (uint8_t)(rxBuf[i].SID >> 16);
//    	g_test_tx_buffer[6] = (uint8_t)(rxBuf[i].SID >> 8);
//    	g_test_tx_buffer[7] = (uint8_t)(rxBuf[i].SID); //
//		vc_tx_test_packet_t(t_id, g_test_tx_buffer);
//		////////////// For Test ///////////////////////////

        for(int j=0; j<MAX_NUM_CAN_ID; j++)
        {
            if(rxBuf[i].SID == pCan->can_list[pCan->select].can_id_info[j].id)
            {
                item_idx = pCan->can_list[pCan->select].can_id_info[j].item_idx;


                if(item_idx==FLAG_GEAR)
                {
                	g_gear.vehicle_comm_checker = 1;

                    temp_gear = extractbit(&rxBuf[i].data8[0], pCan->can_list[pCan->select].can_id_info[j].start_bit, pCan->can_list[pCan->select].can_id_info[j].len);
                    if(pCan->select == KUXEN)
                    {
						if(temp_gear == g_gear.netural_number)
						{
							temp_vehicle_info[item_idx] = GEAR_NETURAL;
						}
						else if(temp_gear == g_gear.parking_number)
						{
							temp_vehicle_info[item_idx] = GEAR_PARKING;
						}
						else if(temp_gear > g_gear.netural_number && temp_gear < g_gear.parking_number)
						{
							temp_vehicle_info[item_idx] = GEAR_DRIVE;
						}
						else if(temp_gear < g_gear.netural_number)
						{
							temp_vehicle_info[item_idx] = GEAR_REVERSE;
						}
						else{}
                    }
                    else
					{
					//if((pCan->select == SANTAFE) || (pCan->select == BONGO))
						if(temp_gear == g_gear.netural_number)
						{
							temp_vehicle_info[item_idx] = GEAR_NETURAL;
						}
						else if(temp_gear == g_gear.parking_number)
						{
							temp_vehicle_info[item_idx] = GEAR_PARKING;
						}
						else if(temp_gear == g_gear.drive_number)
						{
							temp_vehicle_info[item_idx] = GEAR_DRIVE;
						}
						else if(temp_gear == g_gear.reverse_number)
						{
							temp_vehicle_info[item_idx] = GEAR_REVERSE;
						}
						else{}
					}
                }
                else if(item_idx==FLAG_FW_UPDATE_REQUSET) // FW Update Request, Send to Radar Change SBL Mode Message
                {

                	// Left Right Port 분리 필요....


                	// g radar 는  확인 후 지정하는걸로 바꿔야함.
                	g_radar_id = rxBuf[i].data8[0];
                	uint8_t radar_id = g_radar_id - RADAR_DIFF_ARR_NUM;
                	uint8_t radar_check_flag = 0;

                	// 고장진단 확인필요
                	// 해당 레이더가 연결된 상태가 맞는지 확인하고 해당 레이더로 메시지 전송 혹은 에러 메시지 응답.
                	// 이 과정이 진행이 되지 않을 때 문제점은 fw update flag의 0으로 전환이 안되며, CAN Dev Tool에서 버튼이 풀리지 않게 됨.
                	// 결론적으로 CAN Dev Tool은 재실행해야 하며, Radar Data가 필요한 경우 ECU도 재실행 해야한다.
                	MRADAR_t *radar_status = get_radar_status();
                	if (radar_id == BSD_L)
                	{
                		if (radar_status->left_BSD_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}
                	else if (radar_id == BSIS_B_L)
                	{
                		if (radar_status->left_BSIS_B_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}
                	else if (radar_id == BSIS_F_L)
                	{
                		if (radar_status->left_BSIS_F_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}
                	else if (radar_id == BSD_R)
                	{
                		if (radar_status->right_BSD_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}
                	else if (radar_id == BSIS_B_R)
                	{
                		if (radar_status->right_BSIS_B_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}
                	else if (radar_id == BSIS_F_R)
                	{
                		if (radar_status->right_BSIS_F_status == RADAR_STATUS_RUN)
                			radar_check_flag = 1;
                	}


                	if (radar_check_flag == 1)
                	{
                		// Set FW update flag
                		g_radar_fw_update_flag = 1;

                    	// turn off other radars
//                    	diag_radar_off_non_update(radar_id);

                    	// Send to Radar Change mode Message
                    	rc_tx_radar_setup_mode_t();
                    	rc_tx_fw_image_flash_request_t();
                	}
                	else
                	{
//                		 send error message
//						////////////// For Test ///////////////////////////
//						t_id = 0x701;
//						g_test_tx_buffer[0] = rxBuf[i].data8[0];	//magic
//						g_test_tx_buffer[1] = rxBuf[i].data8[1];	//magic
//						g_test_tx_buffer[2] = rxBuf[i].data8[2];
//						g_test_tx_buffer[3] = rxBuf[i].data8[3];
//						g_test_tx_buffer[4] = (uint8_t)(rxBuf[i].SID >> 24); //
//						g_test_tx_buffer[5] = (uint8_t)(rxBuf[i].SID >> 16);
//						g_test_tx_buffer[6] = (uint8_t)(rxBuf[i].SID >> 8);
//						g_test_tx_buffer[7] = (uint8_t)(rxBuf[i].SID); //
//						vc_tx_test_packet_t(t_id, g_test_tx_buffer);
//						////////////// For Test ///////////////////////////
                	}

//					////////////// For Test ///////////////////////////
//					t_id = 0x702;
//					g_test_tx_buffer[0] = rxBuf[i].data8[0];					// radar num
//					g_test_tx_buffer[1] = (uint8_t)(radar_status->left_BSD_status);
//					g_test_tx_buffer[2] = (uint8_t)(radar_status->left_BSIS_B_status);
//					g_test_tx_buffer[3] = (uint8_t)(radar_status->left_BSIS_F_status);
//					g_test_tx_buffer[4] = (uint8_t)(radar_status->right_BSD_status);
//					g_test_tx_buffer[5] = (uint8_t)(radar_status->right_BSIS_B_status);
//					g_test_tx_buffer[6] = (uint8_t)(radar_status->right_BSIS_F_status);
//					g_test_tx_buffer[7] = (uint8_t)(rxBuf[i].SID);
//					vc_tx_test_packet_t(t_id, g_test_tx_buffer);
//					////////////// For Test ///////////////////////////

                }
                else if(item_idx==FLAG_FW_TRANSMIT) // FW Update Data Transmit
                {
                	if (rxBuf[i].data8[0] == g_radar_id)
                	{
                    	uint32_t current_counter = (uint32_t)(rxBuf[i].data8[1] << 16) + (uint32_t)(rxBuf[i].data8[2] << 8) + (uint32_t)rxBuf[i].data8[3];
                    	uint8_t current_buf_count = g_radar_image_buffer_counter * 4;
                    	// The First Packet, Start Radar Data Send Thread
                    	if(g_radar_fw_data_uds_packet_counter == 0x00)
                    	{
//                    		g_radar_fw_update_flag = 1;
                    		create_task_radar_downloader();
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[4];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[5];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[6];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[7];
                    		g_radar_image_buffer_counter++;
                    		g_radar_fw_data_uds_packet_counter++;
                    	}
                    	// Collect and send data
                    	else if(g_radar_fw_data_uds_packet_counter == current_counter)
                    	{
                    		// 52개씩 모아서 전달
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[4];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[5];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[6];
                    		g_radar_iamge_buffer[current_buf_count++] = rxBuf[i].data8[7];
                    		g_radar_image_buffer_counter++;
                    		g_radar_fw_data_uds_packet_counter++;

                    		if (g_radar_image_buffer_counter == 13) // 13 == (52 / 4) : MAX_RADAR_IMAGE_BUF_SIZE / data num
                    		{
                    			set_radar_image_data(g_radar_iamge_buffer);
                    			g_radar_image_buffer_counter = 0;
                    		}
                    	}
                    	// Check Lask Packet
                    	else if(rxBuf[i].data8[1] == 0xFF)
                    	{
                    		memset(&g_radar_iamge_buffer[current_buf_count], 0xFF, sizeof(g_radar_iamge_buffer) - current_buf_count);
                    		set_radar_image_data(g_radar_iamge_buffer);
                    		g_radar_image_buffer_counter = 0xFF;
                    	}
                	}
                	// ACK Message 전달.
                	t_id = 0x36;
					g_test_tx_buffer[0] = g_radar_image_buffer_counter; //
					g_test_tx_buffer[1] = (uint8_t)(g_radar_fw_data_uds_packet_counter >> 16);
					g_test_tx_buffer[2] = (uint8_t)(g_radar_fw_data_uds_packet_counter >> 8);
					g_test_tx_buffer[3] = (uint8_t)(g_radar_fw_data_uds_packet_counter); //
					g_test_tx_buffer[4] = (uint8_t)(g_radar_fw_data_sbl_packet_counter >> 8);
					g_test_tx_buffer[5] = (uint8_t)(g_radar_fw_data_sbl_packet_counter);
					g_test_tx_buffer[6] = g_radar_image_tx_use_num; //
					g_test_tx_buffer[7] = g_radar_image_tx_buf_num; //
                	vc_tx_test_packet_t(t_id, g_test_tx_buffer);
                }
                else
                {
                    temp_vehicle_info[item_idx] = extractbit(&rxBuf[i].data8[0], pCan->can_list[pCan->select].can_id_info[j].start_bit, pCan->can_list[pCan->select].can_id_info[j].len);
                }
                check_flag = TRUE;
                //printf("ID:%04x, buf num:%d, vinfo[item_idx]:%04x, idx:%d start bit:%d len:%d\r\n",rxBuf[i].SID,i,vinfo[item_idx],item_idx,pCan->can_id_info[j].start_bit, pCan->can_id_info[j].len);
            }
            else{}
        }   //End for(int j=0; j<MAX_NUM_CAN_ID; j++)

    }//End for(int i=0; i<tot_num; i++)

    if(check_flag==TRUE)
//    if(tot_num > 0)
    {
        pal_lld_togglepad(PORT_LED_B_STATE, LED_B_STATE);
        memcpy((void *)&g_vehicle_info, (void *)&temp_vehicle_info[0], sizeof(vehicle_info_t));

        /*vehicle does not support the CAN protocol.*/
        if(pCan->select == BONGO)
        {
        	temp_TurnSig = pal_lld_readpad(PORT_VHE_SIG_LEFT, VHE_SIG_LEFT);
        	if(temp_TurnSig == 0)    	g_vehicle_info.left_turn_signal = true;
        	else g_vehicle_info.left_turn_signal = false;

        	temp_TurnSig = pal_lld_readpad(PORT_VHE_SIG_RIGHT, VHE_SIG_RIGHT);
        	if(temp_TurnSig == 0)	g_vehicle_info.right_turn_signal = true;
        	else g_vehicle_info.right_turn_signal = false;
        }
        else{}
    }
    else{}
}

//=========================
// Data processing
//=========================
/*the rvs_bits function reverses a 64-bit stream*/
uint64_t rvs_bits(uint64_t val) 
{
    uint64_t result = 0;
    int num_bits = sizeof(uint64_t) * 8;

    for (int i = 0; i < num_bits ; i++)
    {
        result <<= 1;                 
        result |= (val & 0x01); 
        val >>= 1;  
    }
    return result;
}

/*the swap2 is to swap for 2byte data*/
uint16_t swap2(uint16_t sval_in)
{
    uint8_t *cp_in, *cp_out;
    uint16_t sval_out;

    cp_in = (uint8_t *)&sval_in;
    cp_out = (uint8_t *)&sval_out;
    cp_out[0] = cp_in[1];
    cp_out[1] = cp_in[0];
    return (sval_out);
}

/*the swap4 is to swap for 4byte data*/
uint32_t swap4(uint32_t sval_in)
{
    uint8_t *cp_in, *cp_out;
    uint32_t sval_out;

    cp_in = (uint8_t *)&sval_in;
    cp_out = (uint8_t *)&sval_out;

    cp_out[0] = cp_in[3];
    cp_out[1] = cp_in[2];
    cp_out[2] = cp_in[1];
    cp_out[3] = cp_in[0];

    return (sval_out);
}

/*The insertbit is to parse rx data for 8 byte*/
uint16_t extractbit(uint8_t* data, int from, int size)
{
    uint8_t tmp[4] ={0,};
    uint8_t len = 0;
    uint16_t result = 0;

	if((from + size) > 64) 
	{
		return 0;
	}

    if(size % 8 > 0)
    {
        len = (size / 8) + 1;
    }
    else
    {
        len = (size / 8);
    }

    memcpy(&tmp[0], &data[from / 8], len);  
    for (int i=0; i<len; i++)
    {
        result |= tmp[i] << i*8; 
    }
    result = result >> (from % 8);
    result = result & ((1 << size) -1);

    //result = (data[from / 8] >> (from % 8)) & ((1 << size) -1);
    return result;
}
/*The insertbit is to encode tx data for 8 byte*/
uint8_t insertbit(uint8_t* data, int from, int size)
{
	if((from + size) > 64)
	{
		return 0;
	}	
	return (data[from / 8] >> (from % 8)) & ((1 << size) -1);
}


/* tx_bsd_packet: This transfers tx packet based on protocol*/
uint32_t vc_tx_bsd_packet_t(uint8_t *tx_data)
{
	CANTxFrame txf;
	uint32_t  txReturn = 0;
        
	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_STD; //CAN_ID_STD : CAN Normal ID, CAN_ID_XTD: CAN Extended ID
	txf.DLC = 64U;
	txf.ID = 0x100; //;
	txf.OPERATION = CAN_OP_CANFD; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	txf.data8[0] = 0x74;
	txf.data8[1] = 0x83;
	txf.data8[2] = 0x31;
	txf.data8[3] = 0x83;
	txf.data8[4] = 0x00;  	// Radar ID
	txf.data8[5] = 0x01;  	// RADAR ID : ALL
	txf.data8[6] = 0x04;  	// Message Type : Vehicle Info
	txf.data8[7] = 0xFF;  	// Header Padding

	txf.data8[8] = tx_data[0];
	txf.data8[9] = tx_data[1];
	txf.data8[10] = tx_data[2];
	txf.data8[11] = tx_data[3];
	txf.data8[12] = tx_data[4];
	txf.data8[13] = tx_data[5];
	txf.data8[14] = tx_data[6];
	txf.data8[15] = tx_data[7];
	txf.data8[16] = tx_data[8];
	txf.data8[17] = tx_data[9];
	txf.data8[18] = tx_data[10];
	txf.data8[19] = tx_data[11];
	txf.data8[20] = tx_data[12];
	txf.data8[21] = tx_data[13];
	txf.data8[22] = tx_data[14];
	txf.data8[23] = tx_data[15];

	//pal_lld_togglepad(PORT_LED_R_STATE, LED_R_STATE);

//	txReturn = can_lld_transmit(&CAND7, CAN_DEDICATED_TXBUFFER, &txf);//test
	txReturn = can_lld_transmit(&CAND9, CAN_DEDICATED_TXBUFFER, &txf);//test
	return txReturn;
}

//////////////////////////////////////////////////////////////
////////   Send To RADAR FW Update (Image Data)   ////////////
//////////////////////////////////////////////////////////////

void rc_FW_Update_Send(void)
{
    if (g_radar_image_tx_use_num != g_radar_image_tx_buf_num)
    {
        send_radar_image(g_radar_image_tx_use_num, DATAFLASH_REQUEST);
    }
    else if (g_radar_image_buffer_counter == 0xFF)
    {
    	g_radar_fw_data_sbl_packet_counter = 0xFFFF;
        send_radar_image(g_radar_image_tx_use_num, PADDING_REQUEST);
    }
}

void send_radar_image(uint8_t buffer_num, uint8_t packet_type)
{
    uint8_t radar_image_tx_buffer[MAX_RADAR_IMAGE_BUF_SIZE];
    uint8_t (*data_ptr)[MAX_RADAR_IMAGE_BUF_SIZE] = get_radar_image_data(buffer_num);

    memcpy(radar_image_tx_buffer, *data_ptr, MAX_RADAR_IMAGE_BUF_SIZE);
    rc_tx_fw_image_packet_t(radar_image_tx_buffer, packet_type);
}

void set_radar_image_data(uint8_t *radar_image)
{
	memcpy(&g_radar_image_tx_buffer[g_radar_image_tx_buf_num][0], radar_image, MAX_RADAR_IMAGE_BUF_SIZE);
	g_radar_image_tx_buf_num = (g_radar_image_tx_buf_num + 1) % RADAR_TX_IMAGE_BUF_NUM;
}

uint8_t (*get_radar_image_data(uint8_t radar_image_buf_num))[MAX_RADAR_IMAGE_BUF_SIZE]
{
    return &g_radar_image_tx_buffer[radar_image_buf_num];
}

// Request Radar Mode Change
// radar operation mode -> setup mode
uint32_t rc_tx_radar_setup_mode_t(void)
{
	CANTxFrame txf;
	uint32_t  txReturn = 0;

	memset(&txf, 0, sizeof(CANTxFrame));

	// send hello, change radar setup mode
	txf.TYPE = CAN_ID_STD; //CAN_ID_STD : CAN Normal ID, CAN_ID_XTD: CAN Extended ID
	txf.DLC = 64U;
	txf.ID = 0x100; // ECU command to All / Send Vehicle Info;
	txf.OPERATION = CAN_OP_CANFD; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	txf.data8[0] = 0x74;
	txf.data8[1] = 0x83;
	txf.data8[2] = 0x31;
	txf.data8[3] = 0x83;
	txf.data8[4] = 0x00;  	// Radar ID
	txf.data8[5] = 0x01;  	// RADAR ID : ALL
	txf.data8[6] = 0x01;  	// Message Type
	txf.data8[7] = 0xFF;  	// Header Padding

	txf.data8[8] = 0x01;	// CMD (4byte) : Radar Enter Setup Mode
	txf.data8[9] = 0x23;
	txf.data8[10] = 0x76;
	txf.data8[11] = 0x54;
	txf.data8[12] = 0x0A;	// PayLoad : \n

//	txReturn = can_lld_transmit(&CAND7, CAN_DEDICATED_TXBUFFER, &txf);//test
	txReturn = can_lld_transmit(&CAND9, CAN_DEDICATED_TXBUFFER, &txf);//test

	// time delay to eeprom write, 필요 없을지도.
	vTaskDelay(1000);
	return txReturn;
}

// EEPROM Write, FW Flash Flag Setup
// Entering SBL Mode and Flash Second FW
uint32_t rc_tx_fw_image_flash_request_t(void)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	uint8_t radar_id = g_radar_id;

	memset(&txf, 0, sizeof(CANTxFrame));

#ifdef RL_Isolation_Version
	// L, R Port Isolation Version
	if (radar_id > 4)
		radar_id -= 3;
#endif

	// send hello, change radar setup mode
	txf.TYPE = CAN_ID_STD; //CAN_ID_STD : CAN Normal ID, CAN_ID_XTD: CAN Extended ID
	txf.DLC = 64U;
	txf.ID = 0x100; // ECU command to All / Send Vehicle Info;
	txf.OPERATION = CAN_OP_CANFD; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	// send set eeprom to flash fw image
	txf.data8[0] = 0x74;
	txf.data8[1] = 0x83;
	txf.data8[2] = 0x31;
	txf.data8[3] = 0x83;
	txf.data8[4] = 0x00;  			// Radar ID
	txf.data8[5] = radar_id;  		// Radar ID
	txf.data8[6] = 0x01;  			// Message Type
	txf.data8[7] = 0xFF;  			// Header Padding

	txf.data8[8] = 0x01;			// CMD (4byte) : EEPROM Write
	txf.data8[9] = 0x01;
	txf.data8[10] = 0x00;
	txf.data8[11] = 0xFF;
	txf.data8[12] = 0x01;			// PayLoad : addr ( 2byte )
	txf.data8[13] = 0x80;
	txf.data8[14] = 0x01;			// PayLoad : value ( 1byte)

//	txReturn = can_lld_transmit(&CAND7, CAN_DEDICATED_TXBUFFER, &txf);//test
	txReturn = can_lld_transmit(&CAND9, CAN_DEDICATED_TXBUFFER, &txf);//test
	return txReturn;
}

uint32_t rc_tx_fw_image_packet_t(uint8_t *radar_image, uint8_t message_type)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	uint8_t radar_id = g_radar_id;

	memset(&txf, 0, sizeof(CANTxFrame));

#ifdef RL_Isolation_Version
	// L, R Port Isolation Version
	if (radar_id > 4)
		radar_id -= 3;
#endif

	txf.TYPE = CAN_ID_STD; //CAN_ID_STD;
	txf.DLC = 64U;
	// test for change id to 110
	txf.ID = 0x110; // 0x111 BSIS-F response - Operation, SBL;
	txf.OPERATION = CAN_OP_CANFD; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	txf.data8[0] = 0x4D;			// STX : MVRT (ASCII)
	txf.data8[1] = 0x56;
	txf.data8[2] = 0x52;
	txf.data8[3] = 0x54;
	txf.data8[4] = 0x00;  			// Radar ID
	txf.data8[5] = radar_id;  	// Radar ID
	txf.data8[6] = 0xFF;
	txf.data8[7] = 0xFF;

	txf.data8[8] = 0x03;			// Message Type, FW Data Transmit
	txf.data8[9] = message_type;	// FF : FW Data, FA : Data Padding
	txf.data8[10] = (uint8_t)(g_radar_fw_data_sbl_packet_counter >> 8); 	// Data Count
	txf.data8[11] = (uint8_t)(g_radar_fw_data_sbl_packet_counter); 			// Data Count
	memcpy(&txf.data8[12], radar_image, MAX_RADAR_IMAGE_BUF_SIZE);

//	txReturn = can_lld_transmit(&CAND7, CAN_DEDICATED_TXBUFFER, &txf);//test
	txReturn = can_lld_transmit(&CAND9, CAN_DEDICATED_TXBUFFER, &txf);//test

	return txReturn;
}

//////////////////////////////////////////////////////////////
////////   Send To Vehicle CAN Test Message   ////////////////
//////////////////////////////////////////////////////////////
uint32_t vc_tx_test_packet_t(uint32_t ID, uint8_t *tx_data)
{
	CANTxFrame txf;
	uint32_t  txReturn;

	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = ID; // TEST ID;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	txf.data8[0] = tx_data[0];
	txf.data8[1] = tx_data[1];
	txf.data8[2] = tx_data[2];
	txf.data8[3] = tx_data[3];
	txf.data8[4] = tx_data[4];
	txf.data8[5] = tx_data[5];
	txf.data8[6] = tx_data[6];
	txf.data8[7] = tx_data[7];

	//pal_lld_togglepad(PORT_LED_B_STATE, LED_B_STATE);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);

	return txReturn;
}


void vc_tx_update_response_packet_t(uint8_t *update_response_msg)
{
//	t_id = 0x99;
//	g_test_tx_buffer[0] = update_response_msg[0];	// Radar Type
//	g_test_tx_buffer[1] = update_response_msg[1];	// Ack
//	g_test_tx_buffer[2] = update_response_msg[2];
//	g_test_tx_buffer[3] = update_response_msg[3];
//	g_test_tx_buffer[4] = update_response_msg[4];
//	g_test_tx_buffer[5] = update_response_msg[5];
//	g_test_tx_buffer[6] = update_response_msg[6];
//	g_test_tx_buffer[7] = update_response_msg[7];
//	vc_tx_test_packet_t(t_id, g_test_tx_buffer);

	// RADAR FW Flash Mode Change, Recv Ack Message
	if ( update_response_msg[1] == 0x01)	// Change Mode Ack
	{
		// reset radar, to enter SBL Mode
		diag_radar_reset();

    	// turn off other radars, FW B'd 바뀌고 pin이 맞는지 확인 필요.
    	diag_radar_off_non_update((uint8_t)(g_radar_id - RADAR_DIFF_ARR_NUM));
	}
	if ( update_response_msg[1] == 0x02)	// SBL Boot Ack
	{
		// set radars setup mode, To don't send radar data
		rc_tx_radar_setup_mode_t();

		// Packet Define Need, UDS
		// ACK Message, about SBL Book Finish
		t_id = 0x34;
		g_test_tx_buffer[0] = 0x00;	// Radar Type
		g_test_tx_buffer[1] = 0x01;	// Ack
		g_test_tx_buffer[2] = 0x00;
		g_test_tx_buffer[3] = 0x00;
		g_test_tx_buffer[4] = 0x00;
		g_test_tx_buffer[5] = 0x00;
		g_test_tx_buffer[6] = 0x00;
		g_test_tx_buffer[7] = 0x00;
		vc_tx_test_packet_t(t_id, g_test_tx_buffer);
	}
	if ( update_response_msg[1] == 0x03)	// FW Data Transmit Ack
	{
		uint16_t sbl_count = (uint16_t)(update_response_msg[3] << 8) + (uint16_t)(update_response_msg[4]);
		if ( g_radar_fw_data_sbl_packet_counter + 1 == sbl_count)
		{
			g_radar_fw_data_sbl_packet_counter += 1;
			g_radar_image_tx_use_num = g_radar_fw_data_sbl_packet_counter % RADAR_TX_IMAGE_BUF_NUM;
		}
		else if (0xFFFF == sbl_count)
		{
			// Reset 했는데 업데이트한 레이더만 켜짐, Clear도 잘 안되는듯, 왜지??  아래 두줄이 실행되지 않은것 같은데 그건 말이안되긴함. ㅅㅂ
			// end
			set_clear_fwupdate();
			diag_radar_reset();

			// Ack Message
			t_id = 0x36;
			g_radar_fw_data_uds_packet_counter = 0xFFFFFFFF;
			g_test_tx_buffer[0] = g_radar_image_buffer_counter; //
			g_test_tx_buffer[1] = (uint8_t)(g_radar_fw_data_uds_packet_counter >> 16);
			g_test_tx_buffer[2] = (uint8_t)(g_radar_fw_data_uds_packet_counter >> 8);
			g_test_tx_buffer[3] = (uint8_t)(g_radar_fw_data_uds_packet_counter); //
			g_test_tx_buffer[4] = (uint8_t)(g_radar_fw_data_sbl_packet_counter >> 8);
			g_test_tx_buffer[5] = (uint8_t)(g_radar_fw_data_sbl_packet_counter);
			g_test_tx_buffer[6] = g_radar_image_tx_use_num; //
			g_test_tx_buffer[7] = g_radar_image_tx_buf_num; //
			vc_tx_test_packet_t(t_id, g_test_tx_buffer);
		}
	}
}

//////////////////////////////////////////////////////////////
////////   Send To Vehicle CAN TDM            ////////////////
//////////////////////////////////////////////////////////////
uint32_t vc_tx_cwi1_packet_t(uint8_t *CWI1_data)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	//uint32_t _swap4[2];

	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = 0x18E0177A;//TTDW;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	// fpdw_info.REG0._bitval.FPDW_FrontSystemOn =1;
	// fpdw_info.REG0._bitval.FPDW_DisplayFRH =1;
	// fpdw_info.REG0._bitval.FPDW_DisplayFLH =1;
	// fpdw_info.REG0._bitval.FPDW_DisplayFCTR =1;
	// fpdw_info.REG0._bitval.FPDW_Fsound =0;
	// fpdw_info.REG0._bitval.FPDW_FrontPDWDiag =1;

	// fpdw_info.REG0._bitval.t1 =1;
	// fpdw_info.REG0._bitval.t2 =0;
	// fpdw_info.REG0._bitval.t3 =0;
	// fpdw_info.REG0._bitval.t4 =1;
	// fpdw_info.REG0._bitval.t5 =0;
	// fpdw_info.REG0._bitval.t6 =1;
	// fpdw_info.REG0._bitval.t7 =0;
	// /*The bit definitions are reversed in union struct, for rvs_bits function is required*/
	// fpdw_info.REG0._uint64_val = rvs_bits(fpdw_info.REG0._uint64_val);
	
	//memcpy((void *)&_swap4[0], (void *)&fpdw_info, sizeof(fpdw_info));
//	txf.data8[0] = 0x0;
//	txf.data8[1] = 0x0;
//	txf.data8[2] = 0x0;
//	txf.data8[3] = 0x0;
//	txf.data8[4] = warning5;
//	txf.data8[5] = 0x0;
//	txf.data8[6] = 0x0;
//	txf.data8[7] = warning4;

	//pal_lld_togglepad(PORT_LED_B_STATE, LED_B_STATE);
	memcpy(&txf.data8[0], CWI1_data, 8);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);
	return txReturn;
}

uint32_t vc_tx_cwi2_packet_t(uint8_t *CWI2_data)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = 0x18E1177A;//TTDW;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	memcpy(&txf.data8[0], CWI2_data, 8);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);
	return txReturn;
}

/*  dm1 packet send
 *  id_flag 1 : DM1
 *  id_flag 2 : TPDT
 *  id_flag 3 : TPCM_BAM
 */
uint32_t vc_tx_dtc_packet_t(uint8_t id_flag, uint8_t *dtc_data)
{
	uint32_t  txReturn;
	uint32_t id = 0;
	CANTxFrame txf;
	memset(&txf, 0, sizeof(CANTxFrame));

	// DM1
	if (id_flag == 1)
		id = 0x18FECA7A;
	//TPDT
	else if (id_flag == 2)
		id = 0x18EBFF7A;
	//TPCM_BAM
	else
		id = 0x18ECFF7A;

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = id;//TTDW;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	memcpy(&txf.data8[0], dtc_data, 8);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);
	return txReturn;
}

uint32_t vc_tx_xbr_active_packet_t(void)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = 0x0C040B7A;//TTDW;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	memset(&txf.data8[0], 0xFF, 8);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);
	return txReturn;
}

uint32_t vc_tx_xbr_passive_packet_t(void)
{
	CANTxFrame txf;
	uint32_t  txReturn;
	memset(&txf, 0, sizeof(CANTxFrame));

	txf.TYPE = CAN_ID_XTD; //CAN_ID_STD;
	txf.DLC = 8U;
	txf.ID = 0x0C040B7A;//TTDW;
	txf.OPERATION = CAN_OP_NORMAL; // CAN_OP_NORMAL : Classical CAN, CAN_OP_CANFD : CAN FD.

	memset(&txf.data8[0], 0xFF, 8);
	txReturn = can_lld_transmit(&CAND1, CAN_DEDICATED_TXBUFFER, &txf);
	return txReturn;
}


void vc_set_CAN_filter(void)
{
    int num=0,num0=0,num1 = 0;
    uint8_t overlap_flag = 0;
    can_list_t *pCan;               

    pCan = get_can_list(); 

    for(int i=0; i<MAX_NUM_CAN_ID; i++)
    {
        if(pCan->can_id_info[i].id == 0)	continue;
        else{}

        overlap_flag = 0; 
        for(int j=0; j<i; j++)
        {
            if(pCan->can_id_info[i].id == pCan->can_id_info[j].id)
            {
                overlap_flag = 1; 
                printf("overlap:%i\r\n",i);
            }
            else{}
        }

        if(overlap_flag == 0)
        {
            if(pCan->can_id_info[i].id <= MAX_NORMAL_CAN_ID)
            {
                can_config_mcanconf.STD_Filter[num0].SFID1 = pCan->can_id_info[i].id ;
                can_config_mcanconf.STD_Filter[num0].SFID2 = num;
                can_config_mcanconf.STD_Filter[num0].SFT = 0;
                can_config_mcanconf.STD_Filter[num0].SFEC = 7;
                num0++;
                printf("normal id add:%d\r\n",i);
            }
            else 
            {
                can_config_mcanconf.XTD_Filter[num1].EFID1 = pCan->can_id_info[i].id ;
                can_config_mcanconf.XTD_Filter[num1].EFID2 = num;
                can_config_mcanconf.XTD_Filter[num1].EFT = 0;
                can_config_mcanconf.XTD_Filter[num1].EFEC = 7;
                num1++;
                printf("extended id add:%d\r\n",i);
            }    
            num++;
        }
        else{}
    }    

    for (int i=num0; i<sizeof(can_config_mcanconf.STD_Filter)/sizeof(can_config_mcanconf.STD_Filter[0]); i++)
    {
        can_config_mcanconf.STD_Filter[i].SFID1 = 0xfff ;
        can_config_mcanconf.STD_Filter[i].SFID2 = i;
        can_config_mcanconf.STD_Filter[i].SFT = 0;
        can_config_mcanconf.STD_Filter[i].SFEC = 0;
    }

    for (int i=num1; i<sizeof(can_config_mcanconf.XTD_Filter)/sizeof(can_config_mcanconf.XTD_Filter[0]); i++)
    {
        can_config_mcanconf.XTD_Filter[i].EFID1 = 0x800 ;
        can_config_mcanconf.XTD_Filter[i].EFID2 = i;
        can_config_mcanconf.XTD_Filter[i].EFT = 0;
        can_config_mcanconf.XTD_Filter[i].EFEC = 0;
    }

}

int vc_set_CAN_bitRate(uint8_t type)
{
    uint32_t baud, sample;
    uint32_t bit_len, nsjw, ntseg1, ntseg2; 
    uint32_t protocol_clk = PROTOCOL_CAN_CLOCK;
    can_list_t *pCan;         

    pCan = get_can_list();
    protocol_clk = PROTOCOL_CAN_CLOCK / SPC5_CGM_AC8_DC0_DIV_VALUE;    

    if(type == CAN_NORMAL)
    {
        baud = pCan->baudrate[CAN_NORMAL];
        sample = pCan->samplepoint[CAN_NORMAL];

        if((baud == NULL) || (baud > MAX_BAUD_RATE_CAN_NORMAL) || (sample == NULL) || (sample > MAX_SAMPLE_POINT))
            return ERROR_CAN_OUT_OF_RANGE;

        bit_len = protocol_clk/(baud*KBIT_PER_SECOND);
        ntseg1 = (sample * bit_len / SAMPLE_POINT_CONVERT_VARIABLE) -2; // 2 :  Sync_Seg(1) + (ntseg1 index:1)
        nsjw = bit_len - ntseg1 - 2 -1; // 3 :  Sync_Seg(1) + (default index:1) + (default index:1)
        ntseg2 = nsjw;

        can_config_mcanconf.nsjw = nsjw;
        can_config_mcanconf.ntseg1 = ntseg1;
        can_config_mcanconf.ntseg2 = ntseg2;
    }
    else
    {
        baud = pCan->baudrate[CAN_FD];
        sample = pCan->samplepoint[CAN_FD];

        if((baud == NULL) || (baud > MAX_BAUD_RATE_CAN_FD) || (sample == NULL) || (sample > MAX_SAMPLE_POINT))
            return ERROR_CAN_OUT_OF_RANGE;

        bit_len = protocol_clk/(baud*KBIT_PER_SECOND);
        ntseg1 = (sample * bit_len / SAMPLE_POINT_CONVERT_VARIABLE) -2; 
        nsjw = bit_len - ntseg1 - 2 - 1;
        ntseg2 = nsjw;

        can_config_mcanconf.dsjw = nsjw;
        can_config_mcanconf.dtseg1 = ntseg1;
        can_config_mcanconf.dtseg2 = ntseg2;
    }
    return 0;
}

void vc_set_control_CAN_task(uint8_t mode)
{
    task_handle_t *taskhandle;
    taskhandle = get_task_handle();

    if(mode == 0) 
    {
        vTaskSuspend(taskhandle->task_vehicle_communication_handle);     
        vTaskSuspend(taskhandle->task_vehicle_communication_handle);   
        vTaskDelay(500);
        can_lld_stop(&CAND1);
//        can_lld_stop(&CAND7);
        can_lld_stop(&CAND9);
    }
    else 
    {
        can_lld_start(&CAND1, &can_config_mcanconf);  /*MCAN SUB  0 CAN 0*/
//        can_lld_start(&CAND7, &can_config_mcanconf);  /*MCAN SUB  1 CAN 1*/
        can_lld_start(&CAND9, &can_config_mcanconf);  /*MCAN SUB  1 CAN 3*/
        vTaskDelay(500);
        vTaskResume(taskhandle->task_vehicle_communication_handle);     
        vTaskResume(taskhandle->task_vehicle_communication_handle);     
    }
}

void set_clear_reset(void)
{
    /*Initialization reset status of functional reset and destructive reset*/
    MC_RGM.FES.R = 0xFFFFU;    
    MC_RGM.DES.R = 0xFFFFU;
}

void set_clear_fwupdate(void)
{
	g_radar_fw_data_uds_packet_counter = 0;
	g_radar_fw_data_sbl_packet_counter = 0;
	g_radar_image_tx_buf_num = 0;
	g_radar_image_tx_use_num = 0;
	g_radar_image_buffer_counter = 0;

	memset(g_radar_image_tx_buffer, 0, sizeof(g_radar_image_tx_buffer));
	memset(g_radar_iamge_buffer, 0, sizeof(g_radar_iamge_buffer));

	g_radar_fw_update_flag = 0;
}

void Init_CWI1_data_packet(void)
{
	memset(tx_CWI1_data,0x00,8);
	tx_CWI1_data[0]=0; // pas no detection
	tx_CWI1_data[3]=0; // radar no detection
	tx_CWI1_data[7]=0xff; // Switch Status
}

// ERROR가 발생하여도 전달할 수 있는 위험은 전달한다.
uint8_t start_flag = 0;
void vc_can_Broadcast(void)
{
	uint8_t sgas_system_state = get_diag_state();

	if(sgas_system_state == SGAS_DIAG_LOADING)
	{
		uint8_t system_start_sound_packet[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
		vc_tx_cwi1_packet_t(system_start_sound_packet);
	}
	else
	{
		// SGAS Switch Check, (Left SGAS Physical Switch Check, Right SGAS AVN Switch Check)
		if ((pal_lld_readpad(PORT_VHE_SIG_SGAS_SW_MCU, VHE_SIG_SGAS_SW_MCU) == 1) == 1 || g_sgas_avn_switch_flag == 1)
		{
			// Initialize CWI message
			Init_CWI1_data_packet();

			// get Vehicle Data
			vehicle_info_t *vehicle_info;
			vehicle_info = get_vehicle_info();

			// get pas warning msg
			uint8_t pas_warn = get_pas_CWI1_data();

			// get radar warning msg
			warn_message_t *pWarn;
			pWarn = get_warn_mesg();

			// SWITCH STATE
			tx_CWI1_data[7] = 0xf1;

			// RADAR 동작 조건
//			if (pWarn->bsd_packet)
//			{
				// RADAR Warning
				tx_CWI1_data[4] = pWarn->bsd_packet;
				tx_CWI1_data[7] |= 0xF4;	// DIAG STATE : system good 1111 0101
//			}

//			// PAS 동작 조건
//			if(vehicle_info->gear == GEAR_PARKING && vehicle_info->speed <= 10)
//			{
				tx_CWI1_data[0] = pas_warn;
				tx_CWI1_data[7] |= 0xF4; 	// DIAG STATE : system good 1111 0101

//				// CWI2 Send - PAS RearDetectionObjectDistanc
//				uint8_t *pas_CWI2_data;
//				pas_CWI2_data = get_pas_CWI2_data();
//				vc_tx_cwi2_packet_t(pas_CWI2_data);
//			}

			// DIAG STATE 	: ERROR
			if(sgas_system_state == SGAS_DIAG_SYSTEM_ERROR)
			{
				tx_CWI1_data[7] = 0xF9;  // SYSTEM ERROR 1111 1001
			}
		}
//		// SGAS Switch OFF
//		else
//		{
//			// Set Default Data
//			tx_CWI1_data[0] = 0x00;	// PAS STATE
//			tx_CWI1_data[4] = 0x00;	// RADAR STATE
//			tx_CWI1_data[7] = 0xF3;	// SWITCH STATE
//			tx_CWI1_data[7] |= 0xF8; 	// DIAG	STATE
//		}

		// CWI1 Send
		vc_tx_cwi1_packet_t(tx_CWI1_data);
	}
}

// XBR delay is controlled by RTOS manager
void vc_xbr_packet(void)
{
	vc_tx_xbr_active_packet_t();
}























