#include <string.h>
#include "lin_lld.h"
#include "lin_lld_cfg.h"
#include "serial_lld.h"

#include "pas_communication.h"
#include "pas_communication_parameter.h"
#include "diagnosis_errorcode.h"
#include "vehicle_communication.h"

// for delay
#include <FreeRTOS.h>
#include <task.h>

//// pin control
//#include "siul_lld.h"

//#include "lin_lld.h" // LIN 드라이버 헤더 포함
//#include "uart_lld.h" // UART 드라이버 헤더 포함

//indicator tset
//#include "indicator_control.h"

/*The vehicle_info_t structure variable holds vehicle related information to share other task. 
This is obatained through the'polling_receive_vehicle_info' function on can communication*/	

MPAS_t pas_info;
uint8_t CWI1_data[8];
uint8_t CWI2_data[8];
uint8_t Slave2_data[10] = {0, };

uint8_t DM1_data[8];


uint8_t get_pas_CWI1_data(void)
{
//    uint8_t *pPas_CWI1;
//    pPas_CWI1 = &CWI1_data[0];
//    return pPas_CWI1;
    uint8_t CWI1_pas_data = CWI1_data[0];
    return CWI1_pas_data;
}

uint8_t *get_pas_CWI2_data(void)
{
    uint8_t *pPas_CWI2;
    pPas_CWI2 = &CWI2_data[0];
    return pPas_CWI2;
}

MPAS_t *get_pas_info(void)
{
    return &pas_info;
}

uint32_t get_pas_dm1_req(void)
{
	return pas_info.DM1_req;
}


void pas_sensor_start(void)
{
	// init params
	Init_Variable();

	// send start message
	uint8_t sensor_start[3] = {0, };
	uint8_t idMessage_sensor_start = 0x47;

	ld_cmd_master_write(idMessage_sensor_start, sensor_start, 3);
}

uint8_t g_pas_comm_checker = 0;

void pas_read(void)
{
	uint8_t idMessage = 0xC4;
	uint8_t ret = 0;

	if((pas_info.DM1_req & (1<<DM1_LIN_COMM_ERR))==0)
	{
		ld_cmd_master_read(idMessage, Slave2_data, PAS_BUF_SIZE);

		Check_DM1_fromLIN(&Slave2_data[0]);
		Update_Sensing_Data(&Slave2_data[0]);

        // LIN communication Check
        for (uint8_t i = 0; i < PAS_BUF_SIZE; i++) {
            if (Slave2_data[i] != 0) {
            	ret = 1;
                break;
            }
        }

        // LIN communication error set DTC
		if(ret == 0)
		{
			if(g_pas_comm_checker < 10)
			{
				g_pas_comm_checker++;
			}
			else
			{
				Init_Variable();
				pas_info.DM1_req |= (1<<DM1_LIN_COMM_ERR);
			}
		}
		memset(&Slave2_data,0, sizeof(Slave2_data));
	}
}

uint8_t ld_cmd_master_read(uint8_t idMessage, uint8_t *data, uint16_t buf_len)
{

	lin_lld_transmit(&LD10, idMessage, data, 2);

	lin_lld_receive(&LD10, idMessage, data, buf_len);

//	// for test
//	uint8_t g_test_tx_buffer[32] = {0, };
//	uint32_t t_id = 0x83;
//	g_test_tx_buffer[0] = data[0];
//	g_test_tx_buffer[1] = data[1];
//	g_test_tx_buffer[2] = data[2];
//	g_test_tx_buffer[3] = data[3];
//	g_test_tx_buffer[4] = data[4];
//	g_test_tx_buffer[5] = data[5];
//	g_test_tx_buffer[6] = data[6];
//	g_test_tx_buffer[7] = data[7];
////	memset(&data[0], 0, 8);
//	vc_tx_test_packet_t(t_id, g_test_tx_buffer);
	return 0;
}

void ld_cmd_master_write(uint8_t idMessage, uint8_t *data, uint16_t buf_len)
{
	uint16_t checksum, i;

	checksum=0;
	for(i=0 ; i < buf_len - 1 ; i++)
	{
		checksum += data[i];
	}
	checksum += idMessage;
	checksum = ~(checksum % 255);

	data[buf_len - 1] = checksum & 0xFF;

	lin_lld_transmit(&LD10, idMessage, data, buf_len - 1);
}

// algorithms
void Init_Variable(void)
{
	memset(&pas_info,0, sizeof(MPAS_t));
//	pas_info.Reverse_state=ReverseGear_INIT;
//	pas_info.OffSW_state=PAS_SW_INIT;
	memset(DM1_data,0xff,8);
	memset(CWI1_data,0xff,8);
	CWI1_data[0]=0; // pas no detection
	CWI1_data[4]=0; // radar no detection
//	CWI1_data[6]=0x0C;

	// Switch Status
	CWI1_data[7]=0xff; // system standby,PAS off switch not available
	memset(CWI2_data,0xff,8);

	pas_info.system_standby=1;

//	gPasUpdate.state=UPDATE_STATE_NONE;

}


void Check_DM1_fromLIN(uint8_t * data)
{
	uint8_t master,slave1,slave2,slave3;
	uint32_t l_DM1_req;

	master=(data[6]>>3)&0x7;
	slave1=data[6]&0x7;
	slave2=(data[7]>>5)&0x7;
	slave3=(data[7]>>2)&0x7;

	pas_info.PA_Noise=0;
	l_DM1_req=0;
	switch(master)
	{
#if 0//ndef PA_NOISE_IGNORE
		case 1: //snesor noise
		case 5: // transducer error
			pas_info.DM1_req |= (1<<DM1_SENSOR_NOISE_MASTER);
			pas_info.PA_Noise |= (1<<DM1_SENSOR_NOISE_MASTER);
			break;
#endif
		case 6: //calibration data empty
			//pas_info.DM1_req |= (1<<DM1_SENSOR_CAL_DATA_EMPTY);
			l_DM1_req |= (1<<DM1_SENSOR_CAL_DATA_EMPTY);
			break;

		case 2: //signal open
		case 3:// signal short to ground
		case 4: //signal short to power
			break;
		default:

			break;

	}


	switch(slave1)
	{
#if 0//ndef PA_NOISE_IGNORE

		case 1: //snesor noise
			//pas_info.DM1_req |= (1<<DM1_SENSOR_NOISE_S1);
			pas_info.PA_Noise |= (1<<DM1_SENSOR_NOISE_S1);
			break;
#endif
		case 2: //signal open
			//pas_info.DM1_req |= (1<<DM1_SENSOR_OPEN_S1);
			l_DM1_req |= (1<<DM1_SENSOR_OPEN_S1);
			break;

		case 3:// signal short to ground
		case 4: //signal short to power
		case 5: // transducer error
			//pas_info.DM1_req |= (1<<DM1_SENSOR_SHORT_S1);
			l_DM1_req |= (1<<DM1_SENSOR_SHORT_S1);
			break;

		case 6: //calibration data empty
		default:

		break;

	}

	switch(slave2)
	{
#if 0//ndef PA_NOISE_IGNORE

		case 1: //snesor noise
			//pas_info.DM1_req |= (1<<DM1_SENSOR_NOISE_S2);
			pas_info.PA_Noise |= (1<<DM1_SENSOR_NOISE_S2);
			break;
#endif
		case 2: //signal open
			//pas_info.DM1_req |= (1<<DM1_SENSOR_OPEN_S2);
			l_DM1_req |= (1<<DM1_SENSOR_OPEN_S2);
			break;

		case 3:// signal short to ground
		case 4: //signal short to power
		case 5: // transducer error
			//pas_info.DM1_req |= (1<<DM1_SENSOR_SHORT_S2);
			l_DM1_req |= (1<<DM1_SENSOR_SHORT_S2);
			break;

		case 6: //calibration data empty
		default:

		break;

	}

	switch(slave3)
	{
#if 0//ndef PA_NOISE_IGNORE

	case 1: //snesor noise
		//pas_info.DM1_req |= (1<<DM1_SENSOR_NOISE_S3);
		pas_info.PA_Noise |= (1<<DM1_SENSOR_NOISE_S2);
		break;
#endif

	case 2: //signal open
		//pas_info.DM1_req |= (1<<DM1_SENSOR_OPEN_S3);
		l_DM1_req |= (1<<DM1_SENSOR_OPEN_S3);
		break;

	case 3:// signal short to ground
	case 4: //signal short to power
	case 5: // transducer error
		//pas_info.DM1_req |= (1<<DM1_SENSOR_SHORT_S3);
		l_DM1_req |= (1<<DM1_SENSOR_SHORT_S3);
		break;

	case 6: //calibration data empty
	default:

	break;

}

	if(pas_info.DM1_req && l_DM1_req==0)
		printf("DM1 removed old DM1_req=%x data6=%x data7=%x \n\r",pas_info.DM1_req,data[6],data[7]);
	pas_info.DM1_req=l_DM1_req;
}


void Update_Sensing_Data(uint8_t * data)
{
	uint8_t master,slave1,slave2,slave3;
#ifdef SENSOR_DISPLAY_SHORTEST

	uint8_t shortest,i,temp_zone;
#endif
	master=(data[4]>>4)&0xf;
	slave1=data[4]&0xf;
	slave2=(data[5]>>4)&0xf;
	slave3=data[5]&0xf;


	switch(master)
	{
		case 1: //Collision Range : 25cm to 37cm
		case 2: // Collision Range : 38cm to 50cm
			//zone1 red
			CWI1_data[0] &= 0xFC;
			CWI1_data[0] |= 0x3;
#ifdef USE_CWI3
			CWI3_data[0] &= 0xFC;
			CWI3_data[0] |= 0x3;
#endif

			break;

		case 3: //Main-warning Range : 51cm to 75cm
		case 4: //Main-warning Range : 76cm to 100cm
			//zone2 yellow
			CWI1_data[0] &= 0xFC;
			CWI1_data[0] |= 0x2;
#ifdef USE_CWI3
			CWI3_data[0] &= 0xFC;
			CWI3_data[0] |= 0x2;
#endif

			break;
		case 5:// Pre-warning Range : 101cm to 125cm
		case 6: //Pre-warning Range : 126cm to 150cm
			//zone3 green
			CWI1_data[0] &= 0xFC;
			CWI1_data[0] |= 0x1;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xFC;
			CWI3_data[0] |= 0x1;
#endif

			break;

		case 0: //No-warning Range : Over 150cm
			CWI1_data[0] &= 0xFC;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xFC;
#endif

			break;
		default:

			break;

	}

	switch(slave1)
	{
		case 1: //Collision Range : 25cm to 37cm
		case 2: // Collision Range : 38cm to 50cm
			//zone1 red
			CWI1_data[0] &= 0xF3;
			CWI1_data[0] |= 0xC;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xF3;
			CWI3_data[0] |= 0xC;
#endif

			break;

		case 3: //Main-warning Range : 51cm to 75cm
		case 4: //Main-warning Range : 76cm to 100cm
			//zone2 yellow
			CWI1_data[0] &= 0xF3;
			CWI1_data[0] |= 0x8;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xF3;
			CWI3_data[0] |= 0x8;
#endif

			break;
		case 5:// Pre-warning Range : 101cm to 125cm
		case 6: //Pre-warning Range : 126cm to 150cm
			//zone3 green
			CWI1_data[0] &= 0xF3;
			CWI1_data[0] |= 0x4;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xF3;
			CWI3_data[0] |= 0x4;
#endif

			break;

		case 0: //No-warning Range : Over 150cm
			CWI1_data[0] &= 0xF3;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xF3;
#endif

			break;
		default:

			break;

	}

	switch(slave2)
	{
		case 1: //Collision Range : 25cm to 37cm
		case 2: // Collision Range : 38cm to 50cm
			//zone1 red
			CWI1_data[0] &= 0xCF;
			CWI1_data[0] |= 0x30;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xCF;
			CWI3_data[0] |= 0x30;
#endif

			break;

		case 3: //Main-warning Range : 51cm to 75cm
		case 4: //Main-warning Range : 76cm to 100cm
			//zone2 yellow
			CWI1_data[0] &= 0xCF;
			CWI1_data[0] |= 0x20;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xCF;
			CWI3_data[0] |= 0x20;
#endif

			break;
		case 5:// Pre-warning Range : 101cm to 125cm
		case 6: //Pre-warning Range : 126cm to 150cm
			//zone3 green
			CWI1_data[0] &= 0xCF;
			CWI1_data[0] |= 0x10;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xCF;
			CWI3_data[0] |= 0x10;
#endif

			break;

		case 0: //No-warning Range : Over 150cm
			CWI1_data[0] &= 0xCF;

#ifdef USE_CWI3
			CWI3_data[0] &= 0xCF;
#endif

			break;
		default:

			break;

	}

	switch(slave3)
	{
		case 1: //Collision Range : 25cm to 37cm
		case 2: // Collision Range : 38cm to 50cm
			//zone1 red
			CWI1_data[0] &= 0x3F;
			CWI1_data[0] |= 0xC0;

#ifdef USE_CWI3
			CWI3_data[0] &= 0x3F;
			CWI3_data[0] |= 0xC0;
#endif

			break;

		case 3: //Main-warning Range : 51cm to 75cm
		case 4: //Main-warning Range : 76cm to 100cm
			//zone2 yellow
			CWI1_data[0] &= 0x3F;
			CWI1_data[0] |= 0x80;

#ifdef USE_CWI3
			CWI3_data[0] &= 0x3F;
			CWI3_data[0] |= 0x80;
#endif

			break;
		case 5:// Pre-warning Range : 101cm to 125cm
		case 6: //Pre-warning Range : 126cm to 150cm
			//zone3 green
			CWI1_data[0] &= 0x3F;
			CWI1_data[0] |= 0x40;

#ifdef USE_CWI3
			CWI3_data[0] &= 0x3F;
			CWI3_data[0] |= 0x40;
#endif

			break;

		case 0: //No-warning Range : Over 150cm
			CWI1_data[0] &= 0x3F;

#ifdef USE_CWI3
			CWI3_data[0] &= 0x3F;
#endif


			break;
		default:

			break;

	}


#ifdef SENSOR_DISPLAY_SHORTEST

	shortest=0;

	for(i=0;i<4;i++)
	{
		temp_zone =((CWI1_data[0]>> (i*2))&&0x3);

		if(temp_zone >=1 && temp_zone <=3)
		{
			if((shortest == 0) || (temp_zone > shortest ))
				shortest=temp_zone;

		}
	}

	for(i=0;i<4;i++)
	{
		temp_zone =((CWI1_data[0]>> (i*2))&&0x3);

		if(temp_zone < shortest)
			CWI1_data[0] &= (uint8_t)(~(0x3 << (i*2)));
	}
#endif


        CWI2_data[0]=((uint16_t)(data[0])*10) & 0xff;//master low LH-S
        CWI2_data[1]=(((uint16_t)(data[0])*10)>>8) & 0xff;//master high
        CWI2_data[2]=((uint16_t)(data[1])*10) & 0xff;//slave1 low LH-C
        CWI2_data[3]=(((uint16_t)(data[1])*10)>>8) & 0xff;//slave1 high
        CWI2_data[4]=((uint16_t)(data[2])*10) & 0xff;//slave2 low RH-C
        CWI2_data[5]=(((uint16_t)(data[2])*10)>>8) & 0xff;//slave2 high
        CWI2_data[6]=((uint16_t)(data[3])*10) & 0xff;//slave3 low RH-S
        CWI2_data[7]=(((uint16_t)(data[3])*10)>>8) & 0xff;//slave3 high
}
