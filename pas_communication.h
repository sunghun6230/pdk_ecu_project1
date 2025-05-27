#include "lin_lld.h"
#include "pas_communication_parameter.h"
#include "pas_communication_setting.h"

/* 
 ***Maintaining order is crucial within the vehicle_info_t  
 */

typedef struct
{
//	uint8_t KLINE_Rx_Buff[KLINE_RX_BUFF_SIZE];
//	uint8_t KLINE_Tx_Buff[KLINE_RX_BUFF_SIZE];
//	osThreadId LIN_handle;
//	osThreadId CAN_Rx_handle;
//	osThreadId CAN_Tx_handle;
//	osThreadId CWI1_handle;
//	osThreadId DM1_handle;
//	osThreadId KLINE_Rx_handle;
//	osThreadId KLINE_Tx_handle;
//	osThreadId Chk_power_handle;
//	osThreadId IO_chk_handle;
//	osThreadId FLASH_UP_handle;
//
//	osMutexId CAN_TX_Mutex;
	uint32_t Battery_vol;
	uint32_t Battery_cnt;

	uint32_t ADC_val;
	uint32_t Array_ADC_val[5];
	uint32_t power_state;

	uint32_t det_Reverse_high;
	uint32_t det_Reverse_low;
	uint32_t det_OffSW_high;
	uint32_t det_OffSW_low;

	uint16_t	EEC1_Engine_Speed;//rpm 1bit 0.125
	uint16_t	TCO1_Vehicle_Speed;

	uint32_t DM1_req;
	//uint8_t DM1_active[DM1_ERROR_MAX];
	uint32_t g10ms_count;
	uint32_t g1S_count;


	uint8_t	ETC2_Current_Gear;
	uint8_t ETC2_Received;
	uint8_t TCO1_Received;
	uint8_t EEC1_Received;
	uint8_t Reverse_state;
	uint8_t OffSW_state;
	uint8_t	Battery_init;
	uint8_t Send_DTC;
	uint8_t system_standby;
	uint8_t	lin_inited;
//	Uart_queueType KLINE_Rx_Que;
//	Uart_queueType KLINE_Tx_Que;
	uint8_t KLINE_source_addr;
	uint32_t PA_Noise;
}MPAS_t;

extern uint8_t get_pas_CWI1_data(void);
extern uint8_t *get_pas_CWI2_data(void);
extern MPAS_t *get_pas_info(void);
extern uint32_t get_pas_dm1_req(void);
extern void pas_sensor_start(void);
extern void pas_read(void);
extern uint8_t ld_cmd_master_read(uint8_t idMessage, uint8_t *data, uint16_t buf_len);
extern void ld_cmd_master_write(uint8_t idMessage, uint8_t *data, uint16_t buf_len);
void Check_DM1_fromLIN(uint8_t * data);
void Update_Sensing_Data(uint8_t * data);
void Init_Variable(void);






