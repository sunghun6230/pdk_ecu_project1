#include "diagnosis_setting.h"
//#include "diagnosis_parameter.h"
#include "diagnosis_errorcode.h"

#ifndef DIAGNOSIS_H_
#define DIAGNOSIS_H_

typedef	struct
{
	uint8_t loadsw_off;
	uint8_t loadsw_on;
	uint8_t sync_flag;
	uint8_t reset_flag;
	uint16_t sync_reset_flag;
	uint8_t loadsw_off_flag;
	uint8_t sync_ch_select;
}diag_mmwave_t;

extern uint8_t get_diag_state(void);
extern void diag_initialize(void);
void diag_dm1_initialize(void);

extern void cli_reboot(void);
uint8_t *diag_dm1_default(void);
extern void diag_read_of_adc_e_fuse(void);
extern void diag_mmwave_sync(void);
extern void diag_radar_reset(void);
extern void diag_radar_off(void);
extern void diag_radar_off_non_update(uint8_t radar_param);
extern void dg_radar_fault(void);
extern void diag_communication(void);
//extern uint8_t get_radar_system_checker(void);
extern uint32_t diag_system_check(uint32_t DM1_current, uint8_t current_radar_num);
//extern uint32_t diag_system_check(MPAS_t pPas_info, uint32_t DM1_current, uint8_t current_radar_num);


uint8_t *CAN_SEND_DM1(uint32_t DTC);
void CAN_SEND_BAM(uint32_t DTC_req,uint8_t count);

#endif
