#include "components.h"
#include "can_lld_cfg.h"
#include "vehicle_communication.h"
#include "flash_control.h"

uint32_t Testvalue_gg = 0;
void init_data_from_eeprom(void)
{
    uint32_t returnvalue;
    uint32_t size;
    static uint16_t watchdog_cnt =0;
    CANConfig tmp_can_config;
    can_desc_t tmp_can_desc;
	can_desc_t *pCan;
    pCan = get_can_desc();
   
    returnvalue = eeprom_start(&EEPROMD, &eeprom_cfg);
    Testvalue_gg = returnvalue;
    if( (returnvalue == EEPROM_OK) || (returnvalue == EEPROM_FIRST_INIT_OK) )
	{
//        returnvalue = eeprom_read(&EEPROMD, 0, &size, &tmp_can_desc);
//		// if(size != sizeof(can_desc_t))
//		// {
//		// 	eeprom_write(&EEPROMD, 0, sizeof(can_desc_t), pCan);
//		// 	pal_lld_togglepad(PORT_E, LED_R_STATE);
//		// }
//        if(returnvalue != EEPROM_ERROR_ID_NOT_FOUND)
//        {
//            memcpy(pCan, &tmp_can_desc, sizeof(can_desc_t));
//        }
//		else{}
//
//
//        size = 0;
//        eeprom_read(&EEPROMD, 1, &size, &tmp_can_config);
//        if(returnvalue != EEPROM_ERROR_ID_NOT_FOUND)
//        {
//            memcpy(&can_config_mcanconf, &tmp_can_config, sizeof(CANConfig));
//        }
//		else{}
//
//        eeprom_read(&EEPROMD, 2, &size, &watchdog_cnt);
//        /*Check wether watchdog reset has been occured or not*/
//        if(MC_RGM.FES.B.F_FCCU_HARD == 0x1)
//        {
//            watchdog_cnt += 1;
//            eeprom_write(&EEPROMD, 2, sizeof(uint16_t), &watchdog_cnt);
//        }
//		else{}
    }
	else{}
    Testvalue_gg = returnvalue;
}

// uint32_g_avn_switch_flag = get_SGAS_switch_flag();
uint32_t get_SGAS_switch_flag(void)
{
	uint32_t returnvalue;
	uint32_t avn_switch_flag[1];
	uint32_t size;

	returnvalue = eeprom_read(&EEPROMD, (uint16_t)1, &size, (uint32_t)&avn_switch_flag);

	return avn_switch_flag[0];
}

// 		uint32_t avn_switch = 4;
//		set_SGAS_switch_flag(avn_switch);
uint32_t set_SGAS_switch_flag(uint32_t avn_switch_flag)
{
	uint32_t returnvalue;
	uint32_t save_avn_switch_flag[1];

	save_avn_switch_flag[0] = avn_switch_flag;
	returnvalue = eeprom_write(&EEPROMD, (uint16_t)1, (uint32_t)sizeof(save_avn_switch_flag), (uint32_t)&save_avn_switch_flag);

	return returnvalue;
}




















