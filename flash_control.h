#ifndef _FLASH_CONTROL_H_
#define _FLASH_CONTROL_H_
extern void init_data_from_eeprom(void);
extern uint32_t set_SGAS_switch_flag(uint32_t avn_switch_flag);
extern uint32_t get_SGAS_switch_flag(void);

#endif
