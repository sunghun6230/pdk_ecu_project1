/****************************************************************************
*
* Copyright ¬© 2017-2019 STMicroelectronics - All Rights Reserved
*
* This software is licensed under SLA0098 terms that can be found in the
* DM00779817_1_0.pdf file in the licenses directory of this software product.
*
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED,
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*****************************************************************************/

/* Inclusion of the main header files of all the imported components in the
   order specified in the application wizard. The file is generated
   automatically.*/
#include "main.h"
#include "components.h"
#include "siul_lld.h"
#include "can_lld_cfg.h"
#include "lin_lld_cfg.h"
#include "serial_lld_cfg.h"
#include "swt_lld_cfg.h"
#include "pit_lld.h"
#include "pit_lld_cfg.h"
#include "spi_lld_cfg.h"
#include "spi_lld.h"
#include "flash_control.h"
#include "PDK_RTOS_manager.h"
#include "radar_communication.h"
#include "vehicle_communication.h"
#include "command_interface.h"
#include "indicator_control.h"
#include "diagnosis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "source/module/pas/pas_communication.h"

void LSW_Init(void);

// Define a semaphore handle
SemaphoreHandle_t radar_to_algorithm_xSemaphore;
SemaphoreHandle_t algorithm_to_indication_xSemaphore;
//SemaphoreHandle_t fwupdate_vehicle_to_radar_xSemaphore;

int main(void) {
    /* Initialization of all the imported components in the order specified in
      the application wizard. The function is generated automatically.*/
    componentsInit();

    /* Enable Interrupts */
    irqIsrEnable();

    /* HW Init */
    LSW_Init();

    /*Software Watch dog Init*/
	swt_lld_start(&SWTD3, &swt2_config);
	pit_lld_start(&PITD1, pit0_config);

    /*read data from flash(eeprom)*/
    init_data_from_eeprom();

    /* Start CAN Driver */
    can_lld_start(&CAND1, &can_config_mcanconf);  /*MCAN SUB  0 CAN 0 - Vehicle */
    //can_lld_start(&CAND3, &can_config_mcanconf);  /*MCAN SUB  0 CAN 2 - Local */
//    can_lld_start(&CAND7, &can_config_Lcanfdconf);  /*MCAN SUB  1 CAN 1 - Left */
    can_lld_start(&CAND9, &can_config_Rcanfdconf);  /*MCAN SUB  1 CAN 3 - Right */

    /* Start Serial Driver - PAS */
//    sd_lld_start(&SD10, &serial_config_pas_9);
    lin_lld_start(&LD10, &lin_config_lin_config_Pas);


   /* Module Init */
    rc_radar_initialize();
    vc_can_initialize();
    cli_initialize();
    INDC_indicator_control_initialize();
    diag_initialize();


    /*Initialization reset status of functional reset and destructive reset*/
    set_clear_reset();

    //Create the semaphore
    radar_to_algorithm_xSemaphore = xSemaphoreCreateBinary();
    algorithm_to_indication_xSemaphore = xSemaphoreCreateBinary();

	/*Create Task Can*/
	create_task_radar_communication();
	create_task_vehicle_communication();
	create_task_assistant_manager();
	create_task_algorithm();
	create_task_indication();
	create_task_diagnosis();
    //create_task_radar_downloader();
	create_task_pas();

	create_task_xbr();

	// Start the scheduler
	vTaskStartScheduler();

    for ( ; ; ) {
    	//pal_lld_togglepad(PORT_LED_B_STATE, LED_B_STATE);
//		INDC_left_indicator_blink(1);
//		INDC_right_indicator_blink(1);
//        osalThreadDelayMilliseconds(100);

        static uint32_t counter = 0;
        uint8_t data[8] = {
            (counter >> 0) & 0xFF,
            (counter >> 8) & 0xFF,
            (counter >> 16) & 0xFF,
            (counter >> 24) & 0xFF,
            0x55, 0x66, 0x77, 0x88
        };
        can_lld_transmit(&CAND1, 0, 0x123, CAN_ID_STD, data, 8);
        counter++;
        pit_lld_delay_us(500000);  // 500ms µÙ∑π¿Ã


    }
}


void LSW_Init(void)
{
    pal_lld_setpad(PORT_LSW_IGN_EN, LSW_IGN_EN);		//5V ON
    pal_lld_setpad(PORT_LSW_ACC_EN, LSW_ACC_EN);		//
    pal_lld_setpad(PORT_LSW_EXT_EN, LSW_EXT_EN);		//
    pal_lld_setpad(PORT_LSW_FX3_EN, LSW_FX3_EN);		//USB Enable
    pal_lld_setpad(PORT_LSW_CAN_EN, LSW_CAN_EN);		//
    pal_lld_setpad(PORT_LSW_XCVR_EN, LSW_XCVR_EN);	    //
    pal_lld_setpad(PORT_LSW_PAS_EN, LSW_PAS_EN);	    //

    //CAN STB PIN
    pal_lld_clearpad(PORT_RDY_CAN1_VEH, RDY_CAN1_VEH); // STB PIN CAN1_VEH
    pal_lld_clearpad(PORT_RDY_CAN7_LEFT, RDY_CAN7_LEFT); // STB PIN CAN7_LEFT/RIGHT
    //pal_lld_clearpad(PORT_RDY_CAN9_RIGHT, RDY_CAN9_RIGHT); //Error. GPI Port

    //PAS STB PIN
    pal_lld_setpad(PORT_GPIO_WKP_PAS, GPIO_WKP_PAS);
    pal_lld_setpad(PORT_GPIO_SLP_PAS, GPIO_SLP_PAS);

    //INDICATE
    pal_lld_clearpad(PORT_LSW_INDL_EN, LSW_INDL_EN);	    //
    pal_lld_clearpad(PORT_LSW_INDR_EN, LSW_INDR_EN);	    //

	/*Load Switch mmWaveRadar*/
    pal_lld_clearpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);	//pc05
    pal_lld_clearpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);		//pc06
    pal_lld_clearpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);	//pc07
    pal_lld_clearpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);		//pc08

//    pal_lld_setpad(PORT_LSW_BSIS_L_EN,LSW_BSIS_L_EN);	//pc05
//    pal_lld_setpad(PORT_LSW_BSD_L_EN,LSW_BSD_L_EN);		//pc06
//    pal_lld_setpad(PORT_LSW_BSIS_R_EN,LSW_BSIS_R_EN);	//pc07
//    pal_lld_setpad(PORT_LSW_BSD_R_EN,LSW_BSD_R_EN);		//pc08
}

