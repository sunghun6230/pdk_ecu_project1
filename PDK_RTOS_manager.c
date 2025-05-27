/*
 * rtos_task_manager.c
 *
 *  Created on: 2023. 8. 30.
 *      Author:
 */
#include <FreeRTOS.h>
#include <task.h>
#include "PDK_RTOS_manager.h"
#include "command_interface.h"
#include "downloader.h"
#include "radar_communication.h"
#include "vehicle_communication.h"
#include "serial_lld.h"
#include "spi_lld_cfg.h"
#include "algorithm.h"
#include "diagnosis.h"

#include "pas_communication.h"


task_handle_t task_handle;
task_handle_t *get_task_handle(void)
{
    return (&task_handle);
}

#if CLI_STATIC_STACK
UBaseType_t cliWaterMark;
StaticTask_t cliTaskTCB;
StackType_t cliTaskStack[ STACK_SIZE_CLI_TASK ];
#endif                           

#if RADAR_STATIC_STACK
UBaseType_t radarWaterMark;
StaticTask_t radarTaskTCB;
StackType_t radarTaskStack[ STACK_SIZE_RADAR_TASK ];
#endif

#if CLI_STATIC_STACK | RADAR_STATIC_STACK
/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
}
#endif

portTASK_FUNCTION( task_diagnosis, pvParameters )
{
	( void ) pvParameters;

	// System Check
	// Radar, PAS, Indicator Check

	for ( ; ; )
	{
		// radar fault manage, radar reset
		// 안써도 상관 없을거 같기도, 실제로 없어도 잘 돌아감.
		// 실제로 동작하는게 없는거 같긴함.
//        diag_read_of_adc_e_fuse();
//        dg_radar_fault();
//        diag_mmwave_sync();

		diag_communication();
		vTaskDelay(TASK_DELAY_DIAGNOSIS_TASK);
	}
}

portTASK_FUNCTION( task_algorithm, pvParameters )
{
	( void ) pvParameters;

	for ( ; ; )
	{
        ALG_radar_algorithm();
		//vTaskDelay(TASK_DELAY_ALGORITHM);
	}
}

portTASK_FUNCTION( task_indication, pvParameters )
{
	( void ) pvParameters;

	for ( ; ; )
	{
		wrn_warning_indication();
		vTaskDelay(TASK_DELAY_WARNING_INDICATE);
	}
}

portTASK_FUNCTION( task_assistant_manager, pvParameters )
{
	( void ) pvParameters;

	for ( ; ; )
	{
		cli_task_control();
        //dw_task_control();
		vTaskDelay(TASK_DELAY_ASSISTANC_TASK);
	}
}

portTASK_FUNCTION( task_vehicle_communication, pvParameters )
{   
	( void ) pvParameters;

	// System Start Sound play Request
	// if not need this, remove here
	uint8_t cwi1_system_start_sound[8] = {0, 0, 0, 0, 0, 0, 0, 4};
	vc_tx_cwi1_packet_t(cwi1_system_start_sound);

    for ( ; ; ) 
    {  
        vc_polling_get_vehicle_infomation();
        if(g_radar_fw_update_flag == 1)
        {
        	vTaskDelay(TASK_DELAY_VEHCLE_COMMUNICATION_FWUPDATE);
        }
        else
        {
        	vTaskDelay(TASK_DELAY_VEHCLE_COMMUNICATION);
        	vc_can_Broadcast();
        }
    }
}

portTASK_FUNCTION( task_radar_communication, pvParameters )
{
	( void ) pvParameters;
	  TickType_t xLastWakeTime = xTaskGetTickCount();

	for ( ; ; )
	{
//		rc_channel_rxd_pin_check();
		rc_radar_rx_done_check();
		//vTaskDelay(TASK_DELAY_RADAR_COMMUNICATION);
		vTaskDelayUntil( &xLastWakeTime, TASK_DELAY_RADAR_COMMUNICATION);
	}
}

portTASK_FUNCTION( task_command_interface, pvParameters )
{
	( void ) pvParameters;
    /* Inspect our own high water mark on entering the task. */
    for ( ; ; ) 
    { 
        cli_commnadloop();
    }
}

portTASK_FUNCTION( task_radar_downloader, pvParameters )
{    
	( void ) pvParameters;
    //UBaseType_t uxHighWaterMark;
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */ 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

    //suspend task not to execute while xds don't use 
    //vTaskSuspend(xXdsHandle);
    for ( ; ; ) 
    {   
        if (!g_radar_fw_update_flag) // should_stop_task는 종료 조건을 나타내는 변수
        {
            vTaskDelete(NULL); // 자신을 삭제
        }

//        dw_set_radar_download();
    	rc_FW_Update_Send();
        vTaskDelay(TASK_DELAY_RADAR_DOWNLOADER);
    }
}

portTASK_FUNCTION( task_pas_communication, pvParameters )
{
	( void ) pvParameters;

	vTaskDelay(200);
	pas_sensor_start();

	vTaskDelay(100);
	for ( ; ; )
	{
		pas_read();
		vTaskDelay(TASK_DELAY_PAS_COMMUNICATION);
	}
}


portTASK_FUNCTION( task_xbr, pvParameters )
{
	( void ) pvParameters;

	for ( ; ; )
	{
		if ( g_xbf_active_flag )
		{
			vTaskDelay(TASK_Active_DELAY_XBR);
			vc_xbr_packet();
		}
		else
		{
			vTaskDelay(TASK_Passive_DELAY_XBR);
			vc_tx_xbr_passive_packet_t();
		}
	}
}


void create_task_command_line_interface(void)
{
    //BaseType_t xReturned;
#if !CLI_STATIC_STACK    
    /* Start the console task */
	xTaskCreate
    (
        task_command_interface,                            /* Function that implements the task.*/
	    (const char * const)"commandloop",                 /* Text name for the task.*/
	    STACK_SIZE_CLI_TASK,                               /* Stack size in words, not bytes.*/ 
	    NULL,                                              /* Parameter passed into the task.*/
	    PRIORITY_CLI_TASK,                                 /* Priority at which the task is created.*/
	    &task_handle.task_command_interface_handle         /* Handler at which the task is created.*/ 
    );
#else
	TaskHandle_t xHandle;
/* Create the task without using any dynamic memory allocation. */
    xHandle = xTaskCreateStatic(
                commandloop,       /* Function that implements the task. */
                "CommandLoop",          /* Text name for the task. */
                STACK_SIZE_CLI_TASK,      /* Number of indexes in the xStack array. */
                ( void * ) 1,    /* Parameter passed into the task. */
                PRIORITY_CLI_TASK,/* Priority at which the task is created. */
                cliTaskStack,          /* Array to use as the task's stack. */
                &cliTaskTCB );  /* Variable to hold the task's data structure. */
#endif
}

void create_task_vehicle_communication (void)
{
    xTaskCreate
    ( 
        task_vehicle_communication,                        /* Function that implements the task. */
        (const char * const)"CAN RX TASK",                 /* Text name for the task. */
        STACK_VEHICLE_COMMUNICATION,                       /* Stack size in words, not bytes. */
        ( void * ) 1,                                      /* Parameter passed into the task. */
        PRIORITY_VEHICLE_COMMUNICATION,                    /* Priority at which the task is created. */
        &task_handle.task_vehicle_communication_handle     /* Handler at which the task is created. */ 
    );
}

void create_task_radar_communication (void)
{
#if !RADAR_STATIC_STACK
	xTaskCreate
	(
		task_radar_communication,                       /* Function that implements the task. */
		(const char * const)"RADAR TRX TASK",           /* Text name for the task. */
		STACK_RADAR_COMMUNICATION,                      /* Stack size in words, not bytes. */
		NULL,                                           /* Parameter passed into the task. */
		PRIORITY_RADAR_COMMUNICATION,                   /* Priority at which the task is created. */
		&task_handle.task_radar_communication_handle    /* Handler at which the task is created. */
	);
#else    
	TaskHandle_t xHandle;
	xHandle = xTaskCreateStatic
	(
		taskradartrx,               /* Function that implements the task. */
		"RADAR TRX TASK",           /* Text name for the task. */
		STACK_SIZE_RADAR_TASK,      /* Number of indexes in the xStack array. */
		( void * ) 1,               /* Parameter passed into the task. */
		PRIORITY_RADAR_TASK,        /* Priority at which the task is created. */
		radarTaskStack,             /* Array to use as the task's stack. */
		&radarTaskTCB		 		/* Variable to hold the task's data structure. */
	);
#endif

}

void create_task_radar_downloader(void)
{
    xTaskCreate
    ( 
        task_radar_downloader,                                  /* Function that implements the task. */
        (const char * const)"create_task_radar_downloader",     /* Text name for the task. */
        STACK_RADAR_DOWNLOADER,                                 /* Stack size in words, not bytes. */
        ( void * ) 1,                                           /* Parameter passed into the task. */
        PRIORITY_RADAR_DOWNLOADER,                              /* Priority at which the task is created. */
        &task_handle.task_radar_downloader_handle 
    );
}

/* The create_task_manager is to manage tasks of cli and radar download
 */
void create_task_assistant_manager(void)
{    
    xTaskCreate
    ( 
    	task_assistant_manager,                       /* Function that implements the task. */
        (const char * const)"assistant_task_manager", /* Text name for the task. */
		STACK_ASSISTANT_TASK_MANAGER,        		  /* Stack size in words, not bytes. */
        ( void * ) 1,                                 /* Parameter passed into the task. */
        PRIORITY_ASSISTANT_TASK_MANAGER,              /* Priority at which the task is created. */
        &task_handle.task_assistant_handle 
    ); 
}

void create_task_algorithm(void)
{
    xTaskCreate
    ( 
        task_algorithm,                               /* Function that implements the task. */
        (const char * const)"task_algorithm",         /* Text name for the task. */
		STACK_RADAR_ALGORITHM,        		          /* Stack size in words, not bytes. */
        ( void * ) 1,                                 /* Parameter passed into the task. */
        PRIORITY_ALGORITHM,                           /* Priority at which the task is created. */
        &task_handle.task_algorithm_handle 
    ); 
}

void create_task_indication (void)
{
	xTaskCreate
	(
		task_indication,                               	/* Function that implements the task. */
		(const char * const)"task_indication",         	/* Text name for the task. */
		STACK_INDICATION,        		          		/* Stack size in words, not bytes. */
		( void * ) 1,                                 	/* Parameter passed into the task. */
		PRIORITY_INDICATION,                           	/* Priority at which the task is created. */
		&task_handle.task_indication_handle
	);
}

void create_task_diagnosis(void)
{
//    diag_init_initialize();
	xTaskCreate
	(
		task_diagnosis,                               	/* Function that implements the task. */
		(const char * const)"task_diagnosis",         	/* Text name for the task. */
		STACK_INDICATION,        		          		/* Stack size in words, not bytes. */
		( void * ) 1,                                 	/* Parameter passed into the task. */
		PRIORITY_DIAGNOSIS,                           	/* Priority at which the task is created. */
		&task_handle.task_diagnosis_handle
	);
}

void create_task_pas(void)
{
//    diag_init_initialize();
	xTaskCreate
	(
		task_pas_communication,                               	/* Function that implements the task. */
		(const char * const)"task_pas_communication",         	/* Text name for the task. */
		STACK_INDICATION,        		          				/* Stack size in words, not bytes. */
		( void * ) 1,                                 			/* Parameter passed into the task. */
		PRIORITY_PAS_COMMUNICATION,                           	/* Priority at which the task is created. */
		&task_handle.task_pas_communication_handle
	);
}

void create_task_xbr(void)
{
//    diag_init_initialize();
	xTaskCreate
	(
		task_xbr,				                               	/* Function that implements the task. */
		(const char * const)"task_xbr",				         	/* Text name for the task. */
		STACK_INDICATION,        		          				/* Stack size in words, not bytes. */
		( void * ) 1,                                 			/* Parameter passed into the task. */
		PRIORITY_XBR,                           				/* Priority at which the task is created. */
		&task_handle.task_pas_communication_handle
	);
}
