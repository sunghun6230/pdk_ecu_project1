/*
 * Configuration for Task.
 * Definition of Priority and Stack Size for each Task
 */
#include "FreeRTOS.h"
#include "task.h"

#define PRIORITY_VEHICLE_COMMUNICATION 		6
#define PRIORITY_RADAR_COMMUNICATION 		8
#define PRIORITY_RADAR_DOWNLOADER 			4
#define PRIORITY_CLI_TASK 					3
#define PRIORITY_ASSISTANT_TASK_MANAGER 	2 
#define PRIORITY_ALGORITHM					7
#define PRIORITY_INDICATION					5
#define PRIORITY_DIAGNOSIS					1
#define PRIORITY_PAS_COMMUNICATION 			9
#define PRIORITY_XBR			 			10 	// temporary setting

//#define PRIORITY_WARNING_CAN_TASK 			3


#define STACK_VEHICLE_COMMUNICATION 		configMINIMAL_STACK_SIZE*2
#define STACK_RADAR_DOWNLOADER 				configMINIMAL_STACK_SIZE
#define STACK_RADAR_COMMUNICATION 			configMINIMAL_STACK_SIZE*2
#define STACK_ASSISTANT_TASK_MANAGER 		configMINIMAL_STACK_SIZE
#define STACK_SIZE_CLI_TASK 				configMINIMAL_STACK_SIZE*4	
#define STACK_RADAR_ALGORITHM				configMINIMAL_STACK_SIZE*8
#define STACK_INDICATION					configMINIMAL_STACK_SIZE
#define STACK_DIAGNOSIS						configMINIMAL_STACK_SIZE

typedef struct {
	TaskHandle_t task_warning_handle;
	TaskHandle_t task_algorithm_handle;
	TaskHandle_t task_radar_communication_handle;
	TaskHandle_t task_vehicle_communication_handle;
	TaskHandle_t task_command_interface_handle;
	TaskHandle_t task_radar_downloader_handle;
	TaskHandle_t task_assistant_handle;
	TaskHandle_t task_diagnosis_handle;
	TaskHandle_t task_indication_handle;
	TaskHandle_t task_pas_communication_handle;
} task_handle_t;



#define TASK_DELAY_VEHCLE_COMMUNICATION					200
#define TASK_DELAY_VEHCLE_COMMUNICATION_FWUPDATE		1
#define TASK_DELAY_COTROL_TASK							50
#define TASK_DELAY_RADAR_DOWNLOADER						40
#define TASK_DELAY_RADAR_COMMUNICATION					1
#define TASK_DELAY_ALGORITHM							10
#define TASK_DELAY_ASSISTANC_TASK						100
#define TASK_DELAY_DIAGNOSIS_TASK						1000
#define TASK_DELAY_WARNING_INDICATE						50
#define TASK_DELAY_PAS_COMMUNICATION					60
//#define TASK_DELAY_PAS_COMMUNICATION					30

#define TASK_Active_DELAY_XBR							20
#define TASK_Passive_DELAY_XBR							200

/*functions of task creation*/

void create_task_assistant_manager(void);
void create_task_vehicle_communication(void);
void create_task_radar_communication(void);
void create_task_can_warining(void);
void create_task_radar_downloader(void);
void create_command_line_interface(void);
void create_task_algorithm(void);
void create_task_warning(void);
void create_task_diagnosis(void);
void create_task_indication(void);
void create_task_pas(void);
void create_task_xbr(void);
