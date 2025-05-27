#include "can_lld.h"
#include "vehicle_communication_parameter.h"
#include "vehicle_communication_setting.h"

/* 
 ***Maintaining order is crucial within the vehicle_info_t  
 */
typedef struct 
{
	uint16_t  ign;  
	uint16_t  gear;
	uint16_t  speed;
	uint16_t  hazard;
	uint16_t  wheel;
	uint16_t  right_turn_signal;
	uint16_t  left_turn_signal;
	uint16_t  brake; 
	uint16_t  driver_doors_switch;      /*dirver door switch*/
	uint16_t  assist_doors_switch; 		/*assit door switch*/
} vehicle_info_t;

typedef struct 
{
	uint32_t id;
	uint16_t item_idx;
	uint8_t  start_bit;
	uint8_t  len;
} can_id_info_t;

typedef struct 
{
	uint8_t  	  manufacture[10]; 
	uint8_t  	  model[8];
	uint16_t 	  year;           				//production year
	uint16_t  	  baudrate[2];    				//can communication speed
	uint16_t  	  samplepoint[2]; 				//can sample point
	can_id_info_t can_id_info[MAX_NUM_CAN_ID]; //parsing infomation
} can_list_t;

typedef struct
{
	uint8_t			total;   /*The number of total protocl*/
	uint8_t 		select;  /*The protocol in currently using */
	can_list_t  	can_list[TOTAL_CAN_PROTOCOL]; /*each protocol structure*/
} can_desc_t;

typedef struct
{
	uint8_t netural_number;
	uint8_t parking_number;
	uint8_t drive_number;
	uint8_t reverse_number;
	uint8_t vehicle_comm_checker;
} gear_t;

/*byte or bit swap function*/
uint16_t swap2(uint16_t sval_in);
uint32_t swap4(uint32_t sval_in);
uint64_t rvs_bits(uint64_t val);

/*can byte data decoding/encoding*/
uint16_t extractbit(uint8_t* data, int from, int size);
uint8_t insertbit(uint8_t* data, int from, int size);
void set_radar_image_data(uint8_t *radar_image);
void send_radar_image(uint8_t buffer_num, uint8_t packet_type);
uint8_t (*get_radar_image_data(uint8_t radar_image_buf_num))[MAX_RADAR_IMAGE_BUF_SIZE];
void set_clear_fwupdate(void);

extern can_list_t *getCanInfo(void);
extern vehicle_info_t *get_vehicle_info(void);
extern void vc_polling_get_vehicle_infomation(void);
extern uint8_t get_vehicle_diag(void);
extern void vc_can_initialize(void);
extern void set_clear_reset(void);
extern void vc_can_Broadcast(void);

void vc_set_control_CAN_task(uint8_t mode);
int vc_set_CAN_bitRate(uint8_t type);
void vc_set_CAN_filter(void);
uint16_t vc_can_lld_read_buf(CANDriver *canp, CANRxFrame *crfp);
uint32_t vc_tx_cwi1_packet_t(uint8_t *CWI1_data);
uint32_t vc_tx_cwi2_packet_t(uint8_t *CWI2_data);
uint32_t vc_tx_dtc_packet_t(uint8_t id_flag, uint8_t *dtc_data);
uint32_t vc_tx_bsd_packet_t(uint8_t *tx_data);
extern uint32_t vc_tx_test_packet_t(uint32_t ID, uint8_t *tx_data);
void vc_tx_update_response_packet_t(uint8_t *update_response_msg);

// fwupdate functions
uint32_t rc_tx_radar_setup_mode_t(void);
uint32_t rc_tx_fw_image_flash_request_t(void);
uint32_t rc_tx_fw_image_packet_t(uint8_t *radar_image, uint8_t message_type);
extern void rc_FW_Update_Send(void);


// xbr
extern void vc_xbr_packet(void);
uint32_t vc_tx_xbr_active_packet_t(void);
uint32_t vc_tx_xbr_passive_packet_t(void);

// System Global Parameters Init
extern uint8_t g_xbf_active_flag;
extern uint8_t g_radar_fw_update_flag;
