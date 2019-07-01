/*(hardware debouncer) switch debouncer for push button
    bouncing effect 
    the chip works faster than the bouncing, thinks many presses
    different bouncing per button 
    add ceramic capitor to make Resistor-Capitor delay circuit to filter out the bouncing 
    smooths out curve of waveform on oscilliscope
gatt server
gpio
app specific 
BLE theory
*/

// preprocessor directives

// include preprocessor
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h" // free real-time operating system kernal
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include <string.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "esp_bt.h"				// implements bt controller, HCI config procedure for host side
#include "esp_bt_main.h"		// implements init and enabling for bluedroid stack
#include "esp_gap_ble_api.h"	// implements GAP config for advertising
#include "esp_gatts_api.h"		// implements GATT config, e.g., creating services and characteristics

#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

// define preprocessors (define macros)
#define DOTS_PER_BOARD 4

// define pins for player 1 board
#define P1_B_DOT  2
#define P1_R1_DOT 4
#define P1_R2_DOT 5
#define P1_G1_DOT 12
#define P1_G2_DOT 13

// define pins for player 2 board
#define P2_B_DOT  14
#define P2_R1_DOT 15
#define P2_R2_DOT 16
#define P2_G1_DOT 17
#define P2_G2_DOT 18

// define pins for mediator board
#define M_R1_DOT 21
#define M_R2_DOT 22
#define M_G1_DOT 23
#define M_G2_DOT 25

// define pins for mechanical push buttons
#define SWAP_RED_BUTTON 26
#define SWAP_GRE_BUTTON 27
#define JUMP_HOR_BUTTON 32
#define JUMP_VER_BUTTON 33
#define SUB_BUTTON 19


#define GATTS_SERVICE_UUID      	0x0001
#define GATTS_CHAR_NUM		    	2
// number of GATT handles
#define GATTS_NUM_HANDLE        	1 + (3 * GATTS_CHAR_NUM)
// 1 service handle
// 2 characteristic handles
// 2 characteristic value handles
// 2 characteristic descriptor values

#define BLE_DEVICE_NAME            	"DOTS_SERVER"
#define BLE_MANUFACTURER_DATA_LEN  	4
#define GATTS_CHAR_VAL_LEN_MAX		22 // define characteristics length
#define DOTS_PLAYER1_PROFILE_ID 	0

#define GATTS_TAG "GATTS"
#define BLE_SERVICE_UUID_SIZE ESP_UUID_LEN_128

#define BUTTON_PIN_SEL ((1ULL<<SWAP_RED_BUTTON) | (1ULL<<SWAP_GRE_BUTTON) | (1ULL<<JUMP_HOR_BUTTON) | (1ULL<<JUMP_VER_BUTTON) | (1<<SUB_BUTTON))


// structs and enums
// -----------------------------------------------------------

typedef enum {
	B,
    R1, 
    R2, 
    G1, 
    G2
} DOTTYPE; 

typedef enum {
	JUMP,
	SWAP
} DOTACTION;

typedef enum {
	PLAYER1, 
	PLAYER2
} PLAYER;

typedef struct {
    gpio_num_t gpio_num;
    DOTTYPE dottype; 
    int level;
} DOT;

typedef enum {
	SCORE, 
	WIN
} BLINKTYPE;

typedef DOT Board[DOTS_PER_BOARD]; 

// structure for application profile
// organizes functionality designed for client app
// each profile is seen as BLE service, client must discrminate services
// struct members 
// and have a corresponding callback function; in this case: gatts_profile_event_handler

// define application profile
typedef struct {
	// gatt interface
    uint16_t gatts_if;
	// application id
    uint16_t app_id;
	// connection id
    uint16_t conn_id;

	uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    
	uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
	// attribute permissions
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    
	// (client characteristic configuration) descriptor handle
	// for notifications or indications
	// describes if notifications and indications are enabled
	// and how char may be configured for a specific client
	uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;

} gatts_profile_inst;

typedef struct {
    // define characteristic 
	esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t char_perm;
	esp_gatt_char_prop_t char_property;
	esp_attr_value_t *char_val;
    esp_attr_control_t *char_control;
    uint16_t char_handle;
    esp_gatts_cb_t char_read_callback;
	esp_gatts_cb_t char_write_callback;
	// define descriptor
    esp_bt_uuid_t descr_uuid;
    esp_gatt_perm_t descr_perm;
	esp_attr_value_t *descr_val;
    esp_attr_control_t *descr_control;
    uint16_t descr_handle;
    esp_gatts_cb_t descr_read_callback;
	esp_gatts_cb_t descr_write_callback;
} gatts_char_inst;


// prototypes
// -----------------------------------------------------------

int create_num_of_dots();
void determine_which_dots(int num_of_dots, Board board);
void select_gpio_pads(Board board);
void set_gpio_levels(Board board);
void reset_dots(Board board);

void swap_or_jump(DOTACTION da, PLAYER player, DOTTYPE dt1, DOTTYPE dt2, Board board);
void player2_compete_task();
void reward_points(PLAYER player);

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void char_rx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char_rx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr_rx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr_rx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void char_tx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void char_tx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr_tx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void descr_tx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


// dots data 
// -----------------------------------------------------------

QueueHandle_t evt_queue; 
gpio_num_t io_num; 
uint32_t intr_status; 

int player1_points = 0;
int player2_points = 0;

DOT player1_bluedot = { .gpio_num = P1_B_DOT, .dottype = B, .level = 0 };
DOT player2_bluedot = { .gpio_num = P2_B_DOT, .dottype = B, .level = 0 };

static Board player1_board = {
    { .gpio_num = P1_R1_DOT, .dottype = R1, .level = 0 }, 
    { .gpio_num = P1_R2_DOT, .dottype = R2, .level = 0 },
    { .gpio_num = P1_G1_DOT, .dottype = G1, .level = 0 }, 
    { .gpio_num = P1_G2_DOT, .dottype = G2, .level = 0 }
}; 

static Board player2_board = {
    { .gpio_num = P2_R1_DOT, .dottype = R1, .level = 0 }, 
    { .gpio_num = P2_R2_DOT, .dottype = R2, .level = 0 },
    { .gpio_num = P2_G1_DOT, .dottype = G1, .level = 0 }, 
    { .gpio_num = P2_G2_DOT, .dottype = G2, .level = 0 }
}; 

static Board mediator_board = {
    { .gpio_num = M_R1_DOT, .dottype = R1, .level = 0 }, 
    { .gpio_num = M_R2_DOT, .dottype = R2, .level = 0 },
    { .gpio_num = M_G1_DOT, .dottype = G1, .level = 0 }, 
    { .gpio_num = M_G2_DOT, .dottype = G2, .level = 0 }
}; 

// DOT server data
// -----------------------------------------------------------

// dummy attribute values for characteristics and descriptors
// must be nonnull object and length greater than 0
uint8_t char_rx_content[GATTS_CHAR_VAL_LEN_MAX] = {0x11,0x22,0x33};
uint8_t char_tx_content[GATTS_CHAR_VAL_LEN_MAX] = {0x11,0x22,0x33};
uint8_t descr_rx_content[GATTS_CHAR_VAL_LEN_MAX] = {0x00,0x00};
uint8_t descr_tx_content[GATTS_CHAR_VAL_LEN_MAX] = "Hallo ESP32";

esp_attr_value_t char_rx_attr_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len		= sizeof(char_rx_content),
	.attr_value     = char_rx_content,
};

esp_attr_value_t char_tx_attr_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len		= sizeof(char_tx_content),
	.attr_value     = char_tx_content,
};

esp_attr_value_t descr_rx_attr_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len		= sizeof(descr_rx_content),
	.attr_value     = descr_rx_content,
};

esp_attr_value_t descr_tx_attr_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len		= 11,
	.attr_value     = descr_tx_content,
};

// for nordic uART
static uint8_t nordic_uart_service_uuid128[BLE_SERVICE_UUID_SIZE] = {
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static uint8_t ble_manufacturer[BLE_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};

static uint32_t ble_add_char_pos;
/*
typedef struct {
    bool set_scan_rsp;            /*!< Set this advertising data as scan response or not
    bool include_name;            /*!< Advertising data include device name or not 
    bool include_txpower;         /*!< Advertising data include TX power 
    int min_interval;             /*!< Advertising data show slave preferred connection min interval 
    int max_interval;             /*!< Advertising data show slave preferred connection max interval 
    int appearance;               /*!< External appearance of device 
    uint16_t manufacturer_len;    /*!< Manufacturer data length 
    uint8_t *p_manufacturer_data; /*!< Manufacturer data point 
    uint16_t service_data_len;    /*!< Service data length 
    uint8_t *p_service_data;      /*!< Service data point 
    uint16_t service_uuid_len;    /*!< Service uuid length 
    uint8_t *p_service_uuid;      /*!< Service uuid array point 
    uint8_t flag;                 /*!< Advertising flag of discovery mode, see BLE_ADV_DATA_FLAG detail 
} esp_ble_adv_data_t;
*/
// defines for advertising data
// advertising payload max 31 bytes (advertisement packet limit)
// info shown to client
static esp_ble_adv_data_t ble_adv_data = {
    .set_scan_rsp = false,							// not scan response
    .include_name = true,							// include device name 	
    .include_txpower = true,						// include tx power
    .min_interval = 0x20, // 2*16 * 1.25ms =  40ms
    .max_interval = 0x40, // 4*16 * 1.25ms =  80ms
    .appearance = 0x00,
    .manufacturer_len = BLE_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  (uint8_t *)ble_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = BLE_SERVICE_UUID_SIZE,
    .p_service_uuid = nordic_uart_service_uuid128,
	// BLE_ADV_DATA_FLAG data flag bit definition used for advertising data flag
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config required for gap to execute
static esp_ble_adv_params_t ble_adv_params = {
	// advertising interval between 20 and 40 ms
    .adv_int_min        = 0x20, // 32
    .adv_int_max        = 0x40, // 64
	// not directed to a particular device, generic
    .adv_type           = ADV_TYPE_IND,
	// public address
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
	// advertises on all channels
    .channel_map        = ADV_CHNL_ALL,
	// allow both scan and connection events from anyone
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// interface is defined when the service starts
// application profile is not linked to a client yet: ESP_GATT_IF_NONE
// implement app profile structure
gatts_profile_inst gl_profile = {
         .gatts_if = ESP_GATT_IF_NONE,       
};


// describe characteristics and descriptors
gatts_char_inst gl_char[GATTS_CHAR_NUM] = {
		{
				// RX for uART service
				.char_uuid.len = ESP_UUID_LEN_128, 
				// nordic semiconductor vendor specific UUID
				.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E },
				// char_perm, property shown to client to let know, server accepts this
				// to read / to write permitted
				// allows capability to be used
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				// char can be read, written, can notify value changes
				// offers hint of capability
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char_rx_attr_val,
				.char_control = NULL,
				.char_handle = 0,
				.char_read_callback=char_rx_read_handler,
				.char_write_callback=char_rx_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &descr_rx_attr_val,
				// since NULL need to esp_ble_gatts_send_response
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr_rx_read_handler,
				.descr_write_callback=descr_rx_write_handler
		},
		{
				// TX for uART service
				.char_uuid.len = ESP_UUID_LEN_128,  // TX
				// nordic semiconductor vendor specific UUID
				.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E },
				.char_perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				.char_val = &char_tx_attr_val,
				// attribute response control attribute
				.char_control=NULL,
				.char_handle=0,
				.char_read_callback=char_tx_read_handler,
				.char_write_callback=char_tx_write_handler,
				.descr_uuid.len = ESP_UUID_LEN_16,
				.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // ESP_GATT_UUID_CHAR_DESCRIPTION,
				.descr_perm=ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				.descr_val = &descr_tx_attr_val,
				.descr_control=NULL,
				.descr_handle=0,
				.descr_read_callback=descr_tx_read_handler,
				.descr_write_callback=descr_tx_write_handler
		}
};

static  uint16_t notify_conn_id = 0;
static  esp_gatt_if_t notify_gatts_if = NULL;
static uint8_t notify_pos=0;
static uint8_t is_notify=0;

// dots functions
// -----------------------------------------------------------

void set_bluedot_levels()
{
	player1_bluedot.level = rand() % 2;
	player2_bluedot.level = rand() % 2;
	gpio_set_level(player1_bluedot.gpio_num, player1_bluedot.level);
	gpio_set_level(player2_bluedot.gpio_num, player2_bluedot.level);
}

void reset_bluedots()
{
	player1_bluedot.level = 0;
	player2_bluedot.level = 0;
	gpio_set_level(player1_bluedot.gpio_num, player1_bluedot.level);
	gpio_set_level(player2_bluedot.gpio_num, player2_bluedot.level);
}

int create_num_of_dots()
{
    srand(time(NULL));
    short num_of_dots = 0;
    while(num_of_dots == 0) 
        num_of_dots = rand() % DOTS_PER_BOARD;
    return num_of_dots; 
}

void determine_which_dots(int num_of_dots, Board board)
{
    int pos; 
    int i; 
    int on_off[DOTS_PER_BOARD] = { 0 }; 
    on_off[rand() % DOTS_PER_BOARD] = 1;

    for (i = 0; i < num_of_dots - 1; ++i)
    {
        do {
            pos = rand() % DOTS_PER_BOARD; 
            if (on_off[pos] != 1)
            {
                on_off[pos] = 1;
                break; 
            }   
        } while(on_off[pos] == 1);
    }

    for (i = 0; i < DOTS_PER_BOARD; ++i)
        board[i].level = on_off[i];
}

void select_gpio_pads(Board board)
{
    for (int i = 0; i < DOTS_PER_BOARD; ++i)
    {
        gpio_pad_select_gpio(board[i].gpio_num);
        gpio_set_direction(board[i].gpio_num, GPIO_MODE_INPUT_OUTPUT);
    }
}

void set_gpio_levels(Board board)
{
    for (int i = 0; i < DOTS_PER_BOARD; ++i)
        gpio_set_level(board[i].gpio_num, board[i].level);
}

void reset_dots(Board board)
{
    for (int i = 0; i < DOTS_PER_BOARD; ++i)
        board[i].level = 0;
    set_gpio_levels(board);  
}

// button handlers
// -----------------------------------------------------------

void IRAM_ATTR handle_swap_red_button(void* args)
{
	intr_status = READ_PERI_REG(GPIO_STATUS_REG); 
	gpio_num_t gpio_no = (gpio_num_t) SWAP_RED_BUTTON; 
	xQueueSendToBackFromISR(evt_queue, &gpio_no, NULL); 
}

void IRAM_ATTR handle_swap_gre_button(void* args)
{
	intr_status = READ_PERI_REG(GPIO_STATUS_REG); 
	gpio_num_t gpio_no = (gpio_num_t) SWAP_GRE_BUTTON; 
	xQueueSendToBackFromISR(evt_queue, &gpio_no, NULL); 
}

void IRAM_ATTR handle_jump_hor_button(void* args)
{
	intr_status = READ_PERI_REG(GPIO_STATUS_REG); 
	gpio_num_t gpio_no = (gpio_num_t) JUMP_HOR_BUTTON; 
	xQueueSendToBackFromISR(evt_queue, &gpio_no, NULL); 
}

void IRAM_ATTR handle_jump_ver_button(void* args)
{
	intr_status = READ_PERI_REG(GPIO_STATUS_REG); 
	gpio_num_t gpio_no = (gpio_num_t) JUMP_VER_BUTTON; 
	xQueueSendToBackFromISR(evt_queue, &gpio_no, NULL); 
}

void IRAM_ATTR handle_sub_button(void* args)
{
	intr_status = READ_PERI_REG(GPIO_STATUS_REG); 
	gpio_num_t gpio_no = (gpio_num_t) SUB_BUTTON; 
	xQueueSendToBackFromISR(evt_queue, &gpio_no, NULL); 
}

// dots server functions
// -----------------------------------------------------------

void char_rx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[0].char_val!=NULL) {

		rsp.attr_value.len = gl_char[0].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[0].char_val->attr_len&&pos<gl_char[0].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[0].char_val->attr_value[pos];
		}
	}

	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
								ESP_GATT_OK, &rsp);
}

void char_tx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
 
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[1].char_val!=NULL) {
	
		rsp.attr_value.len = gl_char[1].char_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[1].char_val->attr_len&&pos<gl_char[1].char_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[1].char_val->attr_value[pos];
		}
	}
	
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
								ESP_GATT_OK, &rsp);
}

void descr_rx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[0].descr_val!=NULL) {
	
		rsp.attr_value.len = gl_char[0].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[0].descr_val->attr_len&&pos<gl_char[0].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[0].descr_val->attr_value[pos];
		}
	}

	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
								ESP_GATT_OK, &rsp);
}

void descr_tx_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (gl_char[1].descr_val!=NULL) {

		rsp.attr_value.len = gl_char[1].descr_val->attr_len;
		for (uint32_t pos=0;pos<gl_char[1].descr_val->attr_len&&pos<gl_char[1].descr_val->attr_max_len;pos++) {
			rsp.attr_value.value[pos] = gl_char[1].descr_val->attr_value[pos];
		}
	}

	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
								ESP_GATT_OK, &rsp);
}

void char2_notify_handle(esp_gatt_if_t gatts_if, uint16_t conn_id) {
	if (is_notify==1) {
		notify_pos='0';
		for ( uint32_t i = 0; i < 10 ;i++) {
	
			esp_ble_gatts_send_indicate(gatts_if, conn_id, gl_char[1].char_handle,1,&notify_pos,false);
			notify_pos++;
		}
	}
}

void player1_compete_handler(gatts_char_inst inst)
{
	if (strncmp((const char *)inst.char_val->attr_value, "G", 1) == 0)
	{
		swap_or_jump(SWAP, PLAYER1, G1, G2, player1_board);
	}	
	else if (strncmp((const char *)inst.char_val->attr_value, "R", 1) == 0)
	{
		swap_or_jump(SWAP, PLAYER1, R1, R2, player1_board);
	}
	else if (strncmp((const char *)inst.char_val->attr_value, "H", 1) == 0)
	{
		swap_or_jump(SWAP, PLAYER1, R1, G2, player1_board);
	}
	else if (strncmp((const char *)inst.char_val->attr_value, "V", 1) == 0)
	{
		swap_or_jump(SWAP, PLAYER1, R2, G1, player1_board);
	}
	else if (strncmp((const char *)inst.char_val->attr_value, "S", 1) == 0)
	{
		reward_points(PLAYER1);
	}
}

void char_rx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	if (gl_char[0].char_val != NULL) {
	
		gl_char[0].char_val->attr_len = param->write.len;
		
		for (uint32_t pos = 0; pos < param->write.len; pos++) 
		{
			gl_char[0].char_val->attr_value[pos] = param->write.value[pos];
		}
	}

	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	
	player1_compete_handler(gl_char[0]);
}

void char_tx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	

	if (gl_char[1].char_val!=NULL) {
		
		gl_char[1].char_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[1].char_val->attr_value[pos]=param->write.value[pos];
		}
	}

    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

void descr_rx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	if (gl_char[0].descr_val!=NULL) {
	
		gl_char[0].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[0].descr_val->attr_value[pos]=param->write.value[pos];
		}
	}

    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

void descr_tx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


	if (gl_char[1].descr_val!=NULL) {
	
		gl_char[1].descr_val->attr_len = param->write.len;
		for (uint32_t pos=0;pos<param->write.len;pos++) {
			gl_char[1].descr_val->attr_value[pos]=param->write.value[pos];
		}
		is_notify = gl_char[1].descr_val->attr_value[0];
	
	}
	
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

void gatts_check_add_char(esp_bt_uuid_t char_uuid, uint16_t attr_handle) {


	if (attr_handle != 0) {
	

		gl_char[ble_add_char_pos].char_handle=attr_handle;

		// is there a descriptor to add ?
		if (gl_char[ble_add_char_pos].descr_uuid.len!=0 && gl_char[ble_add_char_pos].descr_handle==0) {
		
			esp_ble_gatts_add_char_descr(gl_profile.service_handle, &gl_char[ble_add_char_pos].descr_uuid,
					gl_char[ble_add_char_pos].descr_perm, gl_char[ble_add_char_pos].descr_val, gl_char[ble_add_char_pos].descr_control);
		} else {
			for (uint32_t pos = 0; pos < GATTS_CHAR_NUM; pos++) 
			{
				if (gl_char[pos].char_handle == 0) 
				{
					ble_add_char_pos = pos;
					esp_ble_gatts_add_char(
						gl_profile.service_handle, 
						&gl_char[pos].char_uuid,
						gl_char[pos].char_perm,
						gl_char[pos].char_property,
						gl_char[pos].char_val, 
						gl_char[pos].char_control
					);
					break;
				}
			}
		}
	}
}

void gatts_check_add_descr(esp_bt_uuid_t descr_uuid, uint16_t attr_handle) {

	if (attr_handle != 0) 
	{
		gl_char[ble_add_char_pos].descr_handle=attr_handle;
	}
	for (uint32_t pos = 0; pos < GATTS_CHAR_NUM; pos++) 
	{
		if (gl_char[pos].char_handle == 0) 
		{
			ble_add_char_pos = pos;
			esp_ble_gatts_add_char(
				gl_profile.service_handle, 
				&gl_char[pos].char_uuid,
				gl_char[pos].char_perm,
				gl_char[pos].char_property,
				gl_char[pos].char_val, 
				gl_char[pos].char_control
			);
			break;
		}
	}
}

void gatts_check_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	
	uint16_t handle=0;
	
	uint8_t read=1;

    switch (event) {
		
		case ESP_GATTS_READ_EVT: 
			read=1;
			handle=param->read.handle;
			break;
		
		
		case ESP_GATTS_WRITE_EVT: 
			read=0;
			handle=param->write.handle;
		
		default:
			break;
    }

	for (uint32_t pos = 0; pos < GATTS_CHAR_NUM; pos++) 
	{
		if (gl_char[pos].char_handle==handle) 
		{
			if (read==1) 
			{
				if (gl_char[pos].char_read_callback!=NULL) 
				{
					gl_char[pos].char_read_callback(event, gatts_if, param);
				}
			} 
			else 
			{
				if (gl_char[pos].char_write_callback!=NULL) 
				{
					gl_char[pos].char_write_callback(event, gatts_if, param);
				}
			}
			break;
		}
		if (gl_char[pos].descr_handle==handle) 
		{
			if (read==1) 
			{
				if (gl_char[pos].descr_read_callback!=NULL) 
				{
					gl_char[pos].descr_read_callback(event, gatts_if, param);
				}
			} 
			else 
			{
				if (gl_char[pos].descr_write_callback!=NULL) 
				{
					gl_char[pos].descr_write_callback(event, gatts_if, param);
				}
			}

			break;
		}
	}
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    
	// start advertising once setting completed
	// if advert started successfully this event is generated
	// make advertising is occurring
	if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT)
	{
		// start advertising
		// pass in advertising parameters required by stack to operate
		// makes server start advertising
		esp_ble_gap_start_advertising(&ble_adv_params);
	}
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	
	switch (event) 
	{
		// register event
		// register application event is first one called during life of program
		// configures advertising parameters upon registration
		case ESP_GATTS_REG_EVT:
			
			gl_profile.service_id.is_primary = true;
			gl_profile.service_id.id.inst_id = 0x00;
			gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
			for (uint8_t pos = 0; pos < ESP_UUID_LEN_128; pos++) 
			{
				gl_profile.service_id.id.uuid.uuid.uuid128[pos] = nordic_uart_service_uuid128[pos];
			}

			// set device name 
			esp_ble_gap_set_device_name(BLE_DEVICE_NAME);

			// configure bluetooth advertising data; calls gap profile event handler
			// configs advert data that will be advertised to client
			esp_ble_gap_config_adv_data(&ble_adv_data);

			// creates service; get the create event
			esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
			
			break;

		// triggered when a service is successfully created
		case ESP_GATTS_CREATE_EVT:

			// service handle generated by BLE stack and stored
			// will be used by application layer to refer to his service
			gl_profile.service_handle = param->create.service_handle;

			// set uuid of char and uuid length
			gl_profile.char_uuid.len = gl_char[0].char_uuid.len;
			gl_profile.char_uuid.uuid.uuid16 = gl_char[0].char_uuid.uuid.uuid16;

			// start service with previously generated service handle
			// triggers ESP_GATTS_START_EVT
			esp_ble_gatts_start_service(gl_profile.service_handle);
			
			// add characteristics to service
			// triggers ESP_GATTS_ADD_CHAR_EVT
			for (uint32_t pos = 0; pos < GATTS_CHAR_NUM; pos++) 
			{
				if (gl_char[pos].char_handle == 0) 
				{
					ble_add_char_pos = pos;
					esp_ble_gatts_add_char(
						gl_profile.service_handle, 
						&gl_char[pos].char_uuid,
						gl_char[pos].char_perm,
						gl_char[pos].char_property,
						gl_char[pos].char_val, 
						gl_char[pos].char_control
					);
					break;
				}
			}
					
			break;
		// triggered by esp_ble_gatts_start_service
		// params: status, attr_handle, service_handle, char_uuid
		// handle is generated by the stack for the char just added
		case ESP_GATTS_ADD_CHAR_EVT: 

			// store stack generated handle in profile
			gl_profile.char_handle = param->add_char.attr_handle;

			if (param->add_char.status==ESP_GATT_OK) {
				gatts_check_add_char(param->add_char.char_uuid, param->add_char.attr_handle);
			}
			break;
		
		case ESP_GATTS_ADD_CHAR_DESCR_EVT:

			if (param->add_char_descr.status==ESP_GATT_OK) 
			{
				gatts_check_add_descr(param->add_char.char_uuid, param->add_char.attr_handle);
			}

			break;
		// manage read events
		case ESP_GATTS_READ_EVT: 		
			gatts_check_callback(event, gatts_if, param);
			break;
		// manage write events
		case ESP_GATTS_WRITE_EVT: 
			gatts_check_callback(event, gatts_if, param);
			break;
		case ESP_GATTS_EXEC_WRITE_EVT:
		case ESP_GATTS_MTU_EVT:
		case ESP_GATTS_CONF_EVT:
		case ESP_GATTS_UNREG_EVT:			
			break;
		case ESP_GATTS_ADD_INCL_SRVC_EVT:	
			break;
		case ESP_GATTS_DELETE_EVT:
			break;
		case ESP_GATTS_START_EVT:			
			break;
		case ESP_GATTS_STOP_EVT:
			break;
		// is triggered when a client has connected to the GATT server
		case ESP_GATTS_CONNECT_EVT:
			gl_profile.conn_id = param->connect.conn_id;
			break;
		case ESP_GATTS_DISCONNECT_EVT:
			esp_ble_gap_start_advertising(&ble_adv_params);
			break;
		case ESP_GATTS_OPEN_EVT:
		case ESP_GATTS_CANCEL_OPEN_EVT:
		case ESP_GATTS_CLOSE_EVT:
		case ESP_GATTS_LISTEN_EVT:
		case ESP_GATTS_CONGEST_EVT:	
		default:
			break;
    }
}

// captures ESP_GATTS_REG_EVT
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    
	// when app profile is registered
	// ESP_GATTS_REG_EVT is triggiered
	// params: esp_gatt_status t, app_id, gatt interface (assigned by BLE stack)

	// gatts is called once gap registered
    if (event == ESP_GATTS_REG_EVT) 
	{
        if (param->reg.status == ESP_GATT_OK) 
		{
			// store GATT interface in profile
        	gl_profile.gatts_if = gatts_if;
        } 
		else 
		{
            return;
        }
    }

    gatts_profile_event_handler(event, gatts_if, param);
}



void swap_or_jump(DOTACTION da, PLAYER player, DOTTYPE dt1, DOTTYPE dt2, Board board)
{

	if (da == JUMP && player == PLAYER1 && player1_bluedot.level == 0) return; 
	if (da == JUMP && player == PLAYER2 && player2_bluedot.level == 0) return; 

	for (int i = 0; i < DOTS_PER_BOARD; ++i)
	{
		if (board[i].dottype == dt1 && board[i].level == 1)
		{
			for (int j = 0; j < DOTS_PER_BOARD; ++j)
			{
				if (board[j].dottype == dt2 && board[j].level == 0)
				{
					board[i].level = 0;
					board[j].level = 1;
					gpio_set_level(board[i].gpio_num, board[i].level);
					gpio_set_level(board[j].gpio_num, board[j].level);

				}
			}
			break;
		}
		else if (board[i].dottype == dt2 && board[i].level == 1)
		{
			for (int j = 0; j < DOTS_PER_BOARD; ++j)
			{
				if (board[j].dottype == dt1 && board[j].level == 0)
				{
					board[i].level = 0;
					board[j].level = 1;
					gpio_set_level(board[i].gpio_num, board[i].level);
					gpio_set_level(board[j].gpio_num, board[j].level);
				}
			}
			break;
		}
	}
}

void blinking(BLINKTYPE blink_type, PLAYER player)
{
	switch(blink_type)
	{
		case SCORE:
			for (int i = 0; i < DOTS_PER_BOARD * 3; ++i)
			{
				if (player == PLAYER1) 
				{ 
					gpio_set_level(player1_board[i % DOTS_PER_BOARD].gpio_num, 1);
					vTaskDelay(50 / portTICK_PERIOD_MS);
					gpio_set_level(player1_board[i % DOTS_PER_BOARD].gpio_num, 0);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}
				else if (player == PLAYER2) 
				{ 
					gpio_set_level(player2_board[i % DOTS_PER_BOARD].gpio_num, 1);
					vTaskDelay(50 / portTICK_PERIOD_MS);
					gpio_set_level(player2_board[i % DOTS_PER_BOARD].gpio_num, 0);
					vTaskDelay(50 / portTICK_PERIOD_MS);
				}
			}
			break;
		case WIN:
			for (int i = 0; i < DOTS_PER_BOARD * 3; ++i)
			{
				gpio_set_level(mediator_board[i % DOTS_PER_BOARD].gpio_num, 1);
				vTaskDelay(50 / portTICK_PERIOD_MS);
				gpio_set_level(mediator_board[i % DOTS_PER_BOARD].gpio_num, 0);
				vTaskDelay(50 / portTICK_PERIOD_MS);
			}
		default:
			break;
	}
}


void reward_points(PLAYER player)
{
	int possible_points = 0;
	bool locked_out = true; 

	for (int i = 0; i < DOTS_PER_BOARD; ++i)
	{
		possible_points += mediator_board[i].level; 
	}

	if (player == PLAYER1)
	{	

		for (int i = 0; i< DOTS_PER_BOARD; i++)
		{
			if (player1_board[i].level == 1)
				locked_out = false;
		}
		
		if (locked_out) return; 

		for (int i = 0; i < DOTS_PER_BOARD; ++i)
		{
			if (player1_board[i].level != mediator_board[i].level)
			{
				reset_dots(player1_board);
				player1_points -= possible_points;
				printf("player 1: %d\n", player1_points);
				return;
			}
		}
		reset_dots(player2_board);
		blinking(SCORE, PLAYER1);
		player1_points += possible_points; 
		printf("player 1: %d\n", player1_points);
	} 
	else if (player == PLAYER2)
	{
		for (int i = 0; i< DOTS_PER_BOARD; i++)
		{
			if (player2_board[i].level == 1)
				locked_out = false;
		}
		
		if (locked_out) return; 

		for (int i = 0; i < DOTS_PER_BOARD; ++i)
		{
			if (player2_board[i].level != mediator_board[i].level)
			{
				reset_dots(player2_board);
				player2_points -= possible_points;
				printf("player 2: %d\n", player2_points);
				return;
			}
		}
		reset_dots(player1_board);
		blinking(SCORE, PLAYER2);
		player2_points += possible_points; 
		printf("player 2: %d\n", player2_points);
	}
	return;      
}

void player2_compete_task()
{
	int b_swap_red, b_swap_gre, b_jump_hor, b_jump_ver, b_submit;
	
	for (;;)
	{
		if (xQueueReceive(evt_queue, &io_num, (TickType_t) 1))
		{
			b_swap_red = gpio_get_level(SWAP_RED_BUTTON);
			b_swap_gre = gpio_get_level(SWAP_GRE_BUTTON);
			b_jump_hor = gpio_get_level(JUMP_HOR_BUTTON);
			b_jump_ver = gpio_get_level(JUMP_VER_BUTTON);
			b_submit  = gpio_get_level(SUB_BUTTON);

			if (b_swap_red) swap_or_jump(SWAP, PLAYER2, R1, R2, player2_board);
			if (b_swap_gre) swap_or_jump(SWAP, PLAYER2, G1, G2, player2_board);
			if (b_jump_hor) swap_or_jump(JUMP, PLAYER2, R1, G2, player2_board);
			if (b_jump_ver) swap_or_jump(JUMP, PLAYER2, R2, G1, player2_board);
			if (b_submit) reward_points(PLAYER2);
		}
	}
}




// BLE 
// ---------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------

void app_main()
{

    nvs_flash_init();

    // init and enable controller

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
	
	
/*
	#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {                              \
    .controller_task_stack_size = ESP_TASK_BT_CONTROLLER_STACK,            \
    .controller_task_prio = ESP_TASK_BT_CONTROLLER_PRIO,                   \
    .hci_uart_no = BT_HCI_UART_NO_DEFAULT,                                 \
    .hci_uart_baudrate = BT_HCI_UART_BAUDRATE_DEFAULT,                     \
    .scan_duplicate_mode = SCAN_DUPLICATE_MODE,                            \
    .scan_duplicate_type = SCAN_DUPLICATE_TYPE_VALUE,                      \
    .normal_adv_size = NORMAL_SCAN_DUPLICATE_CACHE_SIZE,                   \
    .mesh_adv_size = MESH_DUPLICATE_SCAN_CACHE_SIZE,                       \
    .send_adv_reserved_size = SCAN_SEND_ADV_RESERVED_SIZE,                 \
    .controller_debug_flag = CONTROLLER_ADV_LOST_DEBUG_BIT,                \
    .mode = BTDM_CONTROLLER_MODE_EFF,                                      \
    .ble_max_conn = CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF,                     \
    .bt_max_acl_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF,           \
    .bt_sco_datapath = CONFIG_BTDM_CTRL_BR_EDR_SCO_DATA_PATH_EFF,          \
    .bt_max_sync_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF,         \
    .ble_sca = CONFIG_BTDM_BLE_SLEEP_CLOCK_ACCURACY_INDEX_EFF,             \
    .magic = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,                           \
};


typedef struct {
    
    Following parameters can be configured runtime, when call esp_bt_controller_init()
     
    uint16_t controller_task_stack_size;    /*!< Bluetooth controller task stack size 
    uint8_t controller_task_prio;           /*!< Bluetooth controller task priority 
    uint8_t hci_uart_no;                    /*!< If use UART1/2 as HCI IO interface, indicate UART number 
    uint32_t hci_uart_baudrate;             /*!< If use UART1/2 as HCI IO interface, indicate UART baudrate 
    uint8_t scan_duplicate_mode;            /*!< scan duplicate mode 
    uint8_t scan_duplicate_type;            /*!< scan duplicate type 
    uint16_t normal_adv_size;               /*!< Normal adv size for scan duplicate 
    uint16_t mesh_adv_size;                 /*!< Mesh adv size for scan duplicate 
    uint16_t send_adv_reserved_size;        /*!< Controller minimum memory value 
    uint32_t  controller_debug_flag;        /*!< Controller debug log flag 
    uint8_t mode;                           /*!< Controller mode: BR/EDR, BLE or Dual Mode 
    uint8_t ble_max_conn;                   /*!< BLE maximum connection numbers 
    uint8_t bt_max_acl_conn;                /*!< BR/EDR maximum ACL connection numbers 
    uint8_t bt_sco_datapath;                /*!< SCO data path, i.e. HCI or PCM module 
    uint8_t bt_max_sync_conn;               /*!< BR/EDR maximum ACL connection numbers. Effective in menuconfig 
    uint8_t ble_sca;                        /*!< BLE low power crystal accuracy index 
    uint32_t magic;                         /*!< Magic number 

} esp_bt_controller_config_t;

#define TASK_EXTRA_STACK_SIZE      (512)
#endif

#define BT_TASK_EXTRA_STACK_SIZE      TASK_EXTRA_STACK_SIZE
#define ESP_TASK_BT_CONTROLLER_STACK  (3584 + TASK_EXTRA_STACK_SIZE)

*/
	// bluetooth controller
	// implements HCI on controller side, LL, PHY
	// deals with the lower layers of BLE stack
	// generate BT controller configuration structure
	// macro defining default structure 
	// default settings
	// sets controller stack size, priority, and HCI baud rate
	// baud rate (symbol rate): speed of communication over a data channel, bits per second, 921600
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	
	// initialize bluetooth controller with default configuration
    esp_bt_controller_init(&bt_cfg);
	
	// enable the blue tooth controller in BLE mode
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
  
    // init and enable BLE (bluedroid) protocol stack 

    esp_bluedroid_init();
    esp_bluedroid_enable();

	// bluedroid stack is running (definitions and APIs for BLE) in program flow
	// need to define functionality of the application
	// react to a connect event and a 'write' event
	// need two managers
	// GATT and GAP event handlers 
	// register a callback function for each event handler so application knows which functions 
	// are going to handle GAP and GATT events
	// register these event handlers to act as callback functions for GAP and GATT events
    // will handle all events pushed to the application from the BLE stack
	
	// register gatt server event handler
    esp_ble_gatts_register_callback(gatts_event_handler);

    // register gap event handler
    esp_ble_gap_register_callback(gap_event_handler);

    // register app
	// register app profile with an app id
	// when app profile is registered an ESP_GATTS_REG_EVT is triggered
    esp_ble_gatts_app_register(DOTS_PLAYER1_PROFILE_ID);

	// setup buttons

	evt_queue = xQueueCreate(10, sizeof(gpio_num_t));

	gpio_config_t cfg; 
	cfg.pin_bit_mask = BUTTON_PIN_SEL;
	cfg.mode = GPIO_MODE_INPUT; 
	cfg.pull_up_en = GPIO_PULLUP_DISABLE;
	cfg.pull_down_en = GPIO_PULLDOWN_ENABLE; 
	cfg.intr_type = GPIO_INTR_POSEDGE; 
	gpio_config(&cfg);

	gpio_install_isr_service(0); 

	gpio_isr_handler_add(SWAP_RED_BUTTON, handle_swap_red_button, NULL);
	gpio_isr_handler_add(SWAP_GRE_BUTTON, handle_swap_gre_button, NULL);
	gpio_isr_handler_add(JUMP_HOR_BUTTON, handle_jump_hor_button, NULL);
	gpio_isr_handler_add(JUMP_VER_BUTTON, handle_jump_ver_button, NULL);
	gpio_isr_handler_add(SUB_BUTTON, handle_sub_button, NULL);
    

	// select gpio for blue dots
	gpio_pad_select_gpio(player1_bluedot.gpio_num);
    gpio_set_direction(player1_bluedot.gpio_num, GPIO_MODE_INPUT_OUTPUT );
	gpio_pad_select_gpio(player2_bluedot.gpio_num);
    gpio_set_direction(player2_bluedot.gpio_num, GPIO_MODE_INPUT_OUTPUT);

    // select gpio for each board
    select_gpio_pads(player1_board);
    select_gpio_pads(player2_board);
    select_gpio_pads(mediator_board);    

    int num_of_dots; 

	vTaskDelay(8000 / portTICK_PERIOD_MS);

    for(;;)
    {
		TaskHandle_t player2_compete_handle;

        reset_dots(player1_board);
        reset_dots(player2_board);
        reset_dots(mediator_board);
		reset_bluedots();

        vTaskDelay(1000/ portTICK_PERIOD_MS);

        num_of_dots = create_num_of_dots();
        determine_which_dots(num_of_dots, player1_board);
        determine_which_dots(num_of_dots, player2_board);
        determine_which_dots(num_of_dots, mediator_board);

        set_gpio_levels(player1_board);
        set_gpio_levels(player2_board);
        set_gpio_levels(mediator_board);
		set_bluedot_levels();

		xTaskCreate(player2_compete_task, "player2_compete_task", 2048, NULL, 10, &player2_compete_handle);
		
        vTaskDelay(8000 / portTICK_PERIOD_MS);

		vTaskDelete(player2_compete_handle);

		if (player1_points > 5)
		{
			blinking(WIN, PLAYER1);
			blinking(SCORE, PLAYER1);
			break; 
		}

		if (player2_points > 5)
		{
			blinking(WIN, PLAYER2);
			blinking(SCORE, PLAYER2);
			break; 
		}
    }

	reset_dots(player1_board);
	reset_dots(player2_board);
	reset_dots(mediator_board);
}
