#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
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
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

#define DOTS_PER_BOARD 4

// master board
#define M_R1_DOT 21
#define M_R2_DOT 22
#define M_G1_DOT 23
#define M_G2_DOT 25

// player 1 board
#define P1_B_DOT  2
#define P1_R1_DOT 4
#define P1_R2_DOT 5
#define P1_G1_DOT 12
#define P1_G2_DOT 13

// player 2 board
#define P2_B_DOT  14
#define P2_R1_DOT 15
#define P2_R2_DOT 16
#define P2_G1_DOT 17
#define P2_G2_DOT 18

// player 2 buttons
#define SWAP_RED_BUTTON 26
#define SWAP_GRE_BUTTON 27
#define JUMP_HOR_BUTTON 32
#define JUMP_VER_BUTTON 33
#define SUB_BUTTON 19

#define BUTTON_PIN_SEL ((1ULL<<SWAP_RED_BUTTON) | (1ULL<<SWAP_GRE_BUTTON) | (1ULL<<JUMP_HOR_BUTTON) | (1ULL<<JUMP_VER_BUTTON) | (1<<SUB_BUTTON))

#define GATTS_SERVICE_UUID      	0x0001
#define GATTS_CHAR_NUM		    	2
#define GATTS_NUM_HANDLE        	1 + (3 * GATTS_CHAR_NUM)
#define BLE_DEVICE_NAME            	"DOTS_SERVER"
#define BLE_MANUFACTURER_DATA_LEN  	4
#define GATTS_CHAR_VAL_LEN_MAX		22
#define DOTS_PLAYER1_PROFILE_ID 	0
#define GATTS_TAG 					"GATTS"
#define BLE_SERVICE_UUID_SIZE 		ESP_UUID_LEN_128

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

typedef enum {
	SCORE, 
	WIN
} BLINKTYPE;

typedef struct {
    gpio_num_t gpio_num;
    DOTTYPE dottype; 
    int level;
} DOT;

typedef DOT Board[DOTS_PER_BOARD]; 

// GATT profile structure
typedef struct {
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
	uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
	uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gatts_profile_inst;

// GATT characteristic structure
typedef struct {
	esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t char_perm;
	esp_gatt_char_prop_t char_property;
	esp_attr_value_t *char_val;
    esp_attr_control_t *char_control;
    uint16_t char_handle;
	esp_gatts_cb_t char_write_callback;
} gatts_char_inst;


// prototypes
// -----------------------------------------------------------

int create_num_of_dots();
void determine_which_dots(int num_of_dots, Board board);
void select_gpio_pads(Board board);
void set_gpio_levels(Board board);
void reset_dots(Board board);

void player2_compete_task();
void reward_points(PLAYER player);
void blinking(BLINKTYPE blink_type, PLAYER player);
void swap_or_jump(DOTACTION da, PLAYER player, DOTTYPE dt1, DOTTYPE dt2, Board board);

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void char_rx_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


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

// DOTS server data
// -----------------------------------------------------------

// attribute values for characteristics and descriptors

uint8_t char_rx_content[GATTS_CHAR_VAL_LEN_MAX] = {0x11,0x22,0x33};

esp_attr_value_t char_rx_attr_val = {
	.attr_max_len 	= GATTS_CHAR_VAL_LEN_MAX,
	.attr_len		= sizeof(char_rx_content),
	.attr_value     = char_rx_content,
};

// for nordic uART
static uint8_t nordic_uart_service_uuid128[BLE_SERVICE_UUID_SIZE] = {
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static uint8_t ble_manufacturer[BLE_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};

static uint32_t ble_add_char_pos;

// advertising data 
static esp_ble_adv_data_t ble_adv_data = {
    .set_scan_rsp = false,							
    .include_name = true,								
    .include_txpower = true,						
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = BLE_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  (uint8_t *)ble_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = BLE_SERVICE_UUID_SIZE,
    .p_service_uuid = nordic_uart_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// advertising parameters
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT profile
gatts_profile_inst gl_profile = {
         .gatts_if = ESP_GATT_IF_NONE,       
};

// characteristics
gatts_char_inst gl_char[GATTS_CHAR_NUM] = {
	{
			// RX for uART service
			.char_uuid.len = ESP_UUID_LEN_128, 
			.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E },
			.char_perm = ESP_GATT_PERM_WRITE,
			.char_property = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
			.char_val = &char_rx_attr_val,
			.char_control = NULL,
			.char_handle = 0,
			.char_write_callback=char_rx_write_handler
	},
	{
			// TX for uART service
			.char_uuid.len = ESP_UUID_LEN_128,  // TX
			.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E },
			.char_val = NULL, // &char_tx_attr_val,
			.char_control=NULL,
			.char_handle=0,
			.char_write_callback =NULL
	}
};

static  uint16_t notify_conn_id = 0;
static  esp_gatt_if_t notify_gatts_if = NULL;

// dots functions
// -----------------------------------------------------------

void set_bluedot_levels()
{
	player1_bluedot.level = esp_random() % 2;
	player2_bluedot.level = esp_random() % 2;
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
    int num_of_dots = 0;
    while(num_of_dots == 0) 
        num_of_dots = esp_random() % DOTS_PER_BOARD; // 1, 2, 3
    return num_of_dots; 
}

void determine_which_dots(int num_of_dots, Board board)
{
	
    int pos, i;
    int on_off[DOTS_PER_BOARD] = { 0 }; 

    on_off[esp_random() % DOTS_PER_BOARD] = 1;

    for (i = 0; i < num_of_dots - 1; ++i)
    {
        do {
            pos = esp_random() % DOTS_PER_BOARD; 
            if (on_off[pos] != 1) { on_off[pos] = 1; break; }   
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
        gpio_set_direction(board[i].gpio_num, GPIO_MODE_OUTPUT);
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
		swap_or_jump(JUMP, PLAYER1, R1, G2, player1_board);
	}
	else if (strncmp((const char *)inst.char_val->attr_value, "V", 1) == 0)
	{
		swap_or_jump(JUMP, PLAYER1, R2, G1, player1_board);
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


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    
	if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT)
	{
		esp_ble_gap_start_advertising(&ble_adv_params);
	}
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	
	switch (event) 
	{

		case ESP_GATTS_REG_EVT:
			
			gl_profile.service_id.is_primary = true;
			gl_profile.service_id.id.inst_id = 0x00;
			gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
			
			for (uint8_t pos = 0; pos < ESP_UUID_LEN_128; pos++) 
			{
				gl_profile.service_id.id.uuid.uuid.uuid128[pos] = nordic_uart_service_uuid128[pos];
			}

			esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
			esp_ble_gap_config_adv_data(&ble_adv_data);
			esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
			
			break;

		case ESP_GATTS_CREATE_EVT:
		
			gl_profile.service_handle = param->create.service_handle;
			gl_profile.char_uuid.len = gl_char[0].char_uuid.len;
			gl_profile.char_uuid.uuid.uuid16 = gl_char[0].char_uuid.uuid.uuid16;

			esp_ble_gatts_start_service(gl_profile.service_handle);
			
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
		
		case ESP_GATTS_ADD_CHAR_EVT: 
			gl_profile.char_handle = param->add_char.attr_handle;

			if (param->add_char.status==ESP_GATT_OK) 
			{
				if (param->add_char.attr_handle != 0) 
				{
					gl_char[ble_add_char_pos].char_handle=param->add_char.attr_handle;

				
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
			break;
		case ESP_GATTS_WRITE_EVT: 
			for (uint32_t pos = 0; pos < GATTS_CHAR_NUM; pos++) 
			{
				if (gl_char[pos].char_handle==param->write.handle) 
				{
					
					if (gl_char[pos].char_write_callback!=NULL) 
					{
						gl_char[pos].char_write_callback(event, gatts_if, param);
					}
				}
			}
			break;
		case ESP_GATTS_CONNECT_EVT:
			gl_profile.conn_id = param->connect.conn_id;
			break;
		case ESP_GATTS_DISCONNECT_EVT:
			esp_ble_gap_start_advertising(&ble_adv_params);
			break;
		default:
			break;
    }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    
	// gatts is called once gap registered
    if (event == ESP_GATTS_REG_EVT) 
	{
        if (param->reg.status == ESP_GATT_OK) 
		{
        	gl_profile.gatts_if = gatts_if;
        } 
		else 
		{
            return;
        }
    }

    gatts_profile_event_handler(event, gatts_if, param);
}

// competition functions
// -----------------------------------------------------------

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
			b_submit   = gpio_get_level(SUB_BUTTON);

			if (b_swap_red) swap_or_jump(SWAP, PLAYER2, R1, R2, player2_board);
			if (b_swap_gre) swap_or_jump(SWAP, PLAYER2, G1, G2, player2_board);
			if (b_jump_hor) swap_or_jump(JUMP, PLAYER2, R1, G2, player2_board);
			if (b_jump_ver) swap_or_jump(JUMP, PLAYER2, R2, G1, player2_board);
			if (b_submit)   reward_points(PLAYER2);
		}
	}
}

void app_main()
{
    nvs_flash_init();

    // init and enable controller

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
  
    // init and enable BLE protocol stack 

    esp_bluedroid_init();
    esp_bluedroid_enable();

    // register gatt server event handler
    esp_ble_gatts_register_callback(gatts_event_handler);

    // register gap event handler
    esp_ble_gap_register_callback(gap_event_handler);

    // register app
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
		
        vTaskDelay(10000 / portTICK_PERIOD_MS);

		vTaskDelete(player2_compete_handle);

		if (player1_points > 9)
		{
			blinking(WIN, PLAYER1);
			blinking(SCORE, PLAYER1);
			break; 
		}

		if (player2_points > 9)
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
