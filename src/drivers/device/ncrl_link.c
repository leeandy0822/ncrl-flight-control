#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx_conf.h"
#include "task.h"
#include "uart.h"
#include "imu.h"
#include "gpio.h"
#include "ncrl_link.h"
#include "sys_time.h"
#include "debug_link.h"
#include "proj_config.h"
#include "board_porting.h"
#include "quaternion.h"

#define NCRL_LINK_MSG_SIZE 21 
#define NCRL_LINK_CHECKSUM_INIT_VAL 19
#define NCRL_LINK_QUEUE_SIZE (22 * 10) //~400 packets
#define NCRL_LINK_SERIAL_MSG_SIZE 22

typedef struct {
	char c;
} ncrl_link_buf_c_t;

QueueHandle_t ncrl_link_queue;

ncrl_link_t ncrl_link;

void ncrl_link_init(int id)
{
	ncrl_link.id = id;
	ncrl_link_queue = xQueueCreate(NCRL_LINK_QUEUE_SIZE, sizeof(ncrl_link_buf_c_t));
}

bool ncrl_link_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - ncrl_link.time_now) > 300) {
		return false;
	}
	return true;
}

static uint8_t generate_ncrl_link_checksum_byte(uint8_t *payload, int payload_cnt)
{
	uint8_t result = NCRL_LINK_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_cnt; i++)
		result ^= payload[i];

	return result;
}

void ncrl_link_buf_push(uint8_t c)
{
	if(ncrl_link.buf_pos >=NCRL_LINK_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < NCRL_LINK_SERIAL_MSG_SIZE; i++) {
			ncrl_link.buf[i - 1] = ncrl_link.buf[i];
		}

		/* save new byte to the last array element */
		ncrl_link.buf[NCRL_LINK_SERIAL_MSG_SIZE - 1] = c;
		ncrl_link.buf_pos = NCRL_LINK_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		ncrl_link.buf[ncrl_link.buf_pos] = c;
		ncrl_link.buf_pos++;
	}
}

void ncrl_link_isr_handler(uint8_t c)
{
	ncrl_link_buf_c_t ncrl_link_queue_item;
	ncrl_link_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(ncrl_link_queue, &ncrl_link_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void ncrl_link_update(void)
{
	ncrl_link_buf_c_t recept_c;
	while(xQueueReceive(ncrl_link_queue, &recept_c, 0) == pdTRUE) {

		uint8_t c = recept_c.c;
		ncrl_link_buf_push(c);
		if(c == '+' && ncrl_link.buf[0] == '@') {
			/* decode ncrl_link message */
			if(ncrl_link_serial_decoder(ncrl_link.buf) == 0) {
				ncrl_link.buf_pos = 0; //reset position pointer
			}
		}

	}
}

int ncrl_link_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_ncrl_link_checksum_byte(&buf[3], NCRL_LINK_SERIAL_MSG_SIZE - 4);
	int recv_id = buf[2];
	if(checksum != recv_checksum || ncrl_link.id != recv_id) {
		return 1; //error detected		
	}

	ncrl_link.time_now = get_sys_time_ms();

	/* decode position (enu frame) */
	memcpy(&ncrl_link.mode, &buf[3], sizeof(char));
	memcpy(&ncrl_link.aux_info, &buf[4], sizeof(float));
	memcpy(&ncrl_link.data1, &buf[5], sizeof(float));
	memcpy(&ncrl_link.data2, &buf[9], sizeof(float));
	memcpy(&ncrl_link.data3, &buf[13], sizeof(float));
	memcpy(&ncrl_link.data4, &buf[17], sizeof(float));


	/* calculate update rate */
	float received_period = (ncrl_link.time_now - ncrl_link.time_last) * 0.001;
	ncrl_link.update_rate = 1.0f / received_period;

	/* save time for next iteration */
	ncrl_link.time_last = ncrl_link.time_now;

	return 0;
}

void ncrl_link_get_position_enu(float *pos)
{
	pos[0] = ncrl_link.pos_enu[0];
	pos[1] = ncrl_link.pos_enu[1];
	pos[2] = ncrl_link.pos_enu[2];
}

float ncrl_link_get_position_enu_x(void)
{
	return ncrl_link.pos_enu[0];
}

float ncrl_link_get_position_enu_y(void)
{
	return ncrl_link.pos_enu[1];
}

float ncrl_link_get_position_enu_z(void)
{
	return ncrl_link.pos_enu[2];
}

void ncrl_link_get_velocity_enu(float *vel)
{
	vel[0] = ncrl_link.vel_enu[0];
	vel[1] = ncrl_link.vel_enu[1];
	vel[2] = ncrl_link.vel_enu[2];
}

float ncrl_link_get_velocity_enu_x(void)
{
	return ncrl_link.vel_enu[0];
}

float ncrl_link_get_velocity_enu_y(void)
{
	return ncrl_link.vel_enu[1];
}

float ncrl_link_get_velocity_enu_z(void)
{
	return ncrl_link.vel_enu[2];
}

void ncrl_link_get_position_ned(float *pos)
{
	pos[0] =  ncrl_link.pos_enu[1];
	pos[1] =  ncrl_link.pos_enu[0];
	pos[2] = -ncrl_link.pos_enu[2];
}

float ncrl_link_get_position_ned_x(void)
{
	return ncrl_link.pos_enu[1];
}

float ncrl_link_get_position_ned_y(void)
{
	return ncrl_link.pos_enu[0];
}

float ncrl_link_get_position_ned_z(void)
{
	return -ncrl_link.pos_enu[2];
}

void ncrl_link_get_velocity_ned(float *vel)
{
	vel[0] =  ncrl_link.vel_enu[1];
	vel[1] =  ncrl_link.vel_enu[0];
	vel[2] = -ncrl_link.vel_enu[2];
}

float ncrl_link_get_velocity_ned_x(void)
{
	return ncrl_link.vel_enu[1];
}

float ncrl_link_get_velocity_ned_y(void)
{
	return ncrl_link.vel_enu[0];
}

float ncrl_link_get_velocity_ned_z(void)
{
	return -ncrl_link.vel_enu[2];
}

void ncrl_link_get_quaternion(float *q)
{
	q[0] = ncrl_link.q[0];
	q[1] = ncrl_link.q[1];
	q[2] = ncrl_link.q[2];
	q[3] = ncrl_link.q[3];
}

void send_ncrl_link_fsm_msg(void)
{
	/*+------------+----------+---------+---------+---------+--------+--------+--------+----------+
	 *| start byte | checksum | mode | aux_info | data1 | data2 | data3 | data4 | end byte |
	 *+------------+----------+---------+---------+---------+--------+--------+--------+----------+*/

	float gyro[3] = {0.0f};
	get_gyro_lpf(gyro);

	char msg_buf[NCRL_LINK_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0;
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	memcpy(msg_buf + msg_pos, &ncrl_link.mode, sizeof(char));
	msg_pos += sizeof(char);
	memcpy(msg_buf + msg_pos, &ncrl_link.aux_info, sizeof(char));
	msg_pos += sizeof(char);
	memcpy(msg_buf + msg_pos, &ncrl_link.data1, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ncrl_link.data2, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ncrl_link.data3, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ncrl_link.data4, sizeof(float));
	msg_pos += sizeof(float);

	msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	msg_buf[1] = generate_ncrl_link_checksum_byte((uint8_t *)&msg_buf[3],
	                NCRL_LINK_MSG_SIZE - 3);

	ncrl_link_puts(msg_buf, NCRL_LINK_MSG_SIZE);
}

void ncrl_link_send_fsm_200hz(void)
{
	/* triggered every 2 times since the function is designed to be called by
	 * flight control main loop (400Hz) */
	static int prescaler = 2;
	prescaler--;

	if(prescaler == 0) {
		send_ncrl_link_fsm_msg();
		prescaler = 2;
	}
}

void ncrl_link_camera_trigger_20hz(void)
{
	/* to generate the camera trigger pulse:
	 * (1/20Hz) / (1/400Hz) = 20 (flight control loop is 20x faster than what we need)
	 * 10% on:  20 * 0.1 = 2times
	 * 90% off: 20 * 0.9 = 18times*/

	static int counter = 0;

	if(counter < 2) {
		camera_trigger_gpio_on();
	} else {
		camera_trigger_gpio_off();
	}

	counter = (counter + 1) % 20;
}

// void send_ncrl_link_position_debug_message(debug_msg_t *payload)
// {
// 	//unit: [cm]
// 	float px = ncrl_link.pos_enu[0] * 100.0f;
// 	float py = ncrl_link.pos_enu[1] * 100.0f;
// 	float pz = ncrl_link.pos_enu[2] * 100.0f;

// 	pack_debug_debug_message_header(payload, MESSAGE_ID_NCRL_LINK_POSITION);
// 	pack_debug_debug_message_float(&px, payload);
// 	pack_debug_debug_message_float(&py, payload);
// 	pack_debug_debug_message_float(&pz, payload);
// }

// void send_ncrl_link_quaternion_debug_message(debug_msg_t *payload)
// {
// 	pack_debug_debug_message_header(payload, MESSAGE_ID_NCRL_LINK_QUATERNION);
// 	pack_debug_debug_message_float(&ncrl_link.q[0], payload);
// 	pack_debug_debug_message_float(&ncrl_link.q[1], payload);
// 	pack_debug_debug_message_float(&ncrl_link.q[2], payload);
// 	pack_debug_debug_message_float(&ncrl_link.q[3], payload);
// }

// void send_ncrl_link_velocity_debug_message(debug_msg_t *payload)
// {
// 	//unit: [cm/s]
// 	float vx_raw = ncrl_link.vel_enu[0] * 100.0f;
// 	float vy_raw = ncrl_link.vel_enu[1] * 100.0f;
// 	float vz_raw = ncrl_link.vel_enu[2] * 100.0f;
// 	float vx_filtered = ncrl_link.vel_enu[0] * 100.0f;
// 	float vy_filtered = ncrl_link.vel_enu[1] * 100.0f;
// 	float vz_filtered = ncrl_link.vel_enu[2] * 100.0f;

// 	pack_debug_debug_message_header(payload, MESSAGE_ID_NCRL_LINK_VELOCITY);
// 	pack_debug_debug_message_float(&vx_raw, payload);
// 	pack_debug_debug_message_float(&vy_raw, payload);
// 	pack_debug_debug_message_float(&vz_raw, payload);
// 	pack_debug_debug_message_float(&vx_filtered, payload);
// 	pack_debug_debug_message_float(&vy_filtered, payload);
// 	pack_debug_debug_message_float(&vz_filtered, payload);
// 	pack_debug_debug_message_float(&ncrl_link.update_rate, payload);
// }
