#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "lidar_lite.h"
#include "sw_i2c.h"
#include "debug_link.h"
#include "sys_time.h"
#include "ins_sensor_sync.h"

lidar_lite_t lidar_lite;

bool lidar_lite_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_s();
	if((current_time - lidar_lite.last_read_time) > 0.3) {
		return false;
	}
	return true;
}

float lidar_lite_get_update_freq(void)
{
	return lidar_lite.update_freq;
}

void lidar_blocked_read_byte(uint8_t addr, uint8_t *data)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_stop();
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 1);
	sw_i2c_blocked_wait_ack();
	*data = sw_i2c_blocked_read_byte();
	sw_i2c_blocked_nack();
	sw_i2c_blocked_stop();
}

void lidar_read_byte(uint8_t addr, uint8_t *data)
{
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_stop();
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 1);
	sw_i2c_wait_ack();
	*data = sw_i2c_read_byte();
	sw_i2c_nack();
	sw_i2c_stop();
}

void lidar_blocked_read_bytes(uint8_t addr, uint8_t *data, int size)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_stop();
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 1);
	sw_i2c_blocked_wait_ack();
	for(int i = 0; i < size; i++) {
		data[i] = sw_i2c_blocked_read_byte();
		if(i == (size-1)) {
			/* send nack if all bytes are received */
			sw_i2c_blocked_nack();
		} else {
			/* send ack for requesting next byte */
			sw_i2c_blocked_ack();
		}
	}
	sw_i2c_blocked_stop();
}

void lidar_read_bytes(uint8_t addr, uint8_t *data, int size)
{
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_stop();
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 1);
	sw_i2c_wait_ack();
	for(int i = 0; i < size; i++) {
		data[i] = sw_i2c_read_byte();
		if(i == (size-1)) {
			/* send nack if all bytes are received */
			sw_i2c_nack();
		} else {
			/* send ack for requesting next byte */
			sw_i2c_ack();
		}
	}
	sw_i2c_stop();
}

void lidar_blocked_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(data);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_stop();
}

void lidar_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(data);
	sw_i2c_wait_ack();
	sw_i2c_stop();
}

void lidar_lite_init(void)
{
	/* numerical differentiation parameter initialization */
	lidar_lite.prescaler = 1;
	lidar_lite.dt = 0.025;
	lidar_lite.dt *= lidar_lite.prescaler;

	/* reset lidar */
	lidar_write_byte(LIDAR_ACQ_COMMAND_REG, 0x00);
	freertos_task_delay(1000);

	/* continuous reading mode */
	lidar_write_byte(LIDAR_MEASURE_COUNT_REG, 0xff);
	freertos_task_delay(10);

	/* measurement rate */
	lidar_write_byte(LIDAR_MEASURE_DELAY_REG, 0x02);
	freertos_task_delay(10);

	/* fast reading mode */
	lidar_write_byte(LIDAR_ACQ_CONFIG_REG, 0x21);
	freertos_task_delay(10);

	/* start distance measurement */
	lidar_write_byte(LIDAR_ACQ_COMMAND_REG, 0x04);
	freertos_task_delay(10);
}

void lidar_lite_task_handler(void)
{
	uint16_t lidar_dist_buf = 0;
	//int8_t lidar_vel_buf = 0;

	/* sensor reading */
	lidar_read_bytes(LIDAR_DISTANCE_REG, (uint8_t *)&lidar_dist_buf, 2);
	//lidar_read_byte(LIDAR_VELOCITY_REG, (uint8_t *)&lidar_vel_buf);

	/* handle communication failure of the i2c */
	if(lidar_dist_buf == 0) {
		return;
	} else {
		lidar_lite.dist_raw = (float)lidar_dist_buf * 1e-2;
	}

	if(lidar_lite.prescaler_cnt == lidar_lite.prescaler) {
		/* numerical differention */
		lidar_lite.vel_raw = (lidar_lite.dist_raw - lidar_lite.dist_last) / lidar_lite.dt;
		lidar_lite.dist_last = lidar_lite.dist_raw;
		lidar_lite.prescaler_cnt = 0;
	}
	lidar_lite.prescaler_cnt++;

	/* update frequency calculation */
	float curr_time = get_sys_time_s();
	lidar_lite.update_freq = 1.0f / (curr_time - lidar_lite.last_read_time);
	lidar_lite.last_read_time = curr_time;

	/* push data to the ins synchorization buffer */
	ins_rangefinder_sync_buffer_push(lidar_lite.dist_raw, lidar_lite.vel_raw);
}

float lidar_lite_get_distance(void)
{
	return lidar_lite.dist_raw; //[m]
}

float lidar_lite_get_velocity(void)
{
	return lidar_lite.vel_raw; //[m/s]
}

void send_rangefinder_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_RANGEFINDER);
	pack_debug_debug_message_float(&lidar_lite.dist_raw, payload);
	pack_debug_debug_message_float(&lidar_lite.vel_raw, payload);
	pack_debug_debug_message_float(&lidar_lite.update_freq, payload);
}
