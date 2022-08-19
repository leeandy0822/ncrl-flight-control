#ifndef __NCRL_LINK_H__
#define __NCRL_LINK_H__

#include "debug_link.h"

#define NCRL_LINK_SERIAL_MSG_SIZE 22

typedef struct {
	uint8_t id;

	/* position [m] */
	float pos_enu[3];

	/* velocity [m/s] */
	float vel_enu[3];

	/* orientation (quaternion) */
	float q[4];

	float time_now;
	float time_last;
	float update_rate;

	volatile int buf_pos;
	uint8_t buf[NCRL_LINK_SERIAL_MSG_SIZE];

	bool vel_ready;
	char mode;
	char aux_info;

	/* data[0] ~ data[3] : target position in enu [m] */
	float data[4];

} ncrl_link_t ;

void ncrl_link_init(int id);

/* reception of vins-mon pose and velocity information */
int ncrl_link_serial_decoder(uint8_t *buf);
void ncrl_link_isr_handler(uint8_t c);

/* transmission of imu information for vins-mono */
void send_ncrl_link_fsm_msg(float);
// void ncrl_link_send_fsm_200hz(void);

/* vins-mono camera triggering */
void ncrl_link_camera_trigger_20hz(void);

void ncrl_link_update(void);
bool ncrl_link_available(void);

/* ncrl_link getters */

void ncrl_link_get_target_enu(float *pos);
void ncrl_link_get_position_enu(float *pos);
float ncrl_link_get_position_enu_x(void);
float ncrl_link_get_position_enu_y(void);
float ncrl_link_get_position_enu_z(void);

void ncrl_link_get_velocity_enu(float *vel);
float ncrl_link_get_velocity_enu_x(void);
float ncrl_link_get_velocity_enu_y(void);
float ncrl_link_get_velocity_enu_z(void);

void ncrl_link_get_position_ned(float *pos);
float ncrl_link_get_position_ned_x(void);
float ncrl_link_get_position_ned_y(void);
float ncrl_link_get_position_ned_z(void);

void ncrl_link_get_velocity_ned(float *vel);
float ncrl_link_get_velocity_ned_x(void);
float ncrl_link_get_velocity_ned_y(void);
float ncrl_link_get_velocity_ned_z(void);

void ncrl_link_get_quaternion(float *q);
char ncrl_link_get_mode(void);
char ncrl_link_get_aux_info(void);

void ncrl_link_test_function(float );

/* vins-mono debug messages */
// void send_ncrl_link_position_debug_message(debug_msg_t *payload);
// void send_ncrl_link_quaternion_debug_message(debug_msg_t *payload);
// void send_ncrl_link_velocity_debug_message(debug_msg_t *payload);

#endif
