#ifndef __MULTIROTOR_GEOMETRY_CTRL_H__
#define __MULTIROTOR_GEOMETRY_CTRL_H__

#include "imu.h"
#include "ahrs.h"
#include "debug_link.h"
#include "matrix.h"


/*Force*/
#define N_m 5
typedef struct {
	bool isfull;
	int index;
	int N;
} ICL_data;


/*Moment*/
#define ICL_N 10
typedef struct {
	MAT_ALLOC(Y_CL,3,8);	
	MAT_ALLOC(Mb,3,1);	//M_bar
} ICL_sigma;



void geometry_ctrl_init(void);
void multirotor_geometry_control(radio_t *rc);
void send_geometry_moment_ctrl_debug(debug_msg_t *payload);
void send_geometry_tracking_ctrl_debug(debug_msg_t *payload);
void send_uav_dynamics_debug(debug_msg_t *payload);
void send_controller_estimation_adaptive_debug(debug_msg_t *payload);

#endif
