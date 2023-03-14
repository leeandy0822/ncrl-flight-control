#include "proj_config.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "gpio.h"
#include "sbus_radio.h"
#include "ahrs.h"
#include "matrix.h"
#include "motor_thrust_fitting.h"
#include "motor.h"
#include "bound.h"
#include "se3_math.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "autopilot.h"
#include "debug_link.h"
#include "multirotor_geometry_param.h"
#include "multirotor_geometry_ctrl.h"
#include "system_state.h"
#include "multirotor_rc.h"
#include "barometer.h"
#include "compass.h"
#include "sys_param.h"
#include "sys_time.h"
#include "led.h"
#include "waypoint_following.h"
#include "fence.h"
#include "board_porting.h"
#include "sensor_switching.h"

#define dt 0.0025										 //[s]

#if (UAV_FRAME == F450)
	float length = 16.25f;
#else
	float length = 8.125f;						 //[cm]
#endif


#define MOTOR_TO_CG_LENGTH 16.25						 //[cm]
#define MOTOR_TO_CG_LENGTH_M (MOTOR_TO_CG_LENGTH * 0.01) //[m]
#define COEFFICIENT_YAW 1.0f


MAT_ALLOC(J, 3, 3);
MAT_ALLOC(R, 3, 3);
MAT_ALLOC(Rd, 3, 3);
MAT_ALLOC(Rt, 3, 3);
MAT_ALLOC(Rtd, 3, 3);
MAT_ALLOC(RtdR, 3, 3);
MAT_ALLOC(RtRd, 3, 3);
MAT_ALLOC(RtRdWd, 3, 3);
MAT_ALLOC(Re3, 3, 1);
MAT_ALLOC(W, 3, 1);
MAT_ALLOC(W_dot, 3, 1);
MAT_ALLOC(Wd, 3, 1);
MAT_ALLOC(W_hat, 3, 3);
MAT_ALLOC(Wd_dot, 3, 1);
MAT_ALLOC(JW, 3, 1);
MAT_ALLOC(WJW, 3, 1);
MAT_ALLOC(JWdot, 3, 1);
MAT_ALLOC(M, 3, 1);
MAT_ALLOC(eR_mat, 3, 3);
MAT_ALLOC(eR, 3, 1);
MAT_ALLOC(eW, 3, 1);
MAT_ALLOC(WRt, 3, 3);
MAT_ALLOC(WRtRd, 3, 3);
MAT_ALLOC(WRtRdWd, 3, 1);
MAT_ALLOC(RtRdWddot, 3, 1);
MAT_ALLOC(WRtRdWd_RtRdWddot, 3, 1);
MAT_ALLOC(J_WRtRdWd_RtRdWddot, 3, 1);
MAT_ALLOC(inertia_effect, 3, 1);
MAT_ALLOC(kxex_kvev_mge3_mxd_dot_dot, 3, 1);
MAT_ALLOC(b1d, 3, 1);
MAT_ALLOC(b2d, 3, 1);
MAT_ALLOC(b3d, 3, 1);

/* Allocation*/
MAT_ALLOC(DIS_M, 8, 4);
MAT_ALLOC(CONTROL_OUTPUT, 4, 1);
MAT_ALLOC(DIS_OUTPUT, 8, 1);

// Force controller 
MAT_ALLOC(Y_m, 3, 1);
MAT_ALLOC(Y_mt, 1, 3);
MAT_ALLOC(y_m_cl, 3, 1);
MAT_ALLOC(y_m_clt, 1, 3);
MAT_ALLOC(theta_m_hat, 1, 1);
MAT_ALLOC(theta_m_hat_dot, 1, 1);
MAT_ALLOC(theta_m_hat_dot_adaptive, 1, 1);
MAT_ALLOC(theta_m_hat_dot_ICL, 1, 1);
MAT_ALLOC(ev_C1ex, 3, 1);
MAT_ALLOC(Ymt_evC1ex, 1, 1);
MAT_ALLOC(curr_force, 3, 1);
MAT_ALLOC(F_cl, 3, 1);
MAT_ALLOC(mat_m_now, 1, 1);

// Moment controller
MAT_ALLOC(theta, 8, 1);
MAT_ALLOC(theta_hat_dot, 8, 1);
MAT_ALLOC(Y, 3, 8);
MAT_ALLOC(Yt, 8, 3);
MAT_ALLOC(eW_c2eR, 3, 1);
MAT_ALLOC(Yt_eW_c2eR, 8, 1);
MAT_ALLOC(Ytheta, 3, 1);
MAT_ALLOC(adaptive_theta_hat_dot, 8, 1);
MAT_ALLOC(ICL_theta_hat_dot, 8, 1);
MAT_ALLOC(Y_CL, 3, 8);
MAT_ALLOC(M_last, 3, 1);
MAT_ALLOC(Mb, 3, 1);
MAT_ALLOC(ICL_control_term, 8, 1);	 // x
MAT_ALLOC(Y_CLtheta, 3, 1);			 // Y_CL*theta
MAT_ALLOC(Mb_Y_CLtheta, 3, 1);		 // Mb-Y_CL*theta
MAT_ALLOC(Y_CLT, 8, 3);				 // Y_CL.transpose
MAT_ALLOC(Y_CLT_Mb_Y_CLtheta, 8, 1); // Y_CLT*Mb-Y_CL*theta
MAT_ALLOC(W, 3, 1);
MAT_ALLOC(W_obs, 3, 1);


float pos_error[3];
float vel_error[3];
float tracking_error_integral[3];

float krx, kry, krz;
float kwx, kwy, kwz;
float kpx, kpy, kpz;
float kvx, kvy, kvz;
float yaw_rate_ctrl_gain;
float k_tracking_i_gain[3];

float uav_mass;

// M = (J * W_dot) + (W X JW)
float uav_dynamics_m[3] = {0.0f};
// M_rot = (J * W_dot)
float uav_dynamics_m_rot_frame[3] = {0.0f};
float coeff_cmd_to_thrust[6] = {0.0f};
float coeff_thrust_to_cmd[6] = {0.0f};
float motor_thrust_max = 0.0f;
bool height_ctrl_only = false;

/* Force ICL and Adaptive gain*/
ICL_data force_ICL;
// float Gamma_m_gain = 0.2f;
float Gamma_m_gain = 0.2;
float C1_gain = 0.1f;
float k_cl_m_gain = 0.003f;
// float k_cl_m_gain = 0.0001f;
float mat_m_matrix[N_m] = {0.0f};
float mat_m_sum = 0.0f;

/* Moment ICL and Adaptive gain*/
int ICL_sigma_index = 0;
ICL_sigma sigma_array[ICL_N];
float adaptive_gamma[8] = {0.000001, 0.000001, 0.002, 0.002, 0.002, 0.002, 0.002, 0.002};
float c2 = 0.1; 
float output_force_last = 0;
float k_icl[8] = {0.015, 0.015, 0, 0, 0, 0, 0, 0}; 
float adaptive_gamma_k_icl[8];



void ICL_matrix_init(void)
{
	force_ICL.index = 0;
	force_ICL.isfull = false;
	force_ICL.N = N_m;
}

void force_ff_ctrl_use_adaptive_ICL(float *accel_ff, float *force_ff, float *pos_err, float *vel_err, float *curr_vel, float *acc)
{
	/* with mass of uav unknown */
	/* Y_m and Y_m transpose */
	mat_data(Y_m)[0] = accel_ff[0];
	mat_data(Y_m)[1] = accel_ff[1];
	mat_data(Y_m)[2] = accel_ff[2] - 9.81;
	mat_data(Y_mt)[0] = mat_data(Y_m)[0];
	mat_data(Y_mt)[1] = mat_data(Y_m)[1];
	mat_data(Y_mt)[2] = mat_data(Y_m)[2];

	// ev_C1ex = ev + C1*ex
	mat_data(ev_C1ex)[0] = vel_err[0] + C1_gain * pos_err[0];
	mat_data(ev_C1ex)[1] = vel_err[1] + C1_gain * pos_err[1];
	mat_data(ev_C1ex)[2] = vel_err[2] + C1_gain * pos_err[2];

	/* first term of theta_m update law */
	MAT_MULT(&Y_mt, &ev_C1ex, &Ymt_evC1ex);

#if (SELECT_FORCE_ADAPTIVE_W_WO_ICL == FORCE_ADAPTIVE_WITHOUT_ICL)
	// theta_m_dot = Gamma*Y_mt*ev_C1ex
	mat_data(theta_m_hat_dot)[0] = -(Gamma_m_gain * mat_data(Ymt_evC1ex)[0]);

#elif (SELECT_FORCE_ADAPTIVE_W_WO_ICL == FORCE_ADAPTIVE_WITH_ICL)

	mat_data(y_m_cl)[0] = -acc[0];
	mat_data(y_m_cl)[1] = -acc[1];
	mat_data(y_m_cl)[2] = -acc[2] - 9.81;

	
	mat_data(y_m_clt)[0] = mat_data(y_m_cl)[0];
	mat_data(y_m_clt)[1] = mat_data(y_m_cl)[1];
	mat_data(y_m_clt)[2] = mat_data(y_m_cl)[2];

	/* prepare force control input used in ICL */
	mat_data(F_cl)[0] = mat_data(curr_force)[0];
	mat_data(F_cl)[1] = mat_data(curr_force)[1];
	mat_data(F_cl)[2] = mat_data(curr_force)[2];

	/* prepare past data */
	mat_data(mat_m_now)[0] = mat_data(y_m_clt)[0] * (mat_data(F_cl)[0] - mat_data(y_m_cl)[0] * mat_data(theta_m_hat)[0]);
	mat_data(mat_m_now)[0] += mat_data(y_m_clt)[1] * (mat_data(F_cl)[1] - mat_data(y_m_cl)[1] * mat_data(theta_m_hat)[0]);
	mat_data(mat_m_now)[0] += mat_data(y_m_clt)[2] * (mat_data(F_cl)[2] - mat_data(y_m_cl)[2] * mat_data(theta_m_hat)[0]);

	/* summation of past data */
	if (force_ICL.index >= force_ICL.N)
	{
		force_ICL.index = 0;
		force_ICL.isfull = true;
	}
	mat_m_matrix[force_ICL.index] = mat_data(mat_m_now)[0];
	force_ICL.index++;
	if (!force_ICL.isfull)
	{
		mat_m_sum = 0;
		for (int i = 0; i < force_ICL.index; i++)
		{
			mat_m_sum += mat_m_matrix[i];
		}
	}else{
		mat_m_sum = 0;
		for (int i = 0; i < force_ICL.N; i++)
		{
			mat_m_sum += mat_m_matrix[i];
		}
	}

	/* prepare components of theta_m_hat_dot */
	
	mat_data(theta_m_hat_dot_adaptive)[0] = - (Gamma_m_gain * mat_data(Ymt_evC1ex)[0]);
	mat_data(theta_m_hat_dot_ICL)[0] = k_cl_m_gain  * Gamma_m_gain* mat_m_sum;

	/* theta_m_dot = adaptive law + ICL update law */
	mat_data(theta_m_hat_dot)[0] = mat_data(theta_m_hat_dot_adaptive)[0] + mat_data(theta_m_hat_dot_ICL)[0];
#endif

	mat_data(theta_m_hat)[0] += mat_data(theta_m_hat_dot)[0] * dt;

	/* translational adaptive feedforward term */
	// Y_m*theta_m_hat
	force_ff[0] = mat_data(Y_m)[0] * mat_data(theta_m_hat)[0];
	force_ff[1] = mat_data(Y_m)[1] * mat_data(theta_m_hat)[0];
	force_ff[2] = mat_data(Y_m)[2] * mat_data(theta_m_hat)[0];
	
	if (SELECT_UAV_MISSION == TRANSPORTATION){
		bound_float(&force_ff[0], 10, -10);
		bound_float(&force_ff[1], 10, -10);
		bound_float(&force_ff[2], 40, -40);
	}else{
		bound_float(&force_ff[0], 2.8, -2.8);
		bound_float(&force_ff[1], 2.8, -2.8);
		bound_float(&force_ff[2], 18, -18);
	}
}

void geometry_ctrl_init(void)
{
	init_multirotor_geometry_param_list();

	autopilot_init();

	ICL_matrix_init();

	navigation_manager_init();

	float geo_fence_origin[3] = {0.0f, 0.0f, 0.0f};
	autopilot_set_enu_rectangular_fence(geo_fence_origin, 3.3f, 1.3f, 3.0f);

	MAT_INIT(J, 3, 3);
	MAT_INIT(R, 3, 3);
	MAT_INIT(Rd, 3, 3);
	MAT_INIT(Rt, 3, 3);
	MAT_INIT(Rtd, 3, 3);
	MAT_INIT(RtdR, 3, 3);
	MAT_INIT(RtRd, 3, 3);
	MAT_INIT(RtRdWd, 3, 3);
	MAT_INIT(Re3, 3, 1);
	MAT_INIT(W, 3, 1);
	MAT_INIT(W_dot, 3, 1);
	MAT_INIT(Wd, 3, 1);
	MAT_INIT(W_hat, 3, 3);
	MAT_INIT(Wd_dot, 3, 1);
	MAT_INIT(JW, 3, 1);
	MAT_INIT(WJW, 3, 1);
	MAT_INIT(JWdot, 3, 1);
	MAT_INIT(M, 3, 1);
	MAT_INIT(eR_mat, 3, 3);
	MAT_INIT(eR, 3, 1);
	MAT_INIT(eW, 3, 1);
	MAT_INIT(WRt, 3, 3);
	MAT_INIT(WRtRd, 3, 3);
	MAT_INIT(WRtRdWd, 3, 1);
	MAT_INIT(RtRdWddot, 3, 1);
	MAT_INIT(WRtRdWd_RtRdWddot, 3, 1);
	MAT_INIT(J_WRtRdWd_RtRdWddot, 3, 1);
	MAT_INIT(inertia_effect, 3, 1);
	MAT_INIT(kxex_kvev_mge3_mxd_dot_dot, 3, 1);
	MAT_INIT(b1d, 3, 1);
	MAT_INIT(b2d, 3, 1);
	MAT_INIT(b3d, 3, 1);

	MAT_INIT(DIS_M, 8, 4);
	MAT_INIT(CONTROL_OUTPUT, 4, 1);
	MAT_INIT(DIS_OUTPUT, 8, 1);


	mat_data(DIS_M)[4 * 0 + 0] = 0.5f;
	mat_data(DIS_M)[4 * 0 + 1] = 0.79051383f;
	mat_data(DIS_M)[4 * 1 + 1] = 0.24703557f;
	mat_data(DIS_M)[4 * 2 + 2] = 0.5f;
	mat_data(DIS_M)[4 * 3 + 3] = 0.5f;

	mat_data(DIS_M)[4 * 4 + 0] = 0.5f;
	mat_data(DIS_M)[4 * 4 + 1] = -0.79051383f;
	mat_data(DIS_M)[4 * 5 + 1] = 0.24703557f;
	mat_data(DIS_M)[4 * 6 + 2] = 0.5f;
	mat_data(DIS_M)[4 * 7 + 3] = 0.5f;


	// *************************
	// adaptive & ICL parameters
	// *************************


	// force
	MAT_INIT(Y_m, 3, 1);
	MAT_INIT(Y_mt, 1, 3);
	MAT_INIT(y_m_cl, 3, 1);
	MAT_INIT(y_m_clt, 1, 3);
	MAT_INIT(theta_m_hat, 1, 1);
	MAT_INIT(theta_m_hat_dot, 1, 1);
	MAT_INIT(theta_m_hat_dot_adaptive, 1, 1);
	MAT_INIT(theta_m_hat_dot_ICL, 1, 1);
	MAT_INIT(ev_C1ex, 3, 1);
	MAT_INIT(Ymt_evC1ex, 1, 1);
	MAT_INIT(curr_force, 3, 1);
	MAT_INIT(F_cl, 3, 1);
	MAT_INIT(mat_m_now, 1, 1);

	/* initialize value of mass estimation */
	if (SELECT_UAV_MISSION == SINGLE_UAV){
		mat_data(theta_m_hat)[0] = 0.5f;
	}else{
		mat_data(theta_m_hat)[0] = 3.2f;
	}

	// Moment
	MAT_INIT(theta, 8, 1);
	
	mat_data(theta)[0] = 0.0f;
	mat_data(theta)[1] = 0.0f;
	mat_data(theta)[2] = 0.4f;
	mat_data(theta)[3] = 0.02f;
	mat_data(theta)[4] = 0.8f;
	mat_data(theta)[5] = 0.0f;
	mat_data(theta)[6] = 0.0f;
	mat_data(theta)[7] = 0.0f;

	MAT_INIT(theta_hat_dot, 8, 1);
	MAT_INIT(Y, 3, 8);
	MAT_INIT(Yt, 8, 3);
	MAT_INIT(eW_c2eR, 3, 1);
	MAT_INIT(Yt_eW_c2eR, 8, 1);
	MAT_INIT(Ytheta, 3, 1);

	MAT_INIT(adaptive_theta_hat_dot, 8, 1);
	MAT_INIT(ICL_theta_hat_dot, 8, 1);
	MAT_INIT(Y_CL, 3, 8);
	MAT_INIT(M_last, 3, 1);
	mat_data(M_last)[0] = 0;
	mat_data(M_last)[1] = 0;
	mat_data(M_last)[2] = 0;
	MAT_INIT(Mb, 3, 1);
	MAT_INIT(Y_CL, 3, 8);
	MAT_INIT(ICL_control_term, 8, 1);	// x
	MAT_INIT(Y_CLtheta, 3, 1);			// Y_CL*theta
	MAT_INIT(Mb_Y_CLtheta, 3, 1);		// Mb-Y_CL*theta
	MAT_INIT(Y_CLT, 8, 3);				// Y_CL.transpose
	MAT_INIT(Y_CLT_Mb_Y_CLtheta, 8, 1); // Y_CLT*Mb-Y_CL*theta
	MAT_INIT(W, 3, 1);

	mat_data(W)[0] = 0;
	mat_data(W)[1] = 0;
	mat_data(W)[2] = 0;

	for (int i = 0; i < ICL_N; i++)
	{
		MAT_INIT(sigma_array[i].Y_CL, 3, 8);
		MAT_INIT(sigma_array[i].Mb, 3, 1);

		for (int j = 0; j < 3 * 8; j++)
		{
			sigma_array[i].mat_data(Y_CL)[j] = 0;
		}
		sigma_array[i].mat_data(Mb)[0] = 0;
		sigma_array[i].mat_data(Mb)[1] = 0;
		sigma_array[i].mat_data(Mb)[2] = 0;
	}
	for (int i = 0; i < 8; i++)
	{
		adaptive_gamma_k_icl[i] = adaptive_gamma[i] * k_icl[i];
	}

	/* modify local variables when user change them via ground station */
	set_sys_param_update_var_addr(MR_GEO_GAIN_ROLL_P, &krx);
	set_sys_param_update_var_addr(MR_GEO_GAIN_ROLL_D, &kwx);
	set_sys_param_update_var_addr(MR_GEO_GAIN_PITCH_P, &kry);
	set_sys_param_update_var_addr(MR_GEO_GAIN_PITCH_D, &kwy);
	set_sys_param_update_var_addr(MR_GEO_GAIN_YAW_P, &krz);
	set_sys_param_update_var_addr(MR_GEO_GAIN_YAW_D, &kwz);
	set_sys_param_update_var_addr(MR_GEO_GAIN_RATE_YAW, &yaw_rate_ctrl_gain);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_X, &kpx);
	set_sys_param_update_var_addr(MR_GEO_GAIN_VEL_X, &kvx);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_Y, &kpy);
	set_sys_param_update_var_addr(MR_GEO_GAIN_VEL_Y, &kvy);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_Z, &kpz);
	set_sys_param_update_var_addr(MR_GEO_GAIN_VEL_Z, &kvz);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_X_I, &k_tracking_i_gain[0]);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_Y_I, &k_tracking_i_gain[1]);
	set_sys_param_update_var_addr(MR_GEO_GAIN_POS_Z_I, &k_tracking_i_gain[2]);
	set_sys_param_update_var_addr(MR_GEO_UAV_MASS, &uav_mass);
	set_sys_param_update_var_addr(MR_GEO_INERTIA_JXX, &mat_data(J)[0 * 3 + 0]);
	set_sys_param_update_var_addr(MR_GEO_INERTIA_JYY, &mat_data(J)[1 * 3 + 1]);
	set_sys_param_update_var_addr(MR_GEO_INERTIA_JZZ, &mat_data(J)[2 * 3 + 2]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C1, &coeff_cmd_to_thrust[0]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C2, &coeff_cmd_to_thrust[1]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C3, &coeff_cmd_to_thrust[2]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C4, &coeff_cmd_to_thrust[3]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C5, &coeff_cmd_to_thrust[4]);
	set_sys_param_update_var_addr(PWM_TO_THRUST_C6, &coeff_cmd_to_thrust[5]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C1, &coeff_thrust_to_cmd[0]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C2, &coeff_thrust_to_cmd[1]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C3, &coeff_thrust_to_cmd[2]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C4, &coeff_thrust_to_cmd[3]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C5, &coeff_thrust_to_cmd[4]);
	set_sys_param_update_var_addr(THRUST_TO_PWM_C6, &coeff_thrust_to_cmd[5]);
	set_sys_param_update_var_addr(THRUST_MAX, &motor_thrust_max);

	/* load local variables previously stored in internal flash */
	get_sys_param_float(MR_GEO_GAIN_ROLL_P, &krx);
	get_sys_param_float(MR_GEO_GAIN_ROLL_D, &kwx);
	get_sys_param_float(MR_GEO_GAIN_PITCH_P, &kry);
	get_sys_param_float(MR_GEO_GAIN_PITCH_D, &kwy);
	get_sys_param_float(MR_GEO_GAIN_YAW_P, &krz);
	get_sys_param_float(MR_GEO_GAIN_YAW_D, &kwz);
	get_sys_param_float(MR_GEO_GAIN_RATE_YAW, &yaw_rate_ctrl_gain);
	get_sys_param_float(MR_GEO_GAIN_POS_X, &kpx);
	get_sys_param_float(MR_GEO_GAIN_VEL_X, &kvx);
	get_sys_param_float(MR_GEO_GAIN_POS_Y, &kpy);
	get_sys_param_float(MR_GEO_GAIN_VEL_Y, &kvy);
	get_sys_param_float(MR_GEO_GAIN_POS_Z, &kpz);
	get_sys_param_float(MR_GEO_GAIN_VEL_Z, &kvz);
	get_sys_param_float(MR_GEO_GAIN_POS_X_I, &k_tracking_i_gain[0]);
	get_sys_param_float(MR_GEO_GAIN_POS_Y_I, &k_tracking_i_gain[1]);
	get_sys_param_float(MR_GEO_GAIN_POS_Z_I, &k_tracking_i_gain[2]);
	get_sys_param_float(MR_GEO_UAV_MASS, &uav_mass);
	get_sys_param_float(MR_GEO_INERTIA_JXX, &mat_data(J)[0 * 3 + 0]);
	get_sys_param_float(MR_GEO_INERTIA_JYY, &mat_data(J)[1 * 3 + 1]);
	get_sys_param_float(MR_GEO_INERTIA_JZZ, &mat_data(J)[2 * 3 + 2]);
	get_sys_param_float(PWM_TO_THRUST_C1, &coeff_cmd_to_thrust[0]);
	get_sys_param_float(PWM_TO_THRUST_C2, &coeff_cmd_to_thrust[1]);
	get_sys_param_float(PWM_TO_THRUST_C3, &coeff_cmd_to_thrust[2]);
	get_sys_param_float(PWM_TO_THRUST_C4, &coeff_cmd_to_thrust[3]);
	get_sys_param_float(PWM_TO_THRUST_C5, &coeff_cmd_to_thrust[4]);
	get_sys_param_float(PWM_TO_THRUST_C6, &coeff_cmd_to_thrust[5]);
	get_sys_param_float(THRUST_TO_PWM_C1, &coeff_thrust_to_cmd[0]);
	get_sys_param_float(THRUST_TO_PWM_C2, &coeff_thrust_to_cmd[1]);
	get_sys_param_float(THRUST_TO_PWM_C3, &coeff_thrust_to_cmd[2]);
	get_sys_param_float(THRUST_TO_PWM_C4, &coeff_thrust_to_cmd[3]);
	get_sys_param_float(THRUST_TO_PWM_C5, &coeff_thrust_to_cmd[4]);
	get_sys_param_float(THRUST_TO_PWM_C6, &coeff_thrust_to_cmd[5]);
	get_sys_param_float(THRUST_MAX, &motor_thrust_max);



	set_motor_max_thrust(motor_thrust_max);
	set_motor_cmd_to_thrust_coeff(coeff_cmd_to_thrust[0], coeff_cmd_to_thrust[1], coeff_cmd_to_thrust[2],
								  coeff_cmd_to_thrust[3], coeff_cmd_to_thrust[4], coeff_cmd_to_thrust[5]);
	set_motor_thrust_to_cmd_coeff(coeff_thrust_to_cmd[0], coeff_thrust_to_cmd[1], coeff_thrust_to_cmd[2],
								  coeff_thrust_to_cmd[3], coeff_thrust_to_cmd[4], coeff_thrust_to_cmd[5]);
}

void estimate_uav_dynamics(float *gyro, float *moments, float *m_rot_frame)
{
	static float angular_vel_last[3] = {0.0f};
	float angular_accel[3];

	angular_accel[0] = (gyro[0] - angular_vel_last[0]) / dt;
	angular_accel[1] = (gyro[1] - angular_vel_last[1]) / dt;
	angular_accel[2] = (gyro[2] - angular_vel_last[2]) / dt;
	angular_vel_last[0] = gyro[0];
	angular_vel_last[1] = gyro[1];
	angular_vel_last[2] = gyro[2];

	lpf_first_order(angular_accel[0], &mat_data(W_dot)[0], 0.01);
	lpf_first_order(angular_accel[1], &mat_data(W_dot)[1], 0.01);
	lpf_first_order(angular_accel[2], &mat_data(W_dot)[2], 0.01);

	// J* W_dot
	MAT_MULT(&J, &W_dot, &JWdot);
	// W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(mat_data(W), mat_data(JW), mat_data(WJW));
	// M = J * W_dot + W X (J * W)
	MAT_ADD(&JWdot, &WJW, &M);

	m_rot_frame[0] = mat_data(JWdot)[0];
	m_rot_frame[1] = mat_data(JWdot)[1];
	m_rot_frame[2] = mat_data(JWdot)[2];
	moments[0] = mat_data(M)[0];
	moments[1] = mat_data(M)[1];
	moments[2] = mat_data(M)[2];
}

void reset_geometry_tracking_error_integral(void)
{
	tracking_error_integral[0] = 0.0f;
	tracking_error_integral[1] = 0.0f;
	tracking_error_integral[2] = 0.0f;
}

void geometry_manual_ctrl(euler_t *rc, float *attitude_q, float *gyro, float *output_moments,
						  bool heading_present)
{
	/* convert radio command (euler angle) to rotation matrix */
	euler_to_rotation_matrix(rc, mat_data(Rd), mat_data(Rtd));

	/* W (angular velocity) */
	mat_data(W)[0] = gyro[0];
	mat_data(W)[1] = gyro[1];
	mat_data(W)[2] = gyro[2];

	/* set Wd and Wd_dot to 0 since there is no predefined trajectory */
	mat_data(Wd)[0] = 0.0f;
	mat_data(Wd)[1] = 0.0f;
	mat_data(Wd)[2] = 0.0f;
	mat_data(Wd_dot)[0] = 0.0f;
	mat_data(Wd_dot)[1] = 0.0f;
	mat_data(Wd_dot)[2] = 0.0f;

	float _krz, _kwz; // switch between full heading control and yaw rate control

	/* switch to yaw rate control mode if no heading information provided */
	if (heading_present == false)
	{
		/* yaw rate control only */
		_krz = 0.0f;
		_kwz = yaw_rate_ctrl_gain;
		mat_data(Wd)[2] = rc->yaw; // set yaw rate desired value
	}
	else
	{
		_krz = krz;
		_kwz = kwz;
	}

	/* calculate attitude error eR */
	MAT_MULT(&Rtd, &R, &RtdR);
	MAT_MULT(&Rt, &Rd, &RtRd);
	MAT_SUB(&RtdR, &RtRd, &eR_mat);
	vee_map_3x3(mat_data(eR_mat), mat_data(eR));
	mat_data(eR)[0] *= 0.5f;
	mat_data(eR)[1] *= 0.5f;
	mat_data(eR)[2] *= 0.5f;

	/* calculate attitude rate error eW */
	// MAT_MULT(&Rt, &Rd, &RtRd); //the term is duplicated
	MAT_MULT(&RtRd, &Wd, &RtRdWd);
	MAT_SUB(&W, &RtRdWd, &eW);

	/* calculate the inertia feedfoward term */
	// W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(mat_data(W), mat_data(JW), mat_data(WJW));
	mat_data(inertia_effect)[0] = mat_data(WJW)[0];
	mat_data(inertia_effect)[1] = mat_data(WJW)[1];
	mat_data(inertia_effect)[2] = mat_data(WJW)[2];

	/* control input M1, M2, M3 */
	output_moments[0] = -krx * mat_data(eR)[0] - kwx * mat_data(eW)[0] + mat_data(inertia_effect)[0];
	output_moments[1] = -kry * mat_data(eR)[1] - kwy * mat_data(eW)[1] + mat_data(inertia_effect)[1];
	output_moments[2] = -_krz * mat_data(eR)[2] - _kwz * mat_data(eW)[2] + mat_data(inertia_effect)[2];
}

void geometry_tracking_ctrl(euler_t *rc, float *attitude_q, float *gyro, float *acc,  
							float *pos_des_enu, float *vel_des_enu, float *accel_ff_enu,
							float *curr_pos_ned, float *curr_vel_ned, float *output_moments,
							float *output_force, bool manual_flight)
{
	/* ex = x - xd */
	float pos_des_ned[3];
	assign_vector_3x1_enu_to_ned(pos_des_ned, pos_des_enu);
	pos_error[0] = curr_pos_ned[0] - pos_des_ned[0];
	pos_error[1] = curr_pos_ned[1] - pos_des_ned[1];
	pos_error[2] = curr_pos_ned[2] - pos_des_ned[2];

	/* ev = v - vd */
	float vel_des_ned[3];
	assign_vector_3x1_enu_to_ned(vel_des_ned, vel_des_enu);
	vel_error[0] = curr_vel_ned[0] - vel_des_ned[0];
	vel_error[1] = curr_vel_ned[1] - vel_des_ned[1];
	vel_error[2] = curr_vel_ned[2] - vel_des_ned[2];


	tracking_error_integral[0] += k_tracking_i_gain[0] * (pos_error[0]) * dt;
	tracking_error_integral[1] += k_tracking_i_gain[1] * (pos_error[1]) * dt;
	tracking_error_integral[2] += k_tracking_i_gain[2] * (pos_error[2]) * dt;

	bound_float(&tracking_error_integral[0], 150, -150);
	bound_float(&tracking_error_integral[1], 150, -150);
	bound_float(&tracking_error_integral[2], 50, -50);

	/* force feedforward control */
	float accel_ff_ned[3] = {0.0f};
	float force_ff_ned[3] = {0.0f};
	assign_vector_3x1_enu_to_ned(accel_ff_ned, accel_ff_enu);

#if (SELECT_FORCE_CONTROLLER_ESTIMATOR == FORCE_EMK)

	/* with mass of uav known */
	force_ff_ned[0] = uav_mass * accel_ff_ned[0];
	force_ff_ned[1] = uav_mass * accel_ff_ned[1];
	force_ff_ned[2] = uav_mass * (accel_ff_ned[2] - 9.81);

#elif (SELECT_FORCE_CONTROLLER_ESTIMATOR == FORCE_ADAPTIVE)

	force_ff_ctrl_use_adaptive_ICL(accel_ff_ned, force_ff_ned, pos_error, vel_error, curr_vel_ned, acc);

#endif

	mat_data(kxex_kvev_mge3_mxd_dot_dot)[0] = -kpx * pos_error[0] - kvx * vel_error[0] +
											  force_ff_ned[0] - tracking_error_integral[0];
	mat_data(kxex_kvev_mge3_mxd_dot_dot)[1] = -kpy * pos_error[1] - kvy * vel_error[1] +
											  force_ff_ned[1] - tracking_error_integral[1];
	mat_data(kxex_kvev_mge3_mxd_dot_dot)[2] = -kpz * pos_error[2] - kvz * vel_error[2] +
											  force_ff_ned[2] - tracking_error_integral[2];

	/* calculate the denominator of b3d */
	float b3d_denominator; // caution: this term should not be 0
	norm_3x1(mat_data(kxex_kvev_mge3_mxd_dot_dot), &b3d_denominator);
	b3d_denominator = -1.0f / b3d_denominator;

	if (manual_flight == true)
	{
		/* enable altitude control only, control roll and pitch manually */
		// convert radio command (euler angle) to rotation matrix
		euler_to_rotation_matrix(rc, mat_data(Rd), mat_data(Rtd));
	}
	else
	{
		/* enable tracking control for x and y axis */
		// b1d
		mat_data(b1d)[0] = arm_cos_f32(rc->yaw);
		mat_data(b1d)[1] = arm_sin_f32(rc->yaw);
		mat_data(b1d)[2] = 0.0f;
		// b3d = -kxex_kvev_mge3_mxd_dot_dot / ||kxex_kvev_mge3_mxd_dot_dot||
		mat_data(b3d)[0] = mat_data(kxex_kvev_mge3_mxd_dot_dot)[0] * b3d_denominator;
		mat_data(b3d)[1] = mat_data(kxex_kvev_mge3_mxd_dot_dot)[1] * b3d_denominator;
		mat_data(b3d)[2] = mat_data(kxex_kvev_mge3_mxd_dot_dot)[2] * b3d_denominator;
		// b2d = b3d X b1d / ||b3d X b1d||
		cross_product_3x1(mat_data(b3d), mat_data(b1d), mat_data(b2d));
		normalize_3x1(mat_data(b2d));
		/* proj[b1d] = b2d X b3d */
		cross_product_3x1(mat_data(b2d), mat_data(b3d), mat_data(b1d));

		// Rd = [b1d; b3d X b1d; b3d]
		mat_data(Rd)[0 * 3 + 0] = mat_data(b1d)[0];
		mat_data(Rd)[1 * 3 + 0] = mat_data(b1d)[1];
		mat_data(Rd)[2 * 3 + 0] = mat_data(b1d)[2];
		mat_data(Rd)[0 * 3 + 1] = mat_data(b2d)[0];
		mat_data(Rd)[1 * 3 + 1] = mat_data(b2d)[1];
		mat_data(Rd)[2 * 3 + 1] = mat_data(b2d)[2];
		mat_data(Rd)[0 * 3 + 2] = mat_data(b3d)[0];
		mat_data(Rd)[1 * 3 + 2] = mat_data(b3d)[1];
		mat_data(Rd)[2 * 3 + 2] = mat_data(b3d)[2];

		// transpose(Rd)
		mat_data(Rtd)[0 * 3 + 0] = mat_data(Rd)[0 * 3 + 0];
		mat_data(Rtd)[1 * 3 + 0] = mat_data(Rd)[0 * 3 + 1];
		mat_data(Rtd)[2 * 3 + 0] = mat_data(Rd)[0 * 3 + 2];
		mat_data(Rtd)[0 * 3 + 1] = mat_data(Rd)[1 * 3 + 0];
		mat_data(Rtd)[1 * 3 + 1] = mat_data(Rd)[1 * 3 + 1];
		mat_data(Rtd)[2 * 3 + 1] = mat_data(Rd)[1 * 3 + 2];
		mat_data(Rtd)[0 * 3 + 2] = mat_data(Rd)[2 * 3 + 0];
		mat_data(Rtd)[1 * 3 + 2] = mat_data(Rd)[2 * 3 + 1];
		mat_data(Rtd)[2 * 3 + 2] = mat_data(Rd)[2 * 3 + 2];
	}

	/* R * e3 */
	mat_data(Re3)[0] = mat_data(R)[0 * 3 + 2];
	mat_data(Re3)[1] = mat_data(R)[1 * 3 + 2];
	mat_data(Re3)[2] = mat_data(R)[2 * 3 + 2];

	/* f = -(-kx * ex - kv * ev - mge3 + m * x_d_dot_dot) . (R * e3) */
	float neg_kxex_kvev_mge3_mxd_dot_dot[3];
	neg_kxex_kvev_mge3_mxd_dot_dot[0] = -mat_data(kxex_kvev_mge3_mxd_dot_dot)[0];
	neg_kxex_kvev_mge3_mxd_dot_dot[1] = -mat_data(kxex_kvev_mge3_mxd_dot_dot)[1];
	neg_kxex_kvev_mge3_mxd_dot_dot[2] = -mat_data(kxex_kvev_mge3_mxd_dot_dot)[2];
	arm_dot_prod_f32(neg_kxex_kvev_mge3_mxd_dot_dot, mat_data(Re3), 3, output_force);


	/* save current force for ICL */
	mat_data(curr_force)[0] = (*output_force)*mat_data(Re3)[0];
	mat_data(curr_force)[1] = (*output_force)*mat_data(Re3)[1];
	mat_data(curr_force)[2] = (*output_force)*mat_data(Re3)[2];


	/* W (angular velocity) */
	mat_data(W)[0] = gyro[0];
	mat_data(W)[1] = gyro[1];
	mat_data(W)[2] = gyro[2];

	/* set Wd and Wd_dot to 0 since there is no predefined trajectory */
	mat_data(Wd)[0] = 0.0f;
	mat_data(Wd)[1] = 0.0f;
	mat_data(Wd)[2] = 0.0f;
	mat_data(Wd_dot)[0] = 0.0f;
	mat_data(Wd_dot)[1] = 0.0f;
	mat_data(Wd_dot)[2] = 0.0f;

	/* calculate attitude error eR */
	MAT_MULT(&Rtd, &R, &RtdR);
	MAT_MULT(&Rt, &Rd, &RtRd);
	MAT_SUB(&RtdR, &RtRd, &eR_mat);
	vee_map_3x3(mat_data(eR_mat), mat_data(eR));
	mat_data(eR)[0] *= 0.5f;
	mat_data(eR)[1] *= 0.5f;
	mat_data(eR)[2] *= 0.5f;

	/* calculate attitude rate error eW */
	// MAT_MULT(&Rt, &Rd, &RtRd); //the term is duplicated
	MAT_MULT(&RtRd, &Wd, &RtRdWd);
	MAT_SUB(&W, &RtRdWd, &eW);

	/* calculate the inertia feedfoward term */
	// W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(mat_data(W), mat_data(JW), mat_data(WJW));
	mat_data(inertia_effect)[0] = mat_data(WJW)[0];
	mat_data(inertia_effect)[1] = mat_data(WJW)[1];
	mat_data(inertia_effect)[2] = mat_data(WJW)[2];

	/* ICL controller */
#if (SELECT_MOMENT_CONTROLLER_ESTIMATOR == MOMENT_EMK)

	/* control input M1, M2, M3 */
	output_moments[0] = -krx * mat_data(eR)[0] - kwx * mat_data(eW)[0] + mat_data(inertia_effect)[0];
	output_moments[1] = -kry * mat_data(eR)[1] - kwy * mat_data(eW)[1] + mat_data(inertia_effect)[1];
	output_moments[2] = -krz * mat_data(eR)[2] - kwz * mat_data(eW)[2] + mat_data(inertia_effect)[2];

#elif (SELECT_MOMENT_CONTROLLER_ESTIMATOR == MOMENT_ADAPTIVE)
	/*adaptive*/
	// Y1 of Y
	//  [0	-f	]
	//  [f	0	]
	//  [0	0	]
	mat_data(Y)[0 * 8 + 0] = 0;
	mat_data(Y)[1 * 8 + 0] = *output_force;
	mat_data(Y)[2 * 8 + 0] = 0;
	mat_data(Y)[0 * 8 + 1] = -*output_force;
	mat_data(Y)[1 * 8 + 1] = 0;
	mat_data(Y)[2 * 8 + 1] = 0;
	// Y2 of Y
	// wb: omega_bar
	// w: angular velocity
	//[0	,    -w1*w2	,w1*w2	, 0 -w0*w2	,  0+w0*w1	,-w2^2+w1^2	]
	//[w0*w2	,0   	,-w0*w2	, 0 +w1*w2	,-w0^2+w2^2	, 0-w0*w1	]
	//[-w0*w1	,w0*w1	,0  	,-w1^2+w0^2	,  0-w1*w2	, 0+w0*w2	]
	mat_data(Y)[0 * 8 + 2] = 0;
	mat_data(Y)[1 * 8 + 2] = mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 2] = -mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[0 * 8 + 3] = -mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 3] = 0;
	mat_data(Y)[2 * 8 + 3] = mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[0 * 8 + 4] = mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 4] = -mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 4] = 0;
	mat_data(Y)[0 * 8 + 5] = -mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 5] = mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 5] = -mat_data(W)[1] * mat_data(W)[1] + mat_data(W)[0] * mat_data(W)[0];
	mat_data(Y)[0 * 8 + 6] = mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[1 * 8 + 6] = -mat_data(W)[0] * mat_data(W)[0] + mat_data(W)[2] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 6] = -mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[0 * 8 + 7] = -mat_data(W)[2] * mat_data(W)[2] + mat_data(W)[1] * mat_data(W)[1];
	mat_data(Y)[1 * 8 + 7] = -mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[2 * 8 + 7] = mat_data(W)[0] * mat_data(W)[2];
	// Yt*(eW_c2eR)
	MAT_TRANS(&Y, &Yt);
	mat_data(eW_c2eR)[0] = c2 * mat_data(eR)[0] + mat_data(eW)[0];
	mat_data(eW_c2eR)[1] = c2 * mat_data(eR)[1] + mat_data(eW)[1];
	mat_data(eW_c2eR)[2] = c2 * mat_data(eR)[2] + mat_data(eW)[2];
	MAT_MULT(&Yt, &eW_c2eR, &Yt_eW_c2eR);

	// adaptivetheta_hat_dot
	for (int i = 0; i < 8; i++)
	{
		mat_data(theta_hat_dot)[i] = -adaptive_gamma[i] * mat_data(Yt_eW_c2eR)[i];
	}
	// theta update
	mat_data(theta)[0] = mat_data(theta)[0] + mat_data(theta_hat_dot)[0];
	mat_data(theta)[1] = mat_data(theta)[1] + mat_data(theta_hat_dot)[1];
	mat_data(theta)[2] = mat_data(theta)[2] + mat_data(theta_hat_dot)[2];
	mat_data(theta)[3] = mat_data(theta)[3] + mat_data(theta_hat_dot)[3];
	mat_data(theta)[4] = mat_data(theta)[4] + mat_data(theta_hat_dot)[4];
	mat_data(theta)[5] = mat_data(theta)[5] + mat_data(theta_hat_dot)[5];
	mat_data(theta)[6] = mat_data(theta)[6] + mat_data(theta_hat_dot)[6];
	mat_data(theta)[7] = mat_data(theta)[7] + mat_data(theta_hat_dot)[7];

	// Y_theta
	MAT_MULT(&Y, &theta, &Ytheta);

	/* control input M1, M2, M3 */
	output_moments[0] = -krx * mat_data(eR)[0] - kwx * mat_data(eW)[0] + mat_data(Ytheta)[0];
	output_moments[1] = -kry * mat_data(eR)[1] - kwy * mat_data(eW)[1] + mat_data(Ytheta)[1];
	output_moments[2] = -krz * mat_data(eR)[2] - kwz * mat_data(eW)[2] + mat_data(Ytheta)[2];

#else
	/*adaptive*/
	// Y_cog of Y
	//  [0	-f	]
	//  [f	0	]
	//  [0	0	]
	//
	mat_data(Y)[0 * 8 + 0] = 0;
	mat_data(Y)[1 * 8 + 0] = *output_force;
	mat_data(Y)[2 * 8 + 0] = 0;
	mat_data(Y)[0 * 8 + 1] = -*output_force;
	mat_data(Y)[1 * 8 + 1] = 0;
	mat_data(Y)[2 * 8 + 1] = 0;

	// Y_J of Y
	// wb: omega_bar
	// w: angular velocity
	//[0	,    -w1*w2	,w1*w2	, 0 -w0*w2	,  0+w0*w1	,-w2^2+w1^2	]
	//[w0*w2	,0   	,-w0*w2	, 0 +w1*w2	,-w0^2+w2^2	, 0-w0*w1	]
	//[-w0*w1	,w0*w1	,0  	,-w1^2+w0^2	,  0-w1*w2	, 0+w0*w2	]
	mat_data(Y)[0 * 8 + 2] = 0;
	mat_data(Y)[1 * 8 + 2] = mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 2] = -mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[0 * 8 + 3] = -mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 3] = 0;
	mat_data(Y)[2 * 8 + 3] = mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[0 * 8 + 4] = mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 4] = -mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 4] = 0;
	mat_data(Y)[0 * 8 + 5] = -mat_data(W)[0] * mat_data(W)[2];
	mat_data(Y)[1 * 8 + 5] = mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 5] = -mat_data(W)[1] * mat_data(W)[1] + mat_data(W)[0] * mat_data(W)[0];
	mat_data(Y)[0 * 8 + 6] = mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[1 * 8 + 6] = -mat_data(W)[0] * mat_data(W)[0] + mat_data(W)[2] * mat_data(W)[2];
	mat_data(Y)[2 * 8 + 6] = -mat_data(W)[1] * mat_data(W)[2];
	mat_data(Y)[0 * 8 + 7] = -mat_data(W)[2] * mat_data(W)[2] + mat_data(W)[1] * mat_data(W)[1];
	mat_data(Y)[1 * 8 + 7] = -mat_data(W)[0] * mat_data(W)[1];
	mat_data(Y)[2 * 8 + 7] = mat_data(W)[0] * mat_data(W)[2];

	// Yt*(eW_c2eR)
	MAT_TRANS(&Y, &Yt);
	mat_data(eW_c2eR)[0] = c2 * mat_data(eR)[0] + mat_data(eW)[0];
	mat_data(eW_c2eR)[1] = c2 * mat_data(eR)[1] + mat_data(eW)[1];
	mat_data(eW_c2eR)[2] = c2 * mat_data(eR)[2] + mat_data(eW)[2];
	MAT_MULT(&Yt, &eW_c2eR, &Yt_eW_c2eR);

	// theta_hat_dot
	for (int i = 0; i < 8; i++)
	{
		mat_data(adaptive_theta_hat_dot)[i] = -adaptive_gamma[i] * mat_data(Yt_eW_c2eR)[i];
	}

	/*CL*/
	// Y_cog of YCL
	// [0	-f	]
	// [f	0	]
	// [0	0	]
	mat_data(Y_CL)[0 * 8 + 0] = 0;
	mat_data(Y_CL)[1 * 8 + 0] = output_force_last;
	mat_data(Y_CL)[2 * 8 + 0] = 0;
	mat_data(Y_CL)[0 * 8 + 1] = -output_force_last;
	mat_data(Y_CL)[1 * 8 + 1] = 0;
	mat_data(Y_CL)[2 * 8 + 1] = 0;

	// Y_J of YCL
	// Since we don't need to estimate the MOI, so the Y_J_cl term will be zero
	mat_data(Y_CL)[0 * 8 + 2] = 0;
	mat_data(Y_CL)[1 * 8 + 2] = 0;
	mat_data(Y_CL)[2 * 8 + 2] = 0;
	mat_data(Y_CL)[0 * 8 + 3] = 0;
	mat_data(Y_CL)[1 * 8 + 3] = 0;
	mat_data(Y_CL)[2 * 8 + 3] = 0;
	mat_data(Y_CL)[0 * 8 + 4] = 0;
	mat_data(Y_CL)[1 * 8 + 4] = 0;
	mat_data(Y_CL)[2 * 8 + 4] = 0;
	mat_data(Y_CL)[0 * 8 + 5] = 0;
	mat_data(Y_CL)[1 * 8 + 5] = 0;
	mat_data(Y_CL)[2 * 8 + 5] = 0;
	mat_data(Y_CL)[0 * 8 + 6] = 0;
	mat_data(Y_CL)[1 * 8 + 6] = 0;
	mat_data(Y_CL)[2 * 8 + 6] = 0;
	mat_data(Y_CL)[0 * 8 + 7] = 0;
	mat_data(Y_CL)[1 * 8 + 7] = 0;
	mat_data(Y_CL)[2 * 8 + 7] = 0;

	// MAT_ADD(&Y_CLdt ,&W_dot_Matrix ,&sigma_array[ICL_sigma_index].Y_CL );
	sigma_array[ICL_sigma_index].Y_CL = Y_CL;
	sigma_array[ICL_sigma_index].Mb = M_last;
	ICL_sigma_index++;
	ICL_sigma_index %= ICL_N;

	//============= calculate ICL sigma control term ===============
	mat_data(ICL_control_term)[0] = 0;
	mat_data(ICL_control_term)[1] = 0;
	mat_data(ICL_control_term)[2] = 0;
	mat_data(ICL_control_term)[3] = 0;
	mat_data(ICL_control_term)[4] = 0;
	mat_data(ICL_control_term)[5] = 0;
	mat_data(ICL_control_term)[6] = 0;
	mat_data(ICL_control_term)[7] = 0;

	for (int i = 0; i < ICL_N; i++)
	{
		// Y_CL.T*(Mb - Y_CL_theta)
		/*
		ICL_control_term, 8, 1		//x
		Y_CLtheta ,3, ,1			//Y_CL*theta
		Mb_Y_CLtheta ,3 ,1			//Mb-Y_CL*theta
		Y_CLT ,8 ,3					//Y_CL.transpose
		Y_CLT_Mb_Y_CLtheta ,8 ,1	//Y_CLT*Mb-Y_CL*theta
		*/
		// Y_CL_theta
		MAT_MULT(&sigma_array[i].Y_CL, &theta, &Y_CLtheta);
		// Mb - Y_CL_theta
		MAT_SUB(&sigma_array[i].Mb, &Y_CLtheta, &Mb_Y_CLtheta);
		// Y_CL.T*(Mb - Y_CL_theta)
		MAT_TRANS(&sigma_array[i].Y_CL, &Y_CLT);
		MAT_MULT(&Y_CLT, &Mb_Y_CLtheta, &Y_CLT_Mb_Y_CLtheta);

		mat_data(ICL_control_term)[0] = mat_data(ICL_control_term)[0] + mat_data(Y_CLT_Mb_Y_CLtheta)[0];
		mat_data(ICL_control_term)[1] = mat_data(ICL_control_term)[1] + mat_data(Y_CLT_Mb_Y_CLtheta)[1];
		mat_data(ICL_control_term)[2] = mat_data(ICL_control_term)[2] + mat_data(Y_CLT_Mb_Y_CLtheta)[2];
		mat_data(ICL_control_term)[3] = mat_data(ICL_control_term)[3] + mat_data(Y_CLT_Mb_Y_CLtheta)[3];
		mat_data(ICL_control_term)[4] = mat_data(ICL_control_term)[4] + mat_data(Y_CLT_Mb_Y_CLtheta)[4];
		mat_data(ICL_control_term)[5] = mat_data(ICL_control_term)[5] + mat_data(Y_CLT_Mb_Y_CLtheta)[5];
		mat_data(ICL_control_term)[6] = mat_data(ICL_control_term)[6] + mat_data(Y_CLT_Mb_Y_CLtheta)[6];
		mat_data(ICL_control_term)[7] = mat_data(ICL_control_term)[7] + mat_data(Y_CLT_Mb_Y_CLtheta)[7];
	}

	mat_data(ICL_theta_hat_dot)[0] = adaptive_gamma_k_icl[0] * mat_data(ICL_control_term)[0];
	mat_data(ICL_theta_hat_dot)[1] = adaptive_gamma_k_icl[1] * mat_data(ICL_control_term)[1];
	mat_data(ICL_theta_hat_dot)[2] = adaptive_gamma_k_icl[2] * mat_data(ICL_control_term)[2];
	mat_data(ICL_theta_hat_dot)[3] = adaptive_gamma_k_icl[3] * mat_data(ICL_control_term)[3];
	mat_data(ICL_theta_hat_dot)[4] = adaptive_gamma_k_icl[4] * mat_data(ICL_control_term)[4];
	mat_data(ICL_theta_hat_dot)[5] = adaptive_gamma_k_icl[5] * mat_data(ICL_control_term)[5];
	mat_data(ICL_theta_hat_dot)[6] = adaptive_gamma_k_icl[6] * mat_data(ICL_control_term)[6];
	mat_data(ICL_theta_hat_dot)[7] = adaptive_gamma_k_icl[7] * mat_data(ICL_control_term)[7];
	// theta update
	mat_data(theta)[0] = mat_data(theta)[0] + mat_data(adaptive_theta_hat_dot)[0] + mat_data(ICL_theta_hat_dot)[0];
	mat_data(theta)[1] = mat_data(theta)[1] + mat_data(adaptive_theta_hat_dot)[1] + mat_data(ICL_theta_hat_dot)[1];
	mat_data(theta)[2] = mat_data(theta)[2] + mat_data(adaptive_theta_hat_dot)[2] + mat_data(ICL_theta_hat_dot)[2];
	mat_data(theta)[3] = mat_data(theta)[3] + mat_data(adaptive_theta_hat_dot)[3] + mat_data(ICL_theta_hat_dot)[3];
	mat_data(theta)[4] = mat_data(theta)[4] + mat_data(adaptive_theta_hat_dot)[4] + mat_data(ICL_theta_hat_dot)[4];
	mat_data(theta)[5] = mat_data(theta)[5] + mat_data(adaptive_theta_hat_dot)[5] + mat_data(ICL_theta_hat_dot)[5];
	mat_data(theta)[6] = mat_data(theta)[6] + mat_data(adaptive_theta_hat_dot)[6] + mat_data(ICL_theta_hat_dot)[6];
	mat_data(theta)[7] = mat_data(theta)[7] + mat_data(adaptive_theta_hat_dot)[7] + mat_data(ICL_theta_hat_dot)[7];
	// Y_theta
	MAT_MULT(&Y, &theta, &Ytheta);

	/* control input M1, M2, M3 */
	output_moments[0] = -krx * mat_data(eR)[0] - kwx * mat_data(eW)[0] + mat_data(Ytheta)[0];
	output_moments[1] = -kry * mat_data(eR)[1] - kwy * mat_data(eW)[1] + mat_data(Ytheta)[1];
	output_moments[2] = -krz * mat_data(eR)[2] - kwz * mat_data(eW)[2] + mat_data(Ytheta)[2];

	mat_data(M_last)[0] = output_moments[0];
	mat_data(M_last)[1] = output_moments[1];
	mat_data(M_last)[2] = output_moments[2];

	output_force_last = *output_force;

#endif
}

#define l_div_4 (0.25f * (1.0f / MOTOR_TO_CG_LENGTH_M))
#define b_div_4 (+0.25f * (1.0f / COEFFICIENT_YAW))
void mr_geometry_ctrl_thrust_allocation(float *moment, float total_force)
{
	/* quadrotor thrust allocation */
	float distributed_force = total_force *= 0.25; // split force to 4 motors
	float motor_force[4];
	motor_force[0] = -l_div_4 * moment[0] + l_div_4 * moment[1] +
					 -b_div_4 * moment[2] + distributed_force;
	motor_force[1] = +l_div_4 * moment[0] + l_div_4 * moment[1] +
					 +b_div_4 * moment[2] + distributed_force;
	motor_force[2] = +l_div_4 * moment[0] - l_div_4 * moment[1] +
					 -b_div_4 * moment[2] + distributed_force;
	motor_force[3] = -l_div_4 * moment[0] - l_div_4 * moment[1] +
					 +b_div_4 * moment[2] + distributed_force;

	set_motor_value(MOTOR1, convert_motor_thrust_to_cmd(motor_force[0]));
	set_motor_value(MOTOR2, convert_motor_thrust_to_cmd(motor_force[1]));
	set_motor_value(MOTOR3, convert_motor_thrust_to_cmd(motor_force[2]));
	set_motor_value(MOTOR4, convert_motor_thrust_to_cmd(motor_force[3]));
}

void multi_uav_geometry_ctrl_thrust_allocation(float *dis_control_outputs, float *moment, float total_force)
{
	/*CONTROL_OUTPUT 4 x 1 */
	mat_data(CONTROL_OUTPUT)[0] = total_force;
	mat_data(CONTROL_OUTPUT)[1] = moment[0];
	mat_data(CONTROL_OUTPUT)[2] = moment[1];
	mat_data(CONTROL_OUTPUT)[3] = moment[2];
	MAT_MULT(&DIS_M, &CONTROL_OUTPUT, &DIS_OUTPUT);
	for (int i = 0; i < 8; i++)
	{
		dis_control_outputs[i] = mat_data(DIS_OUTPUT)[i];
	}
}

void rc_mode_handler_geometry_ctrl(radio_t *rc)
{
	static bool auto_flight_mode_last = false;

	multirotor_rc_special_function_handler(rc);

	if (rc->safety == true)
	{
		if (rc->auto_flight == true)
		{
			autopilot_set_mode(AUTOPILOT_HOVERING_MODE);
		}
		else
		{
			autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
		}
	}

	// if mode switched to auto-flight
	if (rc->auto_flight == true && auto_flight_mode_last != true)
	{
		autopilot_set_mode(AUTOPILOT_HOVERING_MODE);

		// set desired position to current position
		float curr_pos[3] = {0.0f};
		get_enu_position(curr_pos);
		autopilot_assign_pos_target(curr_pos[0], curr_pos[1], curr_pos[2]);
		autopilot_assign_zero_vel_target();		 // set desired velocity to zero
		autopilot_assign_zero_acc_feedforward(); // set acceleration feedforward to zero

		reset_geometry_tracking_error_integral();
	}

	if (rc->auto_flight == false)
	{
		autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
		autopilot_mission_reset();

		autopilot_assign_pos_target(0.0f, 0.0f, 0.0f);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();

		reset_geometry_tracking_error_integral();
	}

	auto_flight_mode_last = rc->auto_flight;
}

void multirotor_geometry_control(radio_t *rc)
{
	/* handle sensor failure and navigation system switching */
	navigation_manager_handler();

	/* check rc events */
	rc_mode_handler_geometry_ctrl(rc);

	/* get sensor status */
	bool xy_pos_available = is_xy_position_available();
	bool height_availabe = is_height_available();
	bool heading_available = is_heading_available();

	/* get imu data */
	float accel_lpf[3];
	float gyro_lpf[3];
	get_accel_lpf(accel_lpf);
	get_gyro_lpf(gyro_lpf);

	/* get attitude quaternion */
	float attitude_q[4];
	get_attitude_quaternion(attitude_q);

	/* get roll, pitch, yaw angles */
	float attitude_roll, attitude_pitch, attitude_yaw;
	get_attitude_euler_angles(&attitude_roll, &attitude_pitch, &attitude_yaw);

	/* get direction consine matrix of current attitude */
	get_rotation_matrix_b2i(&mat_data(R));
	get_rotation_matrix_i2b(&mat_data(Rt));

	/* prepare position and velocity data */
	float curr_pos_enu[3] = {0.0f}, curr_pos_ned[3] = {0.0f};
	float curr_vel_enu[3] = {0.0f}, curr_vel_ned[3] = {0.0f};
	get_enu_position(curr_pos_enu);
	get_enu_velocity(curr_vel_enu);
	assign_vector_3x1_enu_to_ned(curr_pos_ned, curr_pos_enu);
	assign_vector_3x1_enu_to_ned(curr_vel_ned, curr_vel_enu);

	/* prepare gyroscope data */
	float gyro[3] = {0.0};
	gyro[0] = deg_to_rad(gyro_lpf[0]);
	gyro[1] = deg_to_rad(gyro_lpf[1]);
	gyro[2] = deg_to_rad(gyro_lpf[2]);

	/* guidance loop (autopilot) */
	autopilot_guidance_handler(rc, curr_pos_enu, curr_vel_enu);

	/* prepare desired heading angle, position, velocity and acceleration feedforward */
	float desired_heading, pos_des_enu[3], vel_des_enu[3], accel_ff_enu[3];
	desired_heading = autopilot_get_heading_setpoint();
	autopilot_get_pos_setpoint(pos_des_enu);
	autopilot_get_vel_setpoint(vel_des_enu);
	autopilot_get_accel_feedforward(accel_ff_enu);

	/* prepare manual control attitude commands (euler angles) */
	euler_t attitude_cmd;
	attitude_cmd.roll = deg_to_rad(-rc->roll);
	attitude_cmd.pitch = deg_to_rad(-rc->pitch);
	if (heading_available == true)
	{
		// yaw control mode
		attitude_cmd.yaw = deg_to_rad(desired_heading);
	}
	else
	{
		// yaw rate control mode
		attitude_cmd.yaw = deg_to_rad(-rc->yaw);
	}

	float control_moments[3] = {0.0f}, control_force = 0.0f;

	if (rc->auto_flight == true && height_availabe && heading_available)
	{
		if (xy_pos_available == false)
		{
			height_ctrl_only = true;
		}

		/* auto-flight mode (position, velocity and attitude control) */
		geometry_tracking_ctrl(&attitude_cmd, attitude_q, gyro, accel_lpf,
							   pos_des_enu, vel_des_enu, accel_ff_enu,
							   curr_pos_ned, curr_vel_ned, control_moments,
							   &control_force, height_ctrl_only);
	}
	else
	{
		/* manual flight mode (attitude control only) */
		geometry_manual_ctrl(&attitude_cmd, attitude_q, gyro, control_moments,
							 heading_available);


		/* generate total thrust for quadrotor (open-loop) */
		control_force = 4.0f * convert_motor_cmd_to_thrust(rc->throttle * 0.01 /* [%] */);


		mat_data(M_last)[0] = control_moments[0];
		mat_data(M_last)[1] = control_moments[1];
		mat_data(M_last)[2] = control_moments[2];


		/* R * e3 */
		mat_data(Re3)[0] = mat_data(R)[0 * 3 + 2];
		mat_data(Re3)[1] = mat_data(R)[1 * 3 + 2];
		mat_data(Re3)[2] = mat_data(R)[2 * 3 + 2];

		/* save current force for ICL */
		mat_data(curr_force)[0] = (control_force)*mat_data(Re3)[0];
		mat_data(curr_force)[1] = (control_force)*mat_data(Re3)[1];
		mat_data(curr_force)[2] = (control_force)*mat_data(Re3)[2];

		if (SELECT_UAV_MISSION == TRANSPORTATION)
		{
			/* four uav*/
			control_force = 2 * control_force;
		}


		
		/*CoG estimation*/
		mat_data(Y_CL)[0 * 8 + 0] = 0;
		mat_data(Y_CL)[1 * 8 + 0] = control_force;
		mat_data(Y_CL)[2 * 8 + 0] = 0;
		mat_data(Y_CL)[0 * 8 + 1] = -control_force;
		mat_data(Y_CL)[1 * 8 + 1] = 0;
		mat_data(Y_CL)[2 * 8 + 1] = 0;

		// Y_J of YCL
		// Since we don't need to estimate the MOI, so the Y_J_cl term will be zero
		mat_data(Y_CL)[0 * 8 + 2] = 0;
		mat_data(Y_CL)[1 * 8 + 2] = 0;
		mat_data(Y_CL)[2 * 8 + 2] = 0;
		mat_data(Y_CL)[0 * 8 + 3] = 0;
		mat_data(Y_CL)[1 * 8 + 3] = 0;
		mat_data(Y_CL)[2 * 8 + 3] = 0;
		mat_data(Y_CL)[0 * 8 + 4] = 0;
		mat_data(Y_CL)[1 * 8 + 4] = 0;
		mat_data(Y_CL)[2 * 8 + 4] = 0;
		mat_data(Y_CL)[0 * 8 + 5] = 0;
		mat_data(Y_CL)[1 * 8 + 5] = 0;
		mat_data(Y_CL)[2 * 8 + 5] = 0;
		mat_data(Y_CL)[0 * 8 + 6] = 0;
		mat_data(Y_CL)[1 * 8 + 6] = 0;
		mat_data(Y_CL)[2 * 8 + 6] = 0;
		mat_data(Y_CL)[0 * 8 + 7] = 0;
		mat_data(Y_CL)[1 * 8 + 7] = 0;
		mat_data(Y_CL)[2 * 8 + 7] = 0;

		// MAT_ADD(&Y_CLdt ,&W_dot_Matrix ,&sigma_array[ICL_sigma_index].Y_CL );
		sigma_array[ICL_sigma_index].Y_CL = Y_CL;
		sigma_array[ICL_sigma_index].Mb = M_last;
		ICL_sigma_index++;
		ICL_sigma_index %= ICL_N;

		//============= calculate ICL sigma control term ===============
		mat_data(ICL_control_term)[0] = 0;
		mat_data(ICL_control_term)[1] = 0;
		mat_data(ICL_control_term)[2] = 0;
		mat_data(ICL_control_term)[3] = 0;
		mat_data(ICL_control_term)[4] = 0;
		mat_data(ICL_control_term)[5] = 0;
		mat_data(ICL_control_term)[6] = 0;
		mat_data(ICL_control_term)[7] = 0;

		for (int i = 0; i < ICL_N; i++)
		{
			// Y_CL.T*(Mb - Y_CL_theta)
			/*
			ICL_control_term, 8, 1		//x
			Y_CLtheta ,3, ,1			//Y_CL*theta
			Mb_Y_CLtheta ,3 ,1			//Mb-Y_CL*theta
			Y_CLT ,8 ,3					//Y_CL.transpose
			Y_CLT_Mb_Y_CLtheta ,8 ,1	//Y_CLT*Mb-Y_CL*theta
			*/
			// Y_CL_theta
			MAT_MULT(&sigma_array[i].Y_CL, &theta, &Y_CLtheta);
			// Mb - Y_CL_theta
			MAT_SUB(&sigma_array[i].Mb, &Y_CLtheta, &Mb_Y_CLtheta);
			// Y_CL.T*(Mb - Y_CL_theta)
			MAT_TRANS(&sigma_array[i].Y_CL, &Y_CLT);
			MAT_MULT(&Y_CLT, &Mb_Y_CLtheta, &Y_CLT_Mb_Y_CLtheta);

			mat_data(ICL_control_term)[0] = mat_data(ICL_control_term)[0] + mat_data(Y_CLT_Mb_Y_CLtheta)[0];
			mat_data(ICL_control_term)[1] = mat_data(ICL_control_term)[1] + mat_data(Y_CLT_Mb_Y_CLtheta)[1];
			mat_data(ICL_control_term)[2] = mat_data(ICL_control_term)[2] + mat_data(Y_CLT_Mb_Y_CLtheta)[2];
			mat_data(ICL_control_term)[3] = mat_data(ICL_control_term)[3] + mat_data(Y_CLT_Mb_Y_CLtheta)[3];
			mat_data(ICL_control_term)[4] = mat_data(ICL_control_term)[4] + mat_data(Y_CLT_Mb_Y_CLtheta)[4];
			mat_data(ICL_control_term)[5] = mat_data(ICL_control_term)[5] + mat_data(Y_CLT_Mb_Y_CLtheta)[5];
			mat_data(ICL_control_term)[6] = mat_data(ICL_control_term)[6] + mat_data(Y_CLT_Mb_Y_CLtheta)[6];
			mat_data(ICL_control_term)[7] = mat_data(ICL_control_term)[7] + mat_data(Y_CLT_Mb_Y_CLtheta)[7];
		}

		mat_data(ICL_theta_hat_dot)[0] = 0.000001 * mat_data(ICL_control_term)[0];
		mat_data(ICL_theta_hat_dot)[1] = 0.000001 * mat_data(ICL_control_term)[1];
		mat_data(ICL_theta_hat_dot)[2] = adaptive_gamma_k_icl[2] * mat_data(ICL_control_term)[2];
		mat_data(ICL_theta_hat_dot)[3] = adaptive_gamma_k_icl[3] * mat_data(ICL_control_term)[3];
		mat_data(ICL_theta_hat_dot)[4] = adaptive_gamma_k_icl[4] * mat_data(ICL_control_term)[4];
		mat_data(ICL_theta_hat_dot)[5] = adaptive_gamma_k_icl[5] * mat_data(ICL_control_term)[5];
		mat_data(ICL_theta_hat_dot)[6] = adaptive_gamma_k_icl[6] * mat_data(ICL_control_term)[6];
		mat_data(ICL_theta_hat_dot)[7] = adaptive_gamma_k_icl[7] * mat_data(ICL_control_term)[7];
		// theta update
		mat_data(theta)[0] = mat_data(theta)[0] + mat_data(adaptive_theta_hat_dot)[0] + mat_data(ICL_theta_hat_dot)[0];
		mat_data(theta)[1] = mat_data(theta)[1] + mat_data(adaptive_theta_hat_dot)[1] + mat_data(ICL_theta_hat_dot)[1];
		mat_data(theta)[2] = mat_data(theta)[2] + mat_data(adaptive_theta_hat_dot)[2] + mat_data(ICL_theta_hat_dot)[2];
		mat_data(theta)[3] = mat_data(theta)[3] + mat_data(adaptive_theta_hat_dot)[3] + mat_data(ICL_theta_hat_dot)[3];
		mat_data(theta)[4] = mat_data(theta)[4] + mat_data(adaptive_theta_hat_dot)[4] + mat_data(ICL_theta_hat_dot)[4];
		mat_data(theta)[5] = mat_data(theta)[5] + mat_data(adaptive_theta_hat_dot)[5] + mat_data(ICL_theta_hat_dot)[5];
		mat_data(theta)[6] = mat_data(theta)[6] + mat_data(adaptive_theta_hat_dot)[6] + mat_data(ICL_theta_hat_dot)[6];
		mat_data(theta)[7] = mat_data(theta)[7] + mat_data(adaptive_theta_hat_dot)[7] + mat_data(ICL_theta_hat_dot)[7];
		// Y_theta
		MAT_MULT(&Y, &theta, &Ytheta);

	}

	/*Master to slave force allocation*/

	if (rc->safety == true)
	{
		autopilot_assign_heading_target(attitude_yaw);
		barometer_set_sea_level();
		set_rgb_led_service_motor_lock_flag(true);
	}
	else
	{
		set_rgb_led_service_motor_lock_flag(false);
	}

	bool lock_motor = false;

	// lock motor if throttle values is lower than 10% during manual flight
	lock_motor |= check_motor_lock_condition(rc->throttle < 10.0f &&
											 autopilot_get_mode() == AUTOPILOT_MANUAL_FLIGHT_MODE);
	// lock motor if desired height is lower than threshold value in the takeoff mode
	lock_motor |= check_motor_lock_condition(pos_des_enu[2] < 0.10f &&
											 autopilot_get_mode() == AUTOPILOT_TAKEOFF_MODE);
	// lock motor if current position is very close to ground in the hovering mode
	lock_motor |= check_motor_lock_condition(curr_pos_enu[2] < 0.10f &&
											 autopilot_get_mode() == AUTOPILOT_HOVERING_MODE);
	// lock motor if motors are locked by autopilot
	lock_motor |= check_motor_lock_condition(autopilot_get_mode() == AUTOPILOT_MOTOR_LOCKED_MODE);
	// lock motor if radio safety botton is on
	lock_motor |= check_motor_lock_condition(rc->safety == true);

	/* Below part is the modified part for multiuav transportation 	*/

	float dis_control_output[16] = {0.0f};
	float moment1[4] = {0.0f}, force1 = 0.0f;

	if (SELECT_UAV_MISSION == TRANSPORTATION)
	{

		/*dis_control_output : 16 x 1 matrix for 4 x (4 x 1) control output for 4 UAVs*/
		multi_uav_geometry_ctrl_thrust_allocation(dis_control_output, control_moments, control_force);

		/*UAV1 control input*/
		force1 = dis_control_output[0];
		moment1[0] = dis_control_output[1];
		moment1[1] = dis_control_output[2];
		moment1[2] = dis_control_output[3];

		/*Distribution for uav2, uav3, uav4*/
		assign_command_msg(dis_control_output);
	}
	else
	{
		force1 = control_force;
		moment1[0] = control_moments[0];
		moment1[1] = control_moments[1];
		moment1[2] = control_moments[2];
	}

	if (lock_motor == false)
	{
		mr_geometry_ctrl_thrust_allocation(moment1, force1);
	}
	else
	{
		motor_halt();
	}
}

void send_geometry_moment_ctrl_debug(debug_msg_t *payload)
{
	float roll_error = rad_to_deg(mat_data(eR)[0]);
	float pitch_error = rad_to_deg(mat_data(eR)[1]);
	float yaw_error = rad_to_deg(mat_data(eR)[2]);

	float wx_error = rad_to_deg(mat_data(eW)[0]);
	float wy_error = rad_to_deg(mat_data(eW)[1]);
	float wz_error = rad_to_deg(mat_data(eW)[2]);

	float geometry_ctrl_feedback_moments[3];
	float geometry_ctrl_feedfoward_moments[3];

	/* calculate the feedback moment and convert the unit from [gram force * m] to [newton * m] */
	geometry_ctrl_feedback_moments[0] = (-krx * mat_data(eR)[0] - kwx * mat_data(eW)[0]);
	geometry_ctrl_feedback_moments[1] = (-kry * mat_data(eR)[1] - kwy * mat_data(eW)[1]);
	geometry_ctrl_feedback_moments[2] = (-krz * mat_data(eR)[2] - kwz * mat_data(eW)[2]);

	geometry_ctrl_feedfoward_moments[0] = mat_data(inertia_effect)[0];
	geometry_ctrl_feedfoward_moments[1] = mat_data(inertia_effect)[1];
	geometry_ctrl_feedfoward_moments[2] = mat_data(inertia_effect)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_GEOMETRY_MOMENT_CTRL);
	pack_debug_debug_message_float(&roll_error, payload);
	pack_debug_debug_message_float(&pitch_error, payload);
	pack_debug_debug_message_float(&yaw_error, payload);
	pack_debug_debug_message_float(&wx_error, payload);
	pack_debug_debug_message_float(&wy_error, payload);
	pack_debug_debug_message_float(&wz_error, payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[0], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[1], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[2], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[0], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[1], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[2], payload);
}

void send_geometry_tracking_ctrl_debug(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_GEOMETRY_TRACKING_CTRL);
	pack_debug_debug_message_float(&pos_error[0], payload);
	pack_debug_debug_message_float(&pos_error[1], payload);
	pack_debug_debug_message_float(&pos_error[2], payload);
}

void send_uav_dynamics_debug(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_UAV_DYNAMICS_DEBUG);
	pack_debug_debug_message_float(&uav_dynamics_m[0], payload);
	pack_debug_debug_message_float(&uav_dynamics_m[1], payload);
	pack_debug_debug_message_float(&uav_dynamics_m[2], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[0], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[1], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[2], payload);
}


void send_controller_estimation_adaptive_debug(debug_msg_t *payload)
{
	// float roll_error = rad_to_deg(mat_data(eR)[0]);
	// float pitch_error = rad_to_deg(mat_data(eR)[1]);
	// float yaw_error = rad_to_deg(mat_data(eR)[2]);
	
	float ex_x = pos_error[0];
	float ex_y = pos_error[1];
	float ex_z = pos_error[2];

	float ev_x = vel_error[0];
	float ev_y = vel_error[1];
	float ev_z = vel_error[2];

	float roll_error = rad_to_deg(mat_data(eR)[0]);
	float pitch_error = rad_to_deg(mat_data(eR)[1]);
	float yaw_error = rad_to_deg(mat_data(eR)[2]);

	float wx_error = rad_to_deg(mat_data(eW)[0]);
	float wy_error = rad_to_deg(mat_data(eW)[1]);
	float wz_error = rad_to_deg(mat_data(eW)[2]);

	float mass = mat_data(theta_m_hat)[0];
	float cogX = mat_data(theta)[0];
	float cogY = mat_data(theta)[1];

	float a_x = mat_data(y_m_cl)[0];
	float a_y = mat_data(y_m_cl)[1];
	float a_z = mat_data(y_m_cl)[2];

	float force_x = mat_data(curr_force)[0];
	float force_y = mat_data(curr_force)[1];
	float force_z = mat_data(curr_force)[2];
	float adaptive_hat_dot = mat_data(theta_m_hat_dot_adaptive)[0];
	float cl_hat_dot = mat_data(theta_m_hat_dot_ICL)[0];
	float moment_x = mat_data(M_last)[0];
	float moment_y = mat_data(M_last)[1];
	float moment_z = mat_data(M_last)[2];

	float icl_adaptive_moment_x = mat_data(adaptive_theta_hat_dot)[1];
	float icl_moment_x = mat_data(ICL_theta_hat_dot)[1];

	float time_now = get_sys_time_s();

	pack_debug_debug_message_header(payload, MESSAGE_ID_ADAPTIVE_THETA);

	pack_debug_debug_message_float(&ex_x, payload);
	pack_debug_debug_message_float(&ex_y, payload);
	pack_debug_debug_message_float(&ex_z, payload);

	pack_debug_debug_message_float(&ev_x, payload);
	pack_debug_debug_message_float(&ev_y, payload);
	pack_debug_debug_message_float(&ev_z, payload);

	pack_debug_debug_message_float(&roll_error, payload);
	pack_debug_debug_message_float(&pitch_error, payload);
	pack_debug_debug_message_float(&yaw_error, payload);

	pack_debug_debug_message_float(&wx_error, payload);
	pack_debug_debug_message_float(&wy_error, payload);
	pack_debug_debug_message_float(&wz_error, payload);

	pack_debug_debug_message_float(&force_x, payload);
	pack_debug_debug_message_float(&force_y, payload);
	pack_debug_debug_message_float(&force_z, payload);

	pack_debug_debug_message_float(&moment_x, payload);
	pack_debug_debug_message_float(&moment_y, payload);
	pack_debug_debug_message_float(&moment_z, payload);

	pack_debug_debug_message_float(&mass, payload);
	pack_debug_debug_message_float(&adaptive_hat_dot, payload);
	pack_debug_debug_message_float(&cl_hat_dot, payload);
	pack_debug_debug_message_float(&a_x, payload);
	pack_debug_debug_message_float(&a_y, payload);
	pack_debug_debug_message_float(&a_z, payload);

	pack_debug_debug_message_float(&cogX, payload);
	pack_debug_debug_message_float(&cogY, payload);
	// pack_debug_debug_message_float(&icl_adaptive_moment_x, payload);
	// pack_debug_debug_message_float(&icl_moment_x, payload);




	pack_debug_debug_message_float(&time_now, payload);
}
