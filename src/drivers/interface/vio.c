#include <stdbool.h>
#include <stdint.h>
#include "se3_math.h"
#include "vio.h"
#include "vins_mono.h"
#include "quaternion.h"
#include "gps.h"
#include "compass.h"
#include "ins.h"
#include "optitrack.h"

vio_t vio;

bool vio_available(void)
{
	return vins_mono_available();
}

void vio_enable_frame_alignment(void)
{
	vio.frame_align = true;
}

void vio_disable_frame_alignment(void)
{
	vio.frame_align = false;
}

void vio_calc_frame_alignment_transform(void)
{
	/* get position and quaternion from gnss/ins */
	float p_gnss_ins[3], q_gnss_ins[4];
	optitrack_read_pos(p_gnss_ins); //FIXME
	ins_ahrs_get_attitude_quaternion(q_gnss_ins);

	/* get position and quaternion from local vio */
	float p_local_vio[3], q_local_vio[4], q_local_vio_conj[4];
	vins_mono_read_pos(p_local_vio);                //we get position of bk frame under the c0 frame
	vins_mono_read_quaternion(q_local_vio_conj);    //we get q from b to c0 here
	quaternion_conj(q_local_vio_conj, q_local_vio); //get q from c0 to b

	/* calculate frame rotation */
	float Rt[3*3]; //XXX: dummy variable
	quaternion_mult(q_gnss_ins, q_local_vio, vio.q_l2g); //q_l2g = q_gnss_ins * q_local_vio
	quat_to_rotation_matrix(vio.q_l2g, vio.R_l2g, Rt);   //R_l2g = R(q_l2g)

	/* calculate frame translation
	 * p_l2g = p_gnss_ins - (R_l2g * p_local_vio) */
	float p_tmp[3];
	calc_matrix_multiply_vector_3d(p_tmp, p_local_vio, vio.R_l2g);
	vio.p_l2g[0] = p_gnss_ins[0] - p_tmp[0];
	vio.p_l2g[1] = p_gnss_ins[1] - p_tmp[1];
	vio.p_l2g[2] = p_gnss_ins[2] - p_tmp[2];
}

void vio_get_quaternion(float *q)
{
	if(vio.frame_align == true) {
		/* read local vio quaternion */
		float q_local_vio_conj[4];
		vins_mono_read_quaternion(q_local_vio_conj); //we get q from b to c0 here

		/* calculate q_vio_global = q_l2g * conj(q_local_vio) */
		quaternion_mult(vio.q_l2g, q_local_vio_conj, q);
	} else {
		vins_mono_read_quaternion(q);
	}
}

void vio_get_position(float *pos)
{
	if(vio.frame_align == true) {
		/* get position from local vio */
		float p_local_vio[3];
		vins_mono_read_pos(p_local_vio); //we get position of bk frame under the c0 frame

		/* apply frame translation
		 * p_global = R_l2g * p_local_vio + p_l2g */
		float p_tmp[3];
		calc_matrix_multiply_vector_3d(p_tmp, p_local_vio, vio.R_l2g);
		pos[0] = p_tmp[0];// + vio.p_l2g[0];
		pos[1] = p_tmp[1];// + vio.p_l2g[1];
		pos[2] = p_tmp[2];// + vio.p_l2g[2];
	} else {
		vins_mono_read_pos(pos);
	}
}

float vio_get_position_x(void)
{
	return vins_mono_read_pos_x();
}

float vio_get_position_y(void)
{
	return vins_mono_read_pos_y();
}

float vio_get_position_z(void)
{
	return vins_mono_read_pos_z();
}

void vio_get_velocity(float *vel)
{
	vins_mono_read_vel(vel);
}

float vio_get_velocity_x(void)
{
	return vins_mono_read_vel_x();
}

float vio_get_velocity_y(void)
{
	return vins_mono_read_vel_y();
}

float vio_get_velocity_z(void)
{
	return vins_mono_read_vel_z();
}
