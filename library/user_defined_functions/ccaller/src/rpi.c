#include <rpi.h>

void rpi_init(volatile RPI *rpi_ctrl, volatile float ts, volatile float kp_rpi, volatile float ki_rpi, 
		volatile float omega, volatile float delta, volatile float u_out_lim)
{
	rpi_ctrl->ts = ts;
	rpi_ctrl->kp_rpi = kp_rpi;
	rpi_ctrl->ki_rpi = ki_rpi;
	rpi_ctrl->a11 = 1.0f;
	rpi_ctrl->a12 = ts;
	rpi_ctrl->a21 = -omega*omega*ts;
	rpi_ctrl->a22 = 1.0f - 2*delta*omega*ts;
	rpi_ctrl->u_lim = u_out_lim;
	rpi_reset(rpi_ctrl);	
}

void dqvector_pi_ts(volatile RPI *rpi_ctrl)
{
	rpi_ctrl->ts = ts;
}

void dqvector_pi_reset(volatile RPI *rpi_ctrl)
{
	rpi_ctrl->x1 = 0.0;
	rpi_ctrl->x2 = 0.0;
	rpi_ctrl->y_out = 0.0;
	rpi_ctrl->clip_active = 0;
}

rpi_output_t rpi_process(volatile RPI *rpi_ctrl, volatile float i_ref, volatile float i, volatile float u_dc)
{
	const float i_tilde = i_ref - i;
	const float u_p_out = i_tilde * rpi_ctrl->kp_id;

	float x1_z = rpi_ctrl->x1;
	float x2_z = rpi_ctrl->x1;
	
	if (!rpi_ctrl->clip_active) {
		rpi_ctrl->x1 = rpi_ctrl->a11*x1_z + rpi_ctrl->a12*x2_z;
		rpi_ctrl->x2 = id_tilde * rpi_ctrl->ki_rpi * rpi_ctrl->ts + rpi_ctrl->a21*x1_z + rpi_ctrl->a22*x2_z;
	}
	else {
		rpi_ctrl->x1 = x1_z;
		rpi_ctrl->x2 = x2_z;
	}

	rpi_ctrl->y_out = x2_z;

	const rpi_output_t u_out = rpi_ctrl->y_out;

	return u_out;
}

