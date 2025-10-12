#include <dqpll_grid.h>

param_t param;
int aggancio_pll_ContaFault_RUN;
int aggancio_pll_ContaFault;

void dqpll_grid_init(volatile DQPLL_GRID *dqpll_ctrl) {
	dqpll_ctrl->ts = 0.0;
	dqpll_ctrl->kp_dqpll_grid = KP_DQPLL_GRID;
	dqpll_ctrl->ki1_dqpll_grid = KI1_DQPLL_GRID;
	dqpll_ctrl->ki2_dqpll_grid = KI2_DQPLL_GRID;
	dqpll_grid_reset(dqpll_ctrl);
	
	param.pll_lim_ok = DQPLL_GRID_LIM_OK;
	param.cabcd_global_state = CABCD_STATE_ZVS_RUN;
	param.cabcd_fault = 0x000;
	param.pll_state = DQPLL_GRID_STATE_STOP;
}

void dqpll_grid_ts(volatile DQPLL_GRID *dqpll_ctrl, volatile float ts) {
	dqpll_ctrl->ts = ts;
}

void dqpll_grid_reset(volatile DQPLL_GRID *dqpll_ctrl) {
	dqpll_ctrl->u_xi = 0.0;
	dqpll_ctrl->u_eta = 0.0;
	dqpll_ctrl->omega_hat = 0.0;
	dqpll_ctrl->omega_i_hat = 0.0;
	dqpll_ctrl->gamma_hat = 0.0;
	dqpll_ctrl->clip_active = 0;
	dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_STOP;
	aggancio_pll_ContaFault = 0;
	aggancio_pll_ContaFault_RUN = 0;
}

float dqpll_grid_process(volatile DQPLL_GRID *dqpll_ctrl, volatile float u_phase_r, volatile float u_phase_s, volatile float u_phase_t) {
		
	float omega_i_hat_tilde = 0;
	const float ts = dqpll_ctrl->ts;
	const float kp_dqpll_grid = dqpll_ctrl->kp_dqpll_grid;
	const float ki1_dqpll_grid = dqpll_ctrl->ki1_dqpll_grid;
	const float ki2_dqpll_grid = dqpll_ctrl->ki2_dqpll_grid;

	const float omega_i_hat = dqpll_ctrl->omega_i_hat;
	const float gamma_hat = dqpll_ctrl->gamma_hat;
	
	const float th_sin = sinf(gamma_hat);
	const float th_cos = cosf(gamma_hat);

	const float u_grid_alpha = MATH_2_3 * (u_phase_r - MATH_HALF *  u_phase_s - MATH_HALF *  u_phase_t);
	const float u_grid_beta = MATH_1_SQRT3 * (u_phase_s - u_phase_t);

	const float u_vector_length_inverter = 1.0f / sqrtf(u_grid_alpha * u_grid_alpha + u_grid_beta * u_grid_beta);

	const float u_grid_xi = u_grid_alpha * th_cos + u_grid_beta * th_sin;
	const float u_grid_eta = u_grid_beta * th_cos - u_grid_alpha * th_sin;

	const float u_grid_xi_n = u_grid_xi * u_vector_length_inverter;
	const float u_grid_eta_n = u_grid_eta * u_vector_length_inverter;

	dqpll_ctrl->u_xi = u_grid_xi_n;
	dqpll_ctrl->u_eta = u_grid_eta_n;

	dqpll_ctrl->omega_i_hat = -u_grid_eta_n * ki1_dqpll_grid * ts + omega_i_hat;
	dqpll_ctrl->omega_hat = -u_grid_eta_n * kp_dqpll_grid + dqpll_ctrl->omega_i_hat;
	dqpll_ctrl->gamma_hat = dqpll_ctrl->omega_hat * ki2_dqpll_grid * ts + gamma_hat;
	
	// keep estimated phase between PI and -PI
	dqpll_ctrl->gamma_hat = - MATH_PI + fmodf(dqpll_ctrl->gamma_hat + MATH_3PI, MATH_2PI);

	switch (dqpll_ctrl->dqpll_state) {
		case DQPLL_GRID_STATE_STOP:
			dqpll_ctrl->omega_hat = 0.0;
			dqpll_ctrl->omega_i_hat = 0.0;
			dqpll_ctrl->gamma_hat = 0.0;
			dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_START;
		break;

		case DQPLL_GRID_STATE_FAULT:
			dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_STOP;
			//param.cabcd_fault |= FAULT_PLL_SYNC;
		break;

		case DQPLL_GRID_STATE_START:
			omega_i_hat_tilde = 0;
			/* omega_i_hat_tilde = dqpll_ctrl->omega_i_hat - omega_i_hat; */
		    if ((dqpll_ctrl->omega_hat > OMEGA_HAT_WINDOW1) || (dqpll_ctrl->omega_hat < -OMEGA_HAT_WINDOW1)) {
				if ((omega_i_hat_tilde < param.pll_lim_ok) && (omega_i_hat_tilde > -param.pll_lim_ok)) {
							param.pll_state = DQPLL_GRID_STATE_CONNECTING; 
							dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_CONNECTING; 
							param.cabcd_fault &= ~FAULT_DQPLL_GRID_SYNC;   
							aggancio_pll_ContaFault = 0;           
					}
				else {
						param.cabcd_fault |= FAULT_DQPLL_GRID_SYNC; 
						param.pll_state = DQPLL_GRID_STATE_START;      
						dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_START; 
					}
			}
		   else
		       aggancio_pll_ContaFault++;

		   if(aggancio_pll_ContaFault >= AGGANCIO_PLL_CONTAFAULT_WINDOW1) {
				aggancio_pll_ContaFault = 0;
				param.pll_state = DQPLL_GRID_STATE_START; 
				dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_START;
			}
		break;

		case DQPLL_GRID_STATE_CONNECTING:
			omega_i_hat_tilde = 0;
		/*	omega_i_hat_tilde = dqpll_ctrl->omega_i_hat - omega_i_hat;	*/
			if ((dqpll_ctrl->omega_hat > OMEGA_HAT_WINDOW2) || (dqpll_ctrl->omega_hat < -OMEGA_HAT_WINDOW2)) {
				if ((omega_i_hat_tilde < param.pll_lim_ok) && (omega_i_hat_tilde > -param.pll_lim_ok)) { 
						param.pll_state = DQPLL_GRID_STATE_CONNECTING; 
						dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_CONNECTING; 
				}
				else {
						param.pll_state = DQPLL_GRID_STATE_FAULT;
						dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_FAULT;
						param.cabcd_fault |= FAULT_DQPLL_GRID_SYNC;
				}
			}
			else {
					aggancio_pll_ContaFault_RUN++;
			}

			if(aggancio_pll_ContaFault_RUN >= AGGANCIO_PLL_CONTAFAULT_WINDOW2) {
					param.pll_state = DQPLL_GRID_STATE_FAULT;
					dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_FAULT;
					aggancio_pll_ContaFault_RUN = 0;
					param.cabcd_fault |= FAULT_DQPLL_GRID_SYNC;             
			}

			if (param.cabcd_global_state == CABCD_STATE_ZVS_RUN ) {
					param.pll_state = DQPLL_GRID_STATE_RUN;
					dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_RUN;
					aggancio_pll_ContaFault_RUN = 0;
			}
		break;


		case DQPLL_GRID_STATE_RUN:
			omega_i_hat_tilde = 0;
		/*	omega_i_hat_tilde = dqpll_ctrl->omega_i_hat - omega_i_hat; */
			if ((dqpll_ctrl->omega_hat > OMEGA_HAT_WINDOW2) || (dqpll_ctrl->omega_hat < -OMEGA_HAT_WINDOW2)) {
				if ((omega_i_hat_tilde < param.pll_lim_ok) && (omega_i_hat_tilde > -param.pll_lim_ok)){  
						param.pll_state = DQPLL_GRID_STATE_RUN;
						dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_RUN;
					}
				else {
						param.pll_state = DQPLL_GRID_STATE_FAULT;
						dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_FAULT;
						param.cabcd_fault |= FAULT_DQPLL_GRID_SYNC;
					}
			}
			else {
					aggancio_pll_ContaFault_RUN++;
			}

			if(aggancio_pll_ContaFault_RUN >= AGGANCIO_PLL_CONTAFAULT_WINDOW2 ) {
					param.pll_state = DQPLL_GRID_STATE_FAULT;
					dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_FAULT;
					aggancio_pll_ContaFault_RUN = 0;
					param.cabcd_fault |= FAULT_DQPLL_GRID_SYNC;
			}

			if (param.cabcd_global_state < CABCD_STATE_ZVS_RUN ) {
					param.pll_state = DQPLL_GRID_STATE_STOP;
					dqpll_ctrl->dqpll_state = DQPLL_GRID_STATE_STOP;
			}
		
		break;
	}

	return gamma_hat;
}

