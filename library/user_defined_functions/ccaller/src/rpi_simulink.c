

#include <rpi.h>
#include <rpi_simulink.h>

unsigned int rpi_ctrl_initialized = 0;
RPI rpi_ctrl = {0.0};

// ------------------------------------------------------------------------------
RPI_OUTPUT dqvector_pi_process_simulink(unsigned char reset, const float i_ref, const float i, 
		const float u_dc, const float ts, const float u_lim)
{

	if (rpi_ctrl_initialized == 0) {
	    rpi_init(&rpi_ctrl, kp_rpi, ki_rpi, omega, delta, u_lim);
		rpi_ctrl_initialized = 1;
	}

	rpi_ts(&dqpi_ctrl, ts);

	if (reset) {
		rpi_reset(&dqpi_ctrl);
	}
	
	rpi_output_t rpi_state_out = rpi_process(&rpi_ctrl, i_ref, i, u_dc);

	const RPI_OUTPUT rpi_out = {
		.x1 = rpi_state_out.x1,
		.x2 = rpi_state_out.x2,
		.u_out = rpi_state_out.u_out
	};
	
	return rpi_out;
}

