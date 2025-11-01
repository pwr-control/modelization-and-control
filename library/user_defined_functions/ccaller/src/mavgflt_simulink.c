
#include <mavgflt.h>
#include <mavgflt_simulink.h>

// ------------------------------------------------------------------------------
static void init_allmavgflt_instances(MAVGFLT *const filter_list, 
	const unsigned int filter_num, const float ts)
{
    unsigned int i;
    for (i = 0; i < filter_num; ++i) {
        MAVGFLT *const filter = &filter_list[i];
		mavgflt_init(filter, ts);
		i++;
	}
}

// ------------------------------------------------------------------------------
MAVGFLT_OUTPUT mavgflt_process_simulink(const float input, const float period, 
	const float ts, unsigned char reset, unsigned char instance)
{
	static MAVGFLT filter_instances[NMAVGFLT_INSTANCES] = {0};
	static unsigned int filter_initialized = 0;

	if (!filter_initialized){
	    init_allmavgflt_instances(filter_instances, NMAVGFLT_INSTANCES, ts);
		filter_initialized = 1;
	}

	if (instance < NMAVGFLT_INSTANCES) {
		const MAVGFLT* filter_instance = &filter_instances[instance];

		
		if(reset)
		{
			mavgflt_init(filter_instance, ts);
		}
		
		const float output_value = mavgflt_process(filter_instance, input, period);

		const MAVGFLT_OUTPUT filter_output = {
			.ts = filter_instance->ts,
			.sum_value = filter_instance->sum_value,
			.pinput = filter_instance->pinput,
			.cbpointer = filter_instance->cbpointer,
			.cbsample = filter_instance->cbsample,
			.mavg_output = output_value
		};
		return filter_output;
	}
	else{
		const MAVGFLT_OUTPUT empty_output = {
			0
		};
		return empty_output;
	}
}

