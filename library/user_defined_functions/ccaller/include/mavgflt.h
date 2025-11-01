#ifndef _MAVGFLT_
#define _MAVGFLT_


#define MAVGFLT_SIZE		1520
#define MAVGFLT_SIZE_MIN	780
#define MAVGFLT_SIZE_MAX	4200

typedef struct mavgflt_s {
	float					ts;						    /* time base */
	float					sum_value;		            /* sum buffer */
	float					pinput;			            /* step back input value */
	float					buffer[MAVGFLT_SIZE_MAX];   /* buffer */
	unsigned int			cbpointer;			        /* buffer array pointer */
	unsigned int			cbsample;                  	/* current number of samples for the current period */
} mavgflt_t;
#define MAVGFLT mavgflt_t

void mavgflt_init(volatile MAVGFLT *f, volatile float ts);

float mavgflt_process(volatile MAVGFLT *f, float input, const float period);

#endif
