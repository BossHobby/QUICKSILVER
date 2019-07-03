

typedef struct debug
{
	int gyroid;
	float vbatt_comp;
	float adcfilt;
	float totaltime;	
	float timefilt;
  float adcreffilt;
	float cpu_load;
	float max_cpu_load;
	unsigned int max_cpu_loop_number;
	unsigned int loops_between_max_cpu_load;
	float min_cpu_load;
	unsigned int min_cpu_loop_number;
	unsigned int loops_between_min_cpu_load;
} debug_type;




