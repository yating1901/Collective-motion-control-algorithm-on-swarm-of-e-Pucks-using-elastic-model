#ifndef _PROX
#define _PROX

void e_calibrate_ir();
int e_get_prox(unsigned int sensor_number); // to get a prox value
int e_get_calibrated_prox(unsigned int sensor_number);
int e_get_ambient_light(unsigned int sensor_number); // to get ambient light value

#endif
