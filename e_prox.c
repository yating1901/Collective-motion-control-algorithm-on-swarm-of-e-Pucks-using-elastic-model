#include "e_ad_conv.h"
#include "e_epuck_ports.h"
#include "e_prox.h"

extern int e_ambient_ir[8];						// ambient light measurement
extern int e_ambient_and_reflected_ir[8];		// light when led is on

static int init_value_ir[8] = {0,0,0,0,0,0,0,0};
int redsensor;
int e_a;
int e_f;
int e_i;
/*! \brief To calibrate your ir sensor
 * \warning Call this function one time before calling e_get_calibrated_prox
 */
void e_calibrate_ir()
{
	int i=0,j=0;
	int long t;
	int long tmp[8];
	
	for (;i<8;++i) {
		init_value_ir[i]=0;
		tmp[i]=0;
	}

	for (;j<100;++j) {
		for (i=0;i<8;++i) {
			tmp[i]+=(e_get_prox(i));
			for (t=0;t<1000;++t);
		}
	}

	for (i=0;i<8;++i) {
		init_value_ir[i]=(int)(tmp[i]/(j*1.0));
	}
}

/*! \brief To get the analogic proxy sensor value of a specific ir sensor
 *
 * To estimate the proxymity of an obstacle, we do the following things:
 * - measure the ambient light
 * - turn on the IR led of the sensor
 * - measure the reflected light + ambient light
 * - calculate: reflected light = (reflected light + ambient light) - ambient light
 * - turn off the IR led of the sensor
 *
 * The result value of this function is: reflected light. More this value is great,
 * more the obsacle is near.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_prox(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return e_ambient_ir[sensor_number] - e_ambient_and_reflected_ir[sensor_number];
}

/*! \brief To get the calibrated value of the ir sensor
 *
 * This function return the calbrated analogic value of the ir sensor.
 * \warning Befroe using this function you have to calibrate your ir sensor (only one time)
 * by calling e_calibrate_ir().
 * \param sensor_number The proxy sensor's number that you want the calibrated value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_calibrated_prox(unsigned int sensor_number)
{
	int temp;
	if (sensor_number > 7)
		return 0;
	else
	{
	temp=(e_ambient_ir[sensor_number] - e_ambient_and_reflected_ir[sensor_number])
				- init_value_ir[sensor_number];
          e_a=e_ambient_ir[sensor_number];
          e_f=e_ambient_and_reflected_ir[sensor_number];
          e_i=init_value_ir[sensor_number];
          redsensor=temp;	
		if (temp>0)
			return temp;
		else
			return 0;
	}
}

/*! \brief To get the analogic ambient light value of a specific ir sensor
 *
 * This function return the analogic value of the ambient light measurement.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_ambient_light(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return e_ambient_ir[sensor_number];
}