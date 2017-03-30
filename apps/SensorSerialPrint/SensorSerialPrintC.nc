/*
Humidity and Temperature sensors are located in the external Sensirion
sensor.  Their readings can be converted to SI units as follows:

For Temperature, Oscilloscope returns a 14-bit value that can be
converted to degrees Celsius (oC):
(3)  temperature = -39.60 + 0.01*SOt
where SOt is the raw output of the sensor.

Humidity is a 12-bit value that is not temperature compensated.
(4)  humidity = -4 + 0.0405*SOrh + (-2.8 * 10^-6)*(SOrh^2)
where SOrh is the raw output of the relative humidity sensor

Using this calculation and the temperature measurement, you can correct
the humidity measurement with temperature compensation:
(5)  humidity_true = (Tc - 25) * (0.01 + 0.00008*SOrh) + humidity
where Tc is the temperature measured in oC from equation (3),
SOrh is the raw output of the relative humidity sensor,
and humidity is the uncompensated value calculated in equation (4).

*/

#include<Timer.h>
#include<stdio.h>
#include<string.h>
#include<math.h>


module SensorSerialPrintC
{
	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli>;
	
	uses interface Read<uint16_t> as Tempread;
	uses interface Read<uint16_t> as Lightread;
	uses interface Read<uint16_t> as Humread;
	
}
implementation
{
    uint16_t temperature;
    uint16_t luminance;
    uint16_t humidity;	
	uint16_t humidity_true;	


	event void Boot.booted()
	{
		
		call Timer.startPeriodic(5000);
		call Leds.led0On();		
	}

	event void Timer.fired()
	{
          call Tempread.read();
          call Lightread.read() ;
          call Humread.read();	
	}

	 event void Tempread.readDone(error_t result, uint16_t val)
	{
		temperature = (-39.60 + 0.01 * val);
		if (result == SUCCESS)
		{
			printf("current temperature is: %d \r\n", temperature);
		}
		else
		{
			printf("Error reading from sensors \r\n");
		}	
	}

	event void Lightread.readDone(error_t result, uint16_t val)
	{
		luminance = 2.5 *((val/4096.0) *6250.0);
		if (result == SUCCESS)
		{
			printf("current light is: %d \r\n", luminance);
		}
		else
		{
			printf("Error reading from sensors \r\n");
		}
	}

	event void Humread.readDone(error_t result, uint16_t val)
	{
		humidity = -4 + 0.0405*val + (-2.8 * pow(10,-6))*pow(val,2);
		humidity_true = (temperature - 25) * (0.01 + 0.00008*val) + humidity;
		if (result == SUCCESS)
		{
			printf("current humidity is: %d \r\n", humidity_true);
		}
		else
		{
			printf("Error reading from sensors \r\n");
		}
	}

}
