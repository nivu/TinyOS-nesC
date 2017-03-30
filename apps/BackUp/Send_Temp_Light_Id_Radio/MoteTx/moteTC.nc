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
//#include<stdio.h>
//#include<string.h>
#include<UserButton.h>
#include"moteT.h"

module moteTC
{
	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli>;
	
	uses interface Read<uint16_t> as Tempread;
	uses interface Read<uint16_t> as Lightread;
	uses interface Read<uint16_t> as Humread;

	
	//uses interface Notify<button_state_t>;
	//uses interface Get<button_state_t>;
		
	uses // Radio
    {
      interface Packet;
      interface AMPacket;
      interface AMSend;
      interface SplitControl;
     }
	
}
implementation
{
	bool _radioBusy = FALSE;
	message_t _packet;
    uint16_t centiGrade;
    uint16_t luminance;
    uint16_t humidity;	
	

	event void Boot.booted()
	{
		
		call Timer.startPeriodic(5000);
		call Leds.led0On();
		//call Notify.enable();
		call SplitControl.start();
		
	}

	event void Timer.fired()
	{
          call Tempread.read();
          call Lightread.read() ;
          call Humread.read();
       /*if (call Tempread.read() == SUCCESS)
		{
			call Leds.led2Toggle();
		}
		else 
		{
			call Leds.led0Toggle();
		}*/
               if(_radioBusy == FALSE)
		{
			// Creating Packet
			MoteToMoteMsg_t* msg = call Packet.getPayload(& _packet, sizeof(MoteToMoteMsg_t));
			
			msg->NodeId = TOS_NODE_ID;
			msg->Temp = (uint8_t) centiGrade;
			msg->Hum = (uint8_t) humidity;
			msg->Photo = (uint8_t) luminance;
			
			// Sending Packet
			if(call AMSend.send(AM_BROADCAST_ADDR,& _packet, sizeof(MoteToMoteMsg_t)) == SUCCESS)
			{
				_radioBusy = TRUE;
			}
		}	
	}

	 event void Tempread.readDone(error_t result, uint16_t val)
	{
		centiGrade = (-39.60 + 0.01 * val);
		if (result == SUCCESS)
		{
			//printf("current temperature is: %d \r\n", centiGrade);
		}
		else
		{
			//printf("Error reading from sensors \r\n");
		}	
	}

	event void Lightread.readDone(error_t result, uint16_t val)
	{
		luminance = 2.5 *((val/4096.0) *6250.0);
		if (result == SUCCESS)
		{
			//printf("current light is: %d \r\n", luminance);
		}
		else
		{
			//printf("Error reading from sensors \r\n");
		}
	}

	event void Humread.readDone(error_t result, uint16_t val)
	{
		humidity = 2.5 *((val/4096.0) *6250.0);
	}

	event void AMSend.sendDone(message_t *msg, error_t error)
	{
    	if(msg == & _packet)
        {
        	_radioBusy = FALSE;
       	}	
    }

	/*event void Notify.notify(button_state_t val){

		if(_radioBusy == FALSE)
		{
			// Creating Packet
			MoteToMoteMsg_t* msg = call Packet.getPayload(& _packet, sizeof(MoteToMoteMsg_t));
			
			msg->NodeId = TOS_NODE_ID;
			msg->Data = (uint8_t) centiGrade;
			
			// Sending Packet
			if(call AMSend.send(AM_BROADCAST_ADDR,& _packet, sizeof(MoteToMoteMsg_t)) == SUCCESS)
			{
				_radioBusy = TRUE;
			}
		}	}*/

	event void SplitControl.startDone(error_t error)
	{
		if(error == SUCCESS)
		{
			call Leds.led0On();
		}
		else
		{
			call SplitControl.start();
		}	}

	event void SplitControl.stopDone(error_t error)
	{
		
	}
}
