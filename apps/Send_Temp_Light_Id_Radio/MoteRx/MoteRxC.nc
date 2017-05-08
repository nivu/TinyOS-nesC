#include<string.h>
#include<stdio.h>
#include"MsgT.h"


module MoteRxC

{
	uses // Interfaces
	{
      interface Boot;
      interface Leds;
    }
    
    uses // Radio
    {
      interface SplitControl;
      interface Receive;
     }
}

implementation
{
	bool _radioBusy = FALSE;
	message_t _packet;
	
	event void Boot.booted()
	{
		call SplitControl.start();
	}


	event void SplitControl.startDone(error_t error)
	{
		if(error == SUCCESS)
		{
			call Leds.led0On();
		}
		else
		{
			call SplitControl.start();
		}
 	}
	
	event void SplitControl.stopDone(error_t error)
	{
		// TODO Auto-generated method stub
	}

	event message_t * Receive.receive(message_t *msg, void *payload, uint8_t len)
	{
	    if(len == sizeof(MoteToMoteMsg_t))
	    {
	    	MoteToMoteMsg_t * incomingPacket = (MoteToMoteMsg_t*) payload;
	    		    	
	    uint8_t temp = incomingPacket -> Temp;
		uint8_t hum = incomingPacket -> Hum;
		uint8_t photo = incomingPacket -> Photo;
		uint8_t nodeId = incomingPacket->NodeId;
	    	
	    printf("$%d *%d #%d\r\n",nodeId,temp,hum);
		//printf("Node Id = %d Temperature = %d Humidity = %d Light Intensity = %d \r\n", nodeId,temp,hum,photo);

	    }
        return msg;
	}
}
