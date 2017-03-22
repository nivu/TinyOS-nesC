#include<UserButton.h>
#include"MoteRx.h"
#include<string.h>
#include<stdio.h>

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
	    	
	    	//incomingPacket->NodeId == 2;
	    	
	    uint8_t data = incomingPacket->Data;
		uint8_t data2 = incomingPacket->Data2;
		uint8_t nodeId = incomingPacket->NodeId;
	    	
	    	printf("$%d *%d #%d\r\n",nodeId,data,data2);
		printf("Node Id = %d Temperature = %d Light Intensity = %d \r\n", nodeId,data,data2);
	    	    	
	    	if(data == 1)
	    	{
	    		call Leds.led1On();
	    	  //  printf("Packet is %d\r\n",data);
	    		
	    	}
	    	if(data ==0)
	    	{
	    		call Leds.led1Off();
	    	  //  printf("Packet is %d\r\n",data);
	    		
	    	}	
	    }
        return msg;
	}
}
