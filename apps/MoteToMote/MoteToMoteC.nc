#include<UserButton.h>
#include"MoteToMote.h"

module MoteToMoteC

{
    uses // Interfaces
    {
      interface Boot;
      interface Leds;
    }
    
    uses // User Button
    {
      interface Get<button_state_t>;
      interface Notify<button_state_t>;	
    }
    
    uses // Radio
    {
      interface Packet;
      interface AMPacket;
      interface AMSend;
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
		call Notify.enable();
		call SplitControl.start();
	}

	event void Notify.notify(button_state_t val)
	{
		if(_radioBusy == FALSE)
		{
			// Creating Packet
			MoteToMoteMsg_t* msg = call Packet.getPayload(& _packet, sizeof(MoteToMoteMsg_t));
			
			msg->NodeId = TOS_NODE_ID;
			msg->Data = (uint8_t) val;
			
			// Sending Packet
			if(call AMSend.send(AM_BROADCAST_ADDR,& _packet, sizeof(MoteToMoteMsg_t)) == SUCCESS)
			{
				_radioBusy = TRUE;
			}
		}	
	}

	event void AMSend.sendDone(message_t *msg, error_t error)
	{
        if(msg == & _packet)
        {
        	_radioBusy = FALSE;
       	}
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
	    	
	    	if(data == 1)
	    	{
	    		call Leds.led1On();
	    		call Leds.led2On();

	    	}
	    	if(data ==0)
	    	{
	    		call Leds.led1Off();
	    		call Leds.led2Off();
	    	}	
	    }
        return msg;
	}
}

