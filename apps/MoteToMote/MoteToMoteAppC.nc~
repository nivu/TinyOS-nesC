configuration MoteToMoteAppC
{
	
}

implementation
{
	//General
	components MoteToMoteC as App; //Main module file
	components MainC;  //Boot interface
	components LedsC;
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	
	//UserButton
	components UserButtonC;
	App.Get -> UserButtonC;
	App.Notify -> UserButtonC;
	
	//Radio Communication
	components ActiveMessageC;
	components new AMSenderC(AM_RADIO);
	components new AMReceiverC(AM_RADIO);
	
	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
	App.AMSend -> AMSenderC;
	App.SplitControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;
	
}

