configuration MoteRxAppC
{
	
}

implementation
{
	//General
	components MoteRxC as App; //Main module file
	components MainC;  //Boot interface
	components LedsC;
	components SerialPrintfC;
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	
	//Radio Communication
	components ActiveMessageC;
	components new AMReceiverC(AM_RADIO);

	App.SplitControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;
	
}