configuration moteTAppC
{
	
}
implementation
{
	components moteTC as App;
	components MainC;
	components LedsC;
	components new TimerMilliC();
	components UserButtonC;
	//App.Get -> UserButtonC;
	//App.Notify -> UserButtonC;
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.Timer -> TimerMilliC;
	
	components ActiveMessageC;
	components new AMSenderC(AM_RADIO);
	
	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
	App.AMSend -> AMSenderC;
	App.SplitControl -> ActiveMessageC;
	
	//components SerialPrintfC;
	
	components new SensirionSht11C() as Tsensor;
	App.Tempread -> Tsensor.Temperature;
	App.Humread -> Tsensor.Humidity;

	components new HamamatsuS10871TsrC() as Lsensor;
	App.Lightread -> Lsensor;

  	components new HamamatsuS1087ParC() as TotalSolarC;

  	App.Solarread -> TotalSolarC;
	
}
