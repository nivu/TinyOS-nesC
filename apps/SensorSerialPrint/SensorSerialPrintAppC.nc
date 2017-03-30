configuration SensorSerialPrintAppC
{
	
}
implementation
{
	components SensorSerialPrintC as App;
	components MainC;
	components LedsC;
	components new TimerMilliC();
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.Timer -> TimerMilliC;
	
	components new SensirionSht11C() as Tsensor;
	App.Tempread -> Tsensor.Temperature;
	App.Humread -> Tsensor.Humidity;

	components new HamamatsuS10871TsrC() as Lsensor;
	App.Lightread -> Lsensor;

  	//components new HamamatsuS1087ParC() as TotalSolarC;

  	//App.Solarread -> TotalSolarC;
	
}
