/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

#ifdef USE_BASE
   double battery_voltage(void)
   {
     double sensorValue = 0;
     for (int i=0;i<1000;i++){
     sensorValue += analogRead(ADC_BAT_VOLT_PIN);}
     sensorValue = sensorValue/1000;
     double bat_voltage=VREF*(sensorValue/1023);
     //Serial.println(bat_voltage, 4);
     return (bat_voltage*3.24);
   }
   double battery_current(void)
   {
     double temp_volts, bat_current, sensorValue = 0;
     analogReference(DEFAULT);
     for (int i=0;i<1000;i++){
     sensorValue += analogRead(ADC_BAT_CURR_PIN);}
     sensorValue = sensorValue/1000;
     temp_volts=VREF*(sensorValue/1023) * 1000; // in millvolts
     bat_current = (temp_volts - SENS_OFFSET)/SENSITIVITY;
     //Serial.println(bat_current);
     return bat_current;
   }
#endif
