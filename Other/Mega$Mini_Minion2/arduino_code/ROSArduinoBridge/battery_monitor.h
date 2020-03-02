/* *************************************************************
  Battery parameters definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  #define VREF 4.6
  #define SENSITIVITY 100.0 // mili volt/Amps  for 20 Amps module
  #define SENS_OFFSET 2302  
  #define ADC_BAT_VOLT_PIN PC0 
  #define ADC_BAT_CURR_PIN PC1
#endif
   
double battery_voltage(void);
double battery_current(void);
