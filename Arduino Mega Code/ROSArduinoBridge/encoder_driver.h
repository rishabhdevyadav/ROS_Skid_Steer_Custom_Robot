/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
/*
     ^
BM1 --+-- AM1
   | _|_ |
   |  |  |
BM2 --+-- AM2

*/
/* Please refer to encoder_driver.ino -> ISR before changing... */



  
  #define BM1_ENC_PIN_A PK0 // pin A8
  #define BM1_ENC_PIN_B PK1 // pin A9
  
  #define BM2_ENC_PIN_A PK2 // pin A10
  #define BM2_ENC_PIN_B PK3 // pin A11
  
  #define AM1_ENC_PIN_A PK5 // pin A13
  #define AM1_ENC_PIN_B PK4 // pin A12

  #define AM2_ENC_PIN_A PK7 // pin A15
  #define AM2_ENC_PIN_B PK6 // pin A14
  
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
