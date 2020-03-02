/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == BM1) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == BM1) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long BM1_enc_pos = 0L;
  volatile long AM1_enc_pos = 0L;
  volatile long BM2_enc_pos = 0L;
  volatile long AM2_enc_pos = 0L;
  // encoder lookup table
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

  /* Interrupt routine for NE encoder, taking care of actual counting */
  ISR (PCINT2_vect){
    static uint8_t BM1_enc_last = 0;
    static uint8_t BM2_enc_last = 0;
    static uint8_t AM1_enc_last = 0;
    static uint8_t AM2_enc_last = 0;
        
    BM1_enc_last <<= 2; //shift previous state two places
    BM2_enc_last <<= 2; //shift previous state two places
    AM1_enc_last <<= 2; //shift previous state two places
    AM2_enc_last <<= 2; //shift previous state two places

    BM1_enc_last |= (PINK & (3 << 0)) >> 0;
    BM2_enc_last |= (PINK & (3 << 2)) >> 2;
    AM1_enc_last |= (PINK & (3 << 4)) >> 4;
    AM2_enc_last |= (PINK & (3 << 6)) >> 6;
  
    BM1_enc_pos += ENC_STATES[(BM1_enc_last & 0x0f)];
    BM2_enc_pos += ENC_STATES[(BM2_enc_last & 0x0f)];
    AM1_enc_pos += ENC_STATES[(AM1_enc_last & 0x0f)];
    AM2_enc_pos += ENC_STATES[(AM2_enc_last & 0x0f)];
  }
  
  long readEncoder(int i) {
    switch (i){
      case BM1: return BM1_enc_pos;
      case BM2: return BM2_enc_pos;
      case AM2: return AM2_enc_pos;
      case AM1: return AM1_enc_pos;
    }
  }

  void resetEncoder(int i) {
    switch (i){
      case BM1: {BM1_enc_pos = 0L; return;}
      case BM2: {BM2_enc_pos = 0L; return;}
      case AM2: {AM2_enc_pos = 0L; return;}
      case AM1: {AM1_enc_pos = 0L; return;}
    }
  }
#else
  #error A encoder driver must be selected!
#endif

void resetEncoders() {
  resetEncoder(BM1);
  resetEncoder(BM2);
  resetEncoder(AM1);
  resetEncoder(AM2);
}

#endif
