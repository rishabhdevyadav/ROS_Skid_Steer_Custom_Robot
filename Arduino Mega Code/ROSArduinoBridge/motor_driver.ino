#ifdef USE_BASE

void initMotorController() {
      pinMode(AM1_FWD, OUTPUT);
      pinMode(AM1_BCK, OUTPUT);
      pinMode(AM1_ENB, OUTPUT);

      pinMode(AM2_FWD, OUTPUT);
      pinMode(AM2_BCK, OUTPUT);
      pinMode(AM2_ENB, OUTPUT);

      pinMode(BM1_FWD, OUTPUT);
      pinMode(BM1_BCK, OUTPUT);
      pinMode(BM1_ENB, OUTPUT);

      pinMode(BM2_FWD, OUTPUT);
      pinMode(BM2_BCK, OUTPUT);
      pinMode(BM2_ENB, OUTPUT);
   }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    unsigned char f = 1;
    unsigned char r = 0;
    if (spd < 0)
    {
      spd = -spd;
      f = 0;
      r = 1;
    }
    if (spd > 255)
      spd = 255;
    switch(i){
      case AM1: digitalWrite(AM1_FWD, f);
               digitalWrite(AM1_BCK, r);
               analogWrite(AM1_ENB, spd);
               break;
      case BM1: digitalWrite(BM1_FWD, f);
               digitalWrite(BM1_BCK, r);
               analogWrite(BM1_ENB, spd);
               break;
      case AM2: digitalWrite(AM2_FWD, f);
               digitalWrite(AM2_BCK, r);
               analogWrite(AM2_ENB, spd);
               break;
      case BM2: digitalWrite(BM2_FWD, f);
               digitalWrite(BM2_BCK, r);
               analogWrite(BM2_ENB, spd);
               break;
    }
  }
  
  void setMotorSpeeds(int AM1_speed, int BM1_speed, int AM2_speed, int BM2_speed) {
    setMotorSpeed(AM1, AM1_speed);
    setMotorSpeed(BM1, BM1_speed);
    setMotorSpeed(AM2, AM2_speed);
    setMotorSpeed(BM2, BM2_speed);
    // Serial.println("------------- setMotorSpeeds");
    // Serial.print("AM1: ");
    // Serial.print(AM1_speed);
    // Serial.print(" AM2: ");
    // Serial.print(AM2_speed);
    // Serial.print(" BM1: ");
    // Serial.print(BM1_speed);
    // Serial.print(" BM2: ");
    // Serial.println(BM2_speed);
    // Serial.println("setMotorSpeeds -------------\n");
  }

#endif
