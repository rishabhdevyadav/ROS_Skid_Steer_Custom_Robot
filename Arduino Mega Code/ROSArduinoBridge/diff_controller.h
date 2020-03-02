/* Functions and type-defs for PID control.
   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                // last input
  int ITerm;                    //integrated term
  long output;                    // last motor setting
}
SetPointInfo;
SetPointInfo BM1_PID, BM2_PID, AM1_PID, AM2_PID;

int Kp = 40;
int Kd = 10;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

void resetPID(){
   BM1_PID.TargetTicksPerFrame = 0.0;
   BM1_PID.Encoder = readEncoder(BM1);
   BM1_PID.PrevEnc = BM1_PID.Encoder;
   BM1_PID.output = 0;
   BM1_PID.PrevInput = 0;
   BM1_PID.ITerm = 0;

   AM1_PID.TargetTicksPerFrame = 0.0;
   AM1_PID.Encoder = readEncoder(AM1);
   AM1_PID.PrevEnc = AM1_PID.Encoder;
   AM1_PID.output = 0;
   AM1_PID.PrevInput = 0;
   AM1_PID.ITerm = 0;

   BM2_PID.TargetTicksPerFrame = 0.0;
   BM2_PID.Encoder = readEncoder(BM2);
   BM2_PID.PrevEnc = BM2_PID.Encoder;
   BM2_PID.output = 0;
   BM2_PID.PrevInput = 0;
   BM2_PID.ITerm = 0;

   AM2_PID.TargetTicksPerFrame = 0.0;
   AM2_PID.Encoder = readEncoder(AM2);
   AM2_PID.PrevEnc = AM2_PID.Encoder;
   AM2_PID.output = 0;
   AM2_PID.PrevInput = 0;
   AM2_PID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p, int id) {
  long Perror;
  long output;
  int input;
  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;
  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
 /* if (1){
    Serial.print("------------------ doPID ---- ID: ");
    Serial.print(id);
    Serial.print("\nPerror: ");
    Serial.print(Perror);
    Serial.print("  output   ");
    Serial.println(output);
    Serial.print("  TargetTicksPerFrame    ");
    Serial.println(p->TargetTicksPerFrame);
    Serial.print("  input   ");
    Serial.println(input);
    Serial.print("  enc   ");
    Serial.println(p->PrevEnc);
    Serial.println("doPID ------------------\n");
  }*/
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  AM1_PID.Encoder = readEncoder(AM1);
  BM1_PID.Encoder = readEncoder(BM1);
  BM2_PID.Encoder = readEncoder(BM2);
  AM2_PID.Encoder = readEncoder(AM2);

 /* Serial.println("------------- updatePID");
    Serial.print("AM1: ");
    Serial.print(AM1_PID.Encoder);
    Serial.print(" AM2: ");
    Serial.print(AM2_PID.Encoder);
    Serial.print(" BM1: ");
    Serial.print(BM1_PID.Encoder);
    Serial.print(" BM2: ");
    Serial.println(BM2_PID.Encoder);
    Serial.println("updatePID -------------\n");*/

  if (!moving){
    if (
      BM1_PID.PrevInput != 0 ||
      AM1_PID.PrevInput != 0 ||
      BM2_PID.PrevInput != 0 ||
      AM2_PID.PrevInput != 0
      ) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&BM1_PID, BM1);
  doPID(&AM1_PID, AM1);
  doPID(&BM2_PID, BM2);
  doPID(&AM2_PID, AM2);
  /* Set the motor speeds accordingly */
  setMotorSpeeds(AM1_PID.output, BM1_PID.output, AM2_PID.output, BM2_PID.output);
}
