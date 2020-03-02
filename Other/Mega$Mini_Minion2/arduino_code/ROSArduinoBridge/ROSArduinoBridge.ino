
#define USE_BASE      // Enable the base controller code

#ifdef USE_BASE
  #define ARDUINO_ENC_COUNTER
  #define L298_MOTOR_DRIVER
#endif

// #define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sensors.h"
#include "commands.h"

#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  #include "piston.h"
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"
  #include "battery_monitor.h"

  #define PID_RATE 60// Hz /* Run the PID loop at 30 times per second */
  const int PID_INTERVAL = 1000 / PID_RATE; /* Convert the rate into an interval */
  unsigned long nextPID = PID_INTERVAL; /* Track the next time we make a PID calculation */

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 200000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */
// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;
char chr; // Variable to hold an input character
char cmd; // Variable to hold the current single-character command

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);
  // Serial.println("Some Command");
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(AM1));
    Serial.print(" ");
    Serial.print(readEncoder(BM1));
    Serial.print(" ");
    Serial.print(readEncoder(AM2));
    Serial.print(" ");
    Serial.println(readEncoder(BM2));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (!(arg1 || arg2 || arg3 || arg4)) {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    AM1_PID.TargetTicksPerFrame = arg1;
    BM1_PID.TargetTicksPerFrame = arg2;
    AM2_PID.TargetTicksPerFrame = arg3;
    BM2_PID.TargetTicksPerFrame = arg4;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  case PISTON:
    pistonControl(arg1);
    Serial.println("OK"); 
    break;
#endif
  default:
    String bb = "Invalid Command ";
//    String ll = cmd;/
    String gg = bb + cmd;
    Serial.println(gg);
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    cli();
    // set as inputs
    DDRK &= 0x0;
    // pull up
    PORTK |= 0xff;
    PCICR |= (1 << PCIE2);
    // alternatively
    // PCICR |= 0b00000100;
    PCMSK2 |= 0xff; // all pins set to 1
    // alternatively
    PCMSK2 |= 0b11111111; // all pins set to 1
    sei();
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {

  while (Serial.available() > 0) {
    chr = Serial.read();
  
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if (arg == 3) argv3[index] = NULL;
      else if (arg == 4) argv4[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) arg = 2;
      else if (arg == 2) arg = 3;
      else if (arg == 3) arg = 4;
      index = 0;
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
      else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }
  

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
