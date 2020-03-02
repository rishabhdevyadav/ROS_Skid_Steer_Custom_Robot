/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
/*
     ^
NE --+-- NW
	 |
SE --+-- SW

*/

#ifdef L298_MOTOR_DRIVER
/*  #define NE_MOTOR_FORWARD 6
  #define NE_MOTOR_BACKWARD 5
  #define NE_MOTOR_ENABLE 7

  #define SE_MOTOR_FORWARD 3
  #define SE_MOTOR_BACKWARD 4
  #define SE_MOTOR_ENABLE 2

  #define SW_MOTOR_FORWARD 9
  #define SW_MOTOR_BACKWARD 10
  #define SW_MOTOR_ENABLE 8

  #define NW_MOTOR_FORWARD 12
  #define NW_MOTOR_BACKWARD 11   
  #define NW_MOTOR_ENABLE  13*/

/*
     ^
BM1 --+-- AM1
   |
BM2 --+-- AM2

*/

  #define AM1_FWD 22
  #define AM1_BCK 24
  #define AM1_ENB 5

  #define AM2_FWD 26
  #define AM2_BCK 28
  #define AM2_ENB 6

  #define BM1_FWD 23
  #define BM1_BCK 25
  #define BM1_ENB 2

  #define BM2_FWD 27
  #define BM2_BCK 29
  #define BM2_ENB 3
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int AM1_speed, int BM1_speed, int AM2_speed, int BM2_speed);