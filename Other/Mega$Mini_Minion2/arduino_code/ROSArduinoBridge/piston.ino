/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   void pistonControl(int val)
   {
      if (val == 1){
      digitalWrite(PM1, LOW);
      digitalWrite(PM2, HIGH);
      Serial.println("UP");
      }
      else if (val == 2){
      digitalWrite(PM1, HIGH);
      digitalWrite(PM2, LOW);
      Serial.println("DOWN");
      }
      else {
      digitalWrite(PM1, LOW);
      digitalWrite(PM2, LOW);
      Serial.println("STOP");
      }
   }

#endif
