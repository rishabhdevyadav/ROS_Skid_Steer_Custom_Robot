/*
   Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
   Decawave DW1000 library for arduino.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   @file RangingTag.ino
   Use this to test two-way ranging functionality with two DW1000. This is
   the tag component's code which polls for range computation. Addressing and
   frame filtering is currently done in a custom way, as no MAC features are
   implemented yet.

   Complements the "RangingAnchor" example sketch.

   @todo
    - use enum instead of define
    - move strings to flash (less RAM consumption)
*/

#include <SPI.h>
#include <DW1000.h>

#define BAUDRATE     115200

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 20
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 7000;

/**********default Variables which may change************/
uint32_t resetPeriod = 50;

/**********my Variables************/
int pollCount = 0;
int anchorId = 0;
const int noOfAnchors = 3; // To change
int tagId = 21;
bool start = true;


uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
float distances_arr[noOfAnchors];

int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = {'\0'};
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
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
  
  switch(cmd) {
  case 'b':
    Serial.println(BAUDRATE);
    break;

  case 'd':
    for (int ll = 0 ; ll < noOfAnchors ; ll++) {
    Serial.print(distances_arr[ll]);
    Serial.print(' ');
    }
    Serial.println(' ');
    break;
 
  default:
    Serial.println("Invalid Command");
    break;
  }
}

void sendDistances() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = {'\0'};
      else if (arg == 2) argv2[index] = {'\0'};
      
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = {'\0'};
        arg = 2;
        index = 0;
      }
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
    }
  }
}


void setup() {
  // DEBUG monitoring
  Serial.begin(BAUDRATE);
  Serial.println(F("### DW1000-arduino-ranging-tag ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);
  DW1000.setNetworkId(10);
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);//set3
  //DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY); //set2
  //DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_LOWPOWER); //set1

  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message
  receiver();
  transmitPoll();
  noteActivity();
  rangingCountPeriod = millis();

}

void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {
  // tag sends POLL and listens for POLL_ACK
  expectedMsgId = POLL_ACK;
  transmitPoll();
  noteActivity();
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
}

void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  pollCount++;
  data[0] = POLL;
  data[16] = tagId;
  data[17] = anchorId;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  data[16] = tagId;
  data[17] = anchorId;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 1);
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

float getDistance() {
  while (true) {
    if (pollCount > 15)
    {
      expectedMsgId = POLL_ACK;
      return -1.0;
    }
    if (!sentAck && !receivedAck) {
      // check if inactive
      if (millis() - lastActivity > resetPeriod) {
        resetInactive();
      }
      continue;
    }
    // continue on any success confirmation
    if (sentAck) {
      sentAck = false;
      byte msgId = data[0];
      if (msgId == POLL) {
        DW1000.getTransmitTimestamp(timePollSent);
        //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
      } else if (msgId == RANGE) {
        DW1000.getTransmitTimestamp(timeRangeSent);
        noteActivity();
      }
    }
    if (receivedAck) {
      receivedAck = false;
      // get message and parse
      DW1000.getData(data, LEN_DATA);
      /*if (data[17] != tagId)
        {
        expectedMsgId = POLL_ACK;
        transmitPoll();
        noteActivity();
        break;
        }*/
      byte msgId = data[0];
      if (msgId != expectedMsgId) {
        // unexpected message, start over again
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        transmitPoll();
        break;
      }
      if (msgId == POLL_ACK) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT) {
        expectedMsgId = POLL_ACK;
        //Serial.println("Got Range Report...");
        DW1000.getData(data, LEN_DATA);

        /*float curRange;
          memcpy(&curRange, data + 1, 4);
          Serial.println(curRange);
          transmitPoll();*/
        noteActivity();
        return (data[18] + data[19] / 100.0);
      } else if (msgId == RANGE_FAILED) {
        Serial.println("Range Failed*************");
        expectedMsgId = POLL_ACK;
        transmitPoll();
        noteActivity();
      }
    }
  }
  return -1.0;
}

void loop()
{
  if (start)
  {
    int32_t curMillis = millis();

    start = false;
    int i = 0;
    for (i = 1; i <= noOfAnchors; i++)
    {
      anchorId = i;
      pollCount = 0;
      transmitPoll();
      noteActivity();
      //float distance = ;
      distances_arr[i-1] = getDistance();
      //Serial.print("Distance from anchor "); Serial.print(i); Serial.print(" : "); Serial.println(distances_arr[i-1]);
      //Serial.print(distances_arr[i-1]); Serial.print("    ");
    }
    Serial.println( String(distances_arr[0], 4) + ' ' + String(distances_arr[1], 4) + ' ' + String(distances_arr[2], 4) );
    //sendDistances();
    
    //successRangingCount++;
    /*if (curMillis - rangingCountPeriod > 1000) {
      samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
      rangingCountPeriod = curMillis;
      successRangingCount = 0;
    }*/
    //Serial.print(samplingRate); Serial.println("Hz");
  }
  noteActivity();
  delay(120); //170 for 3 bots
  receivedAck = false;
  start = true;
  /*while (true)
  {
    if (millis() - lastActivity > 500)
    {
      start = true;
      receivedAck = false;
      break;
    }
    if (receivedAck)
    {
      receivedAck = false;
      DW1000.getData(data, LEN_DATA);
      if (data[16] == 3 && data[17] == 24 && data[0] == RANGE_REPORT)
      {
        start = true;
      }
    }
    if (start)
    {
      break;
    }
  }*/
}
