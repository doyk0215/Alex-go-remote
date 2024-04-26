#include <stdarg.h>
#include <serialize.h>
#include <math.h>
//#include <MeMCore.h>
#include <Arduino.h>

#include "packet.h"
#include "constants.h"

#define ALEX_BREADTH 15.5
#define ALEX_LENGTH 25.8


// const int thresholdDistance = 20; // Distance in centimeters

float alexDiagonal;
float alexCirc;

volatile TDirection dir;

/*
   Alex's configuration constants
*/
// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 21

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile float leftRevs;
volatile float rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;

//variables to keep track of our turning andle
unsigned long deltaTicks;
unsigned long targetTicks;

/* Alex Ultrasonic Functions */

// Define pins for ultrasonic sensor
const int trigPin = 49;  // Trig pin of the ultrasonic sensor
const int echoPin = 47;  // Echo pin of the ultrasonic sensor

// Function to initialize ultrasonic sensor pins
void initUltrasonic() {
  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int getDistance() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters
  int distance = duration / 58;
  return distance;
}


/* Alex colour Detection Functions */

// basic definitions for colour sensor and audio player
#define S0 37
#define S1 39
#define S2 41
#define S3 43
#define sensorOut 45
#define song_1 A8
#define song_2 A9
#define song_3 A10
#define sound_out A12

void initColour() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  /*//pinMode(sound, OUTPUT);
    pinMode(sound_out, INPUT);*/
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

// colour declaration
int red = 0;
int green = 0;
int blue = 0;

// array for A2, A3 (index 0 = Red, 1 = Green, 2 = Blue)
int ledArray[] = { 0, 1, 0, 0, 1, 1 };

// declaration for colour arrays
float colourArray[] = { 0, 0, 0 };
float colourLookup[6][3] = {
  { 376, 390, 338 },  // white
  { 408, 683, 563 },  // red
  { 641, 563, 566 },  // Green
};

float colourRange[3] = { 0, 0, 0 };
char colourStr[3][5] = { "R = ", "G = ", "B = " };

int getAvgReading(int times) {
  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;

  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = pulseIn(sensorOut, LOW);
    total += reading;
    delay(100);
  }
  //calculate the average and return it
  return total / times;
}

// squares number
float square(float x) {
  return x * x;
}
int getRedPW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getBluePW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

// gets RGB value
void getRGB() {
  for (int c = 0; c <= 2; c += 1) {
    digitalWrite(S2, ledArray[c]);
    digitalWrite(S3, ledArray[c + 3]);
    delay(100);
    //get average of 5 consecutive readings for current colour and store average.
    colourArray[c] = getAvgReading(5);
  }
}

int SumRGB(bool is_white, int dist) {
  if (is_white) {
    return 37 + 129 * dist - 3 * dist * dist;
  }
  return 124 + 226 * dist - (int)(8.6 * dist * dist);
}

int checkColour() {
  startPlay(0);
  int dist = getDistance();
  int colour = 3;
  
  if (dist > 15) {
    delay(500);
    stopPlay(0);
    //delay(500);
    return colour;
  }
  // not aligned, reject
 
  getRGB();

  int currentRGBSum = 0;
  //Serial.println();
  for (int j = 0; j <= 2; j += 1) {
    currentRGBSum += colourArray[j];
  }
  // initialises values
  int error_RGBsum = 0;
  bool is_white = 0;

  // non-existent colour value
  
  int error_assume_green = abs(SumRGB(0, dist) - currentRGBSum);
  int error_assume_white = abs(SumRGB(1, dist) - currentRGBSum);
  if (error_assume_green > error_assume_white) {
    colour = 0; // white
  }
  else if (colourArray[0] < colourArray[1] - 20) {
    colour = 1; // red
  }
  else {
    colour = 2; //green
  }
  
  stopPlay(0);
  //delay(500);
  //delay(100);
  return colour;
}

/*
//songnum: 0~2
void play() {
   digitalWrite(song_1, HIGH);
   //digitalWrite(song_2, HIGH);
   //digitalWrite(song_3, HIGH);
   //digitalWrite(8 + songnum, LOW); // this selects the song 
   digitalWrite(song_1, LOW); // this selects song_1
   
   digitalWrite(sound_out, LOW); // this starts the song 
   delay(5000);
   digitalWrite(sound_out, HIGH); // this ends the song
}
*/
//songnum: 0~2
void startPlay(int songnum) {
   digitalWrite(song_1 + songnum, LOW); //this selects and plays the song 
  // delay needed in between startPlay and stopPlay
}

void stopPlay(int songnum) {
   digitalWrite(song_1 + songnum, HIGH); // this deselects the song
}



/*

   Alex Communication Routines.

*/

void left(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
     //dbprintf("ang %d\n", (int)ang);
    deltaTicks = ang;
    //dbprintf("deltaTicks %d\n", (int)deltaTicks);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  ccw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = ang;
  }

  targetTicks = leftForwardTicksTurns + deltaTicks;

  cw(ang, speed);
}


TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}


void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
  //Serial.print("RIGHT: ");
  //Serial.println(forwardDist);
}

void sendStatus() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}


void sendColour() {
  
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = checkColour();
  colourPacket.params[1] = (int) colourArray[0]; 
  colourPacket.params[2] = (int) colourArray[1]; 
  colourPacket.params[3] = (int) colourArray[2]; 

  
  sendResponse(&colourPacket);
}

void sendUltrasonic() {
  TPacket ultrasonicPacket;
  ultrasonicPacket.packetType = PACKET_TYPE_RESPONSE;
  ultrasonicPacket.command = RESP_ULTRASONIC;
  ultrasonicPacket.params[0] = getDistance();
  //dbprintf("Ultrasonic %d\n", ultrasonicPacket.params[0]);
  
  sendResponse(&ultrasonicPacket);
}


// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EICRA = 0b10100000;
  EIMSK = 0b00001100;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

ISR(INT2_vect) {
  rightISR();
}

ISR(INT3_vect) {
  leftISR();
}
// Implement INT2 and INT3 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;

  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  /*switch(which)
    {
    case 0:
      clearCounters();
      break;

    case 1:
      leftTicks=0;
      break;

    case 2:
      rightTicks=0;
      break;

    case 3:
      leftRevs=0;
      break;

    case 4:
      rightRevs=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
    }*/
  clearCounters();
}
// Intialize Alex's internal states

void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  int x = 10;
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      // int x = 10;
      x = getDistance(); 
      //forward(distance, speed);
      if (x > 8 || x <= 0) {
        forward((double)command->params[0], (float)command->params[1]);
      }
      break;

    case COMMAND_REVERSE:
      sendOK();
      //backward(distance, speed);
      backward((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      //left(distance, speed);
      left((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      //right(distance, speed);
      right((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;

    case COMMAND_GET_COLOUR:
      sendColour();
      break;

    case COMMAND_GET_ULTRASONIC:
      sendUltrasonic();
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {


        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }  // !exit
}

void initSong() {
  pinMode(song_1, OUTPUT);
  digitalWrite(song_1, HIGH);
  // add new songs here
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();

  initUltrasonic();
  initColour();
  initSong();

  float alexDiagonal = sqrt(ALEX_BREADTH * ALEX_BREADTH + ALEX_LENGTH * ALEX_LENGTH);
  float alexCirc = PI * alexDiagonal;
}

void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  //forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2
  // Print distance to serial monitor
  /*Serial.print("Distance: ");
    Serial.print(measureDistance());
    Serial.println(" cm");*/


  // put your main code here, to run repeatedly:
  TPacket recvPacket;  // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  //int y = 10;
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      //y = getDistance();
      if (forwardDist > newDist) { // | y < 8
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      //dbprintf("reverse %d\n", reverseDist); 
      //dbprintf("new %d\n", newDist); 
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == (TDirection)STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (leftForwardTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == (TDirection)STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
