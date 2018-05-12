/*
 * Self balancing skateboard v1
 *
 *  1. Max tilt and lock safety
 * There is an hard maximum tilt at 40 degrees (+-2). This is when 
 * the motors touch the ground. To avoid chock and damages, power is
 * cut above a tilt limit (absolute value: positive or negative):
 *
**/

#define DEBUG

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <PID_v1.h>
#include <MotorsController.h>

// choose an IMU board
// #include "Sparkfun6DOF.h"
// IMUmanager imu = Sparkfun6DOF();
#include <LSM9DS0_9DOF.h>
LSM9DS0_9DOF myIMU = LSM9DS0_9DOF();


#define ledPin 13
#define motorPin 11
// IMU is on I2C so A4-A5 (see Arduino Wire)


// Period for the execution loop in ms; minimum 10.
// DONT FORGET to correct Kd and Ki if you change this value
const int period = 50;
// tilt measure offset, because the IMU is not perfectly parallel to the board.
const double tiltTrim = 7;
// Safety lock; tilt in degrees
const int safeTilt = 25;
bool tiltLock = false;
// Number of measures for averageing for noise control
const int measureAverageCount = 3;
// Worker vars
int exectime, wait, loopcount = 0, i;
boolean ledhigh = true;
unsigned long time1;
// Set the MotorsController object
MotorsController myMotors = MotorsController(motorPin);
// Set the PID object
double Setpoint, Input, Output;
double Kp=1, Ki=0, Kd=0.07;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("setup");
  printKpid();
  Serial.println();
#endif

  //initialize the variables for the PID
  Setpoint = 0;
  Input = 0;
  Output = 0;
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTime(period);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //begin the motor controller serial
  myMotors.init();

  //begin the IMU
  myIMU.init();
  // yaw=0 pitch=1 roll=2
  myIMU.setAxis(1);
  // Wait IMU stabilization
  Serial.println("Wait IMU stabilization");
  for (i = 0; i < 500; i++) {
    if(i%50==0){
      Serial.println("Wait "+String(i/50)+"/10");
    }
    myIMU.getTilt();
    delay(10);
  }

  // end signal
#ifdef DEBUG
  Serial.print("setup done ");
  Serial.print(millis(), DEC);
  Serial.println("ms");
#endif
  digitalWrite(ledPin, HIGH);
}


void loop() {
  // Time debugging shows loop execution time -> dms-off: 13-15ms, dms-on: 22-24ms
  time1 = millis();

  // anti freeze visual check
  fancyLedBlink();

  // Get measure of tilt (average for noise reduction)
  Input = 0;
  for (i = 0; i < measureAverageCount; i++) {
    // SHOULD DO SOMETHING HERE to avoid freeze from Wire/twi.c lib
    Input += myIMU.getTilt();
  }
  Input /= measureAverageCount;
  Input += tiltTrim;

  // Safety lock: check maximum tilt
  if ((!tiltLock && (abs(Input) > safeTilt)) || (tiltLock && (abs(Input) < 1))) {
    if (tiltLock) { // release tiltLock
      myPID.Reset();
    }
    tiltLock = !tiltLock;
  }

  // Command
  if (tiltLock) {
    myMotors.stop();
  } else {
    myPID.Compute();
    myMotors.go((int)Output);
  }

  // dynamic delay time to stick to the period (loop exec time~=8ms)
  exectime = (int)(millis() - time1);
#ifdef DEBUG
  printDebug();
#endif
  wait = exectime > period ? 0 : period - exectime;
  delay(wait);
}


#ifdef DEBUG
void printKpid() {
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(" | Ki: ");
  Serial.print(Ki);
  Serial.print(" | Kd: ");
  Serial.print(Kd);
}

void printDebug(){
  Serial.print(time1);
  Serial.print(" | ");
  Serial.print(Input);
  Serial.print(" | ");
  Serial.print(Output);
  Serial.print(" | ");
  Serial.print(exectime);
  Serial.print(" | ");
  printKpid();
  if (tiltLock)
    Serial.print("| tilt lock");
  Serial.println();
}
#endif

/**
   blink led to enable freeze block detection
*/
void   fancyLedBlink() {
  loopcount++;
  loopcount %= 10;
  if (loopcount == 0) {
    ledhigh = !ledhigh;
    digitalWrite(13, ledhigh ? HIGH : LOW);
  }
}






