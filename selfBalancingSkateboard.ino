
#define DEBUG
//#define PID_TUNING_ZIEGLER_NICHOLS

#include <Arduino.h>

#ifdef PID_TUNING_ZIEGLER_NICHOLS
#include <EEPROM.h>
#endif

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Wire.h>
#include <FreeSixIMU.h>
#include <PID_v1.h>
#include "IMUmanager.h"
#include "MotorsController.h"

#define resetPin 3
#define dischargePin 4
#define ledPin 13
#define dmsPin 7
#define wheelPin A0
#define s1Pin 8
#define s2Pin 9

/*
    Self balancing skateboard v1
   max tilt (+/-)32.0 is when motors touch the ground
*/
// Safety lock; tilt in degrees
double tiltTrim = 4;
const int safeTilt = 20;
// Period for the execution loop in ms; minimum 10
const int period = 50;
// Number of measures for averageing for noise control
const int measureAverageCount = 3;
// Worker vars
int exectime, wait, loopcount = 0, i;
boolean ledhigh = true;
unsigned long time1;
// Set the IMUmanager object
IMUmanager myIMU = IMUmanager();
// Set the MotorsController object
MotorsController myMotors = MotorsController();
// Set the PID object
double Setpoint, Input, Output;
// no load measure: Ku=3 - Tu=0.45
// loaded 83kg measure: Ku=4 - Tu=0.2
double Ku = 4, Tu = 0.2;
/**** PID VARS ****/
// pid classic
// double Kp = 0.6*Ku, Ki = 2*Kp/Tu, Kd = Kp*Tu/8;
// some overshoot
//double Kp = 0.33 * Ku, Ki = 2 * Kp / Tu, Kd = Kp * Tu / 3;
// no overshoot
//double Kp = 0.2 * Ku, Ki = 2 * Kp / Tu, Kd = Kp * Tu / 3;
// tuto vars
//double overallGain = 0.6, degRad = 0.017453, conv = overallGain * degRad * 127;
//double Kp = 4.5 * conv, Ki = 1.0 * conv, Kd = 0.5 * conv;
// manual
//double Kp = 0, Ki = 0, Kd = 0;
double Kp = 6.7, Ki = 0, Kd = 0.07;
/**** END PID VARS ****/
#ifdef PID_TUNING_ZIEGLER_NICHOLS
double maxKp = 10, maxKi = 5, maxKd = 0.5;
int v[3];
byte j = 0, jMem = -1;
#else
int wheel;
#endif
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool dmsRun = false, dmsLock = false, tiltLock = false;

void setup() {
  // start by setting reset pin to low to avoid reset loop
  digitalWrite(resetPin, LOW);
  digitalWrite(dischargePin, LOW);
  pinMode(dischargePin, OUTPUT);
  pinMode(resetPin, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(dmsPin, INPUT_PULLUP);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("setup");
  printKpid();
  Serial.println();
#ifdef PID_TUNING_ZIEGLER_NICHOLS
  loadVars();
  Kp = (double)v[0] * maxKp / 1023;
  Ki = (double)v[1] * maxKi / 1023;
  Kd = (double)v[2] * maxKd / 1023;
  selectK();
  Serial.print(j);
  Serial.print(" - ");
#endif
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
  Wire.begin();
  myIMU.init();
  // yaw=0 pitch=1 roll=2
  myIMU.setAxis(1);
  for (i = 0; i < 200; i++) {
    myIMU.getTilt();
  }

  // end signal
#ifdef DEBUG
  Serial.print("setup done ");
  Serial.print(millis(), DEC);
  Serial.println("ms");
#endif
  digitalWrite(13, HIGH);
}


void loop() {
  // Time debugging shows loop execution time -> dms-off: 13-15ms, dms-on: 22-24ms
  time1 = millis();
#ifdef PID_TUNING_ZIEGLER_NICHOLS
  selectK();
  if (j != jMem) {
    Serial.print(j);
    Serial.print(" - changing ");
    Serial.println(j == 0 ? "Kp" : (j == 1 ? "Ki" : (j == 2 ? "Kd" : "ERROR")));
    jMem = j;
  }
#endif

  // safety lock DMS
  if (digitalRead(dmsPin) ^ dmsLock) {
    if (!dmsLock) { // dms cut -> lock
      myMotors.stop();
#ifdef PID_TUNING_ZIEGLER_NICHOLS
      resetPID();
#else
      myPID.Reset();
#endif
    }
    dmsLock = !dmsLock;
  }

  // anti freeze visual check
  fancyLedBlink();

  // Get measure of tilt (average for noise reduction)
  Input = 0;
  for (i = 0; i < measureAverageCount; i++) {
    // To avoid freeze from Wire/twi.c lib, launch hardware-failsafe delayed reset
    delayedReset(true);
    Input += myIMU.getTilt();
    // Wire reading successful, cancel reset
    delayedReset(false);
  }
  Input /= measureAverageCount;
  Input += tiltTrim;

  // Safety lock Tilt
  if ((!tiltLock && (abs(Input) > safeTilt)) || (tiltLock && (abs(Input) < 1))) {
    if (tiltLock) // release tiltLock
#ifdef PID_TUNING_ZIEGLER_NICHOLS
      resetPID();
#else
      myPID.Reset();
#endif
    tiltLock = !tiltLock;
  }

  // Command
  if (dmsLock || tiltLock) {
    if (myMotors.isOn())
      myMotors.stop();
  } else {
    myPID.Compute();
#ifdef PID_TUNING_ZIEGLER_NICHOLS
    myMotors.go((int)Output);
#else
    wheel = analogRead(wheelPin);
    myMotors.go((int)Output, wheel);
#endif
  }

  // dynamic delay time to stick to the period (loop exec time~=8ms)
  exectime = (int)(millis() - time1);
#ifdef DEBUG
  Serial.print(time1);
  Serial.print(" | ");
  Serial.print(Input);
  Serial.print(" | ");
  Serial.print(Output);
  Serial.print(" | ");
  Serial.print(exectime);
  Serial.print(" | ");
  printKpid();
  if (dmsLock)
    Serial.print("| dms lock");
  if (tiltLock)
    Serial.print("| tilt lock");
  Serial.println();
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
#endif

#ifdef PID_TUNING_ZIEGLER_NICHOLS
/**
   reload with new PID parameters
*/
void resetPID() {
  // PID gain from analog pins
  // loadVars();
  // selectK();
  // selectK sets j to 0=Kp, 1=Ki, 2=Kd from the switches
  v[j] = analogRead(wheelPin);
  // saveVars();
  Kp = (double)v[0] * maxKp / 1023;
  Ki = (double)v[1] * maxKi / 1023;
  Kd = (double)v[2] * maxKd / 1023;
#ifdef DEBUG
  Serial.println();
  Serial.print(j);
  Serial.print(" - ");
  printKpid();
  Serial.println();
#endif
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.Reset();
}
/*
   select K to change from switches, 0=Kp, 1=Ki, 2=Kd
*/
void selectK() {
  j = digitalRead(s2Pin) << 1 | digitalRead(s1Pin);
  j++;
  j %= 4;
}
void loadVars() {
  for (j = 0; j < 3; j++) {
    EEPROM.get(j << 1, v[j]);
    if (v[j] < 0) v[j] = 0;
  }
}
void saveVars() {
  for (j = 0; j < 3; j++)
    EEPROM.put(j << 1, v[j]);
}
void printVars() {
  for (j = 0; j < 3; j++) {
    Serial.print(v[j], DEC);
    Serial.print(" | ");
  }
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

/**
   Failsafe for software bugs from Wire/twi.c infinity loops
   activate=true will set pin HIGH to load capa that will trigger arduino's RESET
   Load delay is between 60 and 70ms
   So expect the ABomb after 60ms: it needs to be deactivated with follow up call activate=false
*/
void delayedReset(bool activate) {
  if (activate) {
    digitalWrite(resetPin, HIGH);
    pinMode(dischargePin, INPUT);
  } else {
    digitalWrite(resetPin, LOW);
    // discharge capa to avoid building up load over sequential calls
    pinMode(dischargePin, OUTPUT);
  }
}

