//#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Wire.h>
#include <FreeSixIMU.h>
#include <PID_v1.h>
#include "IMUmanager.h"
#include "MotorsController.h"

/*
 *  Self balancing skateboard v1
 * max tilt (+/-)32.0 is when motors touch the ground
 */
// Safety lock; tilt in degrees, power in %
const float safeTilt = 20;
// Period for the execution loop in ms; minimum 10
const int period = 50;
// Number of measures for averageing for noise control
const int measureAverageCount = 3;
// Worker vars
int exectime, wait, loopcount = 0, i;
unsigned long time1;
// Set the IMUmanager object
IMUmanager myIMU = IMUmanager();
// Set the MotorsController object
MotorsController myMotors = MotorsController();
// Set the PID object
double Setpoint, Input, Output, Tilt;
double Kp = 0, Ki = 0, Kd = 0;
double maxKp = 5, maxKi = 5, maxKd = 5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // debug
//  Serial.begin(115200);
//  Serial.println("setup");
  time1 = millis();

  // light up led 13 while setting up
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

//  Serial.println("init motors");

  //begin the motor controller serial
  myMotors.init();
  delay(5);

//  Serial.println("init imu");

  //begin the IMU
  Wire.begin();
  delay(5);
  myIMU.init();
  // yaw=0 pitch=1 roll=2
  myIMU.setAxis(1);
  delay(500);
//  Serial.println("sample imu stable");
  for (i = 0; i < 200; i++) {
    Tilt = myIMU.getTilt();
    delay(5);
  }

//  Serial.println("init pid");

  //initialize the variables for the PID
  Setpoint = 0;
  Input = 0;
  Output = 0;
  // PID gain from analog pins
  int akp = analogRead(A0), aki = analogRead(A1), akd = analogRead(A2);
  Kp = (double)akp * maxKp / 1023;
  Ki = (double)aki * maxKi / 1023;
  Kd = (double)akd * maxKd / 1023;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTime(period);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  delay(5);

  // end signal
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);

//  Serial.print("started in ");
//  Serial.print(millis() - time1);
//  Serial.println(" ms.");
}

void loop() {
  digitalWrite(13, HIGH);
  //  Time debugging shows: t1-t2=1ms, t2-t3=6ms, t3-t4=3ms, average total at 9ms
  time1 = millis();
//  Serial.print(time1);
//  Serial.print(" | ");

  // Get measure of tilt (average for noise reduction)
  Tilt = 0;
  for (i = 0; i < measureAverageCount; i++) {
    Tilt += myIMU.getTilt();
  }
  Tilt /= measureAverageCount;

//  Serial.print(Tilt);
//  Serial.print(" | ");

  // Safety lock
  if (abs(Input) > safeTilt) {
    myMotors.stop();
    // safety release
    if (abs(Tilt) < 1)
      Input = 0;
    delay(period);
//    Serial.println(" lock");
    return;
  }

  // Command
  Input = Tilt;

//  Serial.print(" | ");
//  Serial.print(Input);

  myPID.Compute();
  myMotors.go((int)Output); // time debug: takes average 2ms

//  Serial.print(" | ");
//  Serial.println(Output);

  // dynamic delay time to stick to the period (loop exec time~=8ms)
  exectime = (int)(millis() - time1);
  wait = exectime > period ? 0 : period - exectime;
  delay(wait);
}

