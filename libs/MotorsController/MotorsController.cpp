#include "MotorsController.h"

MotorsController::MotorsController(int pin) : SWSerial(NOT_A_PIN, pin), ST(128, SWSerial) {
  //SWSerial: RX on no pin (unused), TX on given pin (to S1).
  //Sabertooth: Address 128, and use SWSerial as the serial port.
}

void MotorsController::init() {
  SWSerial.begin(9600);
  stop();
  delay(100);
}

void MotorsController::stop() {
  go(0);
}

void MotorsController::check() {
  int p = 10;
  for(int pwr(p); pwr>-p; pwr-=p){
    go(pwr);
    delay(100);
  }
  stop();
}

/*
   send command c to both motors, c between -127 and 127
*/
void MotorsController::go(int c) {
  int pwr = constrain(c, POWER_MIN, POWER_MAX);
  ST.motor(1, pwr);
  ST.motor(2, pwr);
  current_power_motor_1 = pwr;
  current_power_motor_2 = pwr;
}

/*
   send command c to both motors, c between -127 and 127
*/
void MotorsController::go(int power, int radius) {
  int e = (int)(((double)radius - 512) * power / 1024);
  ST.motor(1, power - e);
  ST.motor(2, power + e);
  current_power_motor_1 = power;
  current_power_motor_2 = power;
}




