#include "MotorsController.h"

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

MotorsController::MotorsController() {
}

void MotorsController::init() {
  SWSerial.begin(9600);
  stop();
}

void MotorsController::stop() {
  ST.motor(1, 0);
  ST.motor(2, 0);
  current_power_motor_1 = 0;
  current_power_motor_2 = 0;
}

/*
   send command c to both motors, c between -127 and 127
*/
void MotorsController::go(int c) {
  ST.motor(1, c);
  ST.motor(2, c);
  current_power_motor_1 = c;
  current_power_motor_2 = c;
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
