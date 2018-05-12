#include "MotorsController.h"

MotorsController::MotorsController(int pin) : SWSerial(NOT_A_PIN, pin), ST(128, SWSerial) {
  //SWSerial: RX on no pin (unused), TX on given pin (to S1).
  //Sabertooth: Address 128, and use SWSerial as the serial port.
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




