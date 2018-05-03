#include "IMUmanager.h"

IMUmanager::IMUmanager() {
  sixDOF = FreeSixIMU();
  ax = 1; //
}

void IMUmanager::init() {
  sixDOF.init();
}

void IMUmanager::setAxis(char axis) {
  ax = constrain(axis, 0, 2);
}

float IMUmanager::getTilt() {
  sixDOF.getYawPitchRoll(angles);
  return angles[ax];
}

