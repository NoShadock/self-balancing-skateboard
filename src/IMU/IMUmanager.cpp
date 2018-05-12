#include "IMUmanager.h"
#include <Arduino.h>


IMUmanager::IMUmanager()
{
  _axis = Axis::pitch;
}

void IMUmanager::setAxis(char axis) {
  _axis = constrain(axis, 0, 2);
}

























