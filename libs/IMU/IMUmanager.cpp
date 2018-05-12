#include "IMUmanager.h"
#include <Arduino.h>


IMUmanager::IMUmanager()
{
  _axis = Axis::pitch;
}

void IMUmanager::setAxis(char axis, bool direction) {
  _axis = constrain(axis, 0, 2);
  _direction = direction;
}

float IMUmanager::_validateTiltDirection(float tilt) {
	return _direction? tilt : -tilt;
}

