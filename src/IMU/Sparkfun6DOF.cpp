#include "Sparkfun6DOF.h"

Sparkfun6DOF::Sparkfun6DOF() : IMUmanager(), dof() {
}

void Sparkfun6DOF::init() {
  Wire.begin();
  dof.init();
}

float Sparkfun6DOF::getTilt() {
  dof.getYawPitchRoll(angles);
  return angles[_axis];
}
























