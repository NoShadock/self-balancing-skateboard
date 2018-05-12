#include <Arduino.h>


class IMUmanager
{
  protected:
    char _axis;
    float angles[3] = {0.0f, 0.0f, 0.0f}; // yaw pitch roll

  public:
    IMUmanager();
    void setAxis(char axis);
    virtual void init();
    virtual float getTilt();
};

enum Axis : char {
	yaw = 0,
	pitch = 1,
	roll = 2
};


























