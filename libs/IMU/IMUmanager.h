#include <Arduino.h>


class IMUmanager
{
  protected:
    char _axis = 1;
    bool _direction = true; // used to orientate the IMU
    float angles[3] = {0.0f, 0.0f, 0.0f}; // yaw pitch roll
    float _validateTiltDirection(float tilt);

  public:
    IMUmanager();
    void setAxis(char axis, bool direction = true);
    virtual void init();
    virtual float getTilt();
};

enum Axis : char {
	yaw = 0,
	pitch = 1,
	roll = 2
};

