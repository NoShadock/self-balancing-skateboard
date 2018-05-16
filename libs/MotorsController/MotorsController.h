#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Arduino.h>

# define POWER_MIN -127
# define POWER_MAX 127

class MotorsController
{
  private:
    int current_power_motor_1;
    int current_power_motor_2;
    SoftwareSerial SWSerial;
    Sabertooth ST;
  public:
    MotorsController(int motorPin);
    void init();
    void stop();
    void check();
    void go(int power);
    void go(int power, int radius);
    int getCurrentPowerMotor1() {
      return current_power_motor_1;
    }
    int getCurrentPowerMotor2() {
      return current_power_motor_2;
    }
    bool isOn() {
      return (current_power_motor_1 | current_power_motor_2);
    }
};






