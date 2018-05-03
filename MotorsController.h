#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Arduino.h>

class MotorsController
{
  private:
    int current_power_motor_1;
    int current_power_motor_2;
  public:
    MotorsController();
    void init();
    void stop();
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


