
class MotorsController
{
  private:
    int current_power_motor_1;
    int current_power_motor_2;
  public:
    MotorsController();
    void init();
    void stop();
    void go(int c);
    int getCurrentPowerMotor1(){ return current_power_motor_1; }
    int getCurrentPowerMotor2(){ return current_power_motor_2; }
};

