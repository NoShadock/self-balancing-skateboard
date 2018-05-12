#include <FreeSixIMU.h>

class IMUmanager
{
  private:
    char ax;
    FreeSixIMU sixDOF;
    float angles[3]; // yaw pitch roll
  
  public:
    IMUmanager();
    void init();
    void setAxis(char axis);
    float getTilt();
};






