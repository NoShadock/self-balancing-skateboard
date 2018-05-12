#include "IMUmanager.h"
#include "Wire.h"
#include <FreeSixIMU.h>

class Sparkfun6DOF : public IMUmanager
{
  private:
    FreeSixIMU dof;
  
  public:
    Sparkfun6DOF();
    void init();
	float getTilt();
};























