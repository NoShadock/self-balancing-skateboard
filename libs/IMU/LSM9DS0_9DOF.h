#include "IMUmanager.h"
// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>


class LSM9DS0_9DOF : public IMUmanager {
  
  private:
    LSM9DS0 dof;

	float pitch, yaw, roll;
	float deltat = 0.0f;        // integration interval for both filter schemes
	uint32_t lastUpdate = 0;    // used to calculate integration interval
	uint32_t Now = 0;           // used to calculate integration interval
	
	float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
	float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
	float temperature;

  	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
 	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);



  public:
    LSM9DS0_9DOF();
    void init();
    float getTilt();
};
























