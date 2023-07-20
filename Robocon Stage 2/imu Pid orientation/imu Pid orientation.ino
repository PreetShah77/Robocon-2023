#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"

LSM6DSO myIMU;

double z,Yaw,Yaw2;
double timeStep = 0.000144;

void setup() {
  Serial.begin(9600);
  delay(10);
  myIMU.begin();
  myIMU.initialize(BASIC_SETTINGS);
}
void loop()
{
  z = myIMU.readFloatGyroZ();
  Yaw = Yaw + int(z) * timeStep;
  Yaw2 = Yaw *(180/M_PI);
  Serial.println(Yaw2);
}
