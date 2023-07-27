
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION
//***ALWAYS CHECK MOWER.H and .CPP ACCORDING THE PLATFORM BEFORE UPLOAD A NEW VERSION

//to do 
//delete last rfid tag
//change declar location for area1_ip etc....
//imu.readImuTemperature(); is it possible locking teensy ?


//On the first start if beep on error you need to do:
//command/off
//factory setting and DO NOT save setting.
//reboot
//command/off
// clear error counter 
//setting save user setting.
//redo the motor calibration.

#include "robot.h"
#include "mower.h"

void setup()  {
  robot.setup();
}

void loop()  {
  robot.loop();
}
