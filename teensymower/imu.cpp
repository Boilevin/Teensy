/*
  License
  Copyright (c) 2013-2017 by Alexander Grau

  Private-use only! (you need to ask for a commercial-use)

  The code is open: you can modify it under the terms of the
  GNU General Public License as published by the Free Software Foundation,
  either version 3 of the License, or (at your option) any later version.

  The code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)

*/

#include "imu.h"
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps_v6_12.h"
#include "mower.h"
#include "i2c.h"
#include "robot.h"
#include "flashmem.h"

///////////////////////////////////   CONFIGURATION FOR ACCEL GYRO MPU6050 CALIBRATION  /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); //if ad0 is shunt

boolean blinkState = false;
float nextTimeLoop;
float nextSendPitchRollError;
// MPU control/status vars
boolean dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//for accelgyro calib
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

float yprtest[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define ADDR 600
#define MAGIC 6
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)
#define QMC5883L (0x0D)          // HMC5883L compass sensor (GY-80 PCB)

void IMUClass::begin() {
  if (!robot.imuUse) return;

  //initialisation of Compass
  if (robot.CompassUse) {

    if (COMPASS_IS == HMC5883L) {
      Serial.println(F("--------------------------------- COMPASS HMC5883L INITIALISATION ---------------"));
      uint8_t data = 0;
      //while (true) {
      I2CreadFrom(HMC5883L, 10, 1, &data, 1);
      Serial.print(F("COMPASS HMC5883L ID NEED TO BE 72 IF ALL IS OK ------>  ID="));
      Serial.println(data);
      if (data != 72) Serial.println(F("COMPASS HMC5883L FAIL"));
      delay(1000);
      //}
      comOfs.x = comOfs.y = comOfs.z = 0;
      comScale.x = comScale.y = comScale.z = 2;
      useComCalibration = true;
      I2CwriteTo(HMC5883L, 0x00, 0x70);  // config A:  8 samples averaged, 75Hz frequency, no artificial bias.
      I2CwriteTo(HMC5883L, 0x01, 0x20);  // config B: gain
      I2CwriteTo(HMC5883L, 0x02, 00);  // mode: continuous
      delay(2000); //    wait 2 second before doing the first reading
    }

    if (COMPASS_IS == QMC5883L) {
      Serial.println(F("--------------------------------- COMPASS QMC5883L INITIALISATION ---------------"));
      comOfs.x = comOfs.y = comOfs.z = 0;
      comScale.x = comScale.y = comScale.z = 2;
      useComCalibration = true;
      I2CwriteTo(QMC5883L, 0x0b, 0x01);
      /*
         osr512         00
         rng 8g         01
         odr 100hz      10
         mode continous 01

        so 00011001 = 0x19
      */
      I2CwriteTo(QMC5883L, 0x09, 0x19);
      delay(500); //    wait  before doing the first reading
    }
  }

  loadCalib();
  printCalib();

  //initialisation of MPU6050
  Serial.println(F("--------------------------------- GYRO ACCEL INITIALISATION ---------------"));
  mpu.initialize();
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  }
  else
  {
    Serial.println("MPU6050 connection failed");
    robot.imuUse = false;
    return;

  }
  Serial.println(mpu.getDeviceID());
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(ax_offset); //-1929
  mpu.setYAccelOffset(ay_offset); //471
  mpu.setZAccelOffset(az_offset); // 1293
  mpu.setXGyroOffset(gx_offset);//-1
  mpu.setYGyroOffset(gy_offset);//-3
  mpu.setZGyroOffset(gz_offset);//-2

  CompassGyroOffset = 0;

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.print(F("Enabling DMP...  "));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("Packet size "));
    Serial.println(packetSize);
  }
  else {
    Serial.println(F("DMP Initialization failed "));
    robot.imuUse = false;
    return;
  }
  nextTimeAdjustYaw = millis();
  Serial.println(F("Wait 3 secondes to stabilize the Drift"));
  //delay(3000); // wait 3 sec to help DMP stop drift
  // read the AccelGyro and the CompassHMC5883 to find the initial CompassYaw

  run();
  Serial.print(F("AccelGyro Yaw: "));
  Serial.print(ypr.yaw);

  if (robot.CompassUse) {
    Serial.print(F("  Compass Yaw: "));
    Serial.print(comYaw);
    CompassGyroOffset = distancePI(ypr.yaw, comYaw);
    Serial.print(F("  Diff between compass and accelGyro in Radian and Deg"));
    Serial.print(CompassGyroOffset);
    Serial.print(" / ");
    Serial.println(CompassGyroOffset * 180 / PI);
  }
  else {
    CompassGyroOffset = 0;
  }

  Serial.println(F("--------------------------------- IMU READY ------------------------------"));
}

// weight fusion (w=0..1) of two radiant values (a,b)
float IMUClass::fusionPI(float w, float a, float b)
{
  float c;
  if ((b >= PI / 2) && (a <= -PI / 2)) {
    c = w * a + (1.0 - w) * (b - 2 * PI);
  } else if ((b <= -PI / 2) && (a >= PI / 2)) {
    c = w * (a - 2 * PI) + (1.0 - w) * b;
  } else c = w * a + (1.0 - w) * b;
  return scalePI(c);
}

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2(float newAngle, float newRate, int looptime, float angle) {
  float k = 10;
  float dtc2 = float(looptime) / 1000.0;
  float x1 = (newAngle -   angle) * k * k;
  float y1 = dtc2 * x1 + y1;
  float x2 = y1 + (newAngle -   angle) * 2 * k + newRate;
  angle = dtc2 * x2 + angle;
  return angle;
}

// scale setangle, so that both PI angles have the same sign
float scalePIangles(float setAngle, float currAngle) {
  if ((setAngle >= PI / 2) && (currAngle <= -PI / 2)) return (setAngle - 2 * PI);
  else if ((setAngle <= -PI / 2) && (currAngle >= PI / 2)) return (setAngle + 2 * PI);
  else return setAngle;
}


float IMUClass::distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2 * 180;
  else if (d > 180) d = d - 2 * 180;
  return d;
}

//bb
float IMUClass::scale180(float v)
{
  float d = v;
  while (d < 0) d += 2 * 180;
  while (d >= 2 * 180) d -= 2 * 180;
  if (d >= 180) return (-2 * 180 + d);
  else if (d < -180) return (2 * 180 + d);
  else return d;
}
float IMUClass::rotate360(float x)
{
  x += 360;
  if (x >= 360) x -= 360;
  if (x < 0) x += 360;
  return x;
}

void IMUClass::calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    //robot.resetWatchdog();

    meansensors();

    //watchdogReset();
    Serial.print("Wait until accel 3 val are < 8 : ");
    Serial.print(abs(mean_ax));
    Serial.print(" ");
    Serial.print(abs(mean_ay));
    Serial.print(" ");
    Serial.print(abs(16384 - mean_az));
    Serial.print(" and Gyro 3 val are < 1 : ");
    Serial.print(abs(mean_gx));
    Serial.print(" ");
    Serial.print(abs(mean_gy));
    Serial.print(" ");
    Serial.print(abs(mean_gz));
    Serial.println(" ");


    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
void IMUClass::meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {  //default buffersize=1000
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    robot.ResetWatchdog();
    //watchdogReset();
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
void IMUClass::readImuTemperature() {
  int16_t rawTemp = 0;
  rawTemp = mpu.getTemperature();
  robot.temperatureImu = (rawTemp / 340.) + 36.53;

}

void IMUClass::run() {

  if (!robot.imuUse)  return;
  if (devStatus != 0) return;
  if (state == IMU_CAL_COM) { //don't read the MPU6050 if compass calibration
    if (COMPASS_IS == HMC5883L) {
      calibComUpdate();
    }
    if (COMPASS_IS == QMC5883L) {
      calibComQMC5883Update();
    }

    return;
  }
  //-------------------read the mpu6050 DMP into yprtest array--------------------------------

  mpu.resetFIFO();
  double start_fill_fifo = millis();
  while (fifoCount < packetSize) {  //leave time to the DMP to fill the FIFO
    fifoCount = mpu.getFIFOCount();
    if (millis() > start_fill_fifo + 100) {
      Serial.println("fifo read take more than 100 ms");
      fifoCount = 100;
    }

  }

  while (fifoCount >= packetSize) {  //Immediatly read the fifo and verify that it's not filling during reading
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  if (fifoCount != 0) {
    ypr.yaw = scalePI(gyroAccYaw + CompassGyroOffset) ;
    //Serial.println("//////MPU6050 DMP fill the fifo during the reading IMU value are skip //////////////");
    return;  ///the DMP fill the fifo during the reading , all the value are false but without interrupt it's the only way i have find to make it work ???certainly i am wrong
  }



  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yprtest, &q, &gravity);

  //bber4
  //filter to avoid bad reading, not a low pass one because error are only one on 1000 reading and need fast react on blade safety

  if (((abs(yprtest[1]) - abs(ypr.pitch)) > 0.3490) && useComCalibration && millis() > nextSendPitchRollError) //check only after startup finish , avoid trouble if iMU not calibrate
  {
    nextSendPitchRollError = millis() + 2000;
    Serial.print("Last pitch : ");
    Serial.print(ypr.pitch / PI * 180);
    Serial.print(" Actual pitch : ");
    Serial.println(yprtest[1] / PI * 180);
    Serial.println("pitch change 20 deg in less than 50 ms ????????? value is skip");

  }
  else
  {
    ypr.pitch = yprtest[1];
  }

  if (((abs(yprtest[2]) - abs(ypr.roll)) > 0.3490) && useComCalibration && millis() > nextSendPitchRollError) //check only after startup finish , avoid trouble if iMU not calibrate
  {
    nextSendPitchRollError = millis() + 2000;
    Serial.print("Last roll : ");
    Serial.print(ypr.roll / PI * 180);
    Serial.print(" Actual roll : ");
    Serial.println(yprtest[2] / PI * 180);
    Serial.println("roll change 20 deg in less than 50 ms ????????? value is skip");
  }
  else
  {
    ypr.roll = yprtest[2];
  }

  gyroAccYaw = yprtest[0];  // the Gyro Yaw very accurate but drift

  if (robot.CompassUse) {
    // ------------------put the CompassHMC5883 value into comYaw-------------------------------------
    if (COMPASS_IS == HMC5883L) readHMC5883L();
    if (COMPASS_IS == QMC5883L) readQMC5883L();

    //tilt compensed yaw ????????????
    comTilt.x =  com.x  * cos(ypr.pitch) + com.z * sin(ypr.pitch);
    comTilt.y =  com.x  * sin(ypr.roll)         * sin(ypr.pitch) + com.y * cos(ypr.roll) - com.z * sin(ypr.roll) * cos(ypr.pitch);
    comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);

    comYaw = scalePI( atan2(comTilt.y, comTilt.x)  ); // the compass yaw not accurate but reliable
  }
  

  // / CompassGyroOffset=distancePI( scalePI(ypr.yaw-CompassGyroOffset), comYaw);
  ypr.yaw = scalePI(gyroAccYaw + CompassGyroOffset) ;

}

void IMUClass::readQMC5883L() {

  uint8_t buf[6];

  if (I2CreadFrom(QMC5883L, 0x00, 6, (uint8_t*)buf) != 6) {
    Serial.println("error when read compass");
    robot.addErrorCounter(ERR_IMU_COMM);
    return;
  }
  float x = (int16_t) (((uint16_t)buf[1]) << 8 | buf[0]);
  float y = (int16_t) (((uint16_t)buf[3]) << 8 | buf[2]);
  float z = (int16_t) (((uint16_t)buf[5]) << 8 | buf[4]);

  if (useComCalibration) {
    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x * 0.5;
    y /= comScale.y * 0.5;
    z /= comScale.z * 0.5;
    com.x = x;
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }

}
void IMUClass::readHMC5883L() {
  uint8_t buf[6];
  if (I2CreadFrom(HMC5883L, 0x03, 6, (uint8_t*)buf) != 6) {
    Serial.println("error when read compass");
    robot.addErrorCounter(ERR_IMU_COMM);
    return;
  }
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)
  float x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  float y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  float z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);
  if (useComCalibration) {

    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x * 0.5;
    y /= comScale.y * 0.5;
    z /= comScale.z * 0.5;
    com.x = x;
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }

}


void IMUClass::loadSaveCalib(boolean readflag) {
  int addr = ADDR;
  short magic = MAGIC;
  if (readflag) Serial.println(F("Load Calibration"));
  else Serial.println(F("Save Calibration"));
  eereadwrite(readflag, addr, magic); // magic
  //accelgyro offset
  eereadwrite(readflag, addr, ax_offset);
  eereadwrite(readflag, addr, ay_offset);
  eereadwrite(readflag, addr, az_offset);
  eereadwrite(readflag, addr, gx_offset);
  eereadwrite(readflag, addr, gy_offset);
  eereadwrite(readflag, addr, gz_offset);
  //compass offset
  eereadwrite(readflag, addr, comOfs);
  eereadwrite(readflag, addr, comScale);
  Serial.print(F("Calibration address Start = "));
  Serial.println(ADDR);
  Serial.print(F("Calibration address Stop = "));
  Serial.println(addr);
}


void IMUClass::printPt(point_float_t p) {
  Serial.print(p.x);
  Serial.print(",");
  Serial.print(p.y);
  Serial.print(",");
  Serial.println(p.z);
}
void IMUClass::printCalib() {
  Serial.println(F("-------- IMU CALIBRATION  --------"));
  Serial.print("ACCEL GYRO MPU6050 OFFSET ax: ");
  Serial.print(ax_offset);
  Serial.print(" ay: ");
  Serial.print(ay_offset);
  Serial.print(" az: ");
  Serial.print(az_offset);
  Serial.print(" gx: ");
  Serial.print(gx_offset);
  Serial.print(" gy: ");
  Serial.print(gy_offset);
  Serial.print(" gz: ");
  Serial.println(gz_offset);
  Serial.println("COMPASS OFFSET X.Y.Z AND SCALE X.Y.Z");
  Serial.print(F("comOfs="));
  printPt(comOfs);
  Serial.print(F("comScale="));
  printPt(comScale);
  Serial.println(F("."));
}

void IMUClass::loadCalib() {
  short magic = 0;
  int addr = ADDR;
  //magic=EEPROM.read(addr);
  eeread(addr, magic);
  //value = EEPROM.read(address);
  //EEPROM.write(addr, val);
  if (magic != MAGIC) {
    Serial.println(F("IMU Warning: no calib data"));
    ax_offset = 376;
    ay_offset = -1768;
    az_offset = 1512;
    gx_offset = 91;
    gy_offset = 12;
    gz_offset = -2;
    comOfs.x = comOfs.y = comOfs.z = 0;
    comScale.x = comScale.y = comScale.z = 2;
    useComCalibration = false;
    return;
  }
  calibrationAvail = true;
  useComCalibration = true;
  Serial.println(F("IMU: found calib data"));
  loadSaveCalib(true);
}

void IMUClass::saveCalib() {
  loadSaveCalib(false);
}


void IMUClass::deleteCompassCalib() {
  int addr = ADDR;
  //eewrite(addr, (short)0); // magic
  comOfs.x = comOfs.y = comOfs.z = 0;
  comScale.x = comScale.y = comScale.z = 2;
  Serial.println("Compass calibration deleted");
}
void IMUClass::deleteAccelGyroCalib() {
  int addr = ADDR;
  //eewrite(addr, (short)0); // magic
  ax_offset = ay_offset = az_offset = 0;
  gx_offset = gy_offset = gz_offset = 0;
  mpu.setXAccelOffset(1); //-1929
  mpu.setYAccelOffset(1); //471
  mpu.setZAccelOffset(1); // 1293
  mpu.setXGyroOffset(1);//-1
  mpu.setYGyroOffset(1);//-3
  mpu.setZGyroOffset(1);//-2

  Serial.println("AccelGyro calibration deleted ");
}


// calculate gyro offsets
void IMUClass::calibGyro() {

  Serial.println("Reading sensors for first time... without any offset");
  robot.ResetWatchdog();

  meansensors();

  robot.ResetWatchdog();
  Serial.print("Reading ax: ");
  Serial.print(mean_ax);
  Serial.print(" ay: ");
  Serial.print(mean_ay);
  Serial.print(" az: ");
  Serial.print(mean_az);
  Serial.print(" gx: ");
  Serial.print(mean_gx);
  Serial.print(" gy: ");
  Serial.print(mean_gy);
  Serial.print(" gz: ");
  Serial.println(mean_gz);

  Serial.println("\nCalculating offsets...");
  robot.ResetWatchdog();

  calibration();

  robot.ResetWatchdog();
  meansensors();
  robot.ResetWatchdog();
  Serial.println("FINISHED reading Value with new offset,If all is OK need to be close 0 exept the az close to 16384");
  Serial.print(" New reading ax: ");
  Serial.print(mean_ax);
  Serial.print(" ay: ");
  Serial.print(mean_ay);
  Serial.print(" az: ");
  Serial.print(mean_az);
  Serial.print(" gx: ");
  Serial.print(mean_gx);
  Serial.print(" gy: ");
  Serial.print(mean_gy);
  Serial.print(" gz: ");
  Serial.println(mean_gz);
  robot.ResetWatchdog();
  Serial.print("THE NEW OFFSET ax: ");
  Serial.print(ax_offset);
  Serial.print(" ay: ");
  Serial.print(ay_offset);
  Serial.print(" az: ");
  Serial.print(az_offset);
  Serial.print(" gx: ");
  Serial.print(gx_offset);
  Serial.print(" gy: ");
  Serial.print(gy_offset);
  Serial.print(" gz: ");
  Serial.println(gz_offset);
  robot.ResetWatchdog();
  saveCalib();
}


void IMUClass::calibComStartStop() {
  while ((!robot.RaspberryPIUse) && (Serial.available())) Serial.read(); //use to stop the calib
  if (state == IMU_CAL_COM) {
    // stop
    Serial.println(F("com calib completed"));
    calibrationAvail = true;
    float xrange = comMax.x - comMin.x;
    float yrange = comMax.y - comMin.y;
    float zrange = comMax.z - comMin.z;
    comOfs.x = xrange / 2 + comMin.x;
    comOfs.y = yrange / 2 + comMin.y;
    comOfs.z = zrange / 2 + comMin.z;
    comScale.x = xrange;
    comScale.y = yrange;
    comScale.z = zrange;
    //bber18
    if ((comScale.x == 0) || (comScale.y == 0) || (comScale.z == 0)) { //jussip bug found div by 0 later
      Serial.println(F("*********************ERROR WHEN CALIBRATE : VALUE ARE TOTALY WRONG CHECK IF COMPASS IS NOT PERTURBATE **************"));
      comOfs.x = comOfs.y = comOfs.z = 0;
      comScale.x = comScale.y = comScale.z = 2;
    }
    saveCalib();
    printCalib();
    useComCalibration = true;
    state = IMU_RUN;


  } else {
    // start
    Serial.println(F("com calib..."));
    Serial.println(F("rotate sensor 360 degree around all three axis until NO new data are coming"));
    //watchdogReset();
    foundNewMinMax = false;
    useComCalibration = false;
    state = IMU_CAL_COM;
    comMin.x = comMin.y = comMin.z = 99999;
    comMax.x = comMax.y = comMax.z = -99999;
  }
}

void IMUClass::calibComQMC5883Update() {
  comLast = com;
  delay(20);
  readQMC5883L();
  //watchdogReset();
  if (com.x < comMin.x) {
    comMin.x = com.x;
    Serial.print("NEW min x: ");
    Serial.println(comMin.x);
  }
  if (com.y < comMin.y) {
    comMin.y = com.y;
    Serial.print("NEW min y: ");
    Serial.println(comMin.y);
  }
  if (com.z < comMin.z) {
    comMin.z = com.z;
    Serial.print("NEW min z: ");
    Serial.println(comMin.z);
  }
  if (com.x > comMax.x) {
    comMax.x = com.x;
    Serial.print("NEW max x: ");
    Serial.println(comMax.x);
  }
  if (com.y > comMax.y) {
    comMax.y = com.y;
    Serial.print("NEW max y: ");
    Serial.println(comMax.y);
  }
  if (com.z > comMax.z) {
    comMax.z = com.z;
    Serial.print("NEW max z: ");
    Serial.println(comMax.z);
  }
}
void IMUClass::calibComUpdate() {
  comLast = com;
  delay(20);
  readHMC5883L();

  //watchdogReset();
  boolean newfound = false;
  if ( (abs(com.x - comLast.x) < 10) &&  (abs(com.y - comLast.y) < 10) &&  (abs(com.z - comLast.z) < 10) ) {

    if (com.x < comMin.x) {
      comMin.x = com.x;
      newfound = true;
    }
    if (com.y < comMin.y) {
      comMin.y = com.y;
      newfound = true;
    }
    if (com.z < comMin.z) {
      comMin.z = com.z;
      newfound = true;
    }
    if (com.x > comMax.x) {
      comMax.x = com.x;
      newfound = true;
    }
    if (com.y > comMax.y) {
      comMax.y = com.y;
      newfound = true;
    }
    if (com.z > comMax.z) {
      comMax.z = com.z;
      newfound = true;
    }
    if (newfound) {
      foundNewMinMax = true;
      ////watchdogReset();
      Serial.print("x:");
      Serial.print(comMin.x);
      Serial.print(",");
      Serial.print(comMax.x);
      Serial.print("\t  y:");
      Serial.print(comMin.y);
      Serial.print(",");
      Serial.print(comMax.y);
      Serial.print("\t  z:");
      Serial.print(comMin.z);
      Serial.print(",");
      Serial.print(comMax.z);
      Serial.println("\t");
    }
  }
}
