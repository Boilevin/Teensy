/*
  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)

*/



#include "mower.h"

#include <Arduino.h>
#include "drivers.h"

//#define USE_DEVELOPER_TEST     1      // uncomment for new perimeter signal test (developers)

Mower robot;


Mower::Mower() {




  name = "MI632";
  // ------- wheel motors -----------------------------
  motorRightSwapDir     = false;    // inverse right motor direction?
  motorLeftSwapDir      = true;    // inverse left motor direction?
  
  //Enable_Screen = false;  // a OLED 0.96 is connected to I2C2

  motorAccel       = 1500;  // motor wheel acceleration - only functional when odometry is not in use (warning: do not set too low)
  //bb
  motorLeftChange = 500;
  motorRightChange = 500;
  motorOdoAccel = 1500; //Time for accel from 0 to 100% in ms
  motorSpeedMaxRpm       = 30;   // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
  motorSpeedMaxPwm    = 108;  // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  motorPowerMax     = 20;    // motor wheel max power (Watt)
  motorSenseRightScale = 1.870; // normal is 1.536 motor right sense scale (mA=(ADC-zero)/scale)
  motorSenseLeftScale = 1.650; // normal is 1.536 motor left sense scale  (mA=(ADC-zero)/scale)
  motorPowerIgnoreTime = 2000; // time to ignore motor power when start to avoid read the peack on motor start (ms)
  motorZeroSettleTime   = 2000 ; // defaut 3000 how long (ms) to wait for motors to settle at zero speed
  motorRollDegMax    = 100;  // max. roll Deg
  motorRollDegMin    = 20; //min. roll Deg

  motorForwTimeMax   = 120000; // max. forward time (ms) / reverse
  motorBiDirSpeedRatio1 = 0.3;   // bidir mow pattern speed ratio 1
  motorBiDirSpeedRatio2 = 0.92;   // bidir mow pattern speed ratio 2
  motorLeftPID.Kp       = 1.0;    // motor wheel PID controller
  motorLeftPID.Ki       = 0.4;
  motorLeftPID.Kd       = 0.0;

  motorRightOffsetFwd = 0;  //percent offset in PWM use for the 2 wheels motor have the same speed a the same PWM
  motorRightOffsetRev = 0;  //use the 1 ml ODO test to find good value the 2 wheels need to stop at the same time
  motorTickPerSecond = 200; // use to compute the maxodostate duration and computed on the calibration motor

  UseAccelLeft = 1;
  UseBrakeLeft = 1;
  UseAccelRight = 1;
  UseBrakeRight = 1;
  AngleRotate = 100;
  SpeedOdoMin = 50;
  SpeedOdoMax = 140;
  odoLeftRightCorrection     = true;       // left-right correction for straight lines used in manual mode
  autoAdjustSlopeSpeed = true;  //adjust the speed on slope to have same speed on uphill and downhill


  // ------ mower motor -------------------------------
  motorMowAccel       = 1000;  // motor mower acceleration (warning: do not set too low) 2000 seems to fit best considerating start time and power consumption
  motorMowSpeedMaxPwm   = 115;    // motor mower max PWM
  motorMowSpeedMinPwm = 100;   // motor mower minimum PWM (only for cutter modulation)
  motorMowPowerMax = 65.0;     // motor mower max power (Watt)
  highGrassSpeedCoeff = 0.7;  //drive speed coeff when detect high grass in by lane mode

  motorMowPID.Kp = 0.005;    // motor mower RPM PID controller
  motorMowPID.Ki = 0.01;
  motorMowPID.Kd = 0.01;
  //  ------ bumper -----------------------------------
  bumperUse         = 1;      // has bumpers?

  // ------ rain ------------------------------------
  rainUse          = 0;      // use rain sensor?

  // ------ screen ------------------------------------
  Enable_Screen    = 0;       //set to 1 if OLED SCREEN is connected to I2c2

  // ------ DHT22Use ------------------------------------
  //DHT22Use          = 0;      // use DHT22 sensor?
  maxTemperature    = 55;     // max temp before switch off
  //bber35
  // ------ RFID ------------------------------------
  rfidUse          = 0;      // use rfid
  newtagRotAngle1 = -90;
  newtagRotAngle2 = 0;
  newtagDistance1 = 10;
  newtagDistance2 = 0;

  // ------ sonar ------------------------------------
  sonarUse                   = 0;          // use ultra sonic sensor? (WARNING: robot will slow down, if enabled but not connected!)
  sonarLeftUse               = 1;
  sonarRightUse              = 1;
  sonarCenterUse             = 0;
  sonarTriggerBelow          = 87;       // ultrasonic sensor trigger distance in cm (0=off)
  sonarToFrontDist           = 30;        // ultrasonic sensor distance to front mower in cm
  sonarLikeBumper            = false;      //ultrasonic reduce speed vs bumper like


  // ------ perimeter ---------------------------------
  perimeterUse       = 1;      // use perimeter?
  perimeterTriggerMinSmag = 50;      // perimeter minimum smag to use on big area
  //perimeterOutRollTimeMax  = 2000;   // free
  //perimeterOutRollTimeMin = 750;    // free
  perimeterOutRevTime   = 2200;   // free
  perimeterTrackRollTime = 1500; //roll time during perimeter tracking
  perimeterTrackRevTime = 2200;  // reverse time during perimeter tracking
  DistPeriOutRev = 40; // reverse distance when reach the perimeter in cm
  DistPeriObstacleRev = 30; // reverse distance when hit obstacle while tracking in cm
  if ( LEFT_MOTOR_DRIVER == 1) {
    DistPeriOutForw = 15; // distance to accell reduce when BL motor is used
  }
  else
  {
    DistPeriOutForw = 60; // distance to accell
  }
  DistPeriOutStop = 15; //slowing distance after crossover the wire
  DistPeriObstacleForw = 25; //distance while arc circle in peri obstacle avoid
  perimeterPID.Kp    = 16.5;  // perimeter PID controller
  perimeterPID.Ki    = 8;
  perimeterPID.Kd    = 0;
  trackingPerimeterTransitionTimeOut = 1500;
  trackingErrorTimeOut = 10000;
  trakBlockInnerWheel = true;
  //bb
  MaxSpeedperiPwm = 85; // speed max in PWM while perimeter tracking
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //speed in PWM while perimeter tracking
  //timeToResetSpeedPeri = 0; // if millis() > at this var the speed is set to max value
  RollTimeFor45Deg = 1000; //time while roll in peri obstacle avoid if no Odometry
  circleTimeForObstacle = 4000; //time while arc circle in peri obstacle avoid if no Odometry
  DistPeriObstacleAvoid = 100; //distance while arc circle in peri obstacle avoid
  perimeterMagLeftMaxValue = 5000; // Maximum value return when near the perimeter wire (use for tracking and slowing when near wire
  //perimeter.read2Coil = false;
  areaToGo = 1;//initialise the areatogo to the station area


  // ------  IMU (compass/accel/gyro) ----------------------
  imuUse            = 1;       // use IMU?
  CompassUse = 0;       // activate compass?
  stopMotorDuringCalib     = 0;       // correct direction by compass?
  imuDirPID.Kp      = 4.4;     // direction PID controller
  imuDirPID.Ki      = 3.3;
  imuDirPID.Kd      = 0.0;
  imuRollPID.Kp     = 0.8;   // roll PID controller
  imuRollPID.Ki     = 21;
  imuRollPID.Kd     = 0;
  //bb
  yawSet1 = 45;
  yawOppositeLane1RollRight = -133;
  yawOppositeLane1RollLeft = -138;
  yawSet2 = 90;
  yawOppositeLane2RollRight = -88;
  yawOppositeLane2RollLeft = -92;
  yawSet3 = 135;
  yawOppositeLane3RollRight = -47;
  yawOppositeLane3RollLeft = -42;
  laneUseNr = 2;
  maxDriftPerSecond = 0.05; //limit the stop time if small drift
  maxDurationDmpAutocalib = 60; //in sec
  delayBetweenTwoDmpAutocalib = 360; //in sec
  yawCiblePos = 90;
  yawCibleNeg = -90;
  DistBetweenLane = 38;
  maxLenghtByLane = 9;  // distance to run in bylane before simulate a wire detection
  justChangeLaneDir = true;
  mowPatternCurr = MOW_LANES;
  compassRollSpeedCoeff = 40; //speed used when the mower search the compass yaw it's percent of motorSpeedMaxRpm ,Avoid to roll to fast for a correct detection


  // ------ model R/C ------------------------------------
  remoteUse         = 0;       // use model remote control (R/C)?
  // ------ battery -------------------------------------
  batMonitor = false;              // monitor battery and charge voltage?
  batGoHomeIfBelow = 24.3;     // drive home voltage (Volt)
  batSwitchOffIfBelow = 23;  // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = 300;      // switch off battery if idle (minutes)
  batFactor       = 1.00;     //not use
  batChgFactor    = 1.00;     //not use
  batFull          = 29.4;     // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax = 2; // maximum current your charger can devliver
  batFullCurrent  = 0.05;      // current flowing when battery is fully charged
  startChargingIfBelow = 25.0; // start charging if battery Voltage is below
  chargingTimeout = 36000000; // safety timer for charging (ms)  10 hrs

  batSenseFactor  = 1.11;         // charge current conversion factor   - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
  chgSense        = 185.0;      // mV/A empfindlichkeit des Ladestromsensors in mV/A (FÃ¼r ACS712 5A = 185)
  chgChange       = 0;          // Messwertumkehr von - nach +         1 oder 0
  chgNull         = 2;          // Nullduchgang abziehen (1 oder 2)
  // ------  charging station ---------------------------
  stationRevDist     = 50;    // charge station reverse 50 cm
  stationRollAngle    = 45;    // charge station roll after reverse
  stationForwDist    = 30;    // charge station accel distance cm
  stationCheckDist   = 2;    // charge station  check distance to be sure voltage is OK cm
  UseBumperDock = true; //bumper is pressed when docking or not
  dockingSpeed   =  60;   //speed docking is (percent of maxspeed)
  checkDockingSpeed   =  5;   //speed check docking RPM
  autoResetActive  = 0;       // after charging reboot or not
  stationHeading  = 0;  //heading of the charging station to use when no compass


  // ------ odometry ------------------------------------
  odometryUse       = 1;       // use odometry?
  odometryTicksPerRevolution = 2070;   // encoder ticks per one full resolution
  odometryTicksPerCm = 29.6;  // encoder ticks per cm
  odometryWheelBaseCm = 43;    // wheel-to-wheel distance (cm)

  // ----- GPS -------------------------------------------
  gpsUse                     = 0;          // use GPS?
  stuckIfGpsSpeedBelow       = 0.2;        // if Gps speed is below given value the mower is stuck
  gpsSpeedIgnoreTime         = 5000;       // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)


  // ----- other -----------------------------------------
  buttonUse         = 1;       // has digital ON/OFF button?
  RaspberryPIUse = false; // a raspberryPi is connected to USBNative port
  mowPatternDurationMax = 120; //in minutes
  useMqtt = false; //select this to exchange data over mqtt protocol for homeassistant.


  // ----- user-defined switch ---------------------------
  userSwitch1       = 0;       // user-defined switch 1 (default value)
  userSwitch2       = 0;       // user-defined switch 2 (default value)
  userSwitch3       = 0;       // user-defined switch 3 (default value)
  // ----- timer -----------------------------------------
  timerUse          = 0;       // use RTC and timer?

  // ------ mower stats-------------------------------------------
  statsOverride = false; // if set to true mower stats are overwritten - be careful
  statsMowTimeMinutesTotal = 300;
  statsBatteryChargingCounterTotal = 10;  //11
  statsBatteryChargingCapacityTotal = 10000;  //30000
  // -----------configuration end-------------------------------------



}








void Mower::setup() {

  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

  // Buzzer.begin();
  Serial.begin(CONSOLE_BAUDRATE);
  // I2Creset();
  Wire.begin();
  //Wire1.begin();

  Serial.println("SETUP");


  // LED, buzzer, battery

  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);

  pinMode(pinChargeEnable, OUTPUT);
  setActuator(ACT_CHGRELAY, 0);

  //mow motor setting *************************************
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, LOW);

  analogWriteFrequency(pinMotorMowPWM, 20000);//default value
  analogWriteFrequency(pinMotorMowDir, 20000);
  if (MOW_MOTOR_DRIVER == 1) {
    analogWriteFrequency(pinMotorMowPWM, PWM_FREQUENCY_BL500W);
    analogWriteFrequency(pinMotorMowDir, PWM_FREQUENCY_BL500W);
  }
  if (MOW_MOTOR_DRIVER == 2) {
    analogWriteFrequency(pinMotorMowPWM, PWM_FREQUENCY_L298N);
    analogWriteFrequency(pinMotorMowDir, PWM_FREQUENCY_L298N);
  }
  if (MOW_MOTOR_DRIVER == 3) {
    analogWriteFrequency(pinMotorMowPWM, PWM_FREQUENCY_BTS7960);
    analogWriteFrequency(pinMotorMowDir, PWM_FREQUENCY_BTS7960);
  }

  //left motor setting********************************************
  pinMode(pinMotorLeftEnable, OUTPUT);
  digitalWrite(pinMotorLeftEnable, LOW);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);

  analogWriteFrequency(pinMotorLeftPWM, 10000);//default value
  analogWriteFrequency(pinMotorLeftDir, 10000);
  if (LEFT_MOTOR_DRIVER == 1) {
    analogWriteFrequency(pinMotorLeftPWM, PWM_FREQUENCY_BL500W);
    analogWriteFrequency(pinMotorLeftDir, PWM_FREQUENCY_BL500W);
  }
  if (LEFT_MOTOR_DRIVER == 2) {
    analogWriteFrequency(pinMotorLeftPWM, PWM_FREQUENCY_L298N);
    analogWriteFrequency(pinMotorLeftDir, PWM_FREQUENCY_L298N);
  }
  if (LEFT_MOTOR_DRIVER == 3) {
    analogWriteFrequency(pinMotorLeftPWM, PWM_FREQUENCY_BTS7960);
    analogWriteFrequency(pinMotorLeftDir, PWM_FREQUENCY_BTS7960);
  }

  //right motor setting *********************************
  pinMode(pinMotorRightEnable, OUTPUT);
  digitalWrite(pinMotorRightEnable, LOW);
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);

  analogWriteFrequency(pinMotorRightPWM, 10000);//default value
  analogWriteFrequency(pinMotorRightDir, 10000);
  if (RIGHT_MOTOR_DRIVER == 1) {
    analogWriteFrequency(pinMotorRightPWM, PWM_FREQUENCY_BL500W);
    analogWriteFrequency(pinMotorRightDir, PWM_FREQUENCY_BL500W);
  }
  if (RIGHT_MOTOR_DRIVER == 2) {
    analogWriteFrequency(pinMotorRightPWM, PWM_FREQUENCY_L298N);
    analogWriteFrequency(pinMotorRightDir, PWM_FREQUENCY_L298N);
  }
  if (RIGHT_MOTOR_DRIVER == 3) {
    analogWriteFrequency(pinMotorRightPWM, PWM_FREQUENCY_BTS7960);
    analogWriteFrequency(pinMotorRightDir, PWM_FREQUENCY_BTS7960);
  }


  // perimeter
  pinMode(pinPerimeterRight, INPUT);
  pinMode(pinPerimeterLeft, INPUT);


  // button
  pinMode(pinButton, INPUT_PULLUP);

  // bumpers
  if (BUMPER_IS_SWITCH) {
    pinMode(pinBumperLeft, INPUT_PULLUP); //it's contact
    pinMode(pinBumperRight, INPUT_PULLUP);
    if (BUMPER_REAR_EXIST) {
      pinMode(pinBumperRearLeft, INPUT_PULLUP); //it's contact
      pinMode(pinBumperRearRight, INPUT_PULLUP);

    }
  }
  else
  {
    pinMode(pinBumperLeft, INPUT); //it's electronics like KY003
    pinMode(pinBumperRight, INPUT);
    if (BUMPER_REAR_EXIST) {
      pinMode(pinBumperRearLeft, INPUT); //it's contact
      pinMode(pinBumperRearRight, INPUT);
    }
  }
  // rain
  pinMode(pinRain, INPUT);


  // odometry
  if (LEFT_MOTOR_DRIVER == 1) {  //for BL motor no need pull_up
    pinMode(pinOdometryLeft, INPUT);
    pinMode(pinOdometryRight, INPUT);
  }
  else
  {
    pinMode(pinOdometryLeft, INPUT_PULLUP);
    pinMode(pinOdometryRight, INPUT_PULLUP);
  }

  pinMode(pinUserOut1, OUTPUT);
  pinMode(pinUserOut2, OUTPUT);
  pinMode(pinUserOut3, OUTPUT);
  //pinMode(pinUserOut4, OUTPUT);

  /*
    // user switches
    pinMode(pinUserSwitch1, OUTPUT);
    digitalWrite(pinUserSwitch1, LOW);
    pinMode(pinUserSwitch2, OUTPUT);
    digitalWrite(pinUserSwitch2, LOW);
    pinMode(pinUserSwitch3, OUTPUT);
    digitalWrite(pinUserSwitch3, LOW);
  */


  Robot::setup();

}

void checkMotorFault() {
  //bb to test without motor board uncomment return
  //return;
  if ((robot.stateCurr == STATE_OFF) || (robot.stateCurr == STATE_ERROR)  ) return;  //do not generate error if the state if OFF to avoid Buzzer when PI power the DUE via the USB native port

}
/*
  int Mower::readSensor(char type) {

  // the azurit readsensor send an integer to robot.cpp so can't use getVoltage from adcman as it's float
  switch (type) {

    // motors------------------------------------------------------------------------------------------------
    case SEN_MOTOR_MOW: return ADCMan.getValue(pinMotorMowSense); break;
    case SEN_MOTOR_RIGHT: checkMotorFault(); return ADCMan.getValue(pinMotorRightSense); break;
    case SEN_MOTOR_LEFT: checkMotorFault(); return ADCMan.getValue(pinMotorLeftSense); break;



    //case SEN_MOTOR_MOW_RPM: break; // not used - rpm is upated via interrupt

    // perimeter----------------------------------------------------------------------------------------------
    case SEN_PERIM_LEFT: return perimeter.getMagnitude(0); break;
    case SEN_PERIM_RIGHT: return perimeter.getMagnitude(1); break;

    // battery------------------------------------------------------------------------------------------------
    case SEN_BAT_VOLTAGE: return ADCMan.getValue(pinBatteryVoltage) ; break;
    case SEN_CHG_VOLTAGE: return ADCMan.getValue(pinChargeVoltage)  ; break;
    case SEN_CHG_CURRENT: return ADCMan.getValue(pinChargeCurrent) ;  break;

    // buttons------------------------------------------------------------------------------------------------
    case SEN_BUTTON: return (digitalRead(pinButton)); break;

    //bumper----------------------------------------------------------------------------------------------------
    case SEN_BUMPER_RIGHT: return (digitalRead(pinBumperRight)); break;
    case SEN_BUMPER_LEFT: return (digitalRead(pinBumperLeft)); break;


    // sonar---------------------------------------------------------------------------------------------------

    case SEN_SONAR_CENTER: return (NewSonarCenter.ping_cm()); break;
    case SEN_SONAR_LEFT: return (NewSonarLeft.ping_cm()); break;

    case SEN_SONAR_RIGHT: return (NewSonarRight.ping_cm()); break;


    // case SEN_LAWN_FRONT: return(measureLawnCapacity(pinLawnFrontSend, pinLawnFrontRecv)); break;
    //case SEN_LAWN_BACK: return(measureLawnCapacity(pinLawnBackSend, pinLawnBackRecv)); break;


    // rtc--------------------------------------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(datetime)) {
        Serial.println("RTC data error!");
        addErrorCounter(ERR_RTC_DATA);
        setNextState(STATE_ERROR, 0);
      }
      break;


    // rain--------------------------------------------------------------------------------------------------------
    case SEN_RAIN: if (digitalRead(pinRain) == LOW) return 1; break;

  }
  return 0;

  }
*/



void Mower::setActuator(char type, int value) {


  switch (type) {

    //case ACT_MOTOR_MOW: setL298N(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, value); break;// Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

    case ACT_MOTOR_MOW:
      if (MOW_MOTOR_DRIVER == 1) setBL500W(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, abs(value));
      if (MOW_MOTOR_DRIVER == 2) setL298N(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, abs(value));
      if (MOW_MOTOR_DRIVER == 3) setBTS7960(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, abs(value));
      break;

    case ACT_MOTOR_LEFT:
      if (LEFT_MOTOR_DRIVER == 1) setBL500W(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value);
      if (LEFT_MOTOR_DRIVER == 2) setL298N(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value);
      if (LEFT_MOTOR_DRIVER == 3) setBTS7960(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value);
      break;

    case ACT_MOTOR_RIGHT:
      if (RIGHT_MOTOR_DRIVER == 1) setBL500W(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value);
      if (RIGHT_MOTOR_DRIVER == 2) setL298N(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value);
      if (RIGHT_MOTOR_DRIVER == 3) setBTS7960(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value);
      break;



    case ACT_USER_OUT1: digitalWrite(pinUserOut1, value); break;
    case ACT_USER_OUT2: digitalWrite(pinUserOut2, value); break;
    case ACT_USER_OUT3: digitalWrite(pinUserOut3, value); break;
    // case ACT_USER_OUT4: digitalWrite(pinUserOut4, value); break;
    /*
      case ACT_USER_SW1: digitalWrite(pinUserSwitch1, value); break;
      case ACT_USER_SW2: digitalWrite(pinUserSwitch2, value); break;
      case ACT_USER_SW3: digitalWrite(pinUserSwitch3, value); break;
    */
    case ACT_CHGRELAY: digitalWrite(pinChargeEnable, value); break;
    //case ACT_CHGRELAY: digitalWrite(pinChargeEnable, !value); break;
    case ACT_BATTERY_SW: digitalWrite(pinBatterySwitch, value); break;
  }

}
