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
  

  name = "Ardumower";
  // ------- wheel motors -----------------------------
  motorAccel       = 1500;  // motor wheel acceleration - only functional when odometry is not in use (warning: do not set too low)
  //bb
  motorLeftChange = 500;
  motorRightChange = 500;
  motorOdoAccel = 1500; //Time for accel from 0 to 100% in ms
  motorSpeedMaxRpm       = 24;   // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
  motorSpeedMaxPwm    = 190;  // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  motorPowerMax     = 23;    // motor wheel max power (Watt)
  motorSenseRightScale = 1.870; // normal is 1.536 motor right sense scale (mA=(ADC-zero)/scale)
  motorSenseLeftScale = 1.650; // normal is 1.536 motor left sense scale  (mA=(ADC-zero)/scale)
  motorPowerIgnoreTime = 2000; // time to ignore motor power when start to avoid read the peack on motor start (ms)
  motorZeroSettleTime   = 2000 ; // defaut 3000 how long (ms) to wait for motors to settle at zero speed
  motorRollDegMax    = 100;  // max. roll Deg
  motorRollDegMin    = 20; //min. roll Deg

  motorForwTimeMax   = 80000; // not use max. forward time (ms) / timeout
  motorBiDirSpeedRatio1 = 0.3;   // bidir mow pattern speed ratio 1
  motorBiDirSpeedRatio2 = 0.92;   // bidir mow pattern speed ratio 2
  motorLeftPID.Kp       = 1.0;    // motor wheel PID controller
  motorLeftPID.Ki       = 0.4;
  motorLeftPID.Kd       = 0.0;
  motorRightSwapDir     = 0;    // inverse right motor direction?
  motorLeftSwapDir      = 0;    // inverse left motor direction?

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
  motorMowSpeedMaxPwm   = 200;    // motor mower max PWM
  motorMowSpeedMinPwm = 100;   // motor mower minimum PWM (only for cutter modulation)
  motorMowPowerMax = 18.0;     // motor mower max power (Watt)

  motorMowSenseScale = 1.536; // motor mower sense scale (mA=(ADC-zero)/scale)
  motorMowPID.Kp = 0.005;    // motor mower RPM PID controller
  motorMowPID.Ki = 0.01;
  motorMowPID.Kd = 0.01;
  //  ------ bumper -----------------------------------
  bumperUse         = 0;      // has bumpers?
  //  ------ drop -----------------------------------
  dropUse          = 0;     // has drops?                                                                                              Dropsensor - Absturzsensor vorhanden ?
  dropcontact      = 0;     //contact 0-openers 1-closers                                                                              Dropsensor - Kontakt 0-Ã–ffner - 1-SchlieÃŸer betÃ¤tigt gegen GND
  // ------ rain ------------------------------------
  rainUse          = 0;      // use rain sensor?

  // ------ DHT22Use ------------------------------------
  DHT22Use          = 0;      // use DHT22 sensor?
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
  perimeterUse       = 0;      // use perimeter?
  perimeterTriggerMinSmag = 200;      // perimeter minimum smag to use on big area
  //perimeterOutRollTimeMax  = 2000;   // free
  //perimeterOutRollTimeMin = 750;    // free
  perimeterOutRevTime   = 2200;   // free
  perimeterTrackRollTime = 1500; //roll time during perimeter tracking
  perimeterTrackRevTime = 2200;  // reverse time during perimeter tracking
  DistPeriOutRev = 40; // reverse distance when reach the perimeter in cm
  DistPeriObstacleRev = 30; // reverse distance when hit obstacle while tracking in cm
  DistPeriOutForw = 60; // distance to accell
  DistPeriOutStop = 15; //slowing distance after crossover the wire
  DistPeriObstacleForw = 25; //distance while arc circle in peri obstacle avoid
  perimeterPID.Kp    = 16.5;  // perimeter PID controller
  perimeterPID.Ki    = 8;
  perimeterPID.Kd    = 0;
  trackingPerimeterTransitionTimeOut = 1500;
  trackingErrorTimeOut = 10000;
  trakBlockInnerWheel = true;
  //bb
  MaxSpeedperiPwm = 180; // speed max in PWM while perimeter tracking
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //speed in PWM while perimeter tracking
  //timeToResetSpeedPeri = 0; // if millis() > at this var the speed is set to max value
  RollTimeFor45Deg = 1000; //time while roll in peri obstacle avoid if no Odometry
  circleTimeForObstacle = 4000; //time while arc circle in peri obstacle avoid if no Odometry
  DistPeriObstacleAvoid = 100; //distance while arc circle in peri obstacle avoid
  perimeterMagMaxValue = 2000; // Maximum value return when near the perimeter wire (use for tracking and slowing when near wire
  //perimeter.read2Coil = false;
  areaToGo = 1;//initialise the areatogo to the station area

  // ------ lawn sensor --------------------------------
  lawnSensorUse     = 0;       // use capacitive Sensor
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
  yawOppositeLane1RollRight = -125;
  yawOppositeLane1RollLeft = -135;
  yawSet2 = 90;
  yawOppositeLane2RollRight = -92;
  yawOppositeLane2RollLeft = -88;
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
  batFactor       = 10.88;     //depend of the resistor divisor on board R12 and R13
  batChgFactor    = 10.89;     //depend of the resistor divisor on board R9 and R10
  batFull          = 29.4;     // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax = 2; // maximum current your charger can devliver
  batFullCurrent  = 0.1;      // current flowing when battery is fully charged
  startChargingIfBelow = 25.0; // start charging if battery Voltage is below
  chargingTimeout = 25200000; // safety timer for charging (ms)  7 hrs
  chgSenseZero    = 511;        // charge current sense zero point
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
  autoResetActive  = 0;       // after charging reboot or not


  // ------ odometry ------------------------------------
  odometryUse       = 1;       // use odometry?
  odometryTicksPerRevolution = 1010;   // encoder ticks per one full resolution
  odometryTicksPerCm = 12.9;  // encoder ticks per cm
  odometryWheelBaseCm = 43;    // wheel-to-wheel distance (cm)



  // ----- other -----------------------------------------
  buttonUse         = 1;       // has digital ON/OFF button?
  RaspberryPIUse = false; // a raspberryPi is connected to USBNative port
  mowPatternDurationMax = 120; //in minutes

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






/*
  NewPing NewSonarLeft(pinSonarLeftTrigger, pinSonarLeftEcho, 110);
  NewPing NewSonarRight(pinSonarRightTrigger, pinSonarRightEcho, 110);
  NewPing NewSonarCenter(pinSonarCenterTrigger, pinSonarCenterEcho, 110);
*/



void Mower::setup() {

  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

  // Buzzer.begin();
  Console.begin(CONSOLE_BAUDRATE);
  // I2Creset();
  Wire.begin();
  //Wire1.begin();

  Console.println("SETUP");


  // LED, buzzer, battery
  // pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);

  pinMode(pinChargeEnable, OUTPUT);
  setActuator(ACT_CHGRELAY, 0);

  analogWriteFrequency(pinMotorLeftPWM, 10000);
  analogWriteFrequency(pinMotorLeftDir, 10000);
  analogWriteFrequency(pinMotorRightPWM, 10000);
  analogWriteFrequency(pinMotorRightDir, 10000);

 
  // left wheel motor
  pinMode(pinMotorLeftEnable, OUTPUT);
  digitalWrite(pinMotorLeftEnable, LOW);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);


  // right wheel motor
  pinMode(pinMotorRightEnable, OUTPUT);
  digitalWrite(pinMotorRightEnable, LOW);
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);


  // mower motor
  // datashett 8 bit resolution ideal freq 585937.5
  
  analogWriteFrequency(pinMotorMowPWM, 20000);
 

  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, LOW);
  


  // perimeter
  pinMode(pinPerimeterRight, INPUT);
  pinMode(pinPerimeterLeft, INPUT);


  // button
  pinMode(pinButton, INPUT_PULLUP);

  // bumpers
  pinMode(pinBumperLeft, INPUT_PULLUP); //it's contact
  pinMode(pinBumperRight, INPUT_PULLUP);

  // rain
  pinMode(pinRain, INPUT);


  // odometry
  //not sure the pullupis necessary with PCB1.3
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);


  // user switches
  pinMode(pinUserSwitch1, OUTPUT);
  pinMode(pinUserSwitch2, OUTPUT);
  
  
  // PWM frequency setup
  // For obstacle detection, motor torque should be detectable - torque can be computed by motor current.
  // To get consistent current values, PWM frequency should be 3.9 Khz
  // http://wiki.ardumower.de/index.php?title=Motor_driver
  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
  // http://www.atmel.com/images/doc2549.pdf

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

    //drop----------------------------------------------------------------------------------------------------
    case SEN_DROP_RIGHT: return (digitalRead(pinDropRight)); break;                                                                                     // Dropsensor - Absturzsensor
    case SEN_DROP_LEFT: return (digitalRead(pinDropLeft)); break;                                                                                       // Dropsensor - Absturzsensor

    // sonar---------------------------------------------------------------------------------------------------

    case SEN_SONAR_CENTER: return (NewSonarCenter.ping_cm()); break;
    case SEN_SONAR_LEFT: return (NewSonarLeft.ping_cm()); break;

    case SEN_SONAR_RIGHT: return (NewSonarRight.ping_cm()); break;


    // case SEN_LAWN_FRONT: return(measureLawnCapacity(pinLawnFrontSend, pinLawnFrontRecv)); break;
    //case SEN_LAWN_BACK: return(measureLawnCapacity(pinLawnBackSend, pinLawnBackRecv)); break;


    // rtc--------------------------------------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(datetime)) {
        Console.println("RTC data error!");
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

    case ACT_MOTOR_MOW: setL298N(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, value); break;// Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

    case ACT_MOTOR_LEFT: setBTS7960(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value); break;//   Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

    case ACT_MOTOR_RIGHT: setBTS7960(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value); break;


    /*
      if (value >= 0) setBTS7960(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value * (1 + (double)motorRightOffsetFwd / 100));
      else setBTS7960(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value * (1 - (double)motorRightOffsetRev / 100));
      break;
    */
    //case ACT_BUZZER: if (value == 0) Buzzer.noTone(); else Buzzer.tone(value); break;
    //case ACT_LED: digitalWrite(pinLED, value); break;
    //case ACT_USER_SW1: digitalWrite(pinUserSwitch1, value); break;
    //case ACT_USER_SW2: digitalWrite(pinUserSwitch2, value); break;
    //case ACT_USER_SW3: digitalWrite(pinUserSwitch3, value); break;

    case ACT_CHGRELAY: digitalWrite(pinChargeEnable, value); break;
    //case ACT_CHGRELAY: digitalWrite(pinChargeEnable, !value); break;
    case ACT_BATTERY_SW: digitalWrite(pinBatterySwitch, value); break;
  }

}
