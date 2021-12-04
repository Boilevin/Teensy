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
#include "config.h"
#include "mower.h"

#include <Arduino.h>
#include "drivers.h"

//#define USE_DEVELOPER_TEST     1      // uncomment for new perimeter signal test (developers)

Mower robot;

Mower::Mower()
{

  name = "Teensymower";
  // ------- wheel motors -----------------------------
  motorAccel; 
  // bb
  motorLeftChange; 
  motorRightChange; 
  motorOdoAccel;          // Time for accel from 0 to 100% in ms
  motorSpeedMaxRpm;         // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
  motorSpeedMaxPwm;        // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  motorPowerMax;           // motor wheel max power (Watt)
  motorSenseRightScale; // normal is 1.536 motor right sense scale (mA=(ADC-zero)/scale)
  motorSenseLeftScale;  // normal is 1.536 motor left sense scale  (mA=(ADC-zero)/scale)
  motorPowerIgnoreTime;  // time to ignore motor power when start to avoid read the peack on motor start (ms)
  motorZeroSettleTime;   // defaut 3000 how long (ms) to wait for motors to settle at zero speed
  motorRollDegMax;        // max. roll Deg
  motorRollDegMin;         // min. roll Deg

  motorForwTimeMax;     // not use max. forward time (ms) / timeout
  motorBiDirSpeedRatio1;  // bidir mow pattern speed ratio 1
  motorBiDirSpeedRatio2; // bidir mow pattern speed ratio 2
  //NEED CHANGES replace . with _
  motorLeftPID_Kp;        // motor wheel PID controller
  motorLeftPID_Ki;
  motorLeftPID_Kd;
  motorRightSwapDir; // inverse right motor direction?
  motorLeftSwapDir;  // inverse left motor direction?

  motorRightOffsetFwd;  // percent offset in PWM use for the 2 wheels motor have the same speed a the same PWM
  motorRightOffsetRev;  // use the 1 ml ODO test to find good value the 2 wheels need to stop at the same time
  motorTickPerSecond; // use to compute the maxodostate duration and computed on the calibration motor

  UseAccelLeft;
  UseBrakeLeft;
  UseAccelRight;
  UseBrakeRight;
  AngleRotate;
  SpeedOdoMin;
  SpeedOdoMax;
  odoLeftRightCorrection; // left-right correction for straight lines used in manual mode
  autoAdjustSlopeSpeed;   // adjust the speed on slope to have same speed on uphill and downhill

  // ------ mower motor -------------------------------
  motorMowAccel;      // motor mower acceleration (warning: do not set too low) 2000 seems to fit best considerating start time and power consumption
  motorMowSpeedMaxPwm; // motor mower max PWM
  motorMowSpeedMinPwm; // motor mower minimum PWM (only for cutter modulation)
  motorMowPowerMax;   // motor mower max power (Watt)

  motorMowSenseScale; // motor mower sense scale (mA=(ADC-zero)/scale)
  //NEED CHANGES replace . with _
  motorMowPID_Kp;     // motor mower RPM PID controller
  motorMowPID_Ki;
  motorMowPID_Kd;
  //  ------ bumper -----------------------------------
  bumperUse; // has bumpers?
  // ------ rain ------------------------------------
  rainUse; // use rain sensor?

  // ------ DHT22Use ------------------------------------
  DHT22Use;        // use DHT22 sensor?
  maxTemperature; // max temp before switch off
  // bber35
  //  ------ RFID ------------------------------------
  rfidUse; // use rfid
  newtagRotAngle1;
  newtagRotAngle2;
  newtagDistance1;
  newtagDistance2;

  // ------ sonar ------------------------------------
  sonarUse; // use ultra sonic sensor? (WARNING: robot will slow down, if enabled but not connected!)
  sonarLeftUse;
  sonarRightUse;
  sonarTriggerBelow;  // ultrasonic sensor trigger distance in cm (0=off)
  sonarToFrontDist;   // ultrasonic sensor distance to front mower in cm
  sonarLikeBumper; // ultrasonic reduce speed vs bumper like

  // ------ perimeter ---------------------------------
  perimeterUse;              // use perimeter?
  perimeterTriggerMinSmag; // perimeter minimum smag to use on big area
  // perimeterOutRollTimeMax;   // free
  // perimeterOutRollTimeMin;    // free
  perimeterOutRevTime;    // free
  perimeterTrackRollTime; // roll time during perimeter tracking
  perimeterTrackRevTime;  // reverse time during perimeter tracking
  DistPeriOutRev;           // reverse distance when reach the perimeter in cm
  DistPeriObstacleRev;      // reverse distance when hit obstacle while tracking in cm
  DistPeriOutForw;          // distance to accell
  DistPeriOutStop;          // slowing distance after crossover the wire
  DistPeriObstacleForw;     // distance while arc circle in peri obstacle avoid
  //NEED CHANGES replace . with _
  perimeterPID_Kp;        // perimeter PID controller
  perimeterPID_Ki;
  perimeterPID_Kd;
  trackingPerimeterTransitionTimeOut;
  trackingErrorTimeOut;
  trakBlockInnerWheel;
  // bb
  MaxSpeedperiPwm;                // speed max in PWM while perimeter tracking
  ActualSpeedPeriPWM; // speed in PWM while perimeter tracking
  // timeToResetSpeedPeri; // if millis() > at this var the speed is set to max value
  RollTimeFor45Deg;      // time while roll in peri obstacle avoid if no Odometry
  circleTimeForObstacle; // time while arc circle in peri obstacle avoid if no Odometry
  DistPeriObstacleAvoid;  // distance while arc circle in peri obstacle avoid
  perimeterMagMaxValue;  // Maximum value return when near the perimeter wire (use for tracking and slowing when near wire
  // perimeter.read2Coil = false;
  areaToGo; // initialise the areatogo to the station area
  // ------  IMU (compass/accel/gyro) ----------------------
  imuUse;               // use IMU?
  CompassUse;           // activate compass?
  stopMotorDuringCalib; // correct direction by compass?
  //NEED CHANGES replace . with _
  imuDirPID_Kp;       // direction PID controller
  imuDirPID_Ki;
  imuDirPID_Kd;
  imuRollPID_Kp; // roll PID controller
  imuRollPID_Ki;
  imuRollPID_Kd;
  // bb
  yawSet1;
  yawOppositeLane1RollRight;
  yawOppositeLane1RollLeft;
  yawSet2;
  yawOppositeLane2RollRight;
  yawOppositeLane2RollLeft;
  yawSet3;
  yawOppositeLane3RollRight;
  yawOppositeLane3RollLeft;
  laneUseNr;
  maxDriftPerSecond;          // limit the stop time if small drift
  maxDurationDmpAutocalib;      // in sec
  delayBetweenTwoDmpAutocalib; // in sec
  yawCiblePos;
  yawCibleNeg;
  DistBetweenLane;
  maxLenghtByLane; // distance to run in bylane before simulate a wire detection
  justChangeLaneDir;
  mowPatternCurr;
  compassRollSpeedCoeff; // speed used when the mower search the compass yaw it's percent of motorSpeedMaxRpm ,Avoid to roll to fast for a correct detection
  // ------ battery -------------------------------------
  batMonitor;          // monitor battery and charge voltage?
  batGoHomeIfBelow;     // drive home voltage (Volt)
  batSwitchOffIfBelow;    // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle;    // switch off battery if idle (minutes)
  batFactor;           // depend of the resistor divisor on board R12 and R13
  batChgFactor;        // depend of the resistor divisor on board R9 and R10
  batFull;              // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax;   // maximum current your charger can devliver
  batFullCurrent;        // current flowing when battery is fully charged
  startChargingIfBelow; // start charging if battery Voltage is below
  chargingTimeout;  // safety timer for charging (ms)  7 hrs
  chgSenseZero;          // charge current sense zero point
  batSenseFactor;       // charge current conversion factor   - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
  chgSense;            // mV/A empfindlichkeit des Ladestromsensors in mV/A (FÃ¼r ACS712 5A = 185)
  chgChange;               // Messwertumkehr von - nach +         1 oder 0
  chgNull;                 // Nullduchgang abziehen (1 oder 2)
  // ------  charging station ---------------------------
  stationRevDist;   // charge station reverse 50 cm
  stationRollAngle; // charge station roll after reverse
  stationForwDist;  // charge station accel distance cm
  stationCheckDist;  // charge station  check distance to be sure voltage is OK cm
  UseBumperDock;  // bumper is pressed when docking or not
  dockingSpeed;     // speed docking is (percent of maxspeed)
  autoResetActive;   // after charging reboot or not

  // ------ odometry ------------------------------------
  odometryUse;                   // use odometry?
  odometryTicksPerRevolution; // encoder ticks per one full resolution
  odometryTicksPerCm;         // encoder ticks per cm
  odometryWheelBaseCm;          // wheel-to-wheel distance (cm)

  // ----- other -----------------------------------------
  buttonUse;               // has digital ON/OFF button?
  RaspberryPIUse;      // a raspberryPi is connected to USBNative port
  mowPatternDurationMax; // in minutes

  // ----- user-defined switch ---------------------------
  userSwitch1; // user-defined switch 1 (default value)
  userSwitch2; // user-defined switch 2 (default value)
  userSwitch3; // user-defined switch 3 (default value)
  // ----- timer -----------------------------------------
  timerUse; // use RTC and timer?

  // ------ mower stats-------------------------------------------
  statsOverride; // if set to true mower stats are overwritten - be careful
  statsMowTimeMinutesTotal;
  statsBatteryChargingCounterTotal;     // 11
  statsBatteryChargingCapacityTotal; // 30000
  // -----------configuration end-------------------------------------
}

void Mower::setup()
{

  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

  // Buzzer.begin();
  Console.begin(CONSOLE_BAUDRATE);
  // I2Creset();
  Wire.begin();
  // Wire1.begin();

  Console.println("SETUP");

  // LED, buzzer, battery

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
  pinMode(pinBumperLeft, INPUT_PULLUP); // it's contact
  pinMode(pinBumperRight, INPUT_PULLUP);

  // rain
  pinMode(pinRain, INPUT);

  // odometry
  // not sure the pullupis necessary with PCB1.3
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);

  pinMode(pinUserOut1, OUTPUT);
  pinMode(pinUserOut2, OUTPUT);
  pinMode(pinUserOut3, OUTPUT);
  // pinMode(pinUserOut4, OUTPUT);

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

void checkMotorFault()
{
  // bb to test without motor board uncomment return
  // return;
  if ((robot.stateCurr == STATE_OFF) || (robot.stateCurr == STATE_ERROR))
    return; // do not generate error if the state if OFF to avoid Buzzer when PI power the DUE via the USB native port
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

void Mower::setActuator(char type, int value)
{

  switch (type)
  {

    // case ACT_MOTOR_MOW: setL298N(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, value); break;// Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

  case ACT_MOTOR_MOW:
    setBTS7960(pinMotorMowDir, pinMotorMowPWM, pinMotorMowEnable, abs(value));
    break; // limit the rotation to only one direction

  case ACT_MOTOR_LEFT:
    setBTS7960(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value);
    break; //   Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

  case ACT_MOTOR_RIGHT:
    setBTS7960(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value);
    break;

  case ACT_USER_OUT1:
    digitalWrite(pinUserOut1, value);
    break;
  case ACT_USER_OUT2:
    digitalWrite(pinUserOut2, value);
    break;
  case ACT_USER_OUT3:
    digitalWrite(pinUserOut3, value);
    break;
    // case ACT_USER_OUT4: digitalWrite(pinUserOut4, value); break;
    /*
      case ACT_USER_SW1: digitalWrite(pinUserSwitch1, value); break;
      case ACT_USER_SW2: digitalWrite(pinUserSwitch2, value); break;
      case ACT_USER_SW3: digitalWrite(pinUserSwitch3, value); break;
  */
  case ACT_CHGRELAY:
    digitalWrite(pinChargeEnable, value);
    break;
  // case ACT_CHGRELAY: digitalWrite(pinChargeEnable, !value); break;
  case ACT_BATTERY_SW:
    digitalWrite(pinBatterySwitch, value);
    break;
  }
}
