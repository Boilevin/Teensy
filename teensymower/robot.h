/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri

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

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>

#include "pid.h"
#include "gps.h"
#include "pfod.h"
#include "drivers.h"
#include "RunningMedian.h"
#include "imu.h"
#include "perimeter.h"

//the SD card part
#include <SD.h>
#include <SPI.h>
const int chipSelect = BUILTIN_SDCARD;
/*
  Generic robot class - subclass to implement concrete hardware!

*/



// code version
#define VER "1.497-Teensyber"


// sensors
enum {
  SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
  SEN_PERIM_LEFT_EXTRA,  // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT_EXTRA, // 0..MAX_PERIMETER
  SEN_LAWN_FRONT,
  SEN_LAWN_BACK,
  SEN_BAT_VOLTAGE,       // Volt * 100
  SEN_CHG_CURRENT,       // Ampere * 100
  SEN_CHG_VOLTAGE,       // Volt * 100
  SEN_MOTOR_LEFT,        // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_RIGHT,       // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_MOW,         // 0..MAX_MOW_CURRENT
  SEN_BUMPER_LEFT,       // LOW = pressed
  SEN_BUMPER_RIGHT,      // LOW = pressed
  SEN_DROP_LEFT,       // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
  SEN_DROP_RIGHT,      // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor

  SEN_SONAR_CENTER,      // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_LEFT,        // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_RIGHT,       // 0..SONAR_TRIGGER_DISTANCE
  SEN_BUTTON,            // LOW = pressed
  SEN_IMU,
  SEN_MOTOR_MOW_RPM,
  SEN_RTC,
  SEN_RAIN,
};

// actuators
enum {
  ACT_MOTOR_LEFT,
  ACT_MOTOR_RIGHT,
  ACT_MOTOR_MOW,
  ACT_BUZZER,
  ACT_LED,
  ACT_USER_SW1,
  ACT_USER_SW2,
  ACT_USER_SW3,
  ACT_RTC,
  ACT_CHGRELAY,
  ACT_BATTERY_SW,
  //bber80
  ACT_GREEN_LED,
  ACT_RED_LED,
  //bber60
  ACT_USER_OUT1,
  ACT_USER_OUT2,
  ACT_USER_OUT3,
  ACT_USER_OUT4,
};

// error types
enum {
  ERR_MOTOR_LEFT,
  ERR_MOTOR_RIGHT,
  ERR_MOTOR_MOW,
  ERR_MOW_SENSE,
  ERR_IMU_COMM,
  ERR_IMU_TILT,
  ERR_RTC_COMM,
  ERR_RTC_DATA,
  ERR_PERIMETER_TIMEOUT,
  ERR_TRACKING,
  ERR_ODOMETRY_LEFT,
  ERR_ODOMETRY_RIGHT,
  ERR_BATTERY,
  ERR_CHARGER,
  ERR_GPS_COMM,
  ERR_GPS_DATA,
  ERR_ADC_CALIB,
  ERR_IMU_CALIB,
  ERR_EEPROM_DATA,
  ERR_STUCK,
  // <---- add new error types here (NOTE: increase MAGIC to avoid corrupt EEPROM error data!)
  ERR_ENUM_COUNT,
};

// finate state machine states NEVER INSERT ALWAYS ADD TO END
enum {
  STATE_OFF,          // off
  STATE_REMOTE,       // model remote control (R/C)
  STATE_FORWARD,      // drive forward
  STATE_ROLL,         // drive roll right/left
  STATE_REVERSE,      // drive reverse
  STATE_CIRCLE,       // drive circle
  STATE_ERROR,        // error
  STATE_PERI_FIND,    // perimeter find
  STATE_PERI_TRACK,   // perimeter track
  STATE_PERI_ROLL,    // perimeter roll
  STATE_PERI_REV,     // perimeter reverse
  STATE_STATION,       // in station
  STATE_STATION_CHARGING,       // in station charging
  STATE_STATION_CHECK, //checks if station is present
  STATE_STATION_REV,   // charge reverse

  STATE_STATION_ROLL,  // charge roll
  STATE_STATION_FORW,  // charge forward
  STATE_MANUAL,       // manual navigation
  STATE_ROLL_WAIT,    // drive roll right/left
  STATE_PERI_OUT_FORW, // outside perimeter forward driving without checkPerimeterBoundary()
  STATE_PERI_OUT_REV,   // outside perimeter reverse driving without checkPerimeterBoundary()
  STATE_PERI_OUT_ROLL,   // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_PERI_OBSTACLE_REV,     // perimeter reverse when hit obstacle
  STATE_PERI_OBSTACLE_ROLL,     // perimeter roll when hit obstacle
  STATE_PERI_OBSTACLE_FORW,     // perimeter forward after roll obstacle
  STATE_PERI_OBSTACLE_AVOID,     // perimeter circle to avoid obstacle
  STATE_NEXT_LANE_FORW, // use after roll to reach the next lane
  STATE_PERI_OUT_STOP, // use to stop slowly on perimeter before reverse
  STATE_PERI_OUT_LANE_ROLL1,// roll always to 135 deg with brake
  STATE_PERI_OUT_LANE_ROLL2,// roll one wheel to put in straight line with brake

  STATE_PERI_OUT_ROLL_TOINSIDE,   // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_WAIT_AND_REPEAT, // use to repeat a state until something change for example find again perimeter by rolling
  STATE_FORWARD_ODO,      // drive forward odometry control straigh line and not speed in rpm
  STATE_TEST_COMPASS,    // drive roll right/left FOR A SPECIFIQUE ANGLE
  STATE_PERI_OUT_ROLL_TOTRACK,   // outside perimeter rolling driving without checkPerimeterBoundary()
  STATE_PERI_STOP_TOTRACK, // brake when reach the wire before roll to start tracking
  STATE_AUTO_CALIBRATE,  //wait until the drift is stopped and adapt the gyro and compass
  STATE_ROLL_TO_FIND_YAW, //try to find the compass yaw = yawheading
  STATE_TEST_MOTOR, //use to test the odometry motor
  STATE_STOP_TO_FIND_YAW, // brake before roll to find yaw
  STATE_STOP_ON_BUMPER,   //stop immediatly on bumper
  STATE_STOP_CALIBRATE,   //don't move during 15 sec for DMP autocalibration.
  STATE_SONAR_TRIG,      // when sonar detect obstacle brake gently before reverse
  STATE_STOP_BEFORE_SPIRALE,     //brake gently before start the spirale
  STATE_MOW_SPIRALE,      //mowing a CIRCLE ARC of increaseing diameter to make the spirale

  STATE_ROTATE_RIGHT_360,  // simple mowing rotation at the beginning of the spirale
  STATE_NEXT_SPIRE,  //go to next CIRCLE ARC in SPIRALE MODE
  STATE_ESCAPE_LANE,  //grass is high need to reduce the lenght of the lane
  STATE_PERI_STOP_TOROLL,  //use when timer start the mowing or when tracking and find RFID need to stop before roll
  STATE_ROLL_TONEXTTAG,  //use when tracking and find RFID tag the mower roll to new heading
  STATE_PERI_STOP_TO_NEWAREA,  //use when tracking and find RFID need to stop before roll to leave the area
  STATE_ROLL1_TO_NEWAREA,  //use when tracking and find RFID need to roll to leave the area
  STATE_DRIVE1_TO_NEWAREA,  //use when tracking and find RFID need to drive to leave the area
  STATE_ROLL2_TO_NEWAREA,  //use when tracking and find RFID need to roll to leave the area
  STATE_DRIVE2_TO_NEWAREA,  //use when tracking and find RFID need to drive to leave the area

  STATE_WAIT_FOR_SIG2,  //use when the area 2 wait until sender send the signal
  STATE_STOP_TO_NEWAREA,  //use to stop the mower in straight line after long distance moving with ODO and IMU
  STATE_PERI_OUT_STOP_ROLL_TOTRACK, // after the mower rool to track we need to stop the right motor because it's reverse and the track is forward
  STATE_PERI_STOP_TO_FAST_START,  // after the mower find a tag for find a new start entry point
  STATE_CALIB_MOTOR_SPEED,  // we need to know how may ticks the motor can do in 1 ms to compute the maxododuration
  STATE_ACCEL_FRWRD, // when start from calib or off need to accel before motorodo
  STATE_ENDLANE_STOP, //when mower is at the end of the lane avoid to reverse before roll
  STATE_START_FROM_STATION, //when mower is station and a start command is receive need to start again IMU ,perimeter signal etc....
  STATE_BUMPER_REV,   // rev when bumper is hit
  STATE_WAIT_COVER,  // on start wait until user close the security cover

};

// status mode
enum { WAIT, NORMAL_MOWING, SPIRALE_MOWING, BACK_TO_STATION, TRACK_TO_START, MANUAL, REMOTE, IN_ERROR, IN_STATION, TESTING, WAITSIG2, WIRE_MOWING };

// roll types
enum { LEFT, RIGHT };

// rfid todo list
enum { NOTHING, RTS, FAST_START, NEW_AREA, SPEED, AREA1, AREA2, AREA3 };

// mow patterns
enum { MOW_RANDOM, MOW_LANES, MOW_WIRE, MOW_ZIGZAG };

//bb
// console mode
enum { CONSOLE_SENSOR_COUNTERS, CONSOLE_SENSOR_VALUES, CONSOLE_PERIMETER, CONSOLE_OFF , CONSOLE_TRACKING };



#define MAX_TIMERS 5

#define BATTERY_SW_OFF -1

class Robot
{
  public:
    String name;
    boolean sdCardReady;
    int totalLineOnFile;

    char historyFilenameChar[15]; // need a char array for sd card open
    boolean developerActive;
    boolean ConsoleToPfod;
    boolean sdcardToPfod; // use to stop mqtt message went list a file in pfod raw data mode

    // --------- state machine --------------------------
    byte stateCurr;
    byte stateLast;
    byte stateNext;

    byte statusCurr;
    byte rfidToDoCurr;
    //byte statusLast;
    //byte statusNext;

    unsigned long stateTime;
    const char* stateName();
    const char* statusName();
    char* rfidToDoName();

    //const String area1_ip="10.0.0.150";
    //const String area2_ip="10.0.0.151";
    //const String area3_ip="10.0.0.158";

    // char* area1_ip = "10.0.0.151";
    // char* area2_ip = "10.0.0.150";
    // char* area3_ip = "10.0.0.158";

    boolean Enable_Screen ;

    unsigned long stateStartTime;
    unsigned long stateEndTime;
    int idleTimeSec;
    // --------- timer ----------------------------------
    ttimer_t timer[MAX_TIMERS];
    datetime_t datetime;
    boolean timerUse          ;       // use timer?
    unsigned long nextTimeTimer ;
    byte ActualRunningTimer;
    // ----- bluetooth -------------------------------------
    boolean freeboolean;        //use to keep the eeprom integrity


    // -------- mow pattern -----------------------------
    byte mowPatternCurr;
    const char *mowPatternName();
    // -------- gps state -------------------------------
    GPS gps;
    boolean gpsUse            ;       // use GPS?

    float gpsLat;
    float gpsLon;
    float gpsX ;   // X position (m)
    float gpsY ;   // Y position (m)
    unsigned long nextTimeGPS ;
    unsigned long nextTimeCheckIfStuck ;
    int gpsSpeedIgnoreTime ; // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)

    float stuckIfGpsSpeedBelow ;
    int robotIsStuckCounter ;

    // -------- odometry state --------------------------
    boolean odometryUse       ;       // use odometry?
    boolean twoWayOdometrySensorUse;  // free for something else
    int odometryTicksPerRevolution ;   // encoder ticks per one full resolution
    float odometryTicksPerCm ;  // encoder ticks per cm
    float odometryWheelBaseCm ;    // wheel-to-wheel distance (cm)
    int odometryLeft ;   // left wheel counter
    int odometryRight ;  // right wheel counter
    boolean odometryLeftLastState;
    boolean odometryLeftLastState2;
    boolean odometryRightLastState;
    boolean odometryRightLastState2;
    float odometryTheta; // theta angle (radiant)
    float odometryX ;   // X map position (cm)
    float odometryY ;   // Y map position (cm)
    float motorLeftRpmCurr ; // left wheel rpm
    float motorRightRpmCurr ; // right wheel rpm
    unsigned long lastMotorRpmTime ;
    unsigned long nextTimeOdometry ;
    unsigned long nextTimeOdometryInfo ;
    //bb
    int stateEndOdometryRight;  // use to mesure the distance when rev roll etc ...
    int stateEndOdometryLeft;
    int stateStartOdometryLeft;  // use to calculate the accel

    int stateStartOdometryRight;
    int PeriOdoIslandDiff; //use to check if island while tracking
    //float straightLineTheta; //angle read by odometry during the last lane to verify the IMU drift
    int DistPeriOutRev; // Distance in CM when reach perimeter
    int DistPeriObstacleRev; // Distance in CM when prei rev obstacle
    int DistPeriOutForw;//Distance in CM after roll to accel
    int currDistToDrive;//IMU staright line and distance control

    // -------- RC remote control state -----------------
    boolean remoteUse      ;       // use model remote control (R/C)?
    int remoteSteer ;  // range -100..100
    int remoteSpeed ;  // range -100..100
    int remoteMow ;    // range 0..100
    int remoteSwitch ; // range 0..100
    unsigned long remoteSteerLastTime ;
    unsigned long remoteSpeedLastTime ;
    unsigned long remoteMowLastTime ;
    unsigned long remoteSwitchLastTime ;
    boolean remoteSteerLastState ;
    boolean remoteSpeedLastState ;
    boolean remoteMowLastState ;
    boolean remoteSwitchLastState ;
    unsigned long nextTimeRTC ;
    //bber35
    String rfidTagFind;
    boolean rfidUse;
    byte rfidListElementCount = 0;
    struct rfid_list {
      unsigned long TagNr;
      byte TagMowerStatus;
      byte TagToDo;
      int TagSpeed;
      float TagAngle1;
      int TagDist1;
      float TagAngle2;
      int TagDist2;
      struct rfid_list *next;
    };

    struct rfid_list *head = NULL;
    struct rfid_list *ptr = NULL;


    //bber400 gps
    byte gpsPointElementCount = 0;
    struct gpsPoint {
      byte gpsAreaNr;
      byte gpsAreaPointNr;
      float Pt_lon;
      float Pt_lat;
      struct gpsPoint *next;
    };
    struct gpsPoint *headGpsPoint = NULL;
    struct gpsPoint *ptrGpsPoint = NULL;


    // --------- wheel motor state ----------------------------
    // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
    //bb
    int motorLeftChange;
    int motorRightChange;
    int motorAccel  ;  // motor wheel acceleration (warning: do not set too high)
    int motorSpeedMaxRpm   ;   // motor wheel max RPM
    int motorSpeedMaxPwm  ;  // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
    int motorInitialSpeedMaxPwm ;  // motor Initial wheel max Pwm
    float motorPowerMax   ;    // motor wheel max power (Watt)
    PID motorLeftPID;              // motor left wheel PID controller
    PID motorRightPID;              // motor right wheel PID controller
    float motorSenseRightScale ; // motor right sense scale (mA=(ADC-zero)/scale)
    float motorSenseLeftScale ; // motor left sense scale  (mA=(ADC-zero)/scale)
    /*
        int motorRollTimeMax ;  // max. roll time (ms)
        int motorRollTimeMin  ; // min. roll time (ms)
        int motorReverseTime ;  // max. reverse time (ms)
    */

    int motorRollDegMax;
    int motorRollDegMin;
    //int DistPeriOutRev;
    unsigned long stateOffAfter; // using BL Motor the manual PID control can generate a small vibration at end of the movement ,so jump to OFF
    unsigned long motorForwTimeMax; // max. forward time (ms) / timeout
    float motorBiDirSpeedRatio1 ;   // bidir mow pattern speed ratio 1
    float motorBiDirSpeedRatio2 ;   // bidir mow pattern speed ratio 2
    boolean motorRightSwapDir     ;    // inverse right motor direction?
    boolean motorLeftSwapDir      ;    // inverse left motor direction?
    int motorLeftSpeedRpmSet ; // set speed
    int motorRightSpeedRpmSet ;
    //bb
    unsigned long motorOdoAccel;

    int motorLeftPWMCurr ; // current speed
    int motorRightPWMCurr ;
    //float motorLeftPWMCurr ; // current speed
    //float motorRightPWMCurr ;
    int motorRightSenseADC ;
    int motorLeftSenseADC ;
    float motorLeftSenseCurrent ;
    float motorRightSenseCurrent ;
    float motorLeftPower ;      // motor power (range 0..MAX_MOTOR_POWER)
    float motorRightPower ;
    int motorPowerIgnoreTime;
    int motorZeroSettleTime;     // how long (ms) to wait for motor to settle at zero speed
    int motorLeftSenseCounter ;  // motor current counter
    int motorRightSenseCounter ;
    unsigned long nextTimeMotorSense ;
    unsigned long lastSetMotorSpeedTime;
    unsigned long motorLeftZeroTimeout;
    unsigned long motorRightZeroTimeout;
    boolean rotateLeft;
    unsigned long nextTimeRotationChange;
    unsigned long nextTimeSendTagToPi;
    unsigned long nextTimeMotorControl;
    unsigned long nextTimeMotorImuControl ;

    //bb
    int motorRightOffsetFwd;
    int motorRightOffsetRev;
    int motorTickPerSecond;
    //bber400
    float motorLeftRpmCoeff; // coeff use to have a stable RPM on slope or when battery is full or empty
    float motorRightRpmCoeff; // coeff use to have a stable RPM on slope or when battery is full or empty
    
    unsigned long nextTimeMotorOdoControl ;
    unsigned long nextTimePidCompute;
    unsigned long accelDurationRight;
    unsigned long accelDurationLeft;
    unsigned long movingTimeLeft;
    unsigned long movingTimeRight;
    unsigned long nextTimeMotorPerimeterControl;
    unsigned long nextTimeMotorMowControl;
    int lastMowSpeedPWM;
    byte timeToAddMowMedian;
    unsigned long lastSetMotorMowSpeedTime;
    unsigned long nextTimeCheckCurrent;
    unsigned long nextTimeCheckperimeterSpeedCoeff;
    unsigned long lastTimeMotorMowStuck;
    int leftSpeed;
    int rightSpeed;
    int PwmLeftSpeed;
    int PwmRightSpeed;
    int SpeedOdoMaxLeft;
    int SpeedOdoMaxRight;
    int SpeedOdoMin; //Minimum PWM speed to have the wheel turn
    int SpeedOdoMax; //Maximum PWM speed to small mouvment

    float OdoStartBrakeLeft;
    float OdoStartBrakeRight;
    float MaxOdoStateDuration;
    float MaxStateDuration;
    float PrevStateOdoDepassLeft;
    float PrevStateOdoDepassRight;
    boolean UseAccelRight;
    boolean UseBrakeRight;
    boolean UseAccelLeft;
    boolean UseBrakeLeft;
    boolean moveRightFinish;
    boolean moveLeftFinish;
    boolean useMotorDriveBrake;   //for ZS-X11H BL motor driver it's possible to use the brake option for slope management

    boolean odoLeftRightCorrection;
    boolean autoAdjustSlopeSpeed;
    int AngleRotate;
    int newtagRotAngle1;
    int newtagRotAngle2;
    int newtagDistance1;
    int newtagDistance2;

    //unsigned long stateMaxiTime;  // maybe safety if the odometry is not reach in this time error

    int angleCorresp;
    int accelDuration;
    //bber17
    byte RollToInsideQty; //use to stop if roll non stop
    byte rollToTrackQty; //use to stop if roll non stop when lost perimeter wire

    // -------- mower motor state -----------------------
    boolean powerboard_I2c_line_Ok = true;
    float Mow1_Power = 0;  //Use for mower with 3 mow motor and Ina226 sensor
    float Mow2_Power = 0;
    float Mow3_Power = 0;

    int motorMowRpmCounter ;  // mower motor speed state
    boolean motorMowRpmLastState ;
    boolean motorMowEnable ;  // motor can be temporary disabled if stucked etc. with this
    boolean motorMowForceOff ; // user switch for mower motor on/off has highest priority
    // boolean ignoreRfidTag ; // use to stay on wire when mow perimeter
    boolean highGrassDetect;  //detect that the mow motor is on high load so high grass
    float triggerMotorMowHightGrass ;     // motor mower percent of power vs power Max  trigger to start spirale or half lane
    // boolean motorMowEnableOverride ; // user switch for mower motor on/off has highest priority if true the motor is stop
    // mower motor sppeed; range 0..motorMowSpeedMaxPwm
    float motorMowAccel       ;  // motor mower acceleration (warning: do not set too high)
    int motorMowSpeedMaxPwm ;    // motor mower max PWM
    int motorMowSpeedMinPwm ;    // motor mower min PWM (only for speed modulation)
    float motorMowPowerMax ;     // motor mower max power (Watt)

    //bb 8
    byte spiraleNbTurn;  //count the number of revolution of the spirale (10 revolutions for example before stop)
    //byte halfLaneNb; //count the number of lane same as spirale (10  for example before stop)


    float highGrassSpeedCoeff ;   // speed ratio when high grass is detected
    PID motorMowPID ;    // motor mower RPM PID controller
    int motorMowSpeedPWMSet;
    int motorMowPWMCurr ;         // current speed
    int motorMowPwmCoeff ;      // current coeff
    int motorMowSenseADC ;
    float motorMowSenseCurrent ;
    float motorMowPower ;       // motor power (range 0..MAX_MOW_POWER)
    int motorMowSenseCounter ;
    int motorMowSenseErrorCounter ;

    unsigned long lastMotorMowRpmTime;


    // --------- bumper state ---------------------------
    // bumper state (true = pressed)
    boolean bumperUse       ;      // has bumpers?
    boolean tiltUse       ;      // has tilt sensor?
    boolean tilt;
    boolean coverIsClosed;

    int bumper_rev_distance ;
    boolean bumperLeft ;
    int bumperLeftCounter ;
    boolean bumperRight ;
    int bumperRightCounter ;
    boolean bumperRearRight ;
    int bumperRearRightCounter ;
    boolean bumperRearLeft ;
    int bumperRearLeftCounter ;



    unsigned long nextTimeBumper ;
    // --------- drop state ---------------------------
    // bumper state (true = pressed)                                                                                                  // Dropsensor - Absturzsensor vorhanden ?
    //boolean dropUse       ;      // has drops?                                                                                           // Dropsensor - Absturzsensor ZÃ¤hler links
    //int dropLeftCounter ;                                                                                                             // Dropsensor - Absturzsensor
    //boolean dropLeft ;                                                                                                                // Dropsensor - Absturzsensor links betÃ¤tigt ?
    //int dropRightCounter ;                                                                                                            // Dropsensor - Absturzsensor
    //boolean dropRight ;                                                                                                               // Dropsensor - Absturzsensor rechts betÃ¤tigt ?
    //unsigned long nextTimeDrop ;                                                                                                      // Dropsensor - Absturzsensor
    //boolean dropcontact ; // contact 0-openers 1-closers                                                                                 // Dropsensor Kontakt 0 fÃ¼r Ã–ffner - 1 SchlieÃŸer
    // ------- IMU state --------------------------------
    IMUClass imu;
    boolean imuUse            ;       // use IMU?
    boolean CompassUse;       // Deactivate the compass and use only gyro accel from IMU
    boolean stopMotorDuringCalib     ;       // Stop mow motor during auto calibration
    PID imuDirPID  ;    // direction PID controller
    PID imuRollPID ;    // roll PID controller
    float imuDriveHeading ;       // drive heading (IMU)
    float periFindDriveHeading;   // drive heading when search for other rfid tag or go to station
    float remoteDriveHeading;   // drive heading  when rfid tag to go to area2
    float newtagRotAngle1Radian;


    float imuRollHeading ;      // roll heading  (IMU)
    byte  imuRollDir;
    //point_float_t accMin;
    //point_float_t accMax;
    unsigned long nextTimeIMU ; //read IMU data
    unsigned long nextTimeCheckTilt; // check if
    unsigned long nextTimeAddYawMedian;
    //unsigned long nextTimeImuUse;
    unsigned long nextTimeToDmpAutoCalibration;
    unsigned long endTimeCalibration;
    boolean needDmpAutoCalibration;
    //float lastLaneYawMedian; //use to know the last lane direction in bylaneodo mowing
    float YawActualDeg;
    byte compassRollSpeedCoeff;

    RunningMedian compassYawMedian = RunningMedian(60);
    RunningMedian accelGyroYawMedian = RunningMedian(60);
    RunningMedian motorMowPowerMedian = RunningMedian(30);
    // RunningMedian motorSpeedRpmMedian = RunningMedian(35);
    RunningMedian perimeterMedian = RunningMedian(67); //perimeter is read each 20 ms so 1 second



    //bb 5


    float yawToFind;
    float findedYaw;
    float yawSet1;
    float yawOppositeLane1RollRight;
    float yawOppositeLane1RollLeft;
    float yawSet2;
    float yawOppositeLane2RollRight;
    float yawOppositeLane2RollLeft;
    float yawSet3;
    float yawOppositeLane3RollRight;
    float yawOppositeLane3RollLeft;
    float yawCiblePos;
    float yawCibleNeg;
    byte laneUseNr;
    byte DistBetweenLane;
    byte maxLenghtByLane;
    byte actualLenghtByLane;
    int correctLeft;
    int correctRight;
    float YawMedianDeg;
    boolean justChangeLaneDir;  //
    byte actualRollDirToCalibrate;
    float prevYawCalcOdo;
    unsigned long nextTimeImuLoop ;
    unsigned long nextTimeGpsRead ;

    int delayBetweenTwoDmpAutocalib;
    int maxDurationDmpAutocalib;
    float maxDriftPerSecond;

    // ------- perimeter state --------------------------

    //bb
    PerimeterClass perimeter;

    // Perimeter perimeter;
    boolean senderIsRunning;     //check sender is on
    boolean perimeterUse       ;      // use perimeter?
    boolean read2Coil;                // use left and right coil
    // swap coil polarity?
    boolean swapCoilPolarityLeft;     //invert coil signal + -
    boolean swapCoilPolarityRight;
    //int perimeterOutRollTimeMax ;  //free but conserve for eeprom recovery
    //int perimeterOutRollTimeMin ;   //free but conserve for eeprom recovery
    int perimeterOutRevTime  ;
    int perimeterTrackRollTime ; // perimeter tracking roll time (ms)
    int perimeterTrackRevTime ; // perimeter tracking reverse time (ms)
    PID perimeterPID ;             // perimeter PID controller
    int perimeterMagLeft ;             // perimeter magnitude
    int perimeterMagRight ;             // perimeter magnitude
    byte areaInMowing;              //it's the area in mowing nr
    boolean perimeterInsideLeft ;      // is inside perimeter?
    boolean perimeterInsideRight ;      // is inside perimeter?
    unsigned long perimeterTriggerTime; // time to trigger perimeter transition (timeout)
    unsigned long perimeterLeftTriggerTime; // time to trigger left perimeter transition (timeout)
    unsigned long perimeterRightTriggerTime; // time to trigger right perimeter transition (timeout)
    int perimeterTriggerMinSmag;   // perimeter trigger minimum smag use on center of big area ,the Smag can be 200 and transition can occur
    unsigned long perimeterLastTransitionTime;
    int perimeterCounter ;         // counts perimeter transitions
    unsigned long nextTimePerimeter ;
    unsigned long trackingPerimeterTransitionTimeOut;
    unsigned long trackingErrorTimeOut;
    boolean trakBlockInnerWheel;
    float perimeterNoise; //compute each 2 seconde the diff between max and min Mag value help on position of motor wire and ferrite in the chassis
    //add BB
    int leftSpeedperi;
    int rightSpeedperi;
    int lastLeftSpeedperi;
    int lastRightSpeedperi;
    float perimeterSpeedCoeff; // coeff to reduce speed when near perimeter wire
    int uu;
    int vv;
    unsigned long lastTimeForgetWire;
    unsigned long NextTimeNormalSpeed;
    //unsigned long timeToResetSpeedPeri;
    int LastPerimeterMag;
    double CiblePeriValue;
    int MaxSpeedperiPwm;
    int ActualSpeedPeriPWM;
    unsigned long RollTimeFor45Deg;
    unsigned long circleTimeForObstacle;
    int DistPeriObstacleAvoid;
    int DistPeriObstacleForw;
    int DistPeriOutStop;
    int perimeterMagMaxValue;
    int Tempovar;
    boolean lastPerimeterTrackInside; // was inside or outside
    float PeriCoeffAccel;
    float R;
    int smoothPeriMag;
    unsigned long nextTimeReadSmoothPeriMag ; //use when wait for sig2
    boolean reduceSpeedNearPerimeter; //Activate the speed reduction near perimeter
    //End add bb


    // --------- rain -----------------------------------
    boolean rain;
    boolean rainUse;
    int rainCounter;
    unsigned long nextTimeRain ;
    // --------- sonar ----------------------------------
    // ultra sonic sensor distance-to-obstacle (cm)
    boolean sonarUse          ;      // use ultra sonic sensor?
    boolean sonarLikeBumper   ;      // sonar behaviour is the same as bumper
    boolean sonarTest; 
    boolean sonarLeftUse;
    boolean sonarRightUse;
    boolean sonarCenterUse;
    int sonarTriggerBelow ;    // start to reverse
    int sonarSlowBelow ;      // start to slow not use but stay here to keep the compatibily with 1.08
    int sonarDistCenter ;
    int sonarDistRight ;
    int sonarDistLeft ;
    //unsigned int sonarDistCounter ;
    //unsigned int tempSonarDistCounter ;
    unsigned long sonarObstacleTimeout ;
    //unsigned long nextTimeSonar ;
    unsigned long nextTimeCheckSonar ;
    byte distToObstacle; //min distance to obstacle in CM of the 3 sonars
    byte sonarToFrontDist;
    float sonarSpeedCoeff; // coeff to reduce speed when sonar detect something
    float sonarSpeedSettingCoeff;     // setting saved value coeff to reduce speed when sonar detect something
    
    // --------- pfodApp ----------------------------------
    RemoteControl rc; // pfodApp
    unsigned long nextTimePfodLoop ;
    unsigned long next_time_refresh_mqtt;
    // ----- other -----------------------------------------
    boolean buttonUse         ;       // has digital ON/OFF button?
    boolean RaspberryPIUse;  //a raspberryPI is connected to USBNativeport
    boolean useMqtt; // mqtt work to send dat to homeassistant
    boolean MyrpiStatusSync;
    unsigned long beepOnOFFDuration; //variable use for the beeper
    boolean beepState;//for the beeper true when sound
    unsigned long  nextTimeBeeper;// use for beeper
    boolean startByTimer; // use to know if the start is initiate by timer or manual via PFOD
    int whereToStart; // use to know where the mower need to leave the wire and start to mow
    unsigned int whereToResetSpeed; // use with Rfid Speed to know when reset to maxpwm

    int beaconToStart; // use to know where the mower need to leave the wire and start to mow
    byte areaToGo;// use to know the area where to start by timer
    //--------  Temperature humidity ------------------

    unsigned long  nextTimeReadTemperature = 0;
    //float humidityDht;
    float temperatureTeensy;
    float temperatureImu;

    float maxTemperature;  //switch to OFF when reach this temp

    // ----- user-defined switch ---------------------------
    boolean userSwitch1       ;       // user-defined switch 1 (default value)
    boolean userSwitch2       ;       // user-defined switch 2 (default value)
    boolean userSwitch3       ;       // user-defined switch 3 (default value)


    boolean userLed           ;       // Main Led (default value)
    boolean userGreenLed      ;       // Green Led (default value)
    boolean userRedLed        ;       // Red Led (default value)

    boolean userOut1        ;       // output on remote connector Mow (default value)
    boolean userOut2        ;       // output on remote connector steering
    boolean userOut3        ;       // output on remote connector speed
    boolean userOut4        ;       // output on remote connector switch

    boolean invert_userOut  ;       //switch use in case the userOut use a relay board






    // --------- charging -------------------------------

    boolean batMonitor ;              // monitor battery and charge voltage?
    float batGoHomeIfBelow ;     // drive home voltage (Volt)
    float batSwitchOffIfBelow ;  // switch off if below voltage (Volt)
    int batSwitchOffIfIdle;      // switch off battery if idle for minutes
    float batChgFactor       ;     // battery conversion factor
    float batChargingCurrentMax ; // maximum current your charger can devliver
    float batFull         ;      // battery reference Voltage (fully charged)
    float batFullCurrent   ; // current flowing when battery is fully charged
    float batVoltageToStationStart; // // start timer mowing at this voltage (avoid wait for 100 % charge)

    float startChargingIfBelow; // start charging if battery Voltage is below
    unsigned long chargingTimeout; // safety timer for charging
    int chargingMaxDuration; // safety timer for charging in hour
    float stationHeading    ;       // station heading to init the YAW when leave station (in radian)
    float batSenseFactor       ;     // charge current conversion factor
    //float chgSense        ;       // mV/A empfindlichkeit des Ladestromsensors in mV/A (FÃ¼r ACS712 5A = 185)
    //char chgChange        ;       // messwertumkehr von - nach +         1oder 0
    float batVoltage ;  // battery voltage (Volt)

    // byte chgSelection     ;       // Senor Auswahl
    //float batRefFactor ;
    float batCapacity ; // battery capacity (mAh)
    float chgVoltage ;  // charge voltage (Volt)
    float chgCurrent ;  // charge current  (Ampere)
    //int chgNull;        // Nulldurchgang Ladestromsensor
    byte stationRevDist     ;    // charge station reverse distance cm
    byte stationRollAngle    ;    // charge station roll angle
    int stationForwDist    ;    // charge station forward distance cm
    byte stationCheckDist   ;    // charge station check distance cm
    boolean UseBumperDock ;  //bumper is pressed when docking or not
    boolean autoResetActive;       // at the end of the charging all is rebbot to avoid error after 1 or 2 weeks ON
    byte dockingSpeed ;  //speed docking is (percent of maxspeed) when sonar detect something while tracking
    byte checkDockingSpeed ;  //station check speed mower force in station
    unsigned long totalDistDrive;  //use to check when to leave the wire in start timer mode
    unsigned long nextTimeBattery ; // delay between 2 battery reading
    unsigned long nextTimeCheckBattery; // check the battery state
    unsigned long delayToReadVoltageStation; //wait before read the voltage
    unsigned long nextTimeReadStationVoltage; //while tracking delay between 2 station voltage check
    int statsBatteryChargingCounter;
    int statsBatteryChargingCounterTotal;
    float  statsBatteryChargingCapacityTrip;
    float statsBatteryChargingCapacityTotal;
    float statsBatteryChargingCapacityAverage;
    float lastTimeBatCapacity;

    // --------- error counters --------------------------
    byte errorCounterMax[ERR_ENUM_COUNT];
    byte errorCounter[ERR_ENUM_COUNT];
    // --------- other ----------------------------------
    int loopsPerSec ;  // main loops per second
    int loopsPerSecCounter ;
    byte buttonCounter ;
    byte ledState ;
    byte consoleMode ;
    unsigned long nextTimeButtonCheck ;
    unsigned long nextTimeInfo ;
    unsigned long nextTimeScreen ;

    unsigned long nextTimePrintConsole;
    byte rollDir;
    unsigned long nextTimeButton ;
    unsigned long nextTimeErrorCounterReset;
    unsigned long nextTimeErrorBeep ;

    unsigned long endBeepTime; //time in millis when the beep need to stop
    int beepOnDuration; //  duration ON
    int  beepOffDuration; //  duration OFF
    int beepfrequenceOn;  // frequence ON
    int beepfrequenceOff;  //  frequence OFF normaly 0 but can be use to make other tone



    // ------------robot stats---------------------------
    boolean statsOverride ;
    boolean statsMowTimeTotalStart ;
    unsigned int statsMowTimeMinutesTripCounter ;
    unsigned long statsMowTimeMinutesTotal ;
    float statsMowTimeHoursTotal ;
    int statsMowTimeMinutesTrip ;
    unsigned long nextTimeRobotStats ;
    byte mowPatternDuration ; // use to know when need to change, the pattern change each x minutes so byte is OK
    byte mowPatternDurationMax; // value enter into PFOD for setting in Minutes
    //bb


    // --------------------------------------------------
    Robot();
    // robot setup
    virtual void setup();
    // robot main loop
    virtual void loop();
    virtual void resetIdleTime();
    virtual void resetWatchdogForPfod();


    // state machine
    virtual void setNextState(byte stateNew, byte dir);

    // motor
    virtual void setMotorPWM(int pwmLeft, int pwmRight);
    virtual void setMotorMowPWM(int pwm, boolean useAccel);

    // GPS
    virtual void processGPSData();
    virtual void receiveGPSTime();

    // read hardware sensor (HAL)
    //bb
    //virtual int readSensor(char type) {}
    //virtual void readMowerSensor(char type) {}

    // set hardware actuator (HAL)
    virtual void setActuator(char type, int value) {}

    // settings
    virtual void loadRfidList();
    virtual void saveRfidList();
    virtual void deleteUserSettings();
    virtual void saveUserSettings();
    virtual void deleteRobotStats();
    virtual void newTagFind();
    virtual void autoReboot();
    virtual void teensyBootLoader();
    virtual void powerOff_pcb();

    // other
    // virtual void beep(int numberOfBeeps, boolean shortbeep);
    virtual void printInfo(Stream &s);
    virtual void setUserSwitches();
    //bber60
    virtual void setUserOut();

    virtual void addErrorCounter(byte errType);
    virtual void resetErrorCounters();
    virtual void resetMotorFault();// {}
    //bb
    virtual void setBeeper(int totalDuration, int OnDuration, int OffDuration, int frequenceOn, int frequenceOff ); // Set the variable for the beeper
    //virtual void RaspberryPISendStat ();
    virtual void checkSenderIsRunning() ;
    virtual void receivePiPfodCommand (String RpiCmd, float v1, float v2, float v3);

    virtual void ResetWatchdog();


    virtual void writeOnSD(String message);
    virtual void writeOnSDln(String message);
    virtual void ShowMessage(String message);
    virtual void ShowMessageln(String message);
    virtual void ShowMessage(float value);
    virtual void ShowMessageln(float value);

    virtual void printSettingSerial();
    const char* mowPatternNameList(byte mowPatternIndex);
    char* rfidToDoNameList(byte rfidToDoIndex);
    char* statusNameList(byte statusIndex);

    virtual void insert_rfid_list(unsigned long TagNr, byte TagMowerStatus, byte TagToDo, int TagSpeed, float TagAngle1, int TagDist1, float TagAngle2, int TagDist2);
    virtual void print_rfid_list();
    virtual void sort_rfid_list();
    virtual void delete_rfid_list(unsigned long TagNr, byte TagMowerStatus, int pos_into_list);

    virtual void pfodSetDateTime(byte hr1, byte min1, byte sec1, byte day1, byte month1, short year1);

    virtual void startStopSender(int senderNr, boolean startStop);

  protected:
    static void OdoRightCountInt(); //odometry interrupt loop
    static void OdoLeftCountInt();
    static void myCallback();

    // convert ppm time to RC slider value
    virtual int rcValue(int ppmTime);
    virtual void loadSaveErrorCounters(boolean readflag);
    virtual void loadSaveUserSettings(boolean readflag);
    virtual void loadSaveRobotStats(boolean readflag);
    virtual void loadUserSettings();
    virtual void checkErrorCounter();


    // read sensors
    virtual void readSensors();

    // read serial
    virtual void readSerial();

    // check sensor
    virtual void checkButton();
    virtual void checkBattery();
    virtual void checkTimer();
    virtual void checkCurrent();
    virtual void checkBumpers();

    virtual void checkBumpersPerimeter();
    virtual void checkPerimeterBoundary();
    virtual void checkStuckOnIsland();


    virtual void checkSonar();
    virtual void checkSonarPeriTrack();

    virtual void checkTilt();
    virtual void checkRain();
    virtual void checkTimeout();
    virtual void checkOdometryFaults();
    virtual void checkIfStuck();
    virtual void checkRobotStats();

    // motor controllers
    virtual void motorControl();

    //bb

    virtual void OdoRampCompute() ;
    virtual void motorControlOdo() ;
    virtual void motorControlPerimeter();
    virtual void motorControlPerimeter2Coil();

    virtual void readAllTemperature();
    virtual void motorMowControl();

    // date & time
    virtual void setDefaultTime();

    // set reverse
    virtual void reverseOrBidir(byte aRollDir);

    // other

    virtual void printOdometry();
    virtual void printMenu();
    virtual void delayInfo(int ms);
    virtual void delayWithWatchdog(int ms);

    // virtual void testOdometry();
    virtual void testMotors();
    virtual void setDefaults();
    // virtual void receiveGPSTime();
    virtual void calcOdometry();
    virtual void menu();


    virtual void beeper();

    boolean search_rfid_list(unsigned long TagNr);
    void rfidTagTraitement(unsigned long TagNr, byte statusCurr);





};



#endif
