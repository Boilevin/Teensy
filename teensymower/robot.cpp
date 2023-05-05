/*
  Private-use only! (you need to ask for a commercial-use)
*/

#include "robot.h"
#include "mower.h"
#include "imu.h"
#include "flashmem.h"
#include "perimeter.h"
#include "RpiRemote.h"
#include "NewPing.h"
#include "Screen.h"



//the watchdog part
#include "Watchdog_t4.h"
WDT_T4<WDT1> wdt;

//Setting for Raspberry -----------------------------------
RpiRemote MyRpi;

//Setting for Screen -----------------------------------
Screen MyScreen;

//Ina226 part
#include "INA226.h"

INA226 ChargeIna226;
INA226 MotLeftIna226;
INA226 MotRightIna226;

INA226 Mow1Ina226;
INA226 Mow2Ina226;
INA226 Mow3Ina226;

float shuntvoltagec = 0;
float shuntvoltagel = 0;
float shuntvoltager = 0;
//end Ina226

//teensy 4 datetime library
#include "InternalTemperature.h"
#include <TimeLib.h>
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}



NewPing NewSonarLeft(pinSonarLeftTrigger, pinSonarLeftEcho, 110);
NewPing NewSonarRight(pinSonarRightTrigger, pinSonarRightEcho, 110);


#define MAGIC 52  //value 52 is only use to know if the eeprom is OK : 52 is save and read at first byte of memory location
#define ADDR_USER_SETTINGS 2000 //New adress to avoid issue if Azurit1.09 is already instaled
#define ADDR_ERR_COUNTERS 500 //same adress as azurit
//carrefull that the  ADDR 600 is used by the IMU calibration
#define ADDR_ROBOT_STATS 800
#define ADDR_RFID_LIST 3000 //start adress to rfid list value


const char* stateNames[] = {"OFF", "RC", "FORW", "ROLL", "REV", "CIRC", "ERR", "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK", "STREV",
                            "STROL", "STFOR", "MANU", "ROLW", "POUTFOR", "POUTREV", "POUTROLL", "POBSREV", "POBSROLL", "POBSFRWD", "POBSCIRC", "NEXTLANE", "POUTSTOP", "LANEROL1", "LANEROL2",
                            "ROLLTOIN", "WAITREPEAT", "FRWODO", "TESTCOMPAS", "ROLLTOTRACK",
                            "STOPTOTRACK", "AUTOCALIB", "ROLLTOFINDYAW", "TESTMOTOR", "FINDYAWSTOP", "STOPONBUMPER",
                            "STOPCALIB", "SONARTRIG", "STOPSPIRAL", "MOWSPIRAL", "ROT360", "NEXTSPIRE", "ESCAPLANE",
                            "TRACKSTOP", "ROLLTOTAG", "STOPTONEWAREA", "ROLL1TONEWAREA", "DRIVE1TONEWAREA", "ROLL2TONEWAREA", "DRIVE2TONEWAREA", "WAITSIG2", "STOPTONEWAREA", "ROLLSTOPTOTRACK",
                            "STOPTOFASTSTART", "CALIBMOTORSPEED", "ACCELFRWRD", "ENDLANE", "STARTSTATION", "BUMPREV" , "WAITCOVER"
                           };

const char* statusNames[] = {"WAIT", "NORMALMOWING", "SPIRALEMOWING", "BACKTOSTATION", "TRACKTOSTART", "MANUAL", "REMOTE", "ERROR", "STATION", "TESTING", "SIGWAIT" , "WIREMOWING"
                            };


const char* mowPatternNames[] = {"RAND", "LANE",  "WIRE" , "ZIGZAG"};
const char* consoleModeNames[] = {"sen_counters", "sen_values", "perimeter", "off", "Tracking"};
char* rfidToDoNames[] = {"NOTHING", "RTS", "FAST_START", "NEW_AREA", "SPEED", "AREA1", "AREA2", "AREA3"};

//for debug only
unsigned long StartReadAt;
unsigned long EndReadAt;
unsigned long ReadDuration;




Robot::Robot() {

  name = "Generic";
  developerActive = false;
  rc.setRobot(this);
  MyRpi.setRobot(this);

  stateLast = stateCurr = stateNext = STATE_OFF;
  statusCurr = WAIT; //initialise the status on power up

  stateTime = 0;
  idleTimeSec = 0;
  statsMowTimeTotalStart = false;



  odometryLeft = odometryRight = 0;
  PeriOdoIslandDiff = 0;
  odometryLeftLastState = odometryLeftLastState2 = odometryRightLastState = odometryRightLastState2 = LOW;
  odometryTheta = odometryX = odometryY = 0;
  prevYawCalcOdo = 0;
  motorRightRpmCurr = motorLeftRpmCurr = 0;
  lastMotorRpmTime = 0;
  lastSetMotorSpeedTime = 0;
  motorLeftSpeedRpmSet =  motorRightSpeedRpmSet = 0;
  motorLeftPWMCurr = motorRightPWMCurr = 0;
  motorRightSenseADC = motorLeftSenseADC = 0;
  motorLeftSenseCurrent = motorRightSenseCurrent = 0;
  motorLeftPower = motorRightPower = 0;
  motorLeftSenseCounter = motorRightSenseCounter = 0;
  motorZeroSettleTime = 0;
  motorLeftZeroTimeout = 0;
  motorRightZeroTimeout = 0;
  rotateLeft = true;
  moveRightFinish = false;
  moveLeftFinish = false;
  motorRpmCoeff = 1;

  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;
  remoteSteerLastTime = remoteSpeedLastTime = remoteMowLastTime = remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;

  motorMowRpmCounter = 0;
  motorMowRpmLastState = LOW;
  motorMowEnable = false;
  motorMowForceOff = false;
  motorMowSpeedPWMSet = motorMowSpeedMinPwm;  //use to set the speed of the mow motor
  motorMowPWMCurr = 0;
  motorMowSenseADC = 0;
  motorMowSenseCurrent  = 0;
  motorMowPower = 0;
  motorMowSenseCounter = 0;
  motorMowSenseErrorCounter = 0;
  motorMowPwmCoeff = 100;
  lastMowSpeedPWM = 0;
  timeToAddMowMedian = 0;
  lastSetMotorMowSpeedTime = 0;
  nextTimeCheckCurrent = 0;
  lastTimeMotorMowStuck = 0;
  totalDistDrive = 0;
  whereToResetSpeed = 50000;  // initial value to 500 meters

  bumperLeftCounter = bumperRightCounter = 0;
  bumperLeft = bumperRight = false;

  bumperRearLeftCounter = bumperRearRightCounter = 0;
  bumperRearLeft = bumperRearRight = false;

  gpsLat = gpsLon = gpsX = gpsY = 0;

  imuDriveHeading = 0;
  periFindDriveHeading = 0;
  remoteDriveHeading = 0;
  imuRollHeading = 0;
  imuRollDir = LEFT;
  rollDir = LEFT;


  perimeterMagLeft = 0;
  perimeterMagRight = 0;
  perimeterInsideLeft = false;
  perimeterInsideRight = false;
  perimeterCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;
  perimeterLeftTriggerTime = 0;
  perimeterRightTriggerTime = 0;

  areaInMowing = 1;
  perimeterSpeedCoeff = 1;



  rain = false;
  rainCounter = 0;

  sonarLeftUse = sonarRightUse = sonarCenterUse = false;
  sonarDistCenter = sonarDistRight = sonarDistLeft = 0;
  sonarObstacleTimeout = 0;
  distToObstacle = 0;
  sonarSpeedCoeff = 1;

  batVoltage = 0;
  //batRefFactor = 0;
  batCapacity = 0;
  lastTimeBatCapacity = 0;
  chgVoltage = 0;
  chgCurrent = 0;

  memset(errorCounterMax, 0, sizeof errorCounterMax);
  memset(errorCounter, 0, sizeof errorCounterMax);

  loopsPerSec = 0;
  loopsPerSecCounter = 0;
  buttonCounter = 0;
  ledState = 0;

  consoleMode = CONSOLE_OFF;
  nextTimeButtonCheck = 0;
  nextTimeInfo = 0;
  nextTimeScreen = 0;
  nextTimePrintConsole = 0;
  nextTimeMotorSense = 0;
  nextTimeIMU = 0;
  nextTimeCheckTilt = 0;
  nextTimeOdometry = 0;
  nextTimeOdometryInfo = 0;
  nextTimeBumper = 0;

  //nextTimeSonar = 0;
  nextTimeBattery = 0;
  nextTimeCheckBattery = 0;
  nextTimePerimeter = 0;

  nextTimeTimer = millis() + 60000;
  nextTimeRTC = 0;
  nextTimeGPS = 0;

  nextTimePfodLoop = 0;
  nextTimeGpsRead = 0;
  nextTimeImuLoop = 0;
  nextTimeRain = 0;
  lastMotorMowRpmTime = millis();
  nextTimeButton = 0;
  nextTimeErrorCounterReset = 0;
  nextTimeErrorBeep = 0;
  nextTimeMotorControl = 0;
  nextTimeMotorImuControl = 0;
  nextTimeMotorOdoControl = 0;
  nextTimePidCompute = 0;
  nextTimeMotorPerimeterControl = 0;
  nextTimeMotorMowControl = 0;
  nextTimeRotationChange = 0;
  nextTimeAddYawMedian = 0;
  nextTimeRobotStats = 0;
  delayToReadVoltageStation = 0;
  nextTimeReadStationVoltage = 0;

  MaxOdoStateDuration = 2000;
  MaxStateDuration = 2000;

  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
  lastTimeForgetWire = 0; //use in peritrack
  nextTimeToDmpAutoCalibration = 0; //at this time the mower start calibration on first positive lane stop
  //bber17
  RollToInsideQty = 0;
  rollToTrackQty = 0;
  findedYaw = 999; //use the first time set the compass and the Gyro have the same direction with state roll to find yaw
  highGrassDetect = false;
  motorRightPID.Kp = motorLeftPID.Kp;
  motorRightPID.Ki = motorLeftPID.Ki;
  motorRightPID.Kd = motorLeftPID.Kd;

  MyrpiStatusSync = false;
  ConsoleToPfod = false;
  sdcardToPfod = false;

  freeboolean = false;
  totalLineOnFile = 0;
}


const char* Robot::stateName() {
  return stateNames[stateCurr];
}
const char* Robot::statusName() {
  return statusNames[statusCurr];
}
char* Robot::statusNameList(byte statusIndex) {
  return statusNames[statusIndex];
}

char* Robot::rfidToDoName() {
  return rfidToDoNames[rfidToDoCurr];
}

char* Robot::rfidToDoNameList(byte rfidToDoIndex) {
  return rfidToDoNames[rfidToDoIndex];
}

const char* Robot::mowPatternName() {
  return mowPatternNames[mowPatternCurr];
}

const char* Robot::mowPatternNameList(byte mowPatternIndex) {
  return mowPatternNames[mowPatternIndex];
}


void Robot::setDefaultTime() {
  datetime.time.hour = 12;
  datetime.time.minute = 25;
  datetime.date.dayOfWeek = 0;
  datetime.date.day = 26;
  datetime.date.month = 4;
  datetime.date.year = 2021;
  timer[0].active = false;
  timer[0].daysOfWeek = B01111110;
  timer[0].startTime.hour = 9;
  timer[0].stopTime.hour = 11;
}



// check timer
void Robot::checkTimer() {

  if (millis() < nextTimeTimer) return;
  nextTimeTimer = millis() + 60000;  // one minute check
  srand(time2minutes(datetime.time)); // initializes the pseudo-random number generator for c++ rand()
  randomSeed(time2minutes(datetime.time)); // initializes the pseudo-random number generator for arduino random()
  boolean stopTimerTriggered = true;
  if (timerUse) {
    Serial.println("checktimer");
    for (int i = 0; i < MAX_TIMERS; i++) {
      if (timer[i].active) {
        if  ( (timer[i].daysOfWeek & (1 << datetime.date.dayOfWeek)) != 0) {
          int startmin = time2minutes(timer[i].startTime);
          int stopmin =  time2minutes(timer[i].stopTime);
          int currmin =  time2minutes(datetime.time);

          Serial.print("Timer ");
          Serial.print(i);
          Serial.print(" startmin ");
          Serial.print(startmin);

          Serial.print(" stopmin ");
          Serial.print(stopmin);

          Serial.print(" currmin ");
          Serial.println(currmin);

          if ((currmin >= startmin) && (currmin < stopmin)) {
            // start timer triggered
            stopTimerTriggered = false;
            if ((stateCurr == STATE_STATION)) {
              Serial.print("Timer ");
              Serial.print(i);
              Serial.println(F(" start triggered"));
              ActualRunningTimer = i;
              //motorMowEnable = true;
              findedYaw = 999;
              imuDirPID.reset();
              mowPatternCurr = timer[i].startMowPattern;
              laneUseNr = timer[i].startNrLane;
              rollDir = timer[i].startRollDir;
              whereToStart = timer[i].startDistance;
              areaToGo = timer[i].startArea;
              actualLenghtByLane = timer[i].startLaneMaxlengh;
              beaconToStart = timer[i].rfidBeacon;
              startByTimer = true;
              mowPatternDuration = 0;
              totalDistDrive = 0;
              Serial.print(F(" Track for area "));
              Serial.println(areaToGo);
              Serial.print(F(" Distance before start "));
              Serial.println(whereToStart);

              setNextState(STATE_START_FROM_STATION, 1);
              return;
              // setNextState(STATE_STATION_REV, 0);

            }
          }
        }
        if ((stateCurr != STATE_STATION) && (stopTimerTriggered) && (ActualRunningTimer == i)) { //Stop only the running timer

          Serial.println(F("timer stop triggered"));
          ActualRunningTimer = 99;
          if (perimeterUse) {
            setNextState(STATE_PERI_FIND, 0);
          }
          else {
            setNextState(STATE_OFF, 0);
          }
        }

      }
    }
  }
}

void Robot::loadSaveRobotStats(boolean readflag) {

  int addr = ADDR_ROBOT_STATS;

  //create a new history file name to separate the data log on sd card
  // sd.open need a char array to work

  sprintf(historyFilenameChar, "%02d%02d%02d%02d%02d.txt", datetime.date.year - 2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);

  if (sdCardReady) {
    ShowMessage(F("Log Filename : "));
    ShowMessageln(historyFilenameChar);
  }
  if (readflag) {
    ShowMessage(F("Load Stats "));
  }
  else {
    ShowMessage(F("Save Stats "));

  }

  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("********************************************"));
    ShowMessageln(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
    ShowMessageln(F("********************************************"));
  }
  eereadwrite(readflag, addr, statsMowTimeMinutesTrip);
  eereadwrite(readflag, addr, statsMowTimeMinutesTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCounterTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTrip);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityAverage);
  // <----------------------------new robot stats to save goes here!----------------
  ShowMessage(F("Adress Start = "));
  ShowMessage(ADDR_ROBOT_STATS);
  ShowMessage(F(" Stop = "));
  ShowMessageln(addr);

}


void Robot::receiveGPSTime() {
  if (gpsUse) {
    unsigned long chars = 0;
    unsigned short good_sentences = 0;
    unsigned short failed_cs = 0;
    gps.stats(&chars, &good_sentences, &failed_cs);
    if (good_sentences == 0) {
      // no GPS sentences received so far
      ShowMessageln(F("GPS communication error!"));
      addErrorCounter(ERR_GPS_COMM);
      // next line commented out as GPS communication may not be available if GPS signal is poor
      //setNextState(STATE_ERROR, 0);
    }
    ShowMessage(F("GPS sentences: "));
    ShowMessageln(good_sentences);
    ShowMessage(F("GPS satellites in view: "));
    ShowMessageln(gps.satellites());
    if (gps.satellites() == 255) {
      // no GPS satellites received so far
      addErrorCounter(ERR_GPS_DATA);
    }
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long age;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (age != GPS::GPS_INVALID_AGE)
    {
      ShowMessage(F("GPS date received: "));
      ShowMessageln(date2str(datetime.date));
      datetime.date.dayOfWeek = getDayOfWeek(month, day, year, 1);
      datetime.date.day = day;
      datetime.date.month = month;
      datetime.date.year = year;
      datetime.time.hour = hour;
      datetime.time.minute = minute;
      if (timerUse) {
        // set RTC using GPS data
        ShowMessage(F("RTC date set: "));
        ShowMessageln(date2str(datetime.date));
        setActuator(ACT_RTC, 0);
      }
    }
  }
}// check if mower is stuck ToDo: take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck() {   // not use 04/11/22
  if (millis() < nextTimeCheckIfStuck) return;
  nextTimeCheckIfStuck = millis() + 300;
  if ((gpsUse) && (gps.hdop() < 500))  {
    //float gpsSpeedRead = gps.f_speed_kmph();
    float gpsSpeed = gps.f_speed_kmph();

    // low-pass filter
    // double accel = 0.1;
    // float gpsSpeed = (1.0-accel) * gpsSpeed + accel * gpsSpeedRead;
    // ShowMessageln(gpsSpeed);
    // ShowMessageln(robotIsStuckCounter);
    // ShowMessageln(errorCounter[ERR_STUCK]);
    if ((stateCurr != STATE_MANUAL) && (stateCurr != STATE_REMOTE) && (gpsSpeed <= stuckIfGpsSpeedBelow)    // checks if mower is stuck and counts up
        && ((motorLeftRpmCurr && motorRightRpmCurr) != 0) && (millis() > stateStartTime + gpsSpeedIgnoreTime) ) {
      robotIsStuckCounter++;
    }

    else {                         // if mower gets unstuck it resets errorCounterMax to zero and reenabling motorMow
      robotIsStuckCounter = 0;    // resets temporary counter to zero
      if ( (errorCounter[ERR_STUCK] == 0) && (stateCurr != STATE_OFF) && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION)
           && (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION_CHECK)
           && (stateCurr != STATE_STATION_REV) && (stateCurr != STATE_STATION_ROLL)
           && (stateCurr != STATE_REMOTE) && (stateCurr != STATE_ERROR)) {
        motorMowEnable = true;
        errorCounterMax[ERR_STUCK] = 0;
      }
      return;
    }

    if (robotIsStuckCounter >= 5) {
      motorMowEnable = false;
      if (errorCounterMax[ERR_STUCK] >= 3) {  // robot is definately stuck and unable to move
        ShowMessageln(F("Error: Mower is stuck"));
        addErrorCounter(ERR_STUCK);
        setNextState(STATE_ERROR, 0);   //mower is switched into ERROR
        //robotIsStuckCounter = 0;
      }
      else if (errorCounter[ERR_STUCK] < 3) {   // mower tries 3 times to get unstuck
        if (stateCurr == STATE_FORWARD) {
          motorMowEnable = false;
          addErrorCounter(ERR_STUCK);
          setMotorPWM(0, 0);
          reverseOrBidir(RIGHT);
        }
        else if (stateCurr == STATE_ROLL) {
          motorMowEnable = false;
          addErrorCounter(ERR_STUCK);
          setMotorPWM(0, 0);
          setNextState (STATE_FORWARD, 0);
        }
      }
    }
  }
}


void Robot::processGPSData()
{
  if (millis() < nextTimeGPS) return;
  nextTimeGPS = millis() + 1000;
  float nlat, nlon;
  unsigned long age;
  gps.f_get_position(&nlat, &nlon, &age);
  if (nlat == GPS::GPS_INVALID_F_ANGLE ) return;
  if (gpsLon == 0) {
    gpsLon = nlon;  // this is xy (0,0)
    gpsLat = nlat;
    return;
  }
  gpsX = (float)gps.distance_between(nlat,  gpsLon,  gpsLat, gpsLon);
  gpsY = (float)gps.distance_between(gpsLat, nlon,   gpsLat, gpsLon);
  if (RaspberryPIUse) MyRpi.RaspberryPISendGpsLocalisation();
  /*
    Serial.print(" gpsX: ");
    Serial.print(gpsX);
    Serial.print(" gpsY: ");
    Serial.println(gpsY);
  */

}

boolean Robot::search_rfid_list(unsigned long TagNr) {
  boolean tag_exist_in_list = false;
  ptr = head;
  if (ptr != NULL) {
    for (ptr = head; ptr->next != NULL; ptr = ptr->next) {
      if (ptr->TagNr == TagNr) tag_exist_in_list = true;
    }
  }
  return tag_exist_in_list;
}


void Robot::startStopSender(int senderNr, boolean startStop) {
  ShowMessage(F("Sender "));
  ShowMessage(senderNr);
  ShowMessage(F(" / "));
  ShowMessageln(startStop);
  String line01 = "";

  if (senderNr == 1) {
    if (startStop) {
      line01 = "#SENDER," + area1_ip + ",A1";
      Bluetooth.println(line01);
    }
    else
    {
      line01 = "#SENDER," + area1_ip + ",A0";
      Bluetooth.println(line01);
    }
  }

  if (senderNr == 2) {
    if (startStop) {
      line01 = "#SENDER," + area2_ip + ",B1";
      Bluetooth.println(line01);
    }
    else
    {
      line01 = "#SENDER," + area2_ip + ",B0";
      Bluetooth.println(line01);
    }
  }

  if (senderNr == 3) {
    if (startStop) {
      line01 = "#SENDER," + area3_ip + ",B1";
      Bluetooth.println(line01);
    }
    else
    {
      line01 = "#SENDER," + area3_ip + ",B0";
      Bluetooth.println(line01);
    }
  }

}



void Robot::rfidTagTraitement(unsigned long TagNr, byte statusCurr) {
  boolean tagAndStatus_exist_in_list = false;
  String line01 = "";
  //struct rfid_list *temp = (struct rfid_list*) malloc(sizeof(rfid_list));
  ptr = head;
  if (ptr != NULL) {
    for (ptr = head; ptr->next != NULL; ptr = ptr->next) {
      if ((ptr->TagNr == TagNr) && (ptr->TagMowerStatus == statusCurr)) {
        tagAndStatus_exist_in_list = true;
        //ptr is locate on the correct record in the list --> exit from the for loop
        break;
      }
    }
  }
  if (tagAndStatus_exist_in_list)
  {
    //debut du traitement
    ShowMessage(F("Tag and Status find to do is "));
    ShowMessageln(F(rfidToDoNameList(ptr->TagToDo)));
    switch (ptr->TagToDo) {

      case NOTHING:
        ShowMessageln(F("nothing to do ???"));
        break;
      case RTS:
        ShowMessage("Fast return tag : Turning ");
        ShowMessage(ptr->TagAngle1);
        ShowMessage(" degrees and new speed is ");
        ShowMessageln(ptr->TagSpeed);
        newtagRotAngle1 = ptr->TagAngle1;
        motorSpeedMaxPwm = ptr->TagSpeed;
        setNextState(STATE_PERI_STOP_TOROLL, 0);
        break;
      case FAST_START:
        ShowMessage("Faster start tag. Turning ");
        ShowMessage(ptr->TagAngle1);
        ShowMessage("° and new speed is ");
        ShowMessageln(ptr->TagSpeed);
        if (areaToGo != 1) { // if a distance is set for start point we can't use the fast start
          newtagRotAngle1 = ptr->TagAngle1;
          motorSpeedMaxPwm = ptr->TagSpeed;
          setNextState(STATE_PERI_STOP_TO_FAST_START, 0);
        }
        else
        {
          ShowMessageln("Fast start is only valid to change mowing area");
        }
        break;
      case NEW_AREA:
        ShowMessageln("Not use better to use AREA1,2 or 3");
        //not use
        break;
      case AREA1:
        startStopSender(1, 1);
        startStopSender(2, 0);
        startStopSender(3, 0);
        areaToGo = 1;
        ShowMessageln("Return to Station area ");
        motorSpeedMaxPwm = ptr->TagSpeed;
        newtagRotAngle1 = ptr->TagAngle1;
        newtagDistance1 = ptr->TagDist1;
        newtagRotAngle2 = ptr->TagAngle2;
        newtagDistance2 = ptr->TagDist2;
        setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        //#stopsender
        if (areaInMowing == 2) {
          //ButtonStopArea2_click()
        }
        if (areaInMowing == 3) {
          //ButtonStopArea3_click()
        }
        break;
      case AREA2:
        //send data to ESP32 to start AREA2 sender and stop AREA1 one
        line01 = "#SENDER," + area1_ip + ",A0";
        Bluetooth.println(line01);
        line01 = "#SENDER," + area2_ip + ",B1";
        Bluetooth.println(line01);
        if (areaToGo == 2) {
          ShowMessageln("Go to AREA2");
          motorSpeedMaxPwm = ptr->TagSpeed;
          newtagRotAngle1 = ptr->TagAngle1;
          newtagDistance1 = ptr->TagDist1;
          newtagRotAngle2 = ptr->TagAngle2;
          newtagDistance2 = ptr->TagDist2;
          setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        }
        break;

      case AREA3:
        line01 = "#SENDER," + area1_ip + ",A0";
        Bluetooth.println(line01);
        line01 = "#SENDER," + area3_ip + ",B1";
        Bluetooth.println(line01);
        if (areaToGo == 3) {
          ShowMessageln("Go to AREA3");
          motorSpeedMaxPwm = ptr->TagSpeed;
          newtagRotAngle1 = ptr->TagAngle1;
          newtagDistance1 = ptr->TagDist1;
          newtagRotAngle2 = ptr->TagAngle2;
          newtagDistance2 = ptr->TagDist2;
          setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        }
        break;

      case SPEED:
        ActualSpeedPeriPWM = ptr->TagSpeed;
        newtagDistance1 = ptr->TagDist1;
        whereToResetSpeed =  totalDistDrive + newtagDistance1; // when a speed tag is read it's where the speed is back to maxpwm value
        ShowMessage("Change to speed  : ");
        ShowMessage(ActualSpeedPeriPWM);
        ShowMessage(" for next ");
        ShowMessage(newtagDistance1);
        ShowMessageln(" centimeters");

        break;
    }
  }
  else
  {
    ShowMessageln(F("Tag and Status not match"));
  }
}

void Robot::insert_rfid_list(unsigned long TagNr, byte TagMowerStatus, byte TagToDo, int TagSpeed, float TagAngle1, int TagDist1, float TagAngle2, int TagDist2) {
  struct rfid_list *node = (struct rfid_list*) malloc(sizeof(*node));//allocation dynamique de la memoire
  if (node == NULL) {
    ShowMessageln(F("New Rfid tag list insert error "));
    return;
  }

  node->TagNr = TagNr;
  node->TagMowerStatus = TagMowerStatus;
  node->TagToDo = TagToDo;
  node->TagSpeed = TagSpeed;
  node->TagAngle1 = TagAngle1;
  node->TagDist1 = TagDist1;
  node->TagAngle2 = TagAngle2;
  node->TagDist2 = TagDist2;
  node->next = head;  // def du nouveau noeud au premier
  head = node; // tete de la liste devient celui que l on a ajouté.
  rfidListElementCount = rfidListElementCount + 1;
  ShowMessageln(F("1 RFID TAG insertion OK"));
  ShowMessage(F("NEW RFID LIST COUNT = "));
  ShowMessageln(rfidListElementCount);
}
void Robot::delete_rfid_list(unsigned long TagNr, byte TagMowerStatus, int pos_into_list) {
  struct rfid_list *supp_element = NULL;
  ShowMessage(F("Delete element Nr: "));
  ShowMessageln(int(pos_into_list));
  ptr = head;  // move at the beginning of the list
  for (int i = 1; i < pos_into_list; ++i) {
    ptr = ptr->next; // move just before the one to delete
  }


  supp_element = ptr->next; //  the one to delete
  ptr->next = ptr->next->next; //  rewrite the pointer of element before the supress one to the next next one
  // if(ptr->next == NULL)
  //         liste->fin = courant;
  //free (supp_element->donnee);
  free (supp_element);  // free memory  need maybe more free for detail element
  rfidListElementCount = rfidListElementCount - 1;
  ShowMessageln(F("1 RFID TAG suppression OK"));
  ShowMessage(F("NEW RFID LIST COUNT = "));
  ShowMessageln(rfidListElementCount);
}

void Robot::sort_rfid_list() {

  struct rfid_list *p = NULL;
  struct rfid_list PR ;
  struct rfid_list *temp = (struct rfid_list*) malloc(sizeof(rfid_list));
  ptr = head;
  if (ptr != NULL) {
    for (temp = head; temp->next != NULL; temp = temp->next) {
      for (p = temp->next; p != NULL; p = p->next) {

        if (p->TagNr < temp->TagNr) {
          PR.TagNr = p->TagNr;
          PR.TagMowerStatus = p->TagMowerStatus;
          PR.TagToDo = p->TagToDo;
          PR.TagSpeed = p->TagSpeed;
          PR.TagAngle1 = p->TagAngle1;
          PR.TagDist1 = p->TagDist1;
          PR.TagAngle2 = p->TagAngle2;
          PR.TagDist2 = p->TagDist2;

          //p->TagNr = temp->TagNr;
          p->TagNr = temp->TagNr;
          p->TagMowerStatus = temp->TagMowerStatus;
          p->TagToDo = temp->TagToDo;
          p->TagSpeed = temp->TagSpeed;
          p->TagAngle1 = temp->TagAngle1;
          p->TagDist1 = temp->TagDist1;
          p->TagAngle2 = temp->TagAngle2;
          p->TagDist2 = temp->TagDist2;


          //temp->TagNr = PR.TagNr;
          temp->TagNr = PR.TagNr;
          temp->TagMowerStatus = PR.TagMowerStatus;
          temp->TagToDo = PR.TagToDo;
          temp->TagSpeed = PR.TagSpeed;
          temp->TagAngle1 = PR.TagAngle1;
          temp->TagDist1 = PR.TagDist1;
          temp->TagAngle2 = PR.TagAngle2;
          temp->TagDist2 = PR.TagDist2;


        }
      }
    }



  }

}





void Robot::print_rfid_list() {
  ShowMessageln("RFID LIST :");
  ptr = head;
  //struct rfid_list *ptr = head;

  // rfid todo list
  //enum { NOTHING, RTS, FAST_START, NEW_AREA, SPEED, AREA1, AREA2, AREA3 };
  while (ptr != NULL) {  //parcours jusqu au dernier
    ShowMessage(String(ptr->TagNr, HEX));
    ShowMessage(",");
    ShowMessage(statusNames[ptr->TagMowerStatus]);
    ShowMessage(",");
    ShowMessage(rfidToDoNames[ptr->TagToDo]);
    ShowMessage(",");
    ShowMessage(ptr->TagSpeed);
    ShowMessage(",");
    ShowMessage(ptr->TagAngle1);
    ShowMessage(",");
    ShowMessage(ptr->TagDist1);
    ShowMessage(",");
    ShowMessage(ptr->TagAngle2);
    ShowMessage(",");
    ShowMessageln(ptr->TagDist2);
    ptr = ptr->next;
  }

}

void Robot::saveRfidList() {
  boolean readflag = false;
  int addr = ADDR_RFID_LIST;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  ShowMessage("RFID LIST COUNT = ");
  ShowMessageln(rfidListElementCount);
  eereadwrite(readflag, addr, rfidListElementCount); // magic
  ptr = head;
  while (ptr != NULL) {  //parcours jusqu au dernier
    eereadwrite(readflag, addr, ptr->TagNr);
    eereadwrite(readflag, addr, ptr->TagMowerStatus);
    eereadwrite(readflag, addr, ptr->TagToDo);
    eereadwrite(readflag, addr, ptr->TagSpeed);
    eereadwrite(readflag, addr, ptr->TagAngle1);
    eereadwrite(readflag, addr, ptr->TagDist1);
    eereadwrite(readflag, addr, ptr->TagAngle2);
    eereadwrite(readflag, addr, ptr->TagDist2);
    ptr = ptr->next;
  }
  ShowMessage(F("RFID LIST address Start="));
  ShowMessageln(ADDR_RFID_LIST);
  ShowMessage(F("RFID LIST address Stop="));
  ShowMessageln(addr);
}


void Robot::loadRfidList() {
  byte rfidListElementTotal = 0;
  boolean readflag = true;
  int addr = ADDR_RFID_LIST;
  struct rfid_list PR ;
  short magic = 0;
  eereadwrite(readflag, addr, magic); // magic
  if (magic != MAGIC) {
    ShowMessageln(F("RFID LIST USERDATA: NO EEPROM RFID LIST DATA"));
    ShowMessageln(F("PLEASE SAVE YOUR RFID LIST ONCE"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, rfidListElementTotal); // magic
  ShowMessage("rfidListElementTotal = ");
  ShowMessageln(rfidListElementTotal);
  for (int i = 0; i < rfidListElementTotal; i++) {
    ShowMessage(i);
    ShowMessageln ("TAG READ");
    eereadwrite(readflag, addr, PR.TagNr);
    eereadwrite(readflag, addr, PR.TagMowerStatus);
    eereadwrite(readflag, addr, PR.TagToDo);
    eereadwrite(readflag, addr, PR.TagSpeed);
    eereadwrite(readflag, addr, PR.TagAngle1);
    eereadwrite(readflag, addr, PR.TagDist1);
    eereadwrite(readflag, addr, PR.TagAngle2);
    eereadwrite(readflag, addr, PR.TagDist2);
    insert_rfid_list(PR.TagNr, PR.TagMowerStatus, PR.TagToDo, PR.TagSpeed, PR.TagAngle1, PR.TagDist1, PR.TagAngle2, PR.TagDist2);

  }
  ShowMessage(F("RFID LIST address Start="));
  ShowMessageln(ADDR_RFID_LIST);
  ShowMessage(F("RFID LIST address Stop="));
  ShowMessageln(addr);
  sort_rfid_list();
  print_rfid_list();
}


void Robot::loadSaveErrorCounters(boolean readflag) {

  if (readflag) {
    ShowMessage(F("Load ErrorData "));
  }
  else {
    ShowMessage(F("Save ErrorData "));
  }
  int addr = ADDR_ERR_COUNTERS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("*****************************************"));
    ShowMessageln(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
    ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    ShowMessageln(F("*****************************************"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, errorCounterMax);
  ShowMessage(F("Address Start= "));
  ShowMessage(ADDR_ERR_COUNTERS);
  ShowMessage(F(" Stop= "));
  ShowMessageln(addr);

}

void Robot::loadSaveUserSettings(boolean readflag) {


  int addr = ADDR_USER_SETTINGS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic

  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("************************************"));
    ShowMessageln(F("        NO EEPROM USER DATA"));
    ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS "));
    ShowMessageln(F("  FACTORY SETTING ARE USED INSTEAD  "));
    ShowMessageln(F("************************************"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }

  eereadwrite(readflag, addr, developerActive);
  eereadwrite(readflag, addr, motorAccel);
  eereadwrite(readflag, addr, motorSpeedMaxRpm);
  eereadwrite(readflag, addr, motorSpeedMaxPwm);
  eereadwrite(readflag, addr, motorPowerMax);
  eereadwrite(readflag, addr, motorSenseRightScale);
  eereadwrite(readflag, addr, motorSenseLeftScale);
  eereadwrite(readflag, addr, motorRollDegMax);
  eereadwrite(readflag, addr, motorRollDegMin);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorPowerIgnoreTime);
  eereadwrite(readflag, addr, motorForwTimeMax);
  eereadwrite(readflag, addr, motorMowSpeedMaxPwm);
  eereadwrite(readflag, addr, motorMowPowerMax);
  eereadwrite(readflag, addr, motorMowSpeedMinPwm);
  eereadwrite(readflag, addr, highGrassSpeedCoeff);
  eereadwrite(readflag, addr, motorLeftPID.Kp);
  eereadwrite(readflag, addr, motorLeftPID.Ki);
  eereadwrite(readflag, addr, motorLeftPID.Kd);
  eereadwrite(readflag, addr, motorMowPID.Kp);
  eereadwrite(readflag, addr, motorMowPID.Ki);
  eereadwrite(readflag, addr, motorMowPID.Kd);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio1);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio2);
  eereadwrite(readflag, addr, bumperUse);
  eereadwrite(readflag, addr, sonarUse);
  eereadwrite(readflag, addr, sonarCenterUse);
  eereadwrite(readflag, addr, sonarLeftUse);
  eereadwrite(readflag, addr, sonarRightUse);
  eereadwrite(readflag, addr, sonarTriggerBelow);
  eereadwrite(readflag, addr, perimeterUse);
  eereadwrite(readflag, addr, perimeterTriggerMinSmag);
  eereadwrite(readflag, addr, trackingErrorTimeOut);
  eereadwrite(readflag, addr, motorTickPerSecond);
  eereadwrite(readflag, addr, perimeterOutRevTime);
  eereadwrite(readflag, addr, perimeterTrackRollTime );
  eereadwrite(readflag, addr, perimeterTrackRevTime);
  eereadwrite(readflag, addr, perimeterPID.Kp);
  eereadwrite(readflag, addr, perimeterPID.Ki);
  eereadwrite(readflag, addr, perimeterPID.Kd);
  eereadwrite(readflag, addr, trakBlockInnerWheel);
  eereadwrite(readflag, addr, imuUse);
  eereadwrite(readflag, addr, stopMotorDuringCalib);
  eereadwrite(readflag, addr, imuDirPID.Kp);
  eereadwrite(readflag, addr, imuDirPID.Ki);
  eereadwrite(readflag, addr, imuDirPID.Kd);
  eereadwrite(readflag, addr, imuRollPID.Kp);
  eereadwrite(readflag, addr, imuRollPID.Ki);
  eereadwrite(readflag, addr, imuRollPID.Kd);
  eereadwrite(readflag, addr, remoteUse);
  eereadwrite(readflag, addr, batMonitor);
  eereadwrite(readflag, addr, batGoHomeIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfIdle);
  eereadwrite(readflag, addr, batFactor);  //float not use with ina226
  eereadwrite(readflag, addr, batChgFactor);  //float not use with ina226
  eereadwrite(readflag, addr, batSenseFactor); //float not use with ina226
  eereadwrite(readflag, addr, batFullCurrent);
  eereadwrite(readflag, addr, startChargingIfBelow);
  eereadwrite(readflag, addr, stationRevDist);
  eereadwrite(readflag, addr, stationRollAngle);
  eereadwrite(readflag, addr, stationForwDist);
  eereadwrite(readflag, addr, stationCheckDist);
  eereadwrite(readflag, addr, UseBumperDock);
  eereadwrite(readflag, addr, odometryTicksPerRevolution);
  eereadwrite(readflag, addr, odometryTicksPerCm);
  eereadwrite(readflag, addr, odometryWheelBaseCm);
  eereadwrite(readflag, addr, autoResetActive);
  eereadwrite(readflag, addr, CompassUse);
  eereadwrite(readflag, addr, twoWayOdometrySensorUse);   // char YES NO adress free for something else
  eereadwrite(readflag, addr, buttonUse);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainUse);
  eereadwrite(readflag, addr, statsOverride);
  eereadwrite(readflag, addr, reduceSpeedNearPerimeter);
  eereadwrite(readflag, addr, autoAdjustSlopeSpeed);
  //eereadwriteString(readflag, addr, esp8266ConfigString);//string not used
  eereadwrite(readflag, addr, tiltUse);
  eereadwrite(readflag, addr, trackingPerimeterTransitionTimeOut);
  eereadwrite(readflag, addr, motorMowForceOff);
  eereadwrite(readflag, addr, MaxSpeedperiPwm);
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //initialise Actual tracking speed
  eereadwrite(readflag, addr, RollTimeFor45Deg);  //unsigned long adress free for something else
  eereadwrite(readflag, addr, DistPeriObstacleAvoid);
  eereadwrite(readflag, addr, circleTimeForObstacle);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorRightOffsetFwd);
  eereadwrite(readflag, addr, motorRightOffsetRev);
  eereadwrite(readflag, addr, perimeterMagMaxValue);
  eereadwrite(readflag, addr, SpeedOdoMin);
  eereadwrite(readflag, addr, SpeedOdoMax);
  eereadwrite(readflag, addr, yawSet1);
  eereadwrite(readflag, addr, yawSet2);
  eereadwrite(readflag, addr, yawSet3);
  eereadwrite(readflag, addr, yawOppositeLane1RollRight);
  eereadwrite(readflag, addr, yawOppositeLane2RollRight);
  eereadwrite(readflag, addr, yawOppositeLane3RollRight);
  eereadwrite(readflag, addr, yawOppositeLane1RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane2RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane3RollLeft);
  eereadwrite(readflag, addr, DistBetweenLane);
  eereadwrite(readflag, addr, maxLenghtByLane);
  actualLenghtByLane = maxLenghtByLane; //initialise lenght lane
  eereadwrite(readflag, addr, swapCoilPolarityRight);
  eereadwrite(readflag, addr, read2Coil);
  eereadwrite(readflag, addr, maxDriftPerSecond);
  eereadwrite(readflag, addr, delayBetweenTwoDmpAutocalib);
  eereadwrite(readflag, addr, maxDurationDmpAutocalib);
  eereadwrite(readflag, addr, mowPatternDurationMax);
  eereadwrite(readflag, addr, DistPeriOutStop);
  eereadwrite(readflag, addr, Enable_Screen);
  eereadwrite(readflag, addr, RaspberryPIUse);
  //RaspberryPIUse=false;
  eereadwrite(readflag, addr, sonarToFrontDist);
  eereadwrite(readflag, addr, maxTemperature);
  eereadwrite(readflag, addr, dockingSpeed);
  eereadwrite(readflag, addr, rfidUse);
  eereadwrite(readflag, addr, compassRollSpeedCoeff);
  eereadwrite(readflag, addr, gpsUse);
  eereadwrite(readflag, addr, stuckIfGpsSpeedBelow);
  eereadwrite(readflag, addr, gpsSpeedIgnoreTime);
  eereadwrite(readflag, addr, useMqtt);
  eereadwrite(readflag, addr, stationHeading);
  eereadwrite(readflag, addr, checkDockingSpeed);
  eereadwrite(readflag, addr, batVoltageToStationStart);
  eereadwrite(readflag, addr, bumper_rev_distance);
  eereadwrite(readflag, addr, swapCoilPolarityLeft);
  eereadwrite(readflag, addr, useMotorDriveBrake);



  if (readflag)
  {
    ShowMessage(F("UserSettings OK from Address : "));
    ShowMessage(ADDR_USER_SETTINGS);
    ShowMessage(F(" To "));
    ShowMessageln(addr);
    motorInitialSpeedMaxPwm = motorSpeedMaxPwm; //the Pi can change the speed so store the initial value to restore after PFND for example
  }
  else
  {
    ShowMessage(F("UserSettings are saved from Address : "));
    ShowMessage(ADDR_USER_SETTINGS);
    ShowMessage(F(" To "));
    ShowMessageln(addr);
  }

}

void Robot::loadUserSettings() {
  //return; // use in one shot to reset all the usersetting if acces on console is not possible
  loadSaveUserSettings(true);
}


void Robot::printSettingSerial() {

  // ------- wheel motors ---------------------------------------------------------
  ShowMessageln("---------- wheel motors -----------");
  ShowMessage  ("motorAccel                 : ");
  ShowMessageln(motorAccel);
  ShowMessage  ("motorSpeedMaxRpm           : ");
  ShowMessageln(motorSpeedMaxRpm);
  ShowMessage  ("motorSpeedMaxPwm           : ");
  ShowMessageln(motorSpeedMaxPwm);
  ShowMessage  ("motorPowerMax              : ");
  ShowMessageln(motorPowerMax);
  ShowMessage  ("motorSenseRightScale       : ");
  ShowMessageln(motorSenseRightScale);
  ShowMessage  ("motorSenseLeftScale        : ");
  ShowMessageln(motorSenseLeftScale);
  //watchdogReset();
  ShowMessage  ("motorPowerIgnoreTime       : ");
  ShowMessageln(motorPowerIgnoreTime);
  ShowMessage  ("motorZeroSettleTime        : ");
  ShowMessageln(motorZeroSettleTime);
  ShowMessage  ("motorRollDegMax            : ");
  ShowMessageln(motorRollDegMax);
  ShowMessage  ("motorRollDegMin            : ");
  ShowMessageln(motorRollDegMin);
  ShowMessage  ("DistPeriOutRev             : ");
  ShowMessageln(DistPeriOutRev);
  //watchdogReset();
  ShowMessage  ("DistPeriOutStop            : ");
  ShowMessageln(DistPeriOutStop);
  ShowMessage  ("motorForwTimeMax           : ");
  ShowMessageln(motorForwTimeMax);
  ShowMessage  ("DistPeriObstacleAvoid      : ");
  ShowMessageln(DistPeriObstacleAvoid);
  ShowMessage  ("circleTimeForObstacle      : ");
  ShowMessageln(circleTimeForObstacle);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  //watchdogReset();
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("SpeedOdoMin                : ");
  ShowMessageln(SpeedOdoMin);
  ShowMessage  ("SpeedOdoMax                : ");
  ShowMessageln(SpeedOdoMax);
  ShowMessage  ("motorTickPerSecond         : ");
  ShowMessageln(motorTickPerSecond);

  ShowMessage  ("motorLeftPID.Kp            : ");
  ShowMessageln(motorLeftPID.Kp);
  ShowMessage  ("motorLeftPID.Ki            : ");
  ShowMessageln(motorLeftPID.Ki);
  ShowMessage  ("motorLeftPID.Kd            : ");
  ShowMessageln(motorLeftPID.Kd);

  ShowMessage  ("motorRightSwapDir          : ");
  ShowMessageln(motorRightSwapDir);
  ShowMessage  ("motorLeftSwapDir           : ");
  ShowMessageln(motorLeftSwapDir);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("autoAdjustSlopeSpeed       : ");
  ShowMessageln(autoAdjustSlopeSpeed);



  //watchdogReset();
  delayWithWatchdog (500);
  // ------ mower motor -----------------------------------
  ShowMessageln("---------- mower motor -----------------");
  ShowMessage  ("motorMowForceOff         : ");
  ShowMessageln(motorMowForceOff);
  ShowMessage  ("motorMowAccel            : ");
  ShowMessageln(motorMowAccel);
  ShowMessage  ("motorMowSpeedMaxPwm      : ");
  ShowMessageln(motorMowSpeedMaxPwm);
  ShowMessage  ("(motorMowSpeedMinPwm     : ");
  ShowMessageln(motorMowSpeedMinPwm);
  ShowMessage  ("motorMowPowerMax         : ");
  ShowMessageln(motorMowPowerMax);
  ShowMessage  ("highGrassSpeedCoeff      : ");
  ShowMessageln(highGrassSpeedCoeff);

  //watchdogReset();
  // ------ bumper ------------------------------------
  ShowMessageln("---------- bumper -----------------");
  ShowMessage  ("bumperUse           : ");
  ShowMessageln(bumperUse);
  ShowMessage  ("bumper_rev_distance : ");
  ShowMessageln(bumper_rev_distance);




  // ------ rain -------------------------------------
  ShowMessageln("---------- rain ----------------");
  ShowMessage  ("rainUse             : ");
  ShowMessageln(rainUse);

  // ------  Temperature -----------------------
  ShowMessageln("----------  Temperature ---");
  ShowMessage  ("MaxTemperature     : ");
  ShowMessageln(maxTemperature);

  // ------ Screen -----------------------
  ShowMessage  ("Enable_Screen        : ");
  ShowMessageln(Enable_Screen);

  // ------ sonar -----------------------------------
  ShowMessageln(F("---------- sonar ---------------"));
  ShowMessage  ("sonarUse              : ");
  ShowMessageln(sonarUse);
  ShowMessage  ("sonarLikeBumper       : ");
  ShowMessageln(sonarLikeBumper);
  ShowMessage  ("sonarLeftUse        : ");
  ShowMessageln(sonarLeftUse);
  ShowMessage  ("sonarRightUse       : ");
  ShowMessageln(sonarRightUse);
  ShowMessage  ("sonarCenterUse      : ");
  ShowMessageln(sonarCenterUse);
  ShowMessage  ("sonarTriggerBelow   : ");
  ShowMessageln(sonarTriggerBelow);
  ShowMessage  ("sonarToFrontDist    : ");
  ShowMessageln(sonarToFrontDist);

  //watchdogReset();
  delayWithWatchdog (500);
  // ------ perimeter --------------------------
  ShowMessageln("---------- perimeter ------");
  ShowMessage  ("perimeterUse             : ");
  ShowMessageln(perimeterUse);
  ShowMessage  ("perimeterTriggerMinSmag  : ");
  ShowMessageln(perimeterTriggerMinSmag);
  ShowMessage  ("MaxSpeedperiPwm          : ");
  ShowMessageln(MaxSpeedperiPwm);
  ShowMessage  ("perimeterTrackRollTime   : ");
  ShowMessageln(perimeterTrackRollTime);
  ShowMessage  ("perimeterTrackRevTime    : ");
  ShowMessageln(perimeterTrackRevTime);
  ShowMessage  ("perimeterPID.Kp          : ");
  ShowMessageln(perimeterPID.Kp);
  ShowMessage  ("perimeterPID.Ki          : ");
  ShowMessageln( perimeterPID.Ki);
  //watchdogReset();
  ShowMessage  ("perimeterPID.Kd          : ");
  ShowMessageln(perimeterPID.Kd);
  ShowMessage  ("trackingPerimeterTransitionTimeOut: ");
  ShowMessageln(trackingPerimeterTransitionTimeOut);
  ShowMessage  ("trackingErrorTimeOut     : ");
  ShowMessageln(trackingErrorTimeOut);
  ShowMessage  ("perimeterMagMaxValue     : ");
  ShowMessageln(perimeterMagMaxValue);
  ShowMessage  ("swapCoilPolarityRight    : ");
  //watchdogReset();
  ShowMessageln(swapCoilPolarityRight);
  ShowMessage  ("swapCoilPolarityLeft     : ");
  ShowMessageln(swapCoilPolarityLeft);
  ShowMessage  ("read2Coil                : ");
  ShowMessageln(read2Coil);
  ShowMessage  ("trackingBlockInnerWheelWhilePerimeterStrug : ");
  ShowMessageln(trakBlockInnerWheel);
  ShowMessage  ("DistPeriOutRev           : ");
  ShowMessageln(DistPeriOutRev);
  ShowMessage  ("DistPeriObstacleRev      : ");
  ShowMessageln(DistPeriObstacleRev);
  ShowMessage  ("DistPeriOutForw          : ");
  ShowMessageln(DistPeriOutForw);
  ShowMessage  ("DistPeriObstacleForw     : ");
  ShowMessageln(DistPeriObstacleForw);
  //watchdogReset();
  delayWithWatchdog (500);
  // ------ By Lanes mowing ---------------------
  ShowMessageln(F("---------- By Lanes mowing ----------"));
  ShowMessage  (F("yawSet1                   : "));
  ShowMessageln(yawSet1);
  ShowMessage  (F("yawSet2                   : "));
  ShowMessageln(yawSet2);
  ShowMessage  (F("yawSet3                   : "));
  ShowMessageln(yawSet3);
  ShowMessage  (F("yawOppositeLane1RollRight : "));
  ShowMessageln(yawOppositeLane1RollRight);
  ShowMessage  (F("yawOppositeLane2RollRight : "));
  ShowMessageln(yawOppositeLane2RollRight);
  ShowMessage  (F("yawOppositeLane3RollRight : "));
  ShowMessageln(yawOppositeLane3RollRight);
  ShowMessage  (F("yawOppositeLane1RollLeft  : "));
  ShowMessageln(yawOppositeLane1RollLeft);
  //watchdogReset();
  ShowMessage  (F("yawOppositeLane2RollLeft  : "));
  ShowMessageln(yawOppositeLane2RollLeft);
  ShowMessage  (F("yawOppositeLane3RollLeft  : "));
  ShowMessageln(yawOppositeLane3RollLeft);
  ShowMessage  (F("DistBetweenLane           : "));
  ShowMessageln(DistBetweenLane);
  ShowMessage  (F("maxLenghtByLane           : "));
  ShowMessageln(maxLenghtByLane);
  //watchdogReset();


  // ------  IMU (compass/accel/gyro) ------
  ShowMessageln(F("---------- IMU (compass/accel/gyro) ---- "));
  ShowMessage  (F("imuUse                : "));
  ShowMessageln( imuUse);
  ShowMessage  (F("CompassUse            : "));
  ShowMessageln(CompassUse);
  ShowMessage  (F("stopMotorDuringCalib  : "));
  ShowMessageln(stopMotorDuringCalib);
  ShowMessage  (F("imuDirPID.Kp          : "));
  ShowMessageln(imuDirPID.Kp);
  ShowMessage  (F("imuDirPID.Ki          : "));
  ShowMessageln(imuDirPID.Ki);
  ShowMessage  (F("imuDirPID.Kd          : "));
  ShowMessageln( imuDirPID.Kd);
  //watchdogReset();
  ShowMessage  (F("maxDriftPerSecond     : "));
  ShowMessageln(maxDriftPerSecond);
  ShowMessage  (F("delayBetweenTwoDmpAutocalib : "));
  ShowMessageln(delayBetweenTwoDmpAutocalib);
  ShowMessage  (F("maxDurationDmpAutocalib     : "));
  ShowMessageln(maxDurationDmpAutocalib);
  ShowMessage  (F("compassRollSpeedCoeff       : "));
  ShowMessageln(compassRollSpeedCoeff);
  delayWithWatchdog (500);
  //watchdogReset();
  // ------ model R/C ------------------------------
  ShowMessageln(F("---------- model R/C ---------"));
  ShowMessage  (F("remoteUse                   : "));
  ShowMessageln(remoteUse);

  // ------ battery ----------------------------
  ShowMessageln(F("---------- battery --------  "));
  ShowMessage  (F("batMonitor           : "));
  ShowMessageln( batMonitor);
  ShowMessage  (F("batGoHomeIfBelow     : "));
  ShowMessageln(batGoHomeIfBelow);
  ShowMessage  (F("batSwitchOffIfBelow  : "));
  ShowMessageln(batSwitchOffIfBelow);
  ShowMessage  (F("batSwitchOffIfIdle   : "));
  ShowMessageln(batSwitchOffIfIdle);
  ShowMessage  (F("batFactor            : "));
  ShowMessageln( batFactor);
  ShowMessage  (F("batChgFactor         : "));
  ShowMessageln( batChgFactor);
  ShowMessage  (F("batFull              : "));
  ShowMessageln( batFull);
  ShowMessage  (F("batVoltageToStationStart: "));
  ShowMessageln(batVoltageToStationStart);
  ShowMessage  (F("batChargingCurrentMax: "));
  ShowMessageln(batChargingCurrentMax);
  ShowMessage  (F("batFullCurrent       : "));
  ShowMessageln(batFullCurrent);
  ShowMessage  (F("startChargingIfBelow : "));
  ShowMessageln(startChargingIfBelow);
  ShowMessage  (F("chargingTimeout      : "));
  ShowMessageln(chargingTimeout);
  ShowMessage  (F("stationHeading       : "));
  ShowMessageln(stationHeading);
  ShowMessage  (F("batSenseFactor       : "));
  ShowMessageln( batSenseFactor);
  //ShowMessage  (F("chgSense             : "));
  //ShowMessageln(chgSense);
  //ShowMessage  (F("chgChange            : "));
  //ShowMessageln(chgChange);
  //ShowMessage  (F("chgNull              : "));
  //ShowMessageln(chgNull);
  //watchdogReset();
  // ------  charging station -----------------------------------------------------
  ShowMessageln(F("---------- charging station ----------------------------------"));
  ShowMessage  (F("stationRevDist     : "));
  ShowMessageln(stationRevDist);
  ShowMessage  (F("stationRollAngle   : "));
  ShowMessageln(stationRollAngle);
  ShowMessage  (F("stationForwDist    : "));
  ShowMessageln(stationForwDist);
  ShowMessage  (F("stationCheckDist   : "));
  ShowMessageln(stationCheckDist);
  ShowMessage  (F("UseBumperDock      : "));
  ShowMessageln(UseBumperDock);
  ShowMessage  (F("dockingSpeed       : "));
  ShowMessageln(dockingSpeed);
  ShowMessage (F("checkDockingSpeed  : "));
  ShowMessageln(checkDockingSpeed);
  ShowMessage  (F("autoResetActive    : "));
  ShowMessageln(autoResetActive);

  //watchdogReset();


  // ------ odometry --------------------------------------------------------------
  ShowMessageln(F("---------- odometry ------------------------------------------"));
  ShowMessage  (F("odometryTicksPerRevolution : "));
  ShowMessageln( odometryTicksPerRevolution);
  ShowMessage  (F("odometryTicksPerCm         : "));
  ShowMessageln( odometryTicksPerCm);
  ShowMessage  (F("odometryWheelBaseCm        : "));
  ShowMessageln( odometryWheelBaseCm);


  // ----- RFID ----------------------------------------------------------------------
  ShowMessageln(F("---------- RFID ----------- "));
  ShowMessage  (F("rfidUse         : "));
  ShowMessageln(rfidUse);
  //watchdogReset();
  // ----- RASPBERRY PI --------------
  ShowMessageln(F("---------- RASPBERRY PI------ "));
  ShowMessage  (F("RaspberryPIUse  : "));
  ShowMessageln(RaspberryPIUse);
  // ----- MQTT --------------
  ShowMessageln(F("---------- MQTT        ------ "));
  ShowMessage  (F("useMqtt  : "));
  ShowMessageln(useMqtt);


  // ----- GPS ----------------------------------------------------------------------
  ShowMessageln(F("---------- GPS -----------------------------------------------"));
  ShowMessage  (F("gpsUse                                     : "));
  ShowMessageln(gpsUse);
  ShowMessage  (F("stuckIfGpsSpeedBelow                       : "));
  ShowMessageln(stuckIfGpsSpeedBelow);
  ShowMessage  (F("gpsSpeedIgnoreTime                         : "));
  ShowMessageln(gpsSpeedIgnoreTime);





  // ----- other ----------------------------------------------------
  ShowMessageln(F("---------- other ------------"));
  ShowMessage  (F("buttonUse              : "));
  ShowMessageln(buttonUse);
  ShowMessage  (F("mowPatternDurationMax  : "));
  ShowMessageln(mowPatternDurationMax);

  //watchdogReset();

  // ----- user-defined switch ----------------------------------------
  ShowMessageln(F("---------- user-defined switch -------"));
  ShowMessage  (F("userSwitch1       : "));
  ShowMessageln(userSwitch1);
  ShowMessage  (F("userSwitch2       : "));
  ShowMessageln(userSwitch2);
  ShowMessage  (F("userSwitch3       : "));
  ShowMessageln(userSwitch3);
  //watchdogReset();
  // ----- timer --------------------------------------------------------------------
  ShowMessageln(F("---------- timer ----------- "));
  ShowMessage  (F("timerUse       : "));
  ShowMessageln(timerUse);




  //watchdogReset();
  // -------robot stats--------------------------------------------------------------
  ShowMessageln(F("---------- robot stats ---------------------------------------"));
  ShowMessage  (F("statsMowTimeMinutesTrip                    : "));
  ShowMessageln(statsMowTimeMinutesTrip);
  ShowMessage  (F("statsMowTimeMinutesTotal                   : "));
  ShowMessageln(statsMowTimeMinutesTotal);
  ShowMessage  (F("statsBatteryChargingCounterTotal           : "));
  ShowMessageln(statsBatteryChargingCounterTotal);
  ShowMessage  (F("statsBatteryChargingCapacityTrip in mAh    : "));
  ShowMessageln(statsBatteryChargingCapacityTrip);
  ShowMessage  (F("statsBatteryChargingCapacityTotal in Ah    : "));
  ShowMessageln(statsBatteryChargingCapacityTotal / 1000);
  ShowMessage  (F("statsBatteryChargingCapacityAverage in mAh : "));
  ShowMessageln(statsBatteryChargingCapacityAverage);
  //watchdogReset();
  //return;


}


void Robot::saveUserSettings() {
  ShowMessageln(F("START TO SAVE USER SETTINGS PLEASE WAIT"));
  loadSaveUserSettings(false);

}


void Robot::deleteUserSettings() {
  int addr = ADDR_USER_SETTINGS;
  ShowMessageln(F("ALL USER SETTINGS ARE DELETED PLEASE RESTART THE DUE"));
  eewrite(addr, (short)0); // magic
}

void Robot::deleteRobotStats() {
  statsMowTimeMinutesTrip = statsMowTimeMinutesTotal = statsBatteryChargingCounterTotal =
                              statsBatteryChargingCapacityTotal = statsBatteryChargingCapacityTrip = 0;
  loadSaveRobotStats(false);
  ShowMessageln(F("ALL ROBOT STATS ARE DELETED"));
}

void Robot::addErrorCounter(byte errType) {
  // increase error counters (both temporary and maximum error counters)
  if (errorCounter[errType] < 255) errorCounter[errType]++;
  if (errorCounterMax[errType] < 255) errorCounterMax[errType]++;
}

void Robot::resetErrorCounters() {
  ShowMessageln(F("resetErrorCounters"));
  for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = errorCounterMax[i] = 0;
  loadSaveErrorCounters(false);
  resetMotorFault();
}

void Robot::resetMotorFault() {
  /*
    if (digitalRead(pinMotorLeftFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor left fault"));
    }
    if  (digitalRead(pinMotorRightFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor right fault"));
    }
    if (digitalRead(pinMotorMowFault) == LOW) {
    digitalWrite(pinMotorMowEnable, LOW);
    digitalWrite(pinMotorMowEnable, HIGH);
    ShowMessageln(F("Reset motor mow fault"));
    }
  */
}

/*
  void Robot::readMowerSensor(char type) {

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
        ShowMessageln("RTC data error!");
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



void Robot::checkErrorCounter() {
  if (millis() >= nextTimeErrorCounterReset) {
    // reset all temporary error counters after 30 seconds (maximum error counters still continue to count)
    for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = 0;
    nextTimeErrorCounterReset = millis() + 30000; // 30 sec
  }
  if (stateCurr != STATE_OFF) {
    for (int i = 0; i < ERR_ENUM_COUNT; i++) {
      // set to fatal error if any temporary error counter reaches 10
      if (errorCounter[i] > 10) {
        ShowMessage("Error Counter > 10 for counter num ");
        ShowMessageln(i);
        setNextState(STATE_ERROR, 0);
      }
    }
  }
}
void Robot::teensyBootLoader() {
  delayWithWatchdog(8000); //wait for pyteensy to stop and start pi teensy loader
  asm("bkpt #251");

}

void Robot::powerOff_pcb() {
  if (RaspberryPIUse) {
    ShowMessageln(F("PCB power OFF after 30 secondes Wait Until PI Stop "));
    MyRpi.sendCommandToPi("PowerOffPi");
    delayWithWatchdog(30000);//wait 30Sec  until pi is OFF or the USB native power again the due and the undervoltage never switch OFF
  }
  else
  {
    ShowMessageln(F("PCB power OFF"));
  }
  setBeeper(3000, 3000, 0, 2000, 0);
  loadSaveErrorCounters(false); // saves error counters
  loadSaveRobotStats(false);    // saves robot stats
  ShowMessageln(F("BATTERY switching OFF"));
  delayWithWatchdog(2000);
  Serial1.end();//disconnect BT
  delayWithWatchdog(3000);
  setActuator(ACT_BATTERY_SW, 0);  // switch off battery
  //end of working

}

void Robot::autoReboot() {
  //this feature use the watchdog to perform a restart of the due
  if (RaspberryPIUse) {
    ShowMessageln(F("Due reset after 1 secondes, send a command to Pi for restart also"));
    MyRpi.sendCommandToPi("RestartPi");
  }
  else
  {
    ShowMessageln(F("Due reset after 1 secondes"));
  }
  delay(1000);
  //watchdogReset();
  delay(30000); // this IS USED to force watchdog to power off the PCB by generate error.
}



// ---- RC (interrupt) --------------------------------------------------------------
// RC remote control helper
// convert ppm time (us) to percent (-100..+100)
// ppmtime: zero stick pos: 1500 uS
//          right stick pos: 2000 uS
//          left stick pos: 1000 uS
int Robot::rcValue(int ppmTime) {
  int value = (int) (((double)((ppmTime) - 1500)) / 3.4);
  if ((value < 5) && (value > -5)) value = 0;  //  ensures exact zero position
  return value;
}


// sets mower motor actuator
// - ensures that the motor is not switched to 100% too fast (motorMowAccel)
// - ensures that the motor voltage is not higher than motorMowSpeedMaxPwm
void Robot::setMotorMowPWM(int pwm, boolean useAccel) {

  unsigned long TaC = millis() - lastSetMotorMowSpeedTime;    // sampling time in millis
  lastSetMotorMowSpeedTime = millis();
  if (TaC > 1000) TaC = 1;
  //bber13
  if ( (!useAccel)) {  //accel is not use when stop the blade on tilt
    motorMowPWMCurr = pwm;
  }
  else {
    motorMowPWMCurr += int(TaC) * (pwm - motorMowPWMCurr) / motorMowAccel;
  }
  setActuator(ACT_MOTOR_MOW, min(motorMowSpeedMaxPwm, max(0, motorMowPWMCurr)));
}

// ensures that the motors (and gears) are not switched to 0% (or 100%) too fast (motorAccel)
void Robot::setMotorPWM(int pwmLeft, int pwmRight) {
  int TaC = int(millis() - lastSetMotorSpeedTime);    // sampling time in millis
  lastSetMotorSpeedTime = millis();
  if (TaC > 1000) TaC = 1;

  if (stateCurr != STATE_OFF) {
    /*
      ShowMessage(stateNames[stateCurr]);
      ShowMessage(" Voeux a ");
      ShowMessage (millis());
      ShowMessage(" TaC=");
      ShowMessage (TaC);
      ShowMessage(" Useaccel=");
      ShowMessage (useAccel);
      ShowMessage(" pwmLeft ");
      ShowMessage (pwmLeft);
      ShowMessage(" pwmRight ");
      ShowMessage (pwmRight);


      ShowMessage(" OdoRight ");
      ShowMessage (odometryRight);
      ShowMessage(" OdoLeft ");
      ShowMessageln (odometryLeft);

      ShowMessage ("  motorLeftZeroTimeout : ");
      ShowMessage (motorLeftZeroTimeout);

      ShowMessage(" motorLeftPWMCurr=");
      ShowMessage(motorLeftPWMCurr);
      ShowMessage(" motorRightPWMCurr=");
      ShowMessageln (motorRightPWMCurr);
    */
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ( ((pwmLeft < 0) && (motorLeftPWMCurr > 0)) || ((pwmLeft > 0) && (motorLeftPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON LEFT MOTOR ");
      ShowMessage("  motorLeftPWMCurr=");
      ShowMessage (motorLeftPWMCurr);
      ShowMessage("  pwmLeft=");
      ShowMessage (pwmLeft);
      ShowMessage(" state ");
      ShowMessageln(stateNames[stateCurr]);

    }
    if (motorLeftZeroTimeout != 0) pwmLeft = motorLeftPWMCurr - motorLeftPWMCurr * ((float)TaC) / 200.0; // reduce speed
  }
  if ( ((pwmRight < 0) && (motorRightPWMCurr > 0)) || ((pwmRight > 0) && (motorRightPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON RIGHT MOTOR ");
      ShowMessage("  motorRightPWMCurr=");
      ShowMessage (motorRightPWMCurr);
      ShowMessage("  pwmRight=");
      ShowMessage (pwmRight);
      ShowMessage("  On state ");
      ShowMessageln(stateNames[stateCurr]);
    }
    if (motorRightZeroTimeout != 0) pwmRight = motorRightPWMCurr - motorRightPWMCurr * ((float)TaC) / 200.0; // reduce speed

  }

  motorLeftPWMCurr = pwmLeft;
  motorRightPWMCurr = pwmRight;

  if (abs(motorLeftRpmCurr) < 1) motorLeftZeroTimeout = max(0, ((int)(motorLeftZeroTimeout - TaC)) );
  else motorLeftZeroTimeout = 1000;
  if (abs(motorRightRpmCurr) < 1) motorRightZeroTimeout = max(0, ((int)(motorRightZeroTimeout - TaC)) );
  else motorRightZeroTimeout = 1000;

  if (stateCurr != STATE_OFF) {
    /*
      ShowMessage(" result ");
      ShowMessage (millis());
      ShowMessage(" Right/Left ");
      ShowMessage (motorRightPWMCurr);
      ShowMessage(" / ");
      ShowMessageln (motorLeftPWMCurr);
    */

  }

  if (motorLeftSwapDir)  // swap pin polarity?
    setActuator(ACT_MOTOR_LEFT, -motorLeftPWMCurr);
  else
    setActuator(ACT_MOTOR_LEFT, motorLeftPWMCurr);
  if (motorRightSwapDir)   // swap pin polarity?
    setActuator(ACT_MOTOR_RIGHT, -motorRightPWMCurr);
  else
    setActuator(ACT_MOTOR_RIGHT, motorRightPWMCurr);

}



void Robot::OdoRampCompute() { //execute only one time when a new state execution
  //Compute the accel duration (very important for small distance)
  //Compute when you need to brake the 2 wheels to stop at the ODO
  //Compute the estimate duration of the state so can force next state if the mower is stuck



  stateStartOdometryLeft = odometryLeft;
  stateStartOdometryRight = odometryRight;

  motorRightPID.reset();
  PwmRightSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorRightSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  PwmLeftSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorLeftSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  //try to find when we need to brake the wheel (depend of the distance)

  int  distToMoveLeft;
  int  distToMoveRight;
  distToMoveLeft = abs(stateStartOdometryLeft - stateEndOdometryLeft);
  distToMoveRight = abs(stateStartOdometryRight - stateEndOdometryRight);
  //left wheel
  if (distToMoveLeft >= odometryTicksPerRevolution)  {
    OdoStartBrakeLeft =  odometryTicksPerRevolution / 2; //si plus d'1 tour on freine dans la moitie du dernier tour
    SpeedOdoMaxLeft = PwmLeftSpeed; //valeur de vitesse max en fonction de la distance a parcourir
  }
  else {  // si moins d 1 tour
    if (UseAccelLeft && UseBrakeLeft) { //need 2 ramp
      OdoStartBrakeLeft = distToMoveLeft / 2; //on freine a la moitie de la distance a parcourir
      if (PwmLeftSpeed <= 0) {
        // SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      }
      else {
        //SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
      }
    }
    else
    { //need 1 ramp
      OdoStartBrakeLeft = distToMoveLeft ; //on freine sur toute la distance a parcourir
      if (PwmLeftSpeed <= 0) SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
    }
  }

  //right wheel
  if (distToMoveRight >= odometryTicksPerRevolution) { //more than 1 rev
    OdoStartBrakeRight =  odometryTicksPerRevolution / 2;
    SpeedOdoMaxRight = PwmRightSpeed;
  }
  else {  //if less than 1 rev right wheel
    if (UseAccelRight && UseBrakeRight) {
      OdoStartBrakeRight = distToMoveRight / 2; //on freine a la moitie de la distance a parcourir
      if (PwmRightSpeed <= 0) {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      }
      else {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
      }
    }
    else
    {
      OdoStartBrakeRight = distToMoveRight ; //on freine sur toute la distance a parcourir
      if (PwmRightSpeed <= 0) SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
    }
  }


  //compute the approximative moving time in millis()
  //Need to compute in 2 times to avoid overflow  !!!!!

  movingTimeLeft = 1000 * distToMoveLeft / motorTickPerSecond ;
  movingTimeLeft = movingTimeLeft * motorSpeedMaxPwm / abs(SpeedOdoMaxLeft);
  //bber500
  if (movingTimeLeft < 4000 ) movingTimeLeft = 4000;

  //for small mouvement need to increase duration
  movingTimeRight = 1000 * distToMoveRight / motorTickPerSecond ;
  movingTimeRight = movingTimeRight * motorSpeedMaxPwm / abs(SpeedOdoMaxRight);
  //bber500 reduce movement shock
  if (movingTimeRight < 4000 ) movingTimeRight = 4000;

  //for small mouvement need to reduce the accel duration
  if (movingTimeLeft >= motorOdoAccel) accelDurationLeft = motorOdoAccel;
  else   accelDurationLeft =  movingTimeLeft / 2;
  if (movingTimeRight >= motorOdoAccel) accelDurationRight = motorOdoAccel;
  else   accelDurationRight =  movingTimeRight / 2;
  if (statusCurr == TESTING) {  //avoid maxduration stop when use test Odo with Pfod
    MaxOdoStateDuration = 30000 + max(movingTimeRight, movingTimeLeft); //add 30 secondes to the max moving duration of the 2 wheels
  }
  else
  {
    MaxOdoStateDuration = 3000 + max(movingTimeRight, movingTimeLeft); //add 3 secondes to the max moving duration of the 2 wheels
  }
  //check to set the correct heading
  imuDriveHeading = imu.ypr.yaw / PI * 180; //normal mowing heading

  if (statusCurr == BACK_TO_STATION) {  //possible heading change
    imuDriveHeading = periFindDriveHeading / PI * 180;
  }
  if (statusCurr == REMOTE) {   //possible heading change
    imuDriveHeading = remoteDriveHeading / PI * 180;
  }

  /*
    ShowMessage(" **************** compute  at  ");
    ShowMessageln(millis());
    ShowMessage(" UseAccelRight ");
    ShowMessage(UseAccelRight);
    ShowMessage(" UseBrakeRight ");
    ShowMessage(UseBrakeRight);
    ShowMessage(" UseAccelLeft ");
    ShowMessage(UseAccelLeft);
    ShowMessage(" UseBrakeLeft ");
    ShowMessage(UseBrakeLeft);
    ShowMessage(" distToMoveLeft ");
    ShowMessage(distToMoveLeft);
    ShowMessage(" movingTimeLeft ");
    ShowMessage(movingTimeLeft);
    ShowMessage("ms movingTimeRight ");
    ShowMessageln(movingTimeRight);
    ShowMessage("accelDurationLeft ");
    ShowMessage(accelDurationLeft);
    ShowMessage("ms accelDurationRight ");
    ShowMessageln(accelDurationRight);

    ShowMessage (F(stateNames[stateNext]));
    ShowMessage(" RightSpeedRpmSet ");
    ShowMessage(motorRightSpeedRpmSet);
    ShowMessage("  PwmRightSpeed ");
    ShowMessage(PwmRightSpeed);
    ShowMessage("  SpeedOdoMaxRight ");
    ShowMessageln(SpeedOdoMaxRight);






    ShowMessage("OdoStartBrakeLeft ");
    ShowMessage(OdoStartBrakeLeft);
    ShowMessage("Ticks OdoStartBrakeRight ");
    ShowMessageln(OdoStartBrakeRight);
    ShowMessage("MaxOdoStateDuration ");
    ShowMessage(MaxOdoStateDuration);
    ShowMessageln(" ms");
  */

}



void Robot::motorControlOdo() {

  // call to reach a ODO cible on left AND right wheel so they don't stop at the same time accel and slow are used to smooth the movement of the mower
  //Stop motor independently when the cible is reach
  //
  if (UseBrakeLeft && (motorLeftSpeedRpmSet >= 0) && (stateEndOdometryLeft - odometryLeft <= -10)) {//Forward left need -10 because when stop the ticks can move in+ or- so do not stop before
    moveLeftFinish = true;
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (UseBrakeRight && (motorRightSpeedRpmSet >= 0) && (stateEndOdometryRight - odometryRight <= -10)) {//right
    moveRightFinish = true;
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  //Reverse
  if (UseBrakeRight && (motorRightSpeedRpmSet <= 0) && (stateEndOdometryRight - odometryRight >= 10)) {//right
    moveRightFinish = true;
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  if (UseBrakeLeft && (motorLeftSpeedRpmSet <= 0) && (stateEndOdometryLeft - odometryLeft >= 10)) {//left
    moveLeftFinish = true;
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (millis() < nextTimeMotorOdoControl) return;
  nextTimeMotorOdoControl = millis() + 15;

  //LEFT WHEEL

  leftSpeed = PwmLeftSpeed ; //Set first to Normal speed and stay like this if not change  by accel or brake so limit the compute time
  if (motorLeftSpeedRpmSet > 0) { //forward left wheel --------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft > stateEndOdometryLeft - (OdoStartBrakeLeft))) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed > SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (leftSpeed < SpeedOdoMin) leftSpeed = SpeedOdoMin; //Minimum speed to be sure the mower is always moving before stop
  }

  if (motorLeftSpeedRpmSet < 0) { //reverse left wheel ----------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft < stateEndOdometryLeft + OdoStartBrakeLeft)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed < SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (abs(leftSpeed) < SpeedOdoMin) leftSpeed = -SpeedOdoMin;
  }

  //  RIGHT WHEEL
  rightSpeed = PwmRightSpeed ; //Normal speed

  if (motorRightSpeedRpmSet > 0) { //forward Right wheel -----------------------------------------------------------------------------
    // ShowMessage(" FR rotate ");
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight > stateEndOdometryRight - OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (rightSpeed > SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (rightSpeed < SpeedOdoMin) rightSpeed = SpeedOdoMin;
  }
  if (motorRightSpeedRpmSet < 0) { //reverse Right wheel ------------------------------------------------------------------------------
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight < stateEndOdometryRight +  OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }

    if (rightSpeed < SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (abs(rightSpeed) < SpeedOdoMin) rightSpeed = -SpeedOdoMin;

  }

  //DRIVE IN STRAIGHT LINE
  if (stateCurr == STATE_FORWARD_ODO || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_DRIVE1_TO_NEWAREA) || (stateCurr == STATE_DRIVE2_TO_NEWAREA))  { //PID compute to accel or brake the wheel to drive straight
    motorRightPID.Kp = motorLeftPID.Kp;
    motorRightPID.Ki = motorLeftPID.Ki;
    motorRightPID.Kd = motorLeftPID.Kd;
    // USE THE IMU
    if ((imuUse) && (mowPatternCurr == MOW_LANES) && (stateCurr == STATE_FORWARD_ODO)) { //if mow by lane need different cible
      YawActualDeg = imu.ypr.yaw / PI * 180;
      if (laneUseNr == 1) {   //from -45 to 45 deg
        yawCiblePos = yawSet1 ;
        // ImuPidCiblePos= yawSet1+360;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane1RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane1RollLeft;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollLeft);
        }

      }
      if (laneUseNr == 2) {   //from 45 to 135 deg
        yawCiblePos = yawSet2;
        //ImuPidCiblePos= yawSet2;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane2RollRight;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane2RollLeft;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollLeft);
        }
      }
      if (laneUseNr == 3) {    //from 135 to -135 or 225 deg
        yawCiblePos = yawSet3;
        // ImuPidCiblePos= imu.rotate360(yawSet3);
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane3RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane3RollLeft;
          //  ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollLeft);
        }
      }

      if ((imu.ypr.yaw / PI * 180) > 0 ) imuDriveHeading = yawCiblePos;
      else imuDriveHeading = yawCibleNeg;
      imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
      imuDirPID.w = 0;
      imuDirPID.y_min = -motorSpeedMaxPwm / 2;
      imuDirPID.y_max = motorSpeedMaxPwm / 2;
      imuDirPID.max_output = motorSpeedMaxPwm / 2;
      imuDirPID.compute();
      /*
            if ((millis() - stateStartTime) < 1000) { // do not use rpm adjust during acceleration
              //bber402

              rightSpeed =  rightSpeed - (66 - (millis() - stateStartTime) / 30);
              leftSpeed =  leftSpeed - (66 - (millis() - stateStartTime) / 30);
              if (rightSpeed < 0 ) rightSpeed = 0;
              if (leftSpeed < 0 ) leftSpeed = 0;
            }
            else //adjust rpm speed only after 1 seconde
            {
      */
      //bber400 //adjust RPM speed
      //PID version
      motorRightPID.x = motorRightRpmCurr; //16/10/22
      //motorRightPID.w = motorSpeedMaxRpm;
      motorRightPID.w = motorRightSpeedRpmSet;
      motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
      motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
      motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
      motorRightPID.compute();
      //ShowMessageln(motorRightPID.y);
      motorRpmCoeff = (100 + motorRightPID.y) / 100;
      if (motorRpmCoeff < 0.10) motorRpmCoeff = 0.10;
      if (motorRpmCoeff > 2.00) motorRpmCoeff = 2.00;
/*
 * Speed control loop to build !!!
      if ((motorRightSpeedRpmSet / motorRightRpmCurr) < 0.8 ) { //speed real is 20 % too high need a brake
        Serial.print("R speed  ");
        Serial.print((motorRightSpeedRpmSet / motorRightRpmCurr));
        Serial.print(" / ");
        Serial.println(motorRightPID.y);
      }

      if ((motorLeftSpeedRpmSet / motorLeftRpmCurr) < 0.8 ) {
        Serial.print("L speed  ");
        Serial.print((motorLeftSpeedRpmSet / motorLeftRpmCurr));
        Serial.print(" / ");
        Serial.println(motorRightPID.y);
      }
*/
      

      if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
        motorRpmCoeff = 1;
      }
      if (highGrassDetect) motorRpmCoeff = highGrassSpeedCoeff; //reduce speed when mower detect high grass
      rightSpeed = motorRpmCoeff * (rightSpeed + imuDirPID.y / 2);
      leftSpeed =  motorRpmCoeff * (leftSpeed - imuDirPID.y / 2);




    }
    else
      //// NORMAL MOWING OR PERIFIND
    {
      if (imuUse) /// use the IMU for straight line
      {
        YawActualDeg = imu.ypr.yaw / PI * 180;
        // if(abs(YawActualDeg) >90) YawMedianDeg = imu.rotate360(YawActualDeg);
        // else YawMedianDeg= YawActualDeg+90;

        imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
        imuDirPID.w = 0;
        imuDirPID.y_min = -motorSpeedMaxPwm / 2;
        imuDirPID.y_max = motorSpeedMaxPwm / 2;
        imuDirPID.max_output = motorSpeedMaxPwm / 2;
        imuDirPID.compute();

        //bber400 //adjust RPM speed
        //PID version
        motorRightPID.x = motorRightRpmCurr;
        //motorRightPID.w = motorSpeedMaxRpm;
        motorRightPID.w = motorRightSpeedRpmSet;//16/10/22
        motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
        motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
        motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
        motorRightPID.compute();
        //ShowMessageln(motorRightPID.y);
        motorRpmCoeff = (100 + motorRightPID.y) / 100;
        if (motorRpmCoeff < 0.10) motorRpmCoeff = 0.10;
        if (motorRpmCoeff > 2.00) motorRpmCoeff = 2.00;

        if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
          motorRpmCoeff = 1;
        }

        rightSpeed = motorRpmCoeff * (rightSpeed + imuDirPID.y / 2);
        leftSpeed =  motorRpmCoeff * (leftSpeed - imuDirPID.y / 2);

      }
      else   /// use only the odometry  for straight line
      {
        if (millis() >= nextTimePidCompute) { //to go in straight line need to compute only each 200 milliseconde and add the dif to one wheel
          nextTimePidCompute = millis() + 200;
          //stateStartOdometryLeft = stateStartOdometryLeft + ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft)); // very important change the odo to retrieve the line to avoid drift
          motorRightPID.x = ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft));
          motorRightPID.w = 0;
          motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
          motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
          motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
          motorRightPID.compute();
          rightSpeed =  rightSpeed + motorRightPID.y / 2;
          leftSpeed =  leftSpeed - motorRightPID.y / 2;
        }
      }
    }

    //bber200

    //bber200 reduce perimeter speed only if both perimeter and sonar are actif
    if (perimeterSpeedCoeff == 1) {
      rightSpeed = rightSpeed * sonarSpeedCoeff;
      leftSpeed = leftSpeed * sonarSpeedCoeff;
    }
    else
    {
      rightSpeed = rightSpeed * perimeterSpeedCoeff;
      leftSpeed = leftSpeed * perimeterSpeedCoeff;
    }

    if (rightSpeed > 255) rightSpeed = 255;
    if (leftSpeed > 255) leftSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;
    if (leftSpeed < 0) leftSpeed = 0;

  }

  if (stateCurr != STATE_OFF) {
    /*
      if (perimeterSpeedCoeff != 1) {
      ShowMessageln(perimeterSpeedCoeff);
      }

        ShowMessage(millis());
        ShowMessage(" Moving Average Dist= ");
        ShowMessage(currDistToDrive);
        ShowMessage(" ODO **** Lspeed= ");
        ShowMessage(leftSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryLeft);
        ShowMessage("/");
        ShowMessage(odometryLeft);
        ShowMessage("/");
        ShowMessage(stateEndOdometryLeft);
        ShowMessage(" ************************* Rspeed= ");
        ShowMessage(rightSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryRight);
        ShowMessage("/");
        ShowMessage(odometryRight);
        ShowMessage("/");
        ShowMessage(stateEndOdometryRight);
        ShowMessage(" PID reel ");
        ShowMessage(motorRightPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(motorRightPID.y);
        ShowMessage("IMU ***** Line use ");
        ShowMessage(laneUseNr);
        ShowMessage(" imuDriveHeading ");
        ShowMessage(imuDriveHeading);
        ShowMessage(" YawMedianDeg ");
        ShowMessage(YawMedianDeg);
        ShowMessage(" YawActualDeg ");
        ShowMessage(YawActualDeg);
        ShowMessage(" correctRight ");
        ShowMessage(correctRight);
        ShowMessage(" correctLeft ");
        ShowMessage(correctLeft);
        ShowMessage(" PID reel ");
        ShowMessage(imuDirPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(imuDirPID.y);
        ShowMessage(" imu.ypr.yaw ");
        ShowMessageln(imu.ypr.yaw);
    */
  }


  setMotorPWM(leftSpeed, rightSpeed);
}






// PID controller: track perimeter
void Robot::motorControlPerimeter() {

  if (millis() < nextTimeMotorPerimeterControl) return;
  nextTimeMotorPerimeterControl = millis() + 15; //bb read the perimeter each 15 ms
  //never stop the PID compute while turning for the new transition
  //use the perimeterMagLeft as cible to smooth the tracking
  //Value reference perimeterMagMaxValue , maybe need to be calculate in mower setting up procedure



  perimeterPID.x = 5 * (double(perimeterMagLeft) / perimeterMagMaxValue);
  if (perimeterInsideLeft)  perimeterPID.w = -0.5;
  else     perimeterPID.w = 0.5;



  perimeterPID.y_min = -ActualSpeedPeriPWM ;
  perimeterPID.y_max = ActualSpeedPeriPWM ;
  perimeterPID.max_output = ActualSpeedPeriPWM ;
  perimeterPID.compute();

  if ((millis() > stateStartTime + 10000) && (millis() > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut)) {
    // robot is wheel-spinning while tracking => roll to get ground again

    if (trakBlockInnerWheel == 0) {
      if (perimeterInsideLeft) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = -ActualSpeedPeriPWM / 2;
      }
      else {
        rightSpeedperi = -ActualSpeedPeriPWM / 2;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (trakBlockInnerWheel == 1) {
      if (perimeterInsideLeft) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = 0;
      }
      else {
        rightSpeedperi = 0;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SEARCH;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
    if (track_ClockWise) {
      setMotorPWM( leftSpeedperi, rightSpeedperi);
    }
    else {
      setMotorPWM( rightSpeedperi, leftSpeedperi);
    }

    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInsideLeft) {
        ShowMessageln("Tracking Fail and we are inside, So start to find again the perimeter");
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      else
      {
        ShowMessageln("Tracking Fail and we are outside, So start to roll to find again the perimeter");
        if (track_ClockWise) {
          rollDir = 0; //le 02/04/22 need to check if it's the correct roll dir
        }
        else {
          rollDir = 1;
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
      }

    }
    return;
  }


  if ((millis() - lastTimeForgetWire ) < trackingPerimeterTransitionTimeOut) {
    //PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    PeriCoeffAccel = (3000.00 - (millis() - lastTimeForgetWire)) / 1000.00 ;
    if (PeriCoeffAccel < 1.00) PeriCoeffAccel = 1.00;
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 +  perimeterPID.y / PeriCoeffAccel));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 -  perimeterPID.y / PeriCoeffAccel));
    //bber30 we are in sonartrigger ,so maybe near station , so avoid 1 wheel reverse because station check is forward
    if (ActualSpeedPeriPWM != MaxSpeedperiPwm)
    {
      if (rightSpeedperi < 0) rightSpeedperi = 0;
      if (leftSpeedperi < 0) leftSpeedperi = 0;
    }

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SLOW;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }
  else
  {
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5   + perimeterPID.y));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  - perimeterPID.y));

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("FAST;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }

  //bb2
  if ((millis() - stateStartTime ) < 2000) { //at the start of the tracking accelerate slowly during 2 secondes
    //leftSpeedperi = leftSpeedperi - (66 - (millis() - stateStartTime) / 30);
    leftSpeedperi = int(leftSpeedperi * ((millis() - stateStartTime) / 2000));
    //bber300
    if (leftSpeedperi < SpeedOdoMin) leftSpeedperi = SpeedOdoMin;
    //rightSpeedperi = rightSpeedperi - (66 - (millis() - stateStartTime) / 30);
    rightSpeedperi = int(rightSpeedperi * ((millis() - stateStartTime) / 2000));
    if (rightSpeedperi < SpeedOdoMin) rightSpeedperi = SpeedOdoMin;
  }

  if (track_ClockWise) {
    setMotorPWM( leftSpeedperi, rightSpeedperi);
  }
  else {
    setMotorPWM( rightSpeedperi, leftSpeedperi);
  }

  if (abs(perimeterMagLeft) < perimeterMagMaxValue / 4) { //250 can be replace by timedOutIfBelowSmag to be tested
    perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime if perfect sthraith line

  }
}







void Robot::motorControlPerimeter2Coil() {
  if (millis() < nextTimeMotorPerimeterControl) return;
  nextTimeMotorPerimeterControl = millis() + 15; //bb read the perimeter each 15 ms
  //never stop the PID compute while turning for the new transition
  //use the PerimeterMagRight as cible to smooth the tracking
  //Value reference perimeterMagMaxValue , maybe need to be calculate in mower setting up procedure


  perimeterPID.x = 5 * (double(perimeterMagRight) / perimeterMagMaxValue);
  if (perimeterInsideRight)  perimeterPID.w = -0.5;
  else     perimeterPID.w = 0.5;



  perimeterPID.y_min = -ActualSpeedPeriPWM ;
  perimeterPID.y_max = ActualSpeedPeriPWM ;
  perimeterPID.max_output = ActualSpeedPeriPWM ;
  perimeterPID.compute();


  if (!(perimeterInsideLeft) || ((millis() > stateStartTime + 10000) && (millis() > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut))) {
    // robot is wheel-spinning while tracking => roll to get ground again

    if (trakBlockInnerWheel == 0) {
      if (perimeterInsideRight) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = -ActualSpeedPeriPWM / 2;
      }
      else {
        rightSpeedperi = -ActualSpeedPeriPWM / 2;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (trakBlockInnerWheel == 1) {
      if (perimeterInsideRight) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = 0;
      }
      else {
        rightSpeedperi = 0;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SEARCH;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";   ");
      ShowMessage (perimeterPID.x);
      ShowMessage("    ;");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
    if (track_ClockWise) {
      setMotorPWM( leftSpeedperi, rightSpeedperi);
    }
    else {
      setMotorPWM( rightSpeedperi, leftSpeedperi);
    }

    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInsideRight) {
        ShowMessageln("Tracking Fail and we are inside, So start to find again the perimeter");
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      else
      {
        ShowMessageln("Tracking Fail and we are outside, So start to roll to find again the perimeter");
        if (track_ClockWise) {
          rollDir = 0; //le 02/04/22 need to check if it's the correct roll dir
        }
        else {
          rollDir = 1;
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
      }

    }
    return;
  }


  if ((millis() - lastTimeForgetWire ) < trackingPerimeterTransitionTimeOut) {
    //PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    PeriCoeffAccel = (3000.00 - (millis() - lastTimeForgetWire)) / 1000.00 ;
    if (PeriCoeffAccel < 1.00) PeriCoeffAccel = 1.00;
    rightSpeedperi = max(0,  ActualSpeedPeriPWM  + (perimeterPID.y ) / PeriCoeffAccel);
    leftSpeedperi = max(0,  ActualSpeedPeriPWM  - (perimeterPID.y ) / PeriCoeffAccel);
    //bber30 we are in sonartrigger ,so maybe near station , so avoid 1 wheel reverse because station check is forward
    if (ActualSpeedPeriPWM != MaxSpeedperiPwm)
    {
      if (rightSpeedperi < 0) rightSpeedperi = 0;
      if (leftSpeedperi < 0) leftSpeedperi = 0;
    }

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SLOW;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";     ");
      ShowMessage (perimeterPID.x);
      ShowMessage("     ;     ");
      ShowMessage(perimeterPID.y);
      ShowMessage("   ;");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }
  else
  {
    rightSpeedperi = max(0, ActualSpeedPeriPWM +  perimeterPID.y);
    leftSpeedperi = max(0, ActualSpeedPeriPWM - perimeterPID.y);

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("FAST;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";    ");
      ShowMessage (perimeterPID.x);
      ShowMessage("   ;  ");
      ShowMessage(perimeterPID.y);
      ShowMessage("   ;");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }

  //bb2
  if ((millis() - stateStartTime ) < 2000) { //at the start of the tracking accelerate slowly during 2 secondes
    //leftSpeedperi = leftSpeedperi - (66 - (millis() - stateStartTime) / 30);
    leftSpeedperi = int(leftSpeedperi * ((millis() - stateStartTime) / 2000));
    //bber300
    if (leftSpeedperi < SpeedOdoMin) leftSpeedperi = SpeedOdoMin;
    //rightSpeedperi = rightSpeedperi - (66 - (millis() - stateStartTime) / 30);
    rightSpeedperi = int(rightSpeedperi * ((millis() - stateStartTime) / 2000));
    if (rightSpeedperi < SpeedOdoMin) rightSpeedperi = SpeedOdoMin;
  }

  if (track_ClockWise) {
    setMotorPWM( leftSpeedperi, rightSpeedperi);
  }
  else {
    setMotorPWM( rightSpeedperi, leftSpeedperi);
  }

  if (abs(perimeterMagRight) < perimeterMagMaxValue / 4) { //250 can be replace by timedOutIfBelowSmag to be tested
    perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime if perfect sthraith line

  }
}



// check for odometry sensor faults
void Robot::checkOdometryFaults() {
  // if pwm > 1/3 of maxvalue the rpm need to be !=0 or it's error
  boolean leftErr = false;
  boolean rightErr = false;
  if ((stateCurr == STATE_FORWARD_ODO) &&  (millis() - stateStartTime > 8000) ) {
    // just check if odometry sensors may not be working at all
    if ( (motorLeftPWMCurr > motorSpeedMaxPwm / 3) && (abs(motorLeftRpmCurr) < 1)  )  leftErr = true;
    if ( (motorRightPWMCurr > motorSpeedMaxPwm / 3) && (abs(motorRightRpmCurr) < 1)  ) rightErr = true;
  }

  if (leftErr) {
    ShowMessage("Left odometry error: PWM=");
    ShowMessage(motorLeftPWMCurr);
    ShowMessage("\tRPM=");
    ShowMessageln(motorLeftRpmCurr);
    addErrorCounter(ERR_ODOMETRY_LEFT);
    setNextState(STATE_ERROR, 0);
  }
  if (rightErr) {
    ShowMessage("Right odometry error: PWM=");
    ShowMessage(motorRightPWMCurr);
    ShowMessage("\tRPM=");
    ShowMessageln(motorRightRpmCurr);
    addErrorCounter(ERR_ODOMETRY_RIGHT);
    setNextState(STATE_ERROR, 0);
  }

}

void Robot::motorControl() {
  if (millis() < nextTimeMotorControl) return;
  nextTimeMotorControl = millis() + 100;  // 10 at the original
  //static unsigned long nextMotorControlOutputTime = 0;

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorLeftPID.w = motorLeftSpeedRpmSet;               // SOLL
  motorRightPID.w = motorRightSpeedRpmSet;             // SOLL

  float RLdiff = motorLeftRpmCurr - motorRightRpmCurr;

  if (motorLeftSpeedRpmSet == motorRightSpeedRpmSet) {
    // line motion
    if (odoLeftRightCorrection) {
      motorLeftPID.w = motorLeftSpeedRpmSet - RLdiff / 2;
      motorRightPID.w = motorRightSpeedRpmSet + RLdiff / 2;
    }
  }
  motorLeftPID.x = motorLeftRpmCurr;                 // IST
  if ((stateCurr == STATE_OFF)) motorLeftPID.w = 0; // to be sure the motor stop when OFF
  /*
    motorLeftPID.y_min = -motorSpeedMaxPwm;        // Regel-MIN
    motorLeftPID.y_max = motorSpeedMaxPwm;     // Regel-MAX
    motorLeftPID.max_output = motorSpeedMaxPwm;    // Begrenzung
    motorLeftPID.compute();
    leftSpeed = int(motorLeftPWMCurr + motorLeftPID.y);
    if (motorLeftSpeedRpmSet > 0) leftSpeed = min( max(0, leftSpeed), motorSpeedMaxPwm);
    if (motorLeftSpeedRpmSet < 0) leftSpeed = max(-motorSpeedMaxPwm, min(0, leftSpeed));
  */

  motorLeftPID.y_min = -255;        // Regel-MIN
  motorLeftPID.y_max = 255;     // Regel-MAX
  motorLeftPID.max_output = 255;    // Begrenzung
  motorLeftPID.compute();
  leftSpeed = int(motorLeftPWMCurr + motorLeftPID.y);
  if (motorLeftSpeedRpmSet > 0) leftSpeed = min( max(0, leftSpeed), 255);
  if (motorLeftSpeedRpmSet < 0) leftSpeed = max(-255, min(0, leftSpeed));

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorRightPID.Kp = motorLeftPID.Kp;
  motorRightPID.Ki = motorLeftPID.Ki;
  motorRightPID.Kd = motorLeftPID.Kd;
  motorRightPID.x = motorRightRpmCurr;               // IST
  if ((stateCurr == STATE_OFF)) motorRightPID.w = 0; // to be sure the motor stop when OFF

  /*
    motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
    motorRightPID.y_max = motorSpeedMaxPwm;        // Regel-MAX
    motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
    motorRightPID.compute();
    rightSpeed = int(motorRightPWMCurr + motorRightPID.y);
    if (motorRightSpeedRpmSet > 0) rightSpeed = min( max(0, rightSpeed), motorSpeedMaxPwm);
    if (motorRightSpeedRpmSet < 0) rightSpeed = max(-motorSpeedMaxPwm, min(0, rightSpeed));
  */
  motorRightPID.y_min = -255;       // Regel-MIN
  motorRightPID.y_max = 255;        // Regel-MAX
  motorRightPID.max_output = 255;   // Begrenzung
  motorRightPID.compute();
  rightSpeed = int(motorRightPWMCurr + motorRightPID.y);
  if (motorRightSpeedRpmSet > 0) rightSpeed = min( max(0, rightSpeed), 255);
  if (motorRightSpeedRpmSet < 0) rightSpeed = max(-255, min(0, rightSpeed));

  if ( (abs(motorLeftPID.x) < 2) && (abs(motorLeftPID.w) < 0.1) ) leftSpeed = 0; // ensures PWM is really zero
  if ( (abs(motorRightPID.x)  < 2) && (abs(motorRightPID.w) < 0.1) ) rightSpeed = 0; // ensures PWM is really zero

  /*  if (millis() >= nextMotorControlOutputTime){
      nextMotorControlOutputTime = millis() + 200;
      ShowMessage("PID x=");
      ShowMessage(motorLeftPID.x);
      ShowMessage("\tPID w=");
      ShowMessage(motorLeftPID.w);
      ShowMessage("\tPID y=");
      ShowMessage(motorLeftPID.y);
      ShowMessage("\tPWM=");
      ShowMessageln(leftSpeed);
    }
  */

  setMotorPWM(leftSpeed, rightSpeed);


}


void Robot::motorMowControl() {
  if (millis() < nextTimeMotorMowControl) return;
  nextTimeMotorMowControl = millis() + 100;
  if (motorMowForceOff) motorMowEnable = false;
  //Auto adjust the motor speed according to cutting power (The goal is On high grass the motor rotate faster)
  //A runningmedian process is used to check each seconde the power value of mow motor
  //if power is low the speed is reduce to have a longer mowing duration and less noise.
  if (motorMowEnable) {
    motorMowPowerMedian.add(motorMowPower);
    if (motorMowPowerMedian.getCount() > 10) { //check each 1 secondes
      int prevcoeff =  motorMowPwmCoeff;
      motorMowPwmCoeff = int((100 * motorMowPowerMedian.getAverage(4)) / (0.8 * motorMowPowerMax));
      if (motorMowPwmCoeff < prevcoeff) {
        //filter on speed reduce to keep the mow speed high for longuer duration
        motorMowPwmCoeff = int((0.1) * motorMowPwmCoeff + (0.9) * prevcoeff);// use only 10% of the new value
      }

      if (motorMowPwmCoeff > 100) motorMowPwmCoeff = 100;
      if (motorMowEnable) {
        motorMowSpeedPWMSet = motorMowSpeedMinPwm + ((double)(motorMowSpeedMaxPwm - motorMowSpeedMinPwm)) * (((double)motorMowPwmCoeff) / 100.0);
      }
      if (motorMowSpeedPWMSet < motorMowSpeedMinPwm) motorMowSpeedPWMSet = motorMowSpeedMinPwm;
      if (motorMowSpeedPWMSet > motorMowSpeedMaxPwm) motorMowSpeedPWMSet = motorMowSpeedMaxPwm;
      //max speed on wire and spirale
      motorMowPowerMedian.clear();
    }
  }
  else
  {
    motorMowSpeedPWMSet = 0;
  }
  if (stateCurr == STATE_ERROR) {
    setMotorMowPWM(0, false); //stop immediatly on error (tilt etc....)
  }
  else
  {
    setMotorMowPWM(motorMowSpeedPWMSet, true);
  }
}

void Robot::resetIdleTime() {
  if (idleTimeSec == BATTERY_SW_OFF) { // battery switched off?
    ShowMessageln(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1);  // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::setBeeper(int totalDuration, int OnDuration, int OffDuration, int frequenceOn, int frequenceOff ) { // Set the variable for the beeper
  endBeepTime = millis() + totalDuration ;
  beepOnDuration = OnDuration ;
  beepOffDuration = OffDuration ;
  beepfrequenceOn = frequenceOn ;
  beepfrequenceOff = frequenceOff ;
}


void Robot::beeper() {  //beeper avoid to use the delay() fonction to not freeze the DUE
  if (millis() < nextTimeBeeper) return;
  nextTimeBeeper = millis() + 50; //maybe test with 100 if loops is too low
  if (((beepOnDuration == 0) && (beepOffDuration == 0)) || (millis() > endBeepTime)) {
    noTone(pinBuzzer);
    beepOnOFFDuration = 0;
  }
  else {
    if (beepOnOFFDuration == 0) beepOnOFFDuration = millis();
    if (millis() >= beepOnOFFDuration )
    {
      if (beepState) beepOnOFFDuration = beepOnOFFDuration + beepOnDuration;
      else beepOnOFFDuration = beepOnOFFDuration + beepOffDuration;
      beepState = !beepState;
      if (beepState) {
        tone(pinBuzzer, beepfrequenceOn);
        //   Buzzer.tone(beepfrequenceOn);
      }
      else {
        tone(pinBuzzer, beepfrequenceOff);
        //   Buzzer.tone(beepfrequenceOff);
      }
    }
  }
}


// set user-defined switches
void Robot::setUserSwitches() {

  setActuator(ACT_USER_SW1, userSwitch1);
  setActuator(ACT_USER_SW2, userSwitch2);
  setActuator(ACT_USER_SW3, userSwitch3);

}

// set user-Led
/*
  void Robot::setUserLeds() {
  setActuator(ACT_LED, userLed);
  setActuator(ACT_GREEN_LED, userGreenLed);
  setActuator(ACT_RED_LED, userRedLed);
  }
*/
void Robot::setUserOut() {
  if (invert_userOut) {
    setActuator(ACT_USER_OUT1, !userOut1);
    setActuator(ACT_USER_OUT2, !userOut2);
    setActuator(ACT_USER_OUT3, !userOut3);
    setActuator(ACT_USER_OUT4, !userOut4);
  }
  else {
    setActuator(ACT_USER_OUT1, userOut1);
    setActuator(ACT_USER_OUT2, userOut2);
    setActuator(ACT_USER_OUT3, userOut3);
    setActuator(ACT_USER_OUT4, userOut4);
  }

}

/*
  volatile float Pulse1;
  volatile float Pulse2;
  volatile float Pulse01;
  volatile float Pulse02;
*/

void Robot::OdoRightCountInt() {
  if (robot.motorRightPWMCurr >= 0 ) robot.odometryRight++; else robot.odometryRight--;
  asm("dsb");
}
void Robot::OdoLeftCountInt() {
  // Pulse01 = micros() - Pulse02;
  // if (Pulse01 > 10  ) { // debounce for 120uS
  //   Pulse02 = micros();
  if (robot.motorLeftPWMCurr > 0 ) robot.odometryLeft++; else robot.odometryLeft--;
  //}
  asm("dsb");

}

void Robot::myCallback() {
  /*
    robot.ShowMessageln("warning Watchdog detect that loops take too long duration ");
    robot.ShowMessageln("Tennsy can automaticly reboot if issue is not reset ");
  */
  Serial.println("warning Watchdog detect that loops take too long duration ");
  Serial.println("Tennsy can automaticly reboot if issue is not reset ");

  // try to stop everything imediatly
  robot.setNextState(STATE_OFF, 0);
  return;

}





// initialise odometry interrupt

void Robot::setup()  {
  setDefaultTime();
  //  mower.h start before the robot setup
  ShowMessage("++++++++++++++ Start Robot Setup at ");
  ShowMessage(millis());
  ShowMessageln(" ++++++++++++");

  //initialise the date time part
  ShowMessageln("Initialise date time library ");
  setSyncProvider(getTeensy3Time);

  if (timeStatus() != timeSet) {
    ShowMessageln("Unable to sync with the RTC");
  } else {
    ShowMessageln("RTC has set the system time");
    datetime.date.day = day();
    datetime.date.month = month();
    datetime.date.year = year();
    datetime.time.hour = hour();
    datetime.time.minute = minute();
    datetime.date.dayOfWeek = getDayOfWeek(month(), day(), year(), 0);
  }


  ShowMessageln("------------------- Initializing SD card... ---------------------");
  /*
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      ShowMessageln("SD Card failed, or not present");
      sdCardReady = false;
    }
    else {
      ShowMessageln("SD card Ok.");
      sdCardReady = true;
      sprintf(historyFilenameChar, "%02u%02u%02u%02u%02u.txt", datetime.date.year-2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);
      ShowMessage(F("Log Filename : "));
      ShowMessageln(historyFilenameChar);

    }

  */

  // see if the card is present and can be initialized:
  if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) {
    ShowMessageln("SD Card failed, or not present");
    sdCardReady = false;
  }
  else {
    sprintf(historyFilenameChar, "%02d%02d%02d%02d%02d.txt", datetime.date.year - 2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);
    sdCardReady = true;
    ShowMessage("SD card Ok  ");
    //count the number of file present on SD Card
    int totalFileOnSD = 0;
    File root = SD.open("/");
    File entry = root.openNextFile();
    while (entry) {
      entry = root.openNextFile();
      totalFileOnSD = totalFileOnSD + 1;
    }
    ShowMessage(String(totalFileOnSD));
    ShowMessageln(" Log file present on SD Card");
    if (totalFileOnSD >= 2000) {
      ShowMessageln("Warning More than 2000 file present on SD Card !!!");
      ShowMessageln("It can slow down all process !!! ");
    }
  }



  ShowMessage("Version : ");
  ShowMessageln(VER);


  if (ODOMETRY_ONLY_RISING)
  {
    attachInterrupt(digitalPinToInterrupt(pinOdometryRight), OdoRightCountInt, RISING);
    attachInterrupt(digitalPinToInterrupt(pinOdometryLeft), OdoLeftCountInt, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(pinOdometryRight), OdoRightCountInt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinOdometryLeft), OdoLeftCountInt, CHANGE);
  }




  //initialise PFOD com
  rc.initSerial(&Bluetooth, BLUETOOTH_BAUDRATE);

  if (RaspberryPIUse) MyRpi.init();









  //init of timer for factory setting
  for (int i = 0; i < MAX_TIMERS; i++) {
    timer[i].active = false;
    timer[i].startTime.hour = 0;
    timer[i].startTime.minute = 0;
    timer[i].stopTime.hour = 0;
    timer[i].stopTime.minute = 0;
    timer[i].daysOfWeek = 0;
    timer[i].startDistance = 0;
    timer[i].startArea = 1;
    timer[i].startMowPattern = 0;
    timer[i].startNrLane = 0;
    timer[i].startRollDir = 0;
    timer[i].startLaneMaxlengh = 0;
    timer[i].rfidBeacon = 0;
  }
  ActualRunningTimer = 99;
  setMotorPWM(0, 0);
  loadSaveErrorCounters(true);
  loadUserSettings();
  if (!statsOverride) loadSaveRobotStats(true);
  else loadSaveRobotStats(false);
  setUserSwitches();
  if (rfidUse) loadRfidList();
  //------------------------  SCREEN parts  ----------------------------------------
  if (Enable_Screen) {
    MyScreen.init();
  }
  if (imuUse) {
    imu.begin();
  }
  else
  {
    ShowMessageln(" IMU is not activate ");
  }

  if (perimeterUse) {
    ShowMessageln(" ------- Initialize Perimeter Setting ------- ");
    // perimeter.changeArea(1);
    perimeter.begin(pinPerimeterLeft, pinPerimeterRight);
  }

  if (!buttonUse) {
    // robot has no ON/OFF button => start immediately very dangerous
    //setNextState(STATE_FORWARD_ODO, 0);
  }


  stateStartTime = millis();
  //void Robot::setBeeper(int totalDuration, byte OnDuration, byte OffDuration, byte frequenceOn, byte frequenceOff ) { // Set the variable for the beeper

  setBeeper(3000, 500, 250, 2000, 200);//beep for 3 sec
  gps.init();
  ShowMessageln(F("START"));
  ShowMessage(F("Mower "));
  ShowMessageln(VER);

  ShowMessage(F("Config: "));
  ShowMessageln(name);
  ShowMessageln(F("press..."));
  ShowMessageln(F("  d for menu"));
  ShowMessageln(F("  v to change console output (sensor counters, values, perimeter etc.)"));
  ShowMessageln(consoleModeNames[consoleMode]);



  ShowMessageln ("Starting Ina226 current sensor ");
  MotLeftIna226.begin(0x41);
  ChargeIna226.begin(0x40);
  MotRightIna226.begin(0x44);
  Mow1Ina226.begin_I2C1(0x40);  //MOW1 is connect on I2C1
  if (INA226_MOW2_PRESENT) Mow2Ina226.begin_I2C1(0x41);  //MOW2 is connect on I2C1
  if (INA226_MOW3_PRESENT) Mow3Ina226.begin_I2C1(0x44);  //MOW3 is connect on I2C1

  ShowMessageln ("Checking  ina226 current sensor connection");
  //check sense powerboard i2c connection
  powerboard_I2c_line_Ok = true;
  if (!ChargeIna226.isConnected(0x40)) {
    ShowMessageln("INA226 Battery Charge is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!MotRightIna226.isConnected(0x44)) {
    ShowMessageln("INA226 Motor Right is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!MotLeftIna226.isConnected(0x41)) {
    ShowMessageln("INA226 Motor Left is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!Mow1Ina226.isConnected_I2C1(0x40)) {
    ShowMessageln("INA226 MOW1 is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if ((INA226_MOW2_PRESENT) && (!Mow2Ina226.isConnected_I2C1(0x41))) {
    ShowMessageln("INA226 MOW2 is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if ((INA226_MOW3_PRESENT) && (!Mow3Ina226.isConnected_I2C1(0x44))) {
    ShowMessageln("INA226 MOW3 is not OK");
    powerboard_I2c_line_Ok = false;
  }


  if (powerboard_I2c_line_Ok)
  {
    ShowMessageln ("Ina226 Begin OK ");
    // Configure INA226


    ChargeIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    MotLeftIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    MotRightIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    //I2C1 bus
    Mow1Ina226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    if (INA226_MOW2_PRESENT) Mow2Ina226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    if (INA226_MOW3_PRESENT) Mow3Ina226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

    ShowMessageln ("Ina226 Configure OK ");
    // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
    ChargeIna226.calibrate(0.02, 4);
    MotLeftIna226.calibrate(0.02, 4);
    MotRightIna226.calibrate(0.02, 4);
    //I2C1 bus
    Mow1Ina226.calibrate_I2C1(0.02, 4);
    if (INA226_MOW2_PRESENT) Mow2Ina226.calibrate_I2C1(0.02, 4);
    if (INA226_MOW3_PRESENT) Mow3Ina226.calibrate_I2C1(0.02, 4);

    ShowMessageln ("Ina226 Calibration OK ");
  }
  else
  {
    ShowMessageln ("************** WARNING **************");
    ShowMessageln ("INA226 powerboard connection is not OK");
  }


  // watchdog part
  //if wdt is not reset during the trigger duration a myCallback function start.
  // if after timeout wdt is always not reset tennsy reboot

  ShowMessageln("Watchdog configuration start ");
  WDT_timings_t config;
  config.trigger = 2; /* in seconds, 0->128 */
  config.timeout = 10; /* in seconds, 0->128 */
  config.callback = myCallback;
  wdt.begin(config);
  ShowMessageln("Watchdog configuration Finish ");


  nextTimeInfo = millis();
  ShowMessageln("Setup finish");


}
void Robot::resetWatchdogForPfod() {
  wdt.feed();
  delay(200);
  wdt.feed();
  //delay(500);
  //wdt.feed();
  //delay(500);
  //wdt.feed();
  // delay(500);
}

void Robot::printOdometry() {
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
}


void Robot::receivePiPfodCommand (String RpiCmd, float v1, float v2, float v3) {
  rc.processPI(RpiCmd, v1, v2, v3);
}






void Robot::printInfo(Stream & s) {

  if ((consoleMode == CONSOLE_OFF) || (consoleMode == CONSOLE_TRACKING)) {

  }
  else
  {

    Streamprint(s, "t%6u ", (millis() - stateStartTime) / 1000);
    Streamprint(s, "Loops%7u ", loopsPerSec);

    Streamprint(s, "v%1d ", consoleMode);

    Streamprint(s, "%4s ", stateNames[stateCurr]);

    if (consoleMode == CONSOLE_PERIMETER) {

      Streamprint(s, "sig min %4d max %4d avg %4d mag %5d qty %3d",
                  (int)perimeter.getSignalMin(0), (int)perimeter.getSignalMax(0), (int)perimeter.getSignalAvg(0),
                  perimeterMagLeft, (int)(perimeter.getFilterQuality(0) * 100.0));
      Streamprint(s, "  in %2d  cnt %4d  on %1d\r\n",
                  (int)perimeterInsideLeft, perimeterCounter, (int)(!perimeter.signalTimedOut(0)) );
    } else {

      Streamprint(s, "odo %4d %4d ", (int)odometryLeft, (int)odometryRight);
      Streamprint(s, "spd %4d %4d %4d ", (int)motorLeftSpeedRpmSet, (int)motorRightSpeedRpmSet, (int)motorMowPwmCoeff);
      if (consoleMode == CONSOLE_SENSOR_VALUES) {
        // sensor values
        Streamprint(s, "sen %4d %4d %4d ", (int)motorLeftPower, (int)motorRightPower, (int)motorMowPower);
        Streamprint(s, "bum %4d %4d ", bumperLeft, bumperRight);

        Streamprint(s, "son %4d %4d %4d ", sonarDistLeft, sonarDistCenter, sonarDistRight);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        if (perimeterUse) Streamprint(s, "per %3d ", (int)perimeterInsideLeft);

      } else {
        // sensor counters
        Streamprint(s, "sen %4d %4d %4d ", motorLeftSenseCounter, motorRightSenseCounter, motorMowSenseCounter);
        Streamprint(s, "bum %4d %4d ", bumperLeftCounter, bumperRightCounter);

        //Streamprint(s, "son %3d ", sonarDistCounter);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        //Streamprint(s, "per %3d ", perimeterLeft);
        if (perimeterUse) Streamprint(s, "per %3d ", perimeterCounter);

        //if (gpsUse) Streamprint(s, "gps %2d ", (int)gps.satellites());
      }
      Streamprint(s, "bat %2d.%01d ", (int)batVoltage, (int)((batVoltage * 10) - ((int)batVoltage * 10)) );
      Streamprint(s, "chg %2d.%01d %2d.%01d ",
                  (int)chgVoltage, (int)((chgVoltage * 10) - ((int)chgVoltage * 10)),
                  (int)chgCurrent, (int)((abs(chgCurrent) * 10) - ((int)abs(chgCurrent) * 10))
                 );
      //Streamprint(s, "imu%3d ", imu.getCallCounter());
      //   Streamprint(s, "adc%3d ", ADCMan.getCapturedChannels());
      Streamprint(s, "%s\r\n", name.c_str());

    }
  }

}

void Robot::printMenu() {
  Serial.println(" ");
  Serial.println(F(" MAIN MENU:"));
  Serial.println(F("1=test motors"));
  Serial.println(F("To test odometry --> use Arduremote"));
  Serial.println(F("3=communications menu"));
  Serial.println(F("5=Deactivate and Delete GYRO calibration : To calibrate GYRO --> use Arduremote Do not move IMU during the Calib"));
  Serial.println(F("6=Deactivate and Delete Compass calibration : To calibrate Compass --> use Arduremote start/stop"));
  Serial.println(F("9=save user settings"));
  Serial.println(F("l=load factory settings: Do not save setting before restart the mower"));
  Serial.println(F("r=delete robot stats"));
  Serial.println(F("x=read settings"));
  Serial.println(F("e=delete all errors"));
  Serial.println(F("0=exit"));
  Serial.println(" ");
}

void Robot::delayWithWatchdog(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    delay(500);
    wdt.feed();
  }
}

void Robot::delayInfo(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    // readSensors();
    printInfo(Serial);
    //watchdogReset();
    delay(1000);
    //watchdogReset();
  }
}
/*odometryTicksPerRevolution = 720;   // encoder ticks per one full resolution
    odometryTicksPerCm = 9.96;  // encoder ticks per cm
    odometryWheelBaseCm = 42;
*/

void Robot::testMotors() {
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);

  ShowMessageln(F("testing left motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = motorSpeedMaxPwm / 2; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);

  ShowMessageln(F("testing left motor (reverse) full speed..."));
  delay(100);
  motorLeftPWMCurr = -motorSpeedMaxPwm; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);

  ShowMessageln(F("testing right motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = 0; motorRightPWMCurr = motorSpeedMaxPwm / 2;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);

  ShowMessageln(F("testing right motor (reverse) full speed..."));
  delay(100);
  motorLeftPWMCurr = 0; motorRightPWMCurr = -motorSpeedMaxPwm;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
}

void Robot::menu() {
  char ch;
  printMenu();

  while (true) {
    //watchdogReset();
    resetIdleTime();
    // imu.update();
    if ((!RaspberryPIUse) && (Serial.available() > 0)) {  //do not read console is raspberry connected
      ch = (char)Serial.read();
      switch (ch) {
        case '0':
          nextTimeInfo = millis();
          return;
        case '1':
          testMotors();
          printMenu();
          break;

        case '5':
          //     imu.deleteAccelGyroCalib();
          imuUse = false;
          printMenu();
          break;
        case '6':
          //       imu.deleteCompassCalib();
          CompassUse = false;
          printMenu();
          break;

        case '9':
          saveUserSettings();
          printMenu();
          break;
        case 'l':
          printSettingSerial();
          deleteUserSettings();
          printMenu();
          break;
        case 'r':
          printSettingSerial();
          deleteRobotStats();
          printMenu();
          break;
        case 'x':
          printSettingSerial();
          ShowMessageln(F("DONE"));
          printMenu();
          break;
        case 'e':
          resetErrorCounters();
          setNextState(STATE_OFF, 0);
          ShowMessageln(F("ALL ERRORS ARE DELETED"));
          printMenu();
          break;
      }
    }
    delay(10);
  }
}


void Robot::readSerial() {


  if ((!RaspberryPIUse) && (Serial.available() > 0)) {  //do not read console if raspberry connected
    char ch = (char)Serial.read();
    //resetIdleTime();
    switch (ch) {
      case '0':
        // press '0' for OFF
        setNextState(STATE_OFF, 0);
        break;
      case '1':
        // press '1' for Automode
        //motorMowEnable = true;

        setNextState(STATE_ACCEL_FRWRD, 0);
        break;
      case 'd':
        menu(); // menu
        break;

      case 'h':

        setNextState(STATE_PERI_FIND, 0); // press 'h' to drive home
        break;
      case 'l':
        bumperLeft = true; // press 'l' to simulate left bumper
        bumperLeftCounter++;
        break;
      case 'q':
        yawCiblePos = 90;
        setNextState(STATE_ROLL_TO_FIND_YAW, 0); // press 'h' to drive home
        break;
      case 'r':
        //setBeeper(400, 50, 50, 200, 0 );//error
        break;
      case 's':
        //imu.calibComStartStop();
        break;
      case 't':
        setNextState(STATE_PERI_TRACK, 0); // press 't' to track perimeter
        break;
      case 'u':
        setNextState(STATE_ACCEL_FRWRD, RIGHT);
        break;
      case 'v':
        //bb
        consoleMode = (consoleMode + 1) % 5;
        ShowMessageln(consoleModeNames[consoleMode]);
        break;


    }
  }

}

void Robot::checkButton() {
  boolean buttonPressed;
  if ( (!buttonUse) || (millis() < nextTimeButtonCheck) ) return;
  nextTimeButtonCheck = millis() + 100;
  if (START_BUTTON_IS_NC) {
    buttonPressed = !(digitalRead(pinButton) == LOW);
  }
  else
  {
    buttonPressed = (digitalRead(pinButton) == LOW);
  }

  //boolean buttonPressed = false;
  if ( ((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)) )
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed) {
      ShowMessage(F("Button Pressed counter : "));
      ShowMessageln(buttonCounter);
      // ON/OFF button pressed
      setBeeper(500, 500, 0, 2000, 0);//beep for 0.5 sec
      buttonCounter++;
      if (buttonCounter >= 3) buttonCounter = 3;
      //resetIdleTime();
    }
    else
    {
      // ON/OFF button released
      ShowMessage(F("Button Release counter : "));
      ShowMessageln(buttonCounter);
      if ((statusCurr == MANUAL) || (statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (stateCurr == STATE_ERROR) || (statusCurr == WIRE_MOWING) || (statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
        ShowMessageln(F("ButtonPressed Stop Mowing and Reset Error"));
        motorMowEnable = false;
        buttonCounter = 0;
        setNextState(STATE_OFF, 0);
        return;
      }
      if  ((stateCurr == STATE_OFF) || (stateCurr == STATE_STATION)) {
        if (buttonCounter == 1) {
          motorMowEnable = true;
          statusCurr = NORMAL_MOWING;
          findedYaw = 999;
          imuDirPID.reset();
          mowPatternCurr = MOW_LANES;
          laneUseNr = 1;
          rollDir = 1;
          whereToStart = 1;
          areaToGo = 1;
          actualLenghtByLane = maxLenghtByLane; //initialise lenght lane
          beaconToStart = 0;
          mowPatternDuration = 0;
          totalDistDrive = 0;
          buttonCounter = 0;
          if (RaspberryPIUse)  MyRpi.SendStatusToPi();

          if (stateCurr == STATE_STATION) {
            ShowMessageln("MANUAL START FROM STATION");
            setActuator(ACT_CHGRELAY, 0);
            setNextState(STATE_START_FROM_STATION, 1);
          }

          else {
            ShowMessageln("MANUAL START");
            if (MOWER_HAVE_SECURITY_COVER) {
              motorMowEnable = false;
              setNextState(STATE_WAIT_COVER, 0);
            }
            else {
              setNextState(STATE_ACCEL_FRWRD, 0);
            }

            return;
          }



        }
        else if (buttonCounter == 2) {
          // start normal with random mowing
          motorMowEnable = true;
          statusCurr = NORMAL_MOWING;
          mowPatternCurr = MOW_RANDOM;
          buttonCounter = 0;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          if (stateCurr == STATE_STATION) {
            setActuator(ACT_CHGRELAY, 0);
            setNextState(STATE_START_FROM_STATION, 1);
          }

          else {

            if (MOWER_HAVE_SECURITY_COVER) {
              motorMowEnable = false;
              setNextState(STATE_WAIT_COVER, 0);
            }
            else {
              setNextState(STATE_ACCEL_FRWRD, 0);
            }
            return;
          }


        }
        else if (buttonCounter == 3) {
          if (stateCurr == STATE_STATION) return;
          //go to station
          motorMowEnable = false;
          periFindDriveHeading = scalePI(imu.ypr.yaw);
          areaToGo = 1;
          whereToStart = 99999;
          nextTimeTimer = millis() + 3600000; //avoid the mower start again if timer activate.
          statusCurr = BACK_TO_STATION;
          buttonCounter = 0;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          periFindDriveHeading = imu.ypr.yaw;
          if (MOWER_HAVE_SECURITY_COVER) {
            setNextState(STATE_WAIT_COVER, 0);
          }
          else {
            setNextState(STATE_PERI_FIND, 0);
          }

          return;
        }

        buttonCounter = 0;
      }
    }
  }
}


void Robot::newTagFind() {
  if (millis() >= nextTimeSendTagToPi) {
    nextTimeSendTagToPi = millis() + 10000;
    ShowMessage("Find a tag : ");
    ShowMessageln(rfidTagFind);
    unsigned long rfidTagFind_long = hstol(rfidTagFind);

    if (rfidUse) {
      if (search_rfid_list(rfidTagFind_long)) {
        rfidTagTraitement(rfidTagFind_long, statusCurr);
      }
      else
      {
        ShowMessage("Auto insert Wait tag : ");
        ShowMessageln(rfidTagFind);
        insert_rfid_list(rfidTagFind_long , 0, 0, 100, 1, 1, 1, 1);
        sort_rfid_list();
      }
    }
  }
}

void Robot::pfodSetDateTime(byte hr1, byte min1, byte sec1, byte day1, byte month1, short year1) {


  //mkTime(
  setTime(hr1, min1, sec1, day1, month1, year1); // Another way to set;
  Teensy3Clock.set(now()); // set the internal teensy RTC
  ShowMessageln(now());

  //setTime(hr,min,sec,day,month,yr); // Another way to set;
  //setTime(PfodDate);



  /*makeTime(tm);

    Convert normal date & time to a time_t number. The time_t number is returned. The tm input is a TimeElements variable type, which has these fields:

    tm.Second  Seconds   0 to 59
    tm.Minute  Minutes   0 to 59
    tm.Hour    Hours     0 to 23
    tm.Wday    Week Day  0 to 6  (not needed for mktime)
    tm.Day     Day       1 to 31
    tm.Month   Month     1 to 12
    tm.Year    Year      0 to 99 (offset from 1970)
  */




  ShowMessage("Date change : ");
  ShowMessageln(Teensy3Clock.get());
}

void Robot::readSensors() {
  if (millis() >= nextTimeMotorSense) {
    nextTimeMotorSense = millis() +  50;
    double accel = 0.10;  //filter percent
    unsigned long readingDuration;

    if (powerboard_I2c_line_Ok) {
      motorRightPower = MotRightIna226.readBusPower() ;
      motorLeftPower  = MotLeftIna226.readBusPower() ;
      Mow1_Power = Mow1Ina226.readBusPower_I2C1() ;
      if (INA226_MOW2_PRESENT) {
        Mow2_Power = Mow2Ina226.readBusPower_I2C1() ;
      }
      else {
        Mow2_Power = 0;
      }
      if (INA226_MOW3_PRESENT) {
        Mow3_Power = Mow3Ina226.readBusPower_I2C1() ;
      }
      else {
        Mow3_Power = 0;
      }
      readingDuration = millis() - nextTimeMotorSense + 50;
      if (readingDuration > 30 ) {  //leave 30 ms to I2C reading
        ShowMessage("Error in INA226 motor power Timeout reading I2C : ");
        ShowMessageln(readingDuration);
      }
    }
    //Mow2_Power = 0 ;
    //Mow3_Power = 0 ;

    motorMowPower   = max(Mow1_Power, Mow2_Power);
    motorMowPower   = max(motorMowPower, Mow3_Power);

    motorRightPower = motorRightPower * (1.0 - accel) + ((double)motorRightPower) * accel;
    motorLeftPower = motorLeftPower * (1.0 - accel) + ((double)motorLeftPower) * accel;
    motorMowPower = motorMowPower * (1.0 - accel) + ((double)motorMowPower) * accel;




  }
  //perimeter
  if ((stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING) && (perimeterUse) && (millis() >= nextTimePerimeter)) {
    nextTimePerimeter = millis() +  15;
    //right coil
    if (read2Coil) {
      perimeterMagRight = perimeter.getMagnitude(1);
      if ((perimeter.isInside(1) != perimeterInsideRight)) {
        perimeterRightTriggerTime = millis();
        perimeterLastTransitionTime = millis();
        perimeterInsideRight = perimeter.isInside(1);
      }
    }
    //left coil
    perimeterMagLeft = perimeter.getMagnitude(0);
    perimeterMedian.add(perimeterMagLeft);
    if (perimeterMedian.isFull()) {
      perimeterNoise = perimeterMedian.getHighest() - perimeterMedian.getLowest();
      perimeterMedian.clear();
    }

    if ((perimeter.isInside(0) != perimeterInsideLeft)) {
      perimeterCounter++;
      perimeterLeftTriggerTime = millis();
      perimeterLastTransitionTime = millis();
      perimeterInsideLeft = perimeter.isInside(0);
    }

    if (((!perimeterInsideLeft) || ((!perimeterInsideRight) && (read2Coil))) && (perimeterTriggerTime == 0)) {
      // set perimeter trigger time

      //bber2
      //use smooth on left coil only  to avoid big area transition, in the middle of the area with noise the mag can change from + to -
      smoothPeriMag = perimeter.getSmoothMagnitude(0);
      if (smoothPeriMag > perimeterTriggerMinSmag) {
        perimeterTriggerTime = millis();
      }
      else
      {
        if (millis() >= nextTimePrintConsole) {
          nextTimePrintConsole = millis() + 1000;
          if ((developerActive) && (stateCurr == STATE_FORWARD_ODO)) {
            ShowMessageln("Bad reading perimeter In/Out, certainly we are very far the wire");
          }
        }
      }

    }

  }


  if ((MOWER_HAVE_SECURITY_COVER) && (stateCurr != STATE_TEST_MOTOR)) {

    if (digitalRead(pinCover) == 0) {
      coverIsClosed = true;
    }
    else
    {
      coverIsClosed = false;
      if (stateCurr != STATE_WAIT_COVER) {
        if (stateCurr != STATE_OFF)  {
          ShowMessageln("Cover Open ");
          if ((statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (stateCurr == STATE_ERROR) || (statusCurr == WIRE_MOWING) || (statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
            ShowMessageln(F("Stop Mowing and Reset Error"));
            motorMowEnable = false;
            buttonCounter = 0;
            setNextState(STATE_OFF, 0);
            return;
          }
        }
      }
    }


  }


  if ((bumperUse) && (millis() >= nextTimeBumper)) {
    nextTimeBumper = millis() + 30;



    // front bumper

    if (BUMPER_ARE_NORMALY_CLOSED) {
      if (digitalRead(pinBumperLeft) == 1) {
        //ShowMessageln("Bumper left trigger");
        bumperLeftCounter++;
        bumperLeft = true;
      }

      if (digitalRead(pinBumperRight) == 1) {
        //ShowMessageln("Bumper right trigger");
        bumperRightCounter++;
        bumperRight = true;
      }
    }
    else
    {
      if (digitalRead(pinBumperLeft) == 0) {
        //ShowMessageln("Bumper left trigger");
        bumperLeftCounter++;
        bumperLeft = true;
      }

      if (digitalRead(pinBumperRight) == 0) {
        //ShowMessageln("Bumper right trigger");
        bumperRightCounter++;
        bumperRight = true;
      }

    }

    // rear bumper
    if (BUMPER_ARE_NORMALY_CLOSED) {
      if (digitalRead(pinBumperRearLeft) == 1) {
        //ShowMessageln("Bumper left trigger");
        bumperRearLeftCounter++;
        bumperRearLeft = true;
      }

      if (digitalRead(pinBumperRearRight) == 1) {
        //ShowMessageln("Bumper right trigger");
        bumperRearRightCounter++;
        bumperRearRight = true;
      }
    }
    else
    {
      if (digitalRead(pinBumperRearLeft) == 0) {
        //ShowMessageln("Bumper left trigger");
        bumperRearLeftCounter++;
        bumperRearLeft = true;
      }

      if (digitalRead(pinBumperRearRight) == 0) {
        //ShowMessageln("Bumper right trigger");
        bumperRearRightCounter++;
        bumperRearRight = true;
      }

    }






  }
  if (millis() >= nextTimeRTC) {
    //for pfod use : need to adjust the datetime var each second for example
    nextTimeRTC = millis() + 1000;
    datetime.date.day = day();
    datetime.date.month = month();
    datetime.date.year = year();
    datetime.time.hour = hour();
    datetime.time.minute = minute();
    datetime.date.dayOfWeek = getDayOfWeek(month(), day(), year(), 0);

  }




  if (millis() >= nextTimeBattery) {
    // read battery
    double chgvolt = 0.1 ;
    double curramp = 0.1; //  0.1 instead 0 to avoid div/0
    double batvolt = 0.1 ;
    unsigned long readingDuration;
    nextTimeBattery = millis() + 500;
    if ((millis() > 30000) and (millis() < 30550) and (!powerboard_I2c_line_Ok)) {
      ShowMessageln("POWERBOARD I2C LINE is not OK");
    }

    if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5)) {
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    if (powerboard_I2c_line_Ok) {
      chgvolt = ChargeIna226.readBusVoltage() ;
      curramp = ChargeIna226.readBusPower(); //  ?? sense don't work
      batvolt = MotRightIna226.readBusVoltage() ;
      batvolt = batvolt + D5VoltageDrop;
      readingDuration = millis() - nextTimeBattery + 500;
      if (readingDuration > 30 ) {  //leave 30 ms to I2C reading
        ShowMessage("Error in INA226 Bat Voltage Timeout reading I2C : ");
        ShowMessageln(readingDuration);
      }
    }
    if (chgvolt != 0) {
      curramp = curramp / chgvolt;
    }
    else
    {
      curramp = 0;
    }



    double accel = 0.05;  //filter percent

    if (abs(batVoltage - batvolt) > 8)   batVoltage = batvolt; else batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage - chgvolt) > 8)   chgVoltage = chgvolt; else chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    if (abs(chgCurrent - curramp) > 0.4) chgCurrent = curramp; else chgCurrent = (1.0 - accel) * chgCurrent + accel * curramp; //Deaktiviert fÃ¼r Ladestromsensor berechnung
    //bber30 tracking not ok because reduce the speed loop with this but can check the chgvoltage

    /*
       ShowMessage("batVoltage ");
       ShowMessage(batVoltage);
       ShowMessage("/chgVoltage ");
       ShowMessage(chgVoltage);
       ShowMessage("/chgCurrent ");
       ShowMessage(chgCurrent);
       ShowMessage("/curramp ");
       ShowMessageln(curramp);
    */
  }

  if ((rainUse) && (millis() >= nextTimeRain)) {
    // read rain sensor
    nextTimeRain = millis() + 5000;
    rain = (digitalRead(pinRain));
    if (rain) rainCounter++;
  }
}


void Robot::setDefaults() {
  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
  motorMowEnable = false;
}

// set state machine new state
// called *ONCE* to set to a *NEW* state


void Robot::setNextState(byte stateNew, byte dir) {
  stateTime = millis() - stateStartTime; //last state duration
  if (stateNew == stateCurr) return;
  String line01 = "";
  // evaluate new state
  stateNext = stateNew;
  rollDir = dir;
  moveRightFinish = false;
  moveLeftFinish = false;
  switch (stateNew) {

    case STATE_FORWARD: // not use 04/11/22
      if ((stateCurr == STATE_STATION_REV) || (stateCurr == STATE_STATION_ROLL) || (stateCurr == STATE_STATION_CHECK) ) return;
      if ((stateCurr == STATE_STATION) || (stateCurr == STATE_STATION_CHARGING)) {
        setActuator(ACT_CHGRELAY, 0);
        motorMowEnable = false;
      }
      motorLeftSpeedRpmSet = motorSpeedMaxRpm; //use RPM instead of PWM to straight line
      motorRightSpeedRpmSet = motorSpeedMaxRpm;
      statsMowTimeTotalStart = true;

      break;


    case STATE_FORWARD_ODO:
      if (statusCurr != NORMAL_MOWING) {
        statusCurr = NORMAL_MOWING;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
      }
      //bber300 for debug
      ShowMessage("Lane lenght : ");
      ShowMessage(actualLenghtByLane);
      ShowMessage(" change : ");
      ShowMessageln(justChangeLaneDir);
      if (mowPatternCurr == MOW_LANES) { //motor are stop at this moment
        UseAccelRight = 1;
        UseAccelLeft = 1;
      }
      else
      {
        UseAccelRight = 0;
        UseAccelLeft = 0;
      }
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
      //bber60

      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 30000);// set a very large distance 300 ml for random mowing
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 30000);
      if ((mowPatternCurr == MOW_LANES) && (!justChangeLaneDir)) { //it s a not new lane so limit the forward distance
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * actualLenghtByLane * 100); //limit the lenght
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * actualLenghtByLane * 100);
      }

      OdoRampCompute();

      statsMowTimeTotalStart = true;

      break;

    case STATE_ESCAPE_LANE: //not use
      ShowMessageln("Mowing in Half lane width");
      //it's approximation try to go into a parcel already mowed little on left or right
      //halfLaneNb = halfLaneNb + 1; //to avoid repetition the state is lauch only if halfLaneNb=0
      UseAccelLeft = 0;
      UseBrakeLeft = 0;
      UseAccelRight = 0;
      UseBrakeRight = 0;
      if (rollDir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
        stateEndOdometryRight = odometryRight + (odometryTicksPerCm * odometryWheelBaseCm * 2) ;
        stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * odometryWheelBaseCm * 1.5 ) ;
      }
      else
      {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
        stateEndOdometryRight = odometryRight + (odometryTicksPerCm * odometryWheelBaseCm * 1.5 ) ;
        stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * odometryWheelBaseCm * 2) ;
      }


      OdoRampCompute();

      break;

    case STATE_START_FROM_STATION: //when start in auto mode the mower first initialize the IMU and perimeter sender
      motorSpeedMaxPwm = motorInitialSpeedMaxPwm ;
      motorMowEnable = false; //mow motor start later when leave the perimeter wire
      nextTimeToDmpAutoCalibration = millis() + delayBetweenTwoDmpAutocalib * 1000; //set the next time for calib
      //readDHT22();
      ShowMessageln("Start sender1");
      line01 = "#SENDER," + area1_ip + ",A1";
      Bluetooth.println(line01);
      ShowMessageln("Stop sender2");
      line01 = "#SENDER," + area2_ip + ",B0";
      Bluetooth.println(line01);
      ShowMessageln("Stop sender3");
      line01 = "#SENDER," + area3_ip + ",B0";
      Bluetooth.println(line01);
      setBeeper(600, 40, 5, 500, 0 );
      MaxStateDuration = 6000; // 6 secondes beep and pause before rev
      break;

    case STATE_STATION_REV: //when start in auto mode the mower first reverse to leave the station
      statusCurr = TRACK_TO_START;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * stationRevDist);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * stationRevDist);
      OdoRampCompute();
      break;

    case STATE_STATION_ROLL:  //when start in auto after mower reverse it roll for this angle
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (mowPatternCurr == MOW_LANES)       AngleRotate = 90;
      else AngleRotate = random(30, 160);
      if (startByTimer) AngleRotate = stationRollAngle;
      if (track_ClockWise) {
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      }
      else
      {
        Tempovar = -36000 / AngleRotate; //need a value*100 for integer division later
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      }

      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();

      break;

    case STATE_STATION_FORW: //when start in auto after mower  roll this state accel the 2 wheel before forward

      justChangeLaneDir = true;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 60) ;//60CM to accel
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 60) ;
      OdoRampCompute();
      //motorMowEnable = true;

      break;

    case STATE_STATION_CHECK:
      //Move forward in stattion 2 or 3 cm to be sure contact are OK
      //16/10/22 limit the max state duration to 500 ms to avoid overload motor
      if (statusCurr == WIRE_MOWING) { //it is the last status
        ShowMessage("Total distance drive ");
        ShowMessage(totalDistDrive / 100);
        ShowMessageln(" meters ");
        ShowMessage("Total duration ");
        ShowMessage(int(millis() - stateStartTime) / 1000);
        ShowMessageln(" secondes ");
        nextTimeTimer = millis() + 1200000; // only check again the timer after 20 minutes to avoid repetition
      }
      else {
        ShowMessageln("Check station");
      }
      delayToReadVoltageStation = millis() + 500; //the battery is read again after 500 ms to be sure we have always voltage
      UseAccelLeft = 1; //16/10/22
      UseBrakeLeft = 1;
      UseAccelRight = 1; //16/10/22
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = checkDockingSpeed;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * stationCheckDist);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * stationCheckDist);
      OdoRampCompute();


      break;

    //not use actually
    case STATE_PERI_ROLL:
      stateEndTime = millis() + perimeterTrackRollTime + motorZeroSettleTime;
      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;
      } else {
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorLeftSpeedRpmSet = -motorRightSpeedRpmSet;
      }
      break;

    case STATE_PERI_OBSTACLE_REV:
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriObstacleRev);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriObstacleRev);
      OdoRampCompute();
      break;
    case STATE_PERI_OBSTACLE_ROLL:
      AngleRotate = 45;
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (track_ClockWise) {
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      }
      else {
        Tempovar = -36000 / AngleRotate; //need a value*100 for integer division later
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      }
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();
      break;

    case STATE_PERI_OBSTACLE_FORW:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm / 2;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriObstacleForw);//50cm
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriObstacleForw);
      OdoRampCompute();

      break;

    case STATE_PERI_OBSTACLE_AVOID:
      UseAccelLeft = 0;
      UseBrakeLeft = 0;
      UseAccelRight = 0;
      UseBrakeRight = 0;
      if (track_ClockWise) {
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
        motorLeftSpeedRpmSet = int(motorSpeedMaxRpm / 3);
      }
      else {
        motorRightSpeedRpmSet = int(motorSpeedMaxRpm / 3);
        motorLeftSpeedRpmSet = motorSpeedMaxRpm;
      }
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
      OdoRampCompute();



      break;

    case STATE_PERI_REV:  //when obstacle in perifind
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - 1440;
      stateEndOdometryLeft = odometryLeft - 1440;
      OdoRampCompute();
      break;


    case STATE_PERI_OUT_STOP: //in auto mode and forward slow down before stop and reverse

      if (mowPatternCurr == MOW_LANES) {
        if (stateCurr == STATE_NEXT_LANE_FORW) {  // change to mow random if the wire is detected
          //bber201
          mowPatternDuration = mowPatternDurationMax - 1 ; //set the mow_random for the next 1 minutes
          ShowMessageln("Find a corner change to Random for 1 minutes ");
          mowPatternCurr = MOW_RANDOM; //change the pattern each x minutes
          laneUseNr = laneUseNr + 1;
          findedYaw = 999;
          justChangeLaneDir = true;
          nextTimeToDmpAutoCalibration = millis(); // so the at the end of the next line a calibration occur
          if (laneUseNr > 3) laneUseNr = 1;
        }
      }
      //-------------------------------Verify if it's time to change mowing pattern
      if (mowPatternDuration > mowPatternDurationMax) {
        ShowMessageln(" mowPatternCurr  change ");
        mowPatternCurr = (mowPatternCurr + 1) % 2; //change the pattern each x minutes
        mowPatternDuration = 0;
      }
      justChangeLaneDir = !justChangeLaneDir;  //use to know if the lane is not limit distance
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm * 0.7 ; //perimeterSpeedCoeff reduce speed near the wire to 70%
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();

      break;

    case STATE_ENDLANE_STOP: //in auto mode and forward slow down before stop and reverse
      //-------------------------------Verify if it's time to change mowing pattern
      justChangeLaneDir = !justChangeLaneDir;  //use to know if the lane is not limit distance
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm; //perimeterSpeedCoeff reduce speed near the wire to 70%
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 2 * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 2 * DistPeriOutStop);
      OdoRampCompute();


      break;
    case STATE_SONAR_TRIG: //in auto mode and forward slow down before stop and reverse different than stop because reduce speed during a long time and not immediatly
      // justChangeLaneDir = !justChangeLaneDir;  //need to change this feature
      distToObstacle = distToObstacle - sonarToFrontDist; //   the distance between sonar and front of mower
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * distToObstacle);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * distToObstacle);
      OdoRampCompute();

      break;

    case STATE_STOP_TO_FIND_YAW:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();

      break;

    case STATE_STOP_ON_BUMPER:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      //bber500 to stop immediatly
      stateEndOdometryRight = odometryRight;// + (int)(odometryTicksPerCm / 6);
      stateEndOdometryLeft = odometryLeft;// + (int)(odometryTicksPerCm / 6);
      OdoRampCompute();

      break;

    case STATE_PERI_STOP_TOTRACK:
      //bber100 err here
      if (statusCurr == TRACK_TO_START) {
        if (mowPatternCurr == MOW_WIRE) {
          motorMowEnable = true; //time to start the blade
          statusCurr = WIRE_MOWING;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
        }
      }
      else if (statusCurr == WIRE_MOWING) {
        motorMowEnable = true; //time to start the blade
      }
      else {
        statusCurr = BACK_TO_STATION;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
      }

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;




    case STATE_PERI_STOP_TOROLL:
      //imu.run(); //31/08/19 In peritrack the imu is stop so try to add this to start it now and avoid imu tilt error (occur once per week or less) ??????
      if (statusCurr == TRACK_TO_START) {
        startByTimer = false; // cancel because we have reach the start point and avoid repeat search entry
        justChangeLaneDir = false; //the first lane need to be distance control
        motorMowEnable = true; //time to start the blade
      }

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_PERI_STOP_TO_FAST_START:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_PERI_STOP_TO_NEWAREA:
      if ((statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
        statusCurr = REMOTE;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
        //startByTimer = false; // ?? not here                         cancel because we have reach the start point and avoid repeat search entry
        justChangeLaneDir = false; //the first lane need to be distance control
        perimeterUse = false; //disable the perimeter use to leave the area
        ShowMessageln("Stop to read the perimeter wire");
        rollDir = LEFT;

      }
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_ROLL1_TO_NEWAREA:  // when find a tag the mower roll with new heading and drive in straight line
      AngleRotate = abs(newtagRotAngle1);
      newtagRotAngle1Radian = newtagRotAngle1 * PI / 180.0;
      ShowMessage("Actual Heading ");
      ShowMessageln(imu.ypr.yaw * 180 / PI);
      remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New Remote Heading ");
      ShowMessageln(remoteDriveHeading * 180 / PI);

      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (track_ClockWise) {
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm  ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      }
      else {
        Tempovar = -36000 / AngleRotate; //need a value*100 for integer division later
        motorLeftSpeedRpmSet = motorSpeedMaxRpm  ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm ;
      }
      stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();
      break;

    case STATE_ROLL2_TO_NEWAREA:  // when find a tag the mower roll with new heading and drive in straight line
      AngleRotate = newtagRotAngle2;
      newtagRotAngle1Radian = newtagRotAngle2 * PI / 180.0;
      ShowMessage("Actual Heading ");
      //ShowMessageln(imu.ypr.yaw * 180 / PI);
      //remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New Remote Heading ");
      ShowMessageln(remoteDriveHeading * 180 / PI);
      if (AngleRotate >= 0) {
        rollDir = RIGHT;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
      }
      else {
        rollDir = LEFT;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
      }

      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;

      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later  HERE IT CAN BE NEGATIVE WHEN ROLL LEFT
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);





      OdoRampCompute();
      break;

    case STATE_DRIVE1_TO_NEWAREA:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      currDistToDrive = 0; // when use the IMU the distance is not check on each wheel but on averrage
      // newtagDistance1 is the distance to drive
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * newtagDistance1);
      stateEndOdometryLeft = odometryLeft +  (int)(odometryTicksPerCm * newtagDistance1) ;
      OdoRampCompute();

      break;
    case STATE_DRIVE2_TO_NEWAREA:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      currDistToDrive = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * newtagDistance2);
      stateEndOdometryLeft = odometryLeft +  (int)(odometryTicksPerCm * newtagDistance2) ;
      OdoRampCompute();
      break;

    case STATE_STOP_TO_NEWAREA:
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_WAIT_FOR_SIG2:
      statusCurr = WAITSIG2;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //when the raspberry receive this new status it start the sender with the correct area sigcode
      totalDistDrive = 0; //reset the distance to track on the new area
      perimeterUse = true;
      ShowMessageln("Start to read the Perimeter wire");
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight;//+ (odometryTicksPerCm * 10);
      stateEndOdometryLeft = odometryLeft;//+ (odometryTicksPerCm * 10);
      OdoRampCompute();

      break;

    case STATE_ROLL_TONEXTTAG:  // when find a tag the mower roll to leave the wire and go again in peirfind with new heading
      AngleRotate = newtagRotAngle1;
      newtagRotAngle1Radian = newtagRotAngle1 * PI / 180.0;
      ShowMessage("Actual Heading ");
      ShowMessageln(imu.ypr.yaw * 180 / PI);
      periFindDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New PeriFind Heading ");
      ShowMessageln(periFindDriveHeading * 180 / PI);
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      //Always rotate RIGHT to leave the wire
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      OdoRampCompute();
      break;

    case STATE_AUTO_CALIBRATE:
      nextTimeAddYawMedian = millis() + 500; //wait 500 ms to stabilize before record first yaw
      nextTimeToDmpAutoCalibration = millis() + delayBetweenTwoDmpAutocalib * 1000; //set the next time for calib
      //needDmpAutoCalibration = false;  //to avoid repetition
      endTimeCalibration = millis() + maxDurationDmpAutocalib * 1000;  //max duration calibration
      compassYawMedian.clear();
      accelGyroYawMedian.clear();

      break;


    case STATE_STOP_CALIBRATE:
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      if (actualRollDirToCalibrate != LEFT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm / 2 );
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm / 2 );

      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm / 2 ) ;
        stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm / 2 ) ;
      }
      //bber50
      OdoRampCompute();
      break;

    case STATE_STOP_BEFORE_SPIRALE:
      statusCurr = SPIRALE_MOWING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 20 ); //brake in 20CM
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 20);  //brake in 20CM
      OdoRampCompute();
      break;


    case STATE_ROTATE_RIGHT_360:
      spiraleNbTurn = 0;
      //halfLaneNb = 0;
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5  ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5 ;
      stateEndOdometryRight = odometryRight - (int)36000 * (odometryTicksPerCm * PI * odometryWheelBaseCm / 36000);
      stateEndOdometryLeft = odometryLeft + (int)36000 * (odometryTicksPerCm * PI * odometryWheelBaseCm / 36000);
      OdoRampCompute();
      break;

    case STATE_NEXT_SPIRE:
      if (spiraleNbTurn == 0) {
        UseAccelLeft = 1;
        UseAccelRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ; ///adjust to change the access to next arc
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      }
      else {
        UseAccelLeft = 0;
        UseAccelRight = 0;
      }
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      stateEndOdometryRight = odometryRight +  (odometryTicksPerCm * odometryWheelBaseCm / 2);
      stateEndOdometryLeft = odometryLeft +  (odometryTicksPerCm * odometryWheelBaseCm / 2);
      OdoRampCompute();

      break;

    case STATE_MOW_SPIRALE:
      float DistToDriveRight;
      float DistToDriveLeft;
      float Tmp;
      float Tmp1;
      setBeeper(0, 0, 0, 0, 0);
      UseAccelLeft = 0;
      UseAccelRight = 0;
      UseBrakeLeft = 0;
      UseBrakeRight = 0;

      if (spiraleNbTurn == 0) {
        R = (float)(odometryWheelBaseCm * 1.2); //*1.2 to avoid that wheel is completly stop
        DistToDriveRight = PI * (R / 2.00);
        DistToDriveLeft = PI * (R * 1.50);
      }
      else {
        //ShowMessageln(R);
        R = R + (float)(odometryWheelBaseCm / 2);
        //ShowMessageln(R);
        DistToDriveRight = PI * (R - ((float)odometryWheelBaseCm / 2.00));
        DistToDriveLeft = PI * (R + ((float)odometryWheelBaseCm / 2.00));
      }

      motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
      Tmp = 2 * (R - float(odometryWheelBaseCm));
      Tmp1 = 2.00 * (R + float(odometryWheelBaseCm));
      motorRightSpeedRpmSet = (int) (motorLeftSpeedRpmSet * Tmp / Tmp1) ;


      stateEndOdometryRight = odometryRight +  (odometryTicksPerCm * DistToDriveRight);
      stateEndOdometryLeft = odometryLeft +  (odometryTicksPerCm * DistToDriveLeft);
      /*
            ShowMessage("MOW SPIRALE R ");
            ShowMessage(R);
            ShowMessage(" Tmp ");
            ShowMessage(Tmp);
            ShowMessage(" Tmp1 ");
            ShowMessage(Tmp1);
            ShowMessage(" motorLeftSpeedRpmSet ");
            ShowMessage(motorLeftSpeedRpmSet);
            ShowMessage(" motorRightSpeedRpmSet ");
            ShowMessage(motorRightSpeedRpmSet);
            ShowMessage(" stateEndOdometryRight ");
            ShowMessage(stateEndOdometryRight);
            ShowMessage(" stateEndOdometryLeft ");
            ShowMessage(stateEndOdometryLeft);
            ShowMessage(" spiraleNbTurn ");
            ShowMessageln(spiraleNbTurn);

      */



      OdoRampCompute();
      spiraleNbTurn = spiraleNbTurn + 1;

      break;


    case STATE_BUMPER_REV: //in normal mowing reverse after the bumper trigger
      setBeeper(0, 0, 0, 0, 0);

      if (rollDir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        if (mowPatternCurr == MOW_LANES)   UseBrakeRight = 1;
        else UseBrakeRight = 0;
      }
      else
      {
        UseAccelLeft = 1;
        if (mowPatternCurr == MOW_LANES)  UseBrakeLeft = 1;
        else UseBrakeLeft = 0;
        UseAccelRight = 1;
        UseBrakeRight = 1;
      }
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * bumper_rev_distance);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * bumper_rev_distance);
      /*
            ShowMessage("REV left E/A : ");
            ShowMessage(stateEndOdometryLeft);
            ShowMessage(" / ");
            ShowMessageln(odometryLeft);
            ShowMessage("right E/A : ");
            ShowMessage(stateEndOdometryRight);
            ShowMessage(" / ");
            ShowMessageln(odometryRight);
      */



      OdoRampCompute();
      break;


    case STATE_PERI_OUT_REV: //in normal mowing reverse after the wire trigger
      setBeeper(0, 0, 0, 0, 0);
      //perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
      if (mowPatternCurr == MOW_LANES) {
        PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
        PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      }
      else
      {
        PrevStateOdoDepassLeft = 0;
        PrevStateOdoDepassRight = 0;
      }
      if (rollDir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        if (mowPatternCurr == MOW_LANES)   UseBrakeRight = 1;
        else UseBrakeRight = 0;
      }
      else
      {
        UseAccelLeft = 1;
        if (mowPatternCurr == MOW_LANES)  UseBrakeLeft = 1;
        else UseBrakeLeft = 0;
        UseAccelRight = 1;
        UseBrakeRight = 1;
      }
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriOutRev) - PrevStateOdoDepassRight;
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriOutRev) - PrevStateOdoDepassLeft;
      /*
            ShowMessage("REV left E/A : ");
            ShowMessage(stateEndOdometryLeft);
            ShowMessage(" / ");
            ShowMessageln(odometryLeft);
            ShowMessage("right E/A : ");
            ShowMessage(stateEndOdometryRight);
            ShowMessage(" / ");
            ShowMessageln(odometryRight);
      */



      OdoRampCompute();
      break;

    case STATE_PERI_OUT_ROLL: //roll left or right in normal mode

      if (motorRollDegMin > motorRollDegMax) ShowMessageln("Warning : Roll deg Min > Roll deg Max ????? ");
      if (mowPatternCurr == MOW_RANDOM) AngleRotate = random(motorRollDegMin, motorRollDegMax);

      ShowMessage("Heading : ");
      ShowMessage((imu.ypr.yaw / PI * 180.0));

      if (dir == RIGHT) {
        ShowMessage(" Rot Angle : ");
        ShowMessageln(AngleRotate);
        // if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading + 135); //need limit value to valib the rebon
        UseAccelLeft = 1;
        //bb6
        //UseBrakeLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 0;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        Tempovar = 2 * 36000 / AngleRotate; //need a value*100 for integer division later
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        ShowMessage(" Rot Angle : ");
        ShowMessageln(-1 * AngleRotate);
        // if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading - 135); //need limit value to valib the rebon
        UseAccelLeft = 0;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        //bb6 =1
        UseBrakeRight = 0;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        Tempovar = 2 * 36000 / AngleRotate; //need a value*100 for integer division later
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
      }



      OdoRampCompute();
      break;

    case STATE_PERI_OUT_ROLL_TOINSIDE:  //roll left or right in normal mode
      //bber2
      //perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
      if (stateCurr == STATE_WAIT_AND_REPEAT) {
        RollToInsideQty = RollToInsideQty + 1;
        ShowMessage("Not Inside roll nb: ");
        ShowMessageln(RollToInsideQty);
      }
      else {
        RollToInsideQty = 0;
        ShowMessage("Find Inside roll nb: ");
        ShowMessageln(RollToInsideQty);
      }
      if (mowPatternCurr == MOW_LANES) {
        //bber201
        mowPatternDuration = mowPatternDurationMax - 1 ; //set the mow_random for the next 1 minutes
        ShowMessageln("Find a corner change to Random for 1 minutes ");
        mowPatternCurr = MOW_RANDOM; //change the pattern each x minutes
        laneUseNr = laneUseNr + 1;
        findedYaw = 999;
        justChangeLaneDir = true;
        nextTimeToDmpAutoCalibration = millis(); // so the at the end of the next line a calibration occur
        if (laneUseNr > 3) laneUseNr = 1;
      }
      AngleRotate = 50;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);


      }
      OdoRampCompute();
      break;
    case STATE_PERI_OUT_ROLL_TOTRACK:  //roll left or right in normal mode
      AngleRotate = 180;
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (track_ClockWise) {
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm ;
      }
      else {
        Tempovar = -36000 / AngleRotate; //need a value*100 for integer division later
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      }
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);


      OdoRampCompute();
      break;

    case STATE_PERI_OUT_STOP_ROLL_TOTRACK:  //roll right in normal mode when find wire
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      //bber300
      if (track_ClockWise) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm * 5); //stop on 5 cm
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 5);

      }
      else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 5); //stop on 5 cm
        stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm * 5);
      }

      OdoRampCompute();
      break;
    case STATE_PERI_OUT_FORW:  //Accel after roll so the 2 wheel have the same speed when reach the forward state
      ShowMessage("New heading :");
      ShowMessageln((imu.ypr.yaw / PI * 180.0));
      if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG)) {
        PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
        PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      }
      else
      {
        PrevStateOdoDepassLeft = 0;
        PrevStateOdoDepassRight = 0;
      }
      if (dir == RIGHT) {
        if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG))   UseAccelLeft = 1;
        else  UseAccelLeft = 0;
        UseAccelRight = 1;
      } else {

        if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG))   UseAccelRight = 1;
        else  UseAccelRight = 0;
        UseAccelLeft = 1;

      }
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutForw) - PrevStateOdoDepassRight;
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutForw) - PrevStateOdoDepassLeft;
      OdoRampCompute();

      break;


    case STATE_PERI_OUT_LANE_ROLL1: //roll left or right in normal mode for 135 deg

      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      AngleRotate = 135;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight =  odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight ;
        stateEndOdometryLeft =  odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft ;
      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft;
      }
      OdoRampCompute();
      break;

    case STATE_NEXT_LANE_FORW:  //small move to reach the next  parallel lane
      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;

      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistBetweenLane) - PrevStateOdoDepassRight; //forward for  distance between lane
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistBetweenLane) - PrevStateOdoDepassLeft;

      OdoRampCompute();
      break;

    case STATE_PERI_OUT_LANE_ROLL2: //roll left or right in normal mode

      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      AngleRotate = 45;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;

      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight =  odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight ;
        stateEndOdometryLeft =  odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft ;
      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft;
      }
      OdoRampCompute();
      break;

    case STATE_REVERSE:  //hit obstacle in forward state
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 0;
      }
      if (dir == LEFT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 1;
        UseBrakeRight = 1;
      }

      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriObstacleRev);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriObstacleRev);
      OdoRampCompute();
      /*
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.25;
        stateEndTime = millis() + motorReverseTime + motorZeroSettleTime;
      */
      break;
    case STATE_ROLL:  // when hit obstacle in forward mode
      AngleRotate = random(50, 180);
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 0;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        UseAccelLeft = 0;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 0;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      }
      OdoRampCompute();
      break;




    case STATE_TEST_COMPASS:  // to test the imu
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
      stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm * 2 * PI * odometryWheelBaseCm );
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 2 *  PI * odometryWheelBaseCm );



      OdoRampCompute();


      break;


    case STATE_CALIB_MOTOR_SPEED:
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      OdoRampCompute();

      break;

    case STATE_TEST_MOTOR:
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      OdoRampCompute();

      break;


    case STATE_ROLL_TO_FIND_YAW:  // roll slowly 720 deg until find the positive yaw, state will be changed by the IMU
      if (stopMotorDuringCalib) motorMowEnable = false;//stop the mow motor
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      /*
        ShowMessage(" imu.comYaw ");
        ShowMessage(abs(100 * imu.comYaw));
        ShowMessage(" imu.ypr.yaw ");
        ShowMessage(abs(100 * imu.ypr.yaw));
        ShowMessage(" distancePI(imu.comYaw, imu.ypr.yaw) ");
        ShowMessageln(distancePI(imu.comYaw, imu.ypr.yaw));
      */

      /*
            if (distancePI(imu.comYaw, yawCiblePos * PI / 180) > 0) { //rotate in the nearest direction
              actualRollDirToCalibrate = RIGHT;
              //ShowMessageln(" >>> >>> >>> >>> >>> >>> 0");
              motorLeftSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
              motorRightSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
              stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
              stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
            }
            else
            {
              actualRollDirToCalibrate = LEFT;
              //ShowMessageln(" <<< <<< <<< <<< <<< << 0");
              motorLeftSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
              motorRightSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
              stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
              stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
            }
      */

      OdoRampCompute();
      break;




    case STATE_ROLL_WAIT:
      ///use to make test with o in console but never call in normal mode

      //roll slowly 360 deg to find the yaw state is stopped by the IMU
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2;
      stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm *  2 * PI * odometryWheelBaseCm );
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm *  2 * PI * odometryWheelBaseCm );

      OdoRampCompute();
      break;
    case STATE_MANUAL:
      statusCurr = MANUAL;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      break;
    case STATE_REMOTE:
      statusCurr = REMOTE;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      motorMowEnable = false;
      break;
    case STATE_STATION: //stop immediatly
      areaInMowing = 1;
      //ignoreRfidTag = false;
      motorMowEnable = false;
      startByTimer = false;
      totalDistDrive = 0; //reset the tracking distance travel
      whereToResetSpeed = 50000; // initial value to 500 meters
      ActualSpeedPeriPWM = MaxSpeedperiPwm; //reset the tracking speed
      statusCurr = IN_STATION;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //time to reset the speed because the Peri find can use very high speed
      motorSpeedMaxPwm = motorInitialSpeedMaxPwm;
      stateEndOdometryRight = odometryRight;
      stateEndOdometryLeft = odometryLeft ;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      setMotorPWM(0, 0);
      setActuator(ACT_CHGRELAY, 0);
      setDefaults();
      statsMowTimeTotalStart = false;  // stop stats mowTime counter
      //bber30
      loadSaveRobotStats(false);        //save robot stats

      break;

    case STATE_STATION_CHARGING:
      setActuator(ACT_CHGRELAY, 1);
      setDefaults();
      break;

    case STATE_OFF:
      statusCurr = WAIT;
      motorRightPID.reset();
      motorLeftPID.reset();
      if (RaspberryPIUse) MyRpi.SendStatusToPi();

      startByTimer = false;// reset the start timer
      setActuator(ACT_CHGRELAY, 0);
      setDefaults();
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * 20);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * 20);
      // stateMaxiTime = millis() + 5000;
      OdoRampCompute();
      statsMowTimeTotalStart = false; // stop stats mowTime counter
      loadSaveRobotStats(false);      //save robot stats
      break;

    case STATE_ERROR:
      statusCurr = IN_ERROR;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      setActuator(ACT_CHGRELAY, 0);
      motorMowEnable = false;
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * 20);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * 20);
      // stateMaxiTime = millis() + 5000;
      OdoRampCompute();
      statsMowTimeTotalStart = false; // stop stats mowTime counter
      loadSaveRobotStats(false);      //save robot stats
      break;


    case STATE_PERI_FIND:
      //Don't Use accel when start from forward_odo because the 2 wheels are already running
      //if status is change in pfod need to refresh it in PI
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      ShowMessage("Area In Mowing ");
      ShowMessage(areaInMowing);
      ShowMessage(" Area To Go ");
      ShowMessageln(areaToGo);

      if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_OBSTACLE_AVOID)) {
        UseAccelRight = 0;
        UseAccelLeft = 0;
      }
      else {
        UseAccelRight = 1;
        UseAccelLeft = 1;
      }

      UseBrakeLeft = 0;
      UseBrakeRight = 0;

      motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.2;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.2;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 30000);//300 ml
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 30000);
      OdoRampCompute();


      motorMowEnable = false;
      break;


    case STATE_PERI_TRACK:
      rollToTrackQty = 0 ;
      perimeterPID.reset();
      if (track_ClockWise) {
        PeriOdoIslandDiff =  odometryRight - odometryLeft;
      }
      else {
        PeriOdoIslandDiff =  odometryLeft - odometryRight ;
      }
      break;

    case STATE_WAIT_AND_REPEAT:
      //ShowMessageln("WAIT AND REPEAT  ");

      break;

    case STATE_WAIT_COVER:

      ShowMessageln("Close cover to start");
      setBeeper(10000, 300, 300, 1000, 0);//beep for 2 sec
      break;

    //bber202
    case STATE_ACCEL_FRWRD:
      //use to start mow motor at low speed and limit noise on perimeter reading on startup
      motorMowSpeedPWMSet = motorMowSpeedMinPwm;
      motorMowPowerMedian.clear();

      // after this state the mower use pid imu to drive straight so accelerate only at half the max speed
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;

      motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight + int(odometryTicksPerRevolution / 4) ;
      stateEndOdometryLeft = odometryLeft + int(odometryTicksPerRevolution / 4) ;

      OdoRampCompute();

      break;

  }  // end switch

  sonarObstacleTimeout = 0;
  // state has changed

  stateStartTime = millis();

  stateNext = stateNew;
  stateLast = stateCurr;
  stateCurr = stateNext;
  perimeterTriggerTime = 0;
  ShowMessage (F(statusNames[statusCurr]));
  ShowMessage (" / ");
  ShowMessageln (F(stateNames[stateCurr]));







  //ShowMessage (" Dir ");
  //ShowMessage (rollDir);
  //ShowMessage (" State changed at ");
  //ShowMessage (stateStartTime);
  //ShowMessage (" From state ");
  //ShowMessageln (F(stateNames[stateLast]));

}


void Robot::writeOnSD(String message) {
  if (sdCardReady) {
    //filename is reset into :loadSaveRobotStats
    File dataFile = SD.open(historyFilenameChar, FILE_WRITE);
    if (dataFile) {
      dataFile.print(message);
      dataFile.close();
    }
    totalLineOnFile = totalLineOnFile + 1;
    if (totalLineOnFile >= 1000) { // create a new log file if too long
      totalLineOnFile = 0;
      sprintf(historyFilenameChar, "%02u%02u%02u%02u%02u.txt", datetime.date.year - 2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);
    }
  }
}

void Robot::writeOnSDln(String message) {
  if (sdCardReady) {
    //filename is reset into :loadSaveRobotStats
    File dataFile = SD.open(historyFilenameChar, FILE_WRITE);
    if (dataFile) {
      dataFile.println(message);
      dataFile.close();
    }
    totalLineOnFile = totalLineOnFile + 1;
    if (totalLineOnFile >= 1000) { // create a new log file if too long
      totalLineOnFile = 0;
      sprintf(historyFilenameChar, "%02u%02u%02u%02u%02u.txt", datetime.date.year - 2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);
    }
  }
}




void Robot::ShowMessage(String message) {
  writeOnSD (message);
  Serial.print (message);
  if (ConsoleToPfod) {
    Bluetooth.print (message);

  }
}
void Robot::ShowMessageln(String message) {
  writeOnSDln (message);
  Serial.println(message);
  if (ConsoleToPfod) {
    Bluetooth.println(message);
  }
}

void Robot::ShowMessage(float value) {
  writeOnSD (value);
  Serial.print (value);
  if (ConsoleToPfod) {
    Bluetooth.print (value);
  }
}
void Robot::ShowMessageln(float value) {
  writeOnSDln (value);
  Serial.println(value);
  if (ConsoleToPfod) {
    Bluetooth.println(value);
  }
}

// check battery voltage and decide what to do


void Robot::checkBattery() {

  if ((millis() < nextTimeCheckBattery) || (millis() < 30000)) return; //  wait 30 sec after the initial power on before first check to avoid read bad battery voltage
  nextTimeCheckBattery = millis() + 1000; //if change need to adjust the line idleTimeSec= idleTimeSec+1;

  if (batMonitor) {
    // if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_OFF) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING))  {
    if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_OFF))   {
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Switch OFF Voltage : "));
      ShowMessage(batSwitchOffIfBelow);
      ShowMessageln(F("  Bat Voltage is very low the state is changed to OFF, so the undervoltage timer start"));
      addErrorCounter(ERR_BATTERY);
      setBeeper(3000, 500, 0, 2000, 0);//beep for 3 sec
      setNextState(STATE_OFF, 0);
    }
    else if ((batVoltage < batGoHomeIfBelow) && (stateCurr == STATE_FORWARD_ODO) && (perimeterUse)) {    //actualy in mowing mode with station and perimeter
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Minimum Mowing Voltage : "));
      ShowMessageln(batGoHomeIfBelow);
      ShowMessageln(F(" Bat Voltage is low : The mower search the charging Station"));
      setBeeper(3000, 500, 100, 2000, 100);//beep for 3 sec
      statusCurr = BACK_TO_STATION;
      areaToGo = 1;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      periFindDriveHeading = imu.ypr.yaw;
      setNextState(STATE_PERI_FIND, 0);
    }


    // if robot is OFF or Error  we can start to count before shutdown
    if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR)) {
      /*
            ShowMessage("Count before power OFF  ");
            ShowMessage(idleTimeSec);
            ShowMessage(" / ");
            ShowMessageln(batSwitchOffIfIdle * 60);
      */
      if (idleTimeSec != BATTERY_SW_OFF) { // battery already switched off?
        idleTimeSec = idleTimeSec + 1; // this loop is execute each 1 seconde so add 1 second to idle time
        if (idleTimeSec > batSwitchOffIfIdle * 60) {

          if (RaspberryPIUse) {
            ShowMessageln(F("Battery IDLE trigger "));
            ShowMessageln(F("PCB power OFF after 30 secondes Wait Until PI Stop "));
            MyRpi.sendCommandToPi("PowerOffPi");
            delayWithWatchdog(30000);//wait 30Sec  until pi is OFF or the USB native power again the due and the undervoltage never switch OFF
          }
          else
          {
            ShowMessageln(F("PCB power OFF immediatly"));
          }
          setBeeper(3000, 3000, 0, 2000, 0);
          loadSaveErrorCounters(false); // saves error counters
          loadSaveRobotStats(false);    // saves robot stats
          idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
          ShowMessageln(F("BATTERY switching OFF"));
          delayWithWatchdog(2000);
          Serial1.end();//disconnect BT
          delayWithWatchdog(3000);
          setActuator(ACT_BATTERY_SW, 0);  // switch off battery
        }
      }
    }
    else
    {
      resetIdleTime();
    }
  }
}


// check robot stats
void Robot::checkRobotStats() {
  if (millis() < nextTimeRobotStats) return;
  nextTimeRobotStats = millis() + 60000;

  //----------------stats mow time------------------------------------------------------
  statsMowTimeHoursTotal = double(statsMowTimeMinutesTotal) / 60;
  if (statsMowTimeTotalStart) {
    statsMowTimeMinutesTripCounter++;
    statsMowTimeMinutesTrip = statsMowTimeMinutesTripCounter;
    statsMowTimeMinutesTotal++;
  }
  else if (statsMowTimeMinutesTripCounter != 0) {
    statsMowTimeMinutesTripCounter = 0;

  }

  //---------------stats Battery---------------------------------------------------------
  if ((stateCurr == STATE_STATION_CHARGING) && (stateTime >= 60000)) { // count only if mower is charged longer then 60sec
    statsBatteryChargingCounter++; // temporary counter

    if (statsBatteryChargingCounter == 1) statsBatteryChargingCounterTotal += 1;
    statsBatteryChargingCapacityTrip = batCapacity;
    statsBatteryChargingCapacityTotal += (batCapacity - lastTimeBatCapacity); // summ up only the difference between actual batCapacity and last batCapacity
    lastTimeBatCapacity = batCapacity;

  }
  else {                        // resets values to 0 when mower is not charging
    statsBatteryChargingCounter = 0;
    batCapacity = 0;
  }
  /*
    if (isnan(statsBatteryChargingCapacityTrip)) statsBatteryChargingCapacityTrip = 0;
    if (isnan(statsBatteryChargingCounterTotal)) statsBatteryChargingCounterTotal = 0; // for first run ensures that the counter is 0
    if (isnan(statsBatteryChargingCapacityTotal)) statsBatteryChargingCapacityTotal = 0; // for first run ensures that the counter is 0
  */
  if (statsBatteryChargingCapacityTotal <= 0 || statsBatteryChargingCounterTotal == 0) statsBatteryChargingCapacityAverage = 0; // make sure that there is no dividing by zero
  else statsBatteryChargingCapacityAverage = statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;

  //----------------new stats goes here------------------------------------------------------
  //mowPatternJustChange;
  mowPatternDuration++;


}


void Robot::reverseOrBidir(byte aRollDir) {

  if (stateCurr == STATE_PERI_OUT_ROLL_TOINSIDE) {
    ShowMessageln("Bumper hit ! try roll in other dir");
    setNextState(STATE_WAIT_AND_REPEAT, aRollDir);
    return;
  }

  if (mowPatternCurr == MOW_LANES) setNextState(STATE_STOP_ON_BUMPER, rollDir);
  else  setNextState(STATE_STOP_ON_BUMPER, aRollDir);
}

// check motor current
void Robot::checkCurrent() {
  if (millis() < nextTimeCheckCurrent) return;
  nextTimeCheckCurrent = millis() + 100;
  // *************MOW MOTOR***********************
  if ((statusCurr == NORMAL_MOWING) && (!highGrassDetect)) {  //do not start the spirale if in tracking and motor detect high grass
    if (motorMowPower >= 0.8 * motorMowPowerMax) {
      spiraleNbTurn = 0;

      highGrassDetect = true;
      ShowMessageln("Warning  motorMowPower >= 0.8 * motorMowPowerMax ");
      ////  http://forums.parallax.com/discussion/comment/1326585#Comment_1326585
    }
    else {
      if ((spiraleNbTurn >= 8)) {
        spiraleNbTurn = 0;

        highGrassDetect = false; //stop the spirale
      }
    }
  }

  // if (motorMowPower >= motorMowPowerMax)
  if ((motorMowEnable) && (motorMowPower >= motorMowPowerMax))
  {
    motorMowSenseCounter++;
    ShowMessage("Warning  motorMowPower >= motorMowPowerMax : ");
    ShowMessageln(motorMowSenseCounter);
  }
  else
  {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    motorMowSenseCounter = 0;
    if ((lastTimeMotorMowStuck != 0) && (millis() >= lastTimeMotorMowStuck + 60000)) { // wait 60 seconds before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      if ((stateCurr == STATE_FORWARD_ODO)) { //avoid risq of restart not allowed
        motorMowEnable = true;
        lastTimeMotorMowStuck = 0;
        ShowMessageln("Time to restart the mow motor after the 60 secondes pause");
      }
    }
  }
  //need to check this
  if (motorMowSenseCounter >= 10) { //ignore motorMowPower for 1 seconds
    motorMowEnable = false;
    ShowMessageln("Motor mow power overload. Motor STOP and try to start again after 1 minute");
    addErrorCounter(ERR_MOW_SENSE);
    lastTimeMotorMowStuck = millis();
  }

  //**************drive motor***********************
  //bb add test current in manual mode and stop immediatly
  if (statusCurr == MANUAL) {
    if (motorLeftPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);
      setMotorPWM(0, 0);
      setNextState(STATE_OFF, 0);

    }
    if (motorRightPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);
      setMotorPWM(0, 0);
      setNextState(STATE_OFF, 0);

    }
  }


  // here in auto mode
  if (millis() > stateStartTime + motorPowerIgnoreTime) {

    //Motor right****************************************************************
    //First react test to 80 % powerMax
    if (motorRightPower >= 0.8 * motorPowerMax)
    {
      motorRightSenseCounter++;
      setBeeper(500, 500, 0, 2000, 0);
      setMotorPWM(0, 0);
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        }
        else
        {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(LEFT);
        }
      }
    }
    else
    {
      setBeeper(0, 0, 0, 0, 0);
      motorRightSenseCounter = 0; // the sense is OK reset all the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorRightPower >= motorPowerMax) {
      motorRightSenseCounter++;
      // setMotorPWM(0, 0);
      //addErrorCounter(ERR_MOTOR_RIGHT);
      //setNextState(STATE_ERROR, 0);
      ShowMessage("Warning: Motor Right power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorRightSenseCounter);

    }


    //Motor left****************************************************************
    //First react test to 80 % powerMax
    if (motorLeftPower >= 0.8 * motorPowerMax)
    {
      motorLeftSenseCounter++;
      setBeeper(1000, 1000, 0, 2000, 0);
      setMotorPWM(0, 0);
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        }
        else
        {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(RIGHT);
        }
      }
    }
    else
    {
      setBeeper(0, 0, 0, 0, 0);
      motorLeftSenseCounter = 0; // the sense is OK reset the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorLeftPower >= motorPowerMax) {
      motorLeftSenseCounter++;
      // setMotorPWM(0, 0);
      //addErrorCounter(ERR_MOTOR_LEFT);
      //setNextState(STATE_ERROR, 0);
      ShowMessage("Warning: Motor Left power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorLeftSenseCounter);
    }
    //final test on the counter to generate the error and stop the mower
    if (motorLeftSenseCounter >= 10) { //the motor is stuck for more than 1 seconde 10 * 100 ms go to error.
      ShowMessage("Fatal Error: Motor Left power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorLeftPower);
      addErrorCounter(ERR_MOTOR_LEFT);
      setMotorPWM(0, 0);
      setNextState(STATE_ERROR, 0);
    }
    if (motorRightSenseCounter >= 10) { //the motor is stuck for more than 1 seconde go to error.
      ShowMessage("Fatal Error: Motor Right power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorRightPower);
      addErrorCounter(ERR_MOTOR_RIGHT);
      setMotorPWM(0, 0);
      setNextState(STATE_ERROR, 0);
    }

  } //motorpower ignore time

}

// check bumpers
void Robot::checkBumpers() {
  if ((millis() < 3000) || (!bumperUse)) return;

  /*
    if (stateCurr=STATE_PERI_OUT_ROLL_TOINSIDE){
    if (bumperLeft) {
      rollDir=RIGHT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
       setMotorPWM(0, 0);

      return;
    }
    if (bumperRight){
      rollDir=LEFT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
       setMotorPWM(0, 0);

      return;
    }



    }

  */

  if ((bumperLeft || bumperRight || bumperRearLeft || bumperRearRight)) {
    if (statusCurr == MANUAL) {
      ShowMessageln("Bumper trigger in Manual mode ?????????");
      setNextState(STATE_OFF, 0); //the bumper stop all in manual mode
    }
    else
    {
      spiraleNbTurn = 0;
      highGrassDetect = false;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM(0, 0);
      if (bumperLeft || bumperRearLeft) {
        ShowMessageln("Bumper left trigger");
        reverseOrBidir(LEFT);
      } else {
        ShowMessageln("Bumper right trigger");
        reverseOrBidir(RIGHT);
      }
    }

  }
}


// check bumpers while tracking perimeter
void Robot::checkBumpersPerimeter() {
  if ((bumperLeft || bumperRight || bumperRearLeft || bumperRearRight)) { // the bumper is used to detect the station
    motorLeftRpmCurr = motorRightRpmCurr = 0 ;
    setMotorPWM(0, 0);//stop immediatly and station check to see if voltage on pin
    ShowMessageln("Bump on Something check for station");
    setNextState(STATE_STATION_CHECK, rollDir);
    return;
  }

  //if (!UseBumperDock) {   // read the station voltage
  //bber300
  if ((powerboard_I2c_line_Ok) && (millis() >= nextTimeReadStationVoltage)) {
    nextTimeReadStationVoltage = millis() + 20;
    chgVoltage = ChargeIna226.readBusVoltage() ;
  }
  if (chgVoltage > 5) {
    motorLeftRpmCurr = motorRightRpmCurr = 0 ;
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
    setMotorPWM(0, 0);//stop immediatly and wait 2 sec to see if voltage on pin
    ShowMessageln("Detect a voltage on charging contact");
    setNextState(STATE_STATION_CHECK, rollDir);
    return;
  }
  //}
}

//bber401
void Robot::checkStuckOnIsland() {
  //6 * is a test value
  //bber600
  if (track_ClockWise) {
    if ((odometryRight - odometryLeft) - PeriOdoIslandDiff > 6 * odometryTicksPerRevolution) {
      ShowMessageln("Left wheel is 6 full revolution more than Right one --> Island  ??? ");
      newtagRotAngle1 = 90;
      setNextState(STATE_PERI_STOP_TOROLL, 0);
      return;
    }
  }
  else {
    if ((odometryLeft - odometryRight) - PeriOdoIslandDiff > 6 * odometryTicksPerRevolution) {
      ShowMessageln("Right wheel is 6 full revolution more than left one --> Island  ??? ");
      newtagRotAngle1 = 90;
      setNextState(STATE_PERI_STOP_TOROLL, 1);
      return;
    }
  }
}

// check perimeter as a boundary
void Robot::checkPerimeterBoundary() {

  if ((millis() >= nextTimeRotationChange) && (stateCurr == STATE_FORWARD_ODO) && (mowPatternCurr != MOW_LANES)) {// change only when in straight line and random mode
    nextTimeRotationChange = millis() + 600000;  // in random change each 10 minutes
    if (rollDir == LEFT) rollDir = RIGHT; //invert the next rotate
    else rollDir = LEFT;
    ShowMessage(millis());
    ShowMessageln(" Rotation direction change ");
  }
  //bber2
  if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_MOW_SPIRALE) ) {
    //bber200
    //speed coeff between 0.7 and 1 according 50% of perimetermagmax
    if ((millis() >= nextTimeCheckperimeterSpeedCoeff) && (reduceSpeedNearPerimeter)) {
      int miniValue = (int)perimeterMagMaxValue / 5;
      if (perimeter.getSmoothMagnitude(0) > perimeter.getSmoothMagnitude(1)) {
        perimeterSpeedCoeff = (float) map(perimeter.getSmoothMagnitude(0), miniValue, perimeterMagMaxValue, 100, 70) / 100;
      }
      else {
        perimeterSpeedCoeff = (float) map(perimeter.getSmoothMagnitude(1), miniValue, perimeterMagMaxValue, 100, 70) / 100;
      }

      if (perimeterSpeedCoeff < 0.7) {
        perimeterSpeedCoeff = 0.7;
        nextTimeCheckperimeterSpeedCoeff = millis() + 500; //avoid speed coeff increase when mower go accross the wire
      }
      else
      {
        nextTimeCheckperimeterSpeedCoeff = millis() + 15;
      }
      if (perimeterSpeedCoeff > 1) perimeterSpeedCoeff = 1;
    }


    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        ShowMessage(F("Perimeter trigger at : "));
        ShowMessageln(millis());
        //reinit spirale mowing
        spiraleNbTurn = 0;
        highGrassDetect = false; //stop the spirale
        if (stateCurr == STATE_MOW_SPIRALE) {
          setNextState(STATE_PERI_OUT_STOP, 0); //rollDir is always left when spirale mowing ?????
        }
        else {

          if (perimeterRightTriggerTime <= perimeterLeftTriggerTime) { // the right coil detect the wire before the left one
            if (mowPatternCurr == MOW_RANDOM) {
              rollDir = RIGHT;
              setNextState(STATE_PERI_OUT_STOP, rollDir); //rollDir change
            }
            else {
              setNextState(STATE_PERI_OUT_STOP, rollDir); //rollDir don't change in lane mode
            }
          }
          else {
            if (mowPatternCurr == MOW_RANDOM) {
              rollDir = LEFT;
              setNextState(STATE_PERI_OUT_STOP, rollDir); //rollDir change
            }
            else {
              setNextState(STATE_PERI_OUT_STOP, rollDir); //rollDir don't change in lane mode
            }
          }

        }
        return;
      }
    }
  }

  if ((stateCurr == STATE_PERI_OBSTACLE_AVOID)) { //when start in auto mode and quit the station
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        setNextState(STATE_PERI_STOP_TOTRACK, rollDir);
        return;
      }
    }
  }


  if (stateCurr == STATE_SONAR_TRIG) {  //wire is detected during the sonar braking need to stop immediatly
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        setNextState(STATE_STOP_ON_BUMPER, rollDir);  //use to stop immediatly
        return;
      }
    }
  }

  else if ((stateCurr == STATE_ROLL)) {
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        ShowMessageln("Pourquoi je suis la ?? ?? ?? ?? ?? ?? ?? ?? ");
        setMotorPWM(0, 0);
        setNextState(STATE_PERI_OUT_REV, rollDir);
        return;

      }
    }
  }

}






void Robot::checkRain() {
  if (!rainUse) return;
  if (rain) {
    ShowMessageln(F("RAIN"));
    areaToGo = 1;
    if (perimeterUse) {
      periFindDriveHeading = imu.ypr.yaw;
      setNextState(STATE_PERI_FIND, 0);
    }
    else {
      setNextState(STATE_OFF, 0);
    }
  }
}
void Robot::checkSonarPeriTrack() {
  if (!sonarUse) return;
  /*
    if (millis() < nextTimeCheckSonar) return;
    nextTimeCheckSonar = millis() + 200;
    sonarDistRight = NO_ECHO;
    sonarDistLeft = NO_ECHO;
    if (track_ClockWise) { //Track CW
    if (sonarRightUse) {
      sonarDistRight = readSensor(SEN_SONAR_RIGHT);
      if (sonarDistRight < 20 || sonarDistRight > 110) sonarDistRight = NO_ECHO; // Object is too close to the sensor JSN SR04T can't read <20 CM . Sensor value is useless
    }

    else sonarDistRight = NO_ECHO;
    }
    else {//track CCW
    if (sonarLeftUse) { //use the left sonar
      sonarDistLeft = readSensor(SEN_SONAR_LEFT);
      if (sonarDistLeft < 20 || sonarDistLeft  > 110) sonarDistLeft = NO_ECHO;
    }
    else sonarDistLeft = NO_ECHO;
    }

    if (((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) ||  ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow))  ) {
    //setBeeper(1000, 50, 50, 60, 60);
    Serial.println("Sonar reduce speed on tracking for 2 meters");
    whereToResetSpeed =  totalDistDrive + 200; // when a speed tag is read it's where the speed is back to maxpwm value

    nextTimeCheckSonar = millis() + 3000;  //wait before next reading
    ActualSpeedPeriPWM = MaxSpeedperiPwm * dockingSpeed / 100;
    trakBlockInnerWheel = 1; //don't want that a wheel reverse just before station check   /bber30


    }
  */
}

// check sonar
void Robot::checkSonar() {
  if (!sonarUse) return;
  if (millis() < nextTimeCheckSonar) return;
  nextTimeCheckSonar = millis() + 100;
  sonarSpeedCoeff = 1;

  if (sonarRightUse) sonarDistRight = NewSonarRight.ping_cm();
  else sonarDistRight = NO_ECHO;
  if (sonarLeftUse) sonarDistLeft = NewSonarLeft.ping_cm();
  else sonarDistLeft = NO_ECHO ;


  if (stateCurr == STATE_OFF) return; //avoid the mower move when testing

  if (sonarDistRight < 25 || sonarDistRight > 90) sonarDistRight = NO_ECHO; // Object is too close to the sensor JSN SR04T can't read <20 CM . Sensor value is useless
  if (sonarDistLeft < 25 || sonarDistLeft  > 90) sonarDistLeft = NO_ECHO;

  if (((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) ||  ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow))  ) {
    setBeeper(1000, 100, 100, 2000, 50);//beep for 3 sec
    nextTimeCheckSonar = millis() + 1500;  //wait before next reading

    // **************************if sonar during spirale reinit spirale variable*****************
    spiraleNbTurn = 0;

    highGrassDetect = false; //stop the spirale
    // *********************************************************************************
    if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_MOW_SPIRALE)) {
      //avoid the mower move when testing

      if ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) {  //right
        if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.70;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {
          distToObstacle =  sonarDistRight;
          ShowMessage("Sonar Right Trigger at cm : ");
          ShowMessageln (distToObstacle);
          if (mowPatternCurr == MOW_LANES) setNextState(STATE_SONAR_TRIG, rollDir); //don't change the rotation if lane mowing
          else setNextState(STATE_SONAR_TRIG, LEFT);
          return;
        }
      }
      if ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow)) {  //LEFT
        if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.70;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {
          distToObstacle =  sonarDistLeft;
          ShowMessage("Sonar Left Trigger at cm : ");
          ShowMessageln (distToObstacle);
          if (mowPatternCurr == MOW_LANES) setNextState(STATE_SONAR_TRIG, rollDir); //don't change the rotation if lane mowing
          else setNextState(STATE_SONAR_TRIG, RIGHT);
          return;
        }
      }
    }
  }

}






// check IMU (tilt)
void Robot::checkTilt() {
  if (!imuUse) return;
  if (millis() < nextTimeCheckTilt) return;

  nextTimeCheckTilt = millis() + 200; // 5Hz same as nextTimeImu
  int pitchAngle = (imu.ypr.pitch / PI * 180.0);
  int rollAngle  = (imu.ypr.roll / PI * 180.0);
  //at 40 deg mower start to reverse ,so do not test if not in mowing condition
  if ((stateCurr != STATE_MANUAL) && (stateCurr != STATE_OFF) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING)) {
    if ( (abs(pitchAngle) > 70) || (abs(rollAngle) > 70) ) {
      nextTimeCheckTilt = millis() + 5000; // avoid repeat
      ShowMessage(F("ERROR : IMU Roll / Tilt -- > "));
      ShowMessage(rollAngle);
      ShowMessage(F(" / "));
      ShowMessageln(pitchAngle);
      addErrorCounter(ERR_IMU_TILT);
      ShowMessageln("Mower STOP");
      motorMowEnable = false;
      setNextState(STATE_ERROR, 0);
      pitchAngle = 0;
      rollAngle = 0;
    }

    if ( (abs(pitchAngle) > 40) || (abs(rollAngle) > 40) ) {
      nextTimeCheckTilt = millis() + 5000; // avoid repeat
      ShowMessage(F("Warning : IMU Roll / Tilt -- > "));
      ShowMessage(rollAngle);
      ShowMessage(F(" / "));
      ShowMessageln(pitchAngle);
      addErrorCounter(ERR_IMU_TILT);
      ShowMessageln("Motor mow STOP start again after 1 minute");
      motorMowEnable = false;
      lastTimeMotorMowStuck = millis();
      reverseOrBidir(rollDir);

    }
  }

}




// calculate map position by odometry sensors
void Robot::calcOdometry() {

  if ((millis() < nextTimeOdometry) || (stateCurr == STATE_OFF)) return;
  nextTimeOdometry = millis() + 100; //bb 300 at the original but test less
  static int lastOdoLeft = 0;
  static int lastOdoRight = 0;
  int odoLeft = odometryLeft;
  int odoRight = odometryRight;
  int ticksLeft = odoLeft - lastOdoLeft;
  int ticksRight = odoRight - lastOdoRight;
  lastOdoLeft = odoLeft;
  lastOdoRight = odoRight;
  double left_cm = ((double)ticksLeft) / ((double)odometryTicksPerCm);
  double right_cm = ((double)ticksRight) / ((double)odometryTicksPerCm);
  double avg_cm  = (left_cm + right_cm) / 2.0;
  double wheel_theta = (left_cm - right_cm) / ((double)odometryWheelBaseCm);
  //odometryTheta += wheel_theta;
  odometryTheta = scalePI(odometryTheta - wheel_theta);


  motorLeftRpmCurr  = double ((( ((double)ticksLeft) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  motorRightRpmCurr = double ((( ((double)ticksRight) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  lastMotorRpmTime = millis();
  if (stateCurr == STATE_PERI_TRACK)  totalDistDrive = totalDistDrive + int(avg_cm);
  currDistToDrive = currDistToDrive + int(avg_cm);
  if (imuUse) {
    odometryX += avg_cm * sin(prevYawCalcOdo);
    odometryY += avg_cm * cos(prevYawCalcOdo);
    //prevYawCalcOdo = imu.ypr.yaw;

  } else {
    // FIXME: theta should be old theta, not new theta?
    odometryX += avg_cm * sin(odometryTheta);
    odometryY += avg_cm * cos(odometryTheta);
  }


}

void Robot::ResetWatchdog() {
  wdt.feed();
}


void Robot::readAllTemperature() {
  if (millis() > nextTimeReadTemperature) {
    nextTimeReadTemperature = millis() + 3000;
    temperatureTeensy = InternalTemperature.readTemperatureC();
    imu.readImuTemperature();

    if ((temperatureTeensy >= 0.9 * maxTemperature) && (stateCurr == STATE_FORWARD_ODO)) { // at 90% of max temp mower try to find the station
      ShowMessageln("Temperature is 90 % of max ");
      ShowMessageln("Mower search the station ");
      ShowMessage("Maxi Setting = ");
      ShowMessage(maxTemperature);
      ShowMessage(" Actual Temperature = ");
      ShowMessageln(temperatureTeensy);
      nextTimeReadTemperature = nextTimeReadTemperature + 180000; // do not read again the temp for the next 3 minute and set the idle bat to 2 minute to poweroff the PCB
      periFindDriveHeading = scalePI(imu.ypr.yaw);
      areaToGo = 1;
      whereToStart = 99999;
      nextTimeTimer = millis() + 3600000;
      statusCurr = BACK_TO_STATION;
      setNextState(STATE_PERI_FIND, 0);
      return;
    }

    if (temperatureTeensy >= maxTemperature) {
      ShowMessageln("Temperature too high ***************");
      ShowMessageln("PCB AutoStop in the next 2 minutes *");
      ShowMessage("Maxi Setting = ");
      ShowMessage(maxTemperature);
      ShowMessage(" Actual Temperature = ");
      ShowMessageln(temperatureTeensy);
      nextTimeReadTemperature = nextTimeReadTemperature + 180000; // do not read again the temp for the next 3 minute and set the idle bat to 2 minute to poweroff the PCB
      batSwitchOffIfIdle = 2; //use to switch all off after 2 minutes
      setNextState(STATE_ERROR, 0);
      return;
    }

  }
}
void Robot::checkTimeout() {
  if (stateTime > motorForwTimeMax) {
    ShowMessageln("Mower run for a too long duration ?");
    setNextState(STATE_PERI_OUT_STOP, !rollDir); // toggle roll dir
  }
}






void Robot::loop()  {
  stateTime = millis() - stateStartTime;
  int steer;

  if ((useMqtt) && (!sdcardToPfod) && (!ConsoleToPfod) && (millis() > next_time_refresh_mqtt)) {
    next_time_refresh_mqtt = millis() + 3000;
    String line01 = "#RMSTA," + String(statusNames[statusCurr]) + "," + String(stateNames[stateCurr]) + "," + String(temperatureTeensy) + "," + String(batVoltage) + "," + String(loopsPerSec)  ;
    Bluetooth.println(line01);


  }

  if (RaspberryPIUse) {
    MyRpi.run();
    if ((millis() > 28000) && (!MyrpiStatusSync)) { // on initial powerON DUE start faster than PI , so need to send again the status to refresh
      MyRpi.SendStatusToPi();
      MyrpiStatusSync = true;
    }
  }
  else {
    readSerial();
  }


  rc.readSerial();// see the readserial function into pfod.cpp


  readSensors();
  readAllTemperature();
  checkRobotStats();
  checkPerimeterBoundary();
  calcOdometry();
  checkOdometryFaults();
  checkButton();
  motorMowControl();
  checkTilt();
  if ((stateCurr == STATE_PERI_OUT_STOP) && (statusCurr == NORMAL_MOWING)) { //read only timer here for fast processing on odo
    checkTimer();
  }
  beeper();

  // IMU MANAGEMENT

  if ((stateCurr != STATE_START_FROM_STATION) && (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION) && (stateCurr != STATE_PERI_TRACK)) {
    if ((imuUse) && (millis() >= nextTimeImuLoop)) {
      nextTimeImuLoop = millis() + 50;
      StartReadAt = millis();
      imu.run();
      EndReadAt = millis();
      ReadDuration = EndReadAt - StartReadAt;
      if ( ReadDuration > 30) {
        ShowMessage("Error IMU read duration >30 ms : ");
        ShowMessageln(ReadDuration);
        ShowMessageln ("IMU and RFID are DEACTIVATE Mow in safe mode");
        imuUse = false;
        rfidUse = false;
        addErrorCounter(ERR_IMU_COMM);
      }
    }
  }

  if (gpsUse) {

    if (gps.feed()) {
      processGPSData();
    }

  }

  if ((Enable_Screen) && (millis() >= nextTimeScreen))   { // warning : refresh screen take 40 ms
    nextTimeScreen = millis() + 250;
    StartReadAt = millis();

    if ((statusCurr == WAIT) || (statusCurr == MANUAL) || (statusCurr == REMOTE) || (statusCurr == TESTING) || (statusCurr == WAITSIG2)) {
      MyScreen.refreshWaitScreen();
    }
    if ((statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (statusCurr == WIRE_MOWING)) {
      MyScreen.refreshMowScreen();
      nextTimeScreen = millis() + 500; // in mowing mode don't need a big refresh rate and avoid trouble on loop
    }
    if ((statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START) ) {
      MyScreen.refreshTrackScreen();
      nextTimeScreen = millis() + 500;
    }
    if (statusCurr == IN_ERROR ) {
      MyScreen.refreshErrorScreen();
    }
    if (statusCurr == IN_STATION) {
      MyScreen.refreshStationScreen();
    }

    EndReadAt = millis();
    ReadDuration = EndReadAt - StartReadAt;
    //ShowMessage("Screen Duration ");
    //ShowMessageln(ReadDuration);



  }


  if (millis() >= nextTimeInfo) {
    if ((millis() - nextTimeInfo > 250)) {
      if (developerActive) {
        ShowMessage("------ LOOP NOT OK DUE IS OVERLOAD -- Over 1 sec ");
        ShowMessageln((millis() - nextTimeInfo));
      }
    }
    nextTimeInfo = millis() + 1000; //1000

    printInfo(Serial);
    checkErrorCounter();
    //if (stateCurr == STATE_REMOTE) printRemote();
    loopsPerSec = loopsPerSecCounter;
    loopsPerSecCounter = 0;
  }

  rc.run();



  // state machine - things to do *PERMANENTLY* for current state
  // robot state machine

  switch (stateCurr) {

    case STATE_ERROR:
      // fatal-error
      checkBattery();
      if (millis() >= nextTimeErrorBeep) {
        nextTimeErrorBeep = millis() + 5000;
        setBeeper(5000, 200, 200, 2000, 1000);//beep for 5 secs
      }
      motorControlOdo();
      break;

    case STATE_OFF:
      // robot is turned off
      if ((batMonitor) && (millis() - stateStartTime > 2000)) { //the charger is plug
        if (chgVoltage > 5.0)   {
          setNextState(STATE_STATION, 0);
          return;
        }
      }
      imuDriveHeading = imu.ypr.yaw / PI * 180;
      motorControlOdo();
      //bber13
      motorMowEnable = false; //to stop mow motor in OFF mode by pressing OFF again (the one shot OFF is bypass)
      checkSonar();  // only for test never use or the mower can't stay into the station

      checkBattery();


      break;

    case STATE_REMOTE:
      // remote control mode (RC)
      //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      steer = ((double)motorSpeedMaxRpm / 2) * (((double)remoteSteer) / 100.0);
      if (remoteSpeed < 0) steer *= -1;
      motorLeftSpeedRpmSet  = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) - steer;
      motorRightSpeedRpmSet = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) + steer;
      motorLeftSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorLeftSpeedRpmSet));
      motorRightSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorRightSpeedRpmSet));
      motorMowSpeedPWMSet = ((double)motorMowSpeedMaxPwm) * (((double)remoteMow) / 100.0);
      motorControl();


      break;

    case STATE_MANUAL:
      checkCurrent();
      checkBumpers();
      if (millis() > stateOffAfter) setNextState(STATE_OFF, 0);
      motorControl();
      break;

    case STATE_FORWARD:  // not use 04/11/22
      // driving forward
      checkRain();
      checkCurrent();
      checkBumpers();
      checkTimeout();
      motorControl();
      break;







    case STATE_FORWARD_ODO:
      // driving forward with odometry control



      motorControlOdo();


      //manage the imu////////////////////////////////////////////////////////////
      if (imuUse ) {
        //when findedYaw = 999 it's mean that the lane is changed and the imu need to be adjusted to the compass
        //Avoid to stop near perimeter to avoid periout error at the end of calibration
        if ((findedYaw == 999) && (imu.ypr.yaw > 0) && (abs(perimeterMagLeft) <= perimeterMagMaxValue / 2) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000) && (mowPatternCurr == MOW_LANES)) { //try to find compass yaw
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;
        }

        //-----------here and before reverse the mower is stop so mark a pause to autocalibrate DMP-----------
        if ((millis() > nextTimeToDmpAutoCalibration) && (abs(perimeterMagLeft) <= perimeterMagMaxValue / 2) && (mowPatternCurr == MOW_LANES) && (imu.ypr.yaw > 0) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000)  ) {
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;

        }
      }
      //-----------------------------------------------------------------------------
      ////////////////////////////////////////////////////////////////////////////

      //the normal state traitement alternatively the lenght is 300ml or 10 ml for example
      if ((odometryRight > stateEndOdometryRight) || (odometryLeft > stateEndOdometryLeft))
      {
        if ((mowPatternCurr == MOW_LANES) && (!justChangeLaneDir)) {
          ShowMessageln("MAX LANE LENGHT TRIGGER time to reverse");
          setNextState(STATE_ENDLANE_STOP, rollDir);
        }
        else {
          ShowMessageln("more than 300 ML in straight line ?? ?? ?? ?? ?? ??");
          setBeeper(3000, 100, 100, 2000, 50);//beep for 3 sec
          setNextState(STATE_ENDLANE_STOP, rollDir);
        }
      }

      //-----------here need to start to mow in spirale or half lane lenght-----------
      if (highGrassDetect) {
        if ((mowPatternCurr != MOW_LANES)) {
          setNextState(STATE_STOP_BEFORE_SPIRALE, rollDir);
          return;
        }

      }


      checkRain();
      checkCurrent();
      checkBumpers();

      checkSonar();

      //checkLawn();
      checkTimeout();
      checkBattery();

      break;

    case STATE_ESCAPE_LANE:
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) setNextState(STATE_PERI_OUT_STOP, rollDir);
      checkCurrent();
      checkBumpers();
      //checkSonar();

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t escape_lane in time ");
        }
        setNextState(STATE_PERI_OUT_STOP, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROLL_WAIT: //not use ??
      if ((moveRightFinish) && (moveLeftFinish) )  {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          ShowMessage(" OdometryLeft ");
          ShowMessage(odometryLeft);
          ShowMessage(" / stateEndOdometryLeft ");
          ShowMessage(stateEndOdometryLeft);
          ShowMessage(" OdometryRight ");
          ShowMessage(odometryRight);
          ShowMessage(" / stateEndOdometryRight ");
          ShowMessage(stateEndOdometryRight);
          ShowMessage(" yawtofind ");
          ShowMessageln(findedYaw);
          ShowMessage(" odometry find the Opposit Yaw at ");
          ShowMessageln((imu.ypr.yaw / PI * 180));
          setNextState(STATE_OFF, rollDir);
          //setNextState(STATE_FORWARD_ODO, rollDir);
        }
      }
      motorControlOdo();
      break;

    case STATE_CIRCLE:
      // not use
      motorControl();
      break;

    case STATE_PERI_OBSTACLE_REV:
      // perimeter tracking reverse for  x cm
      motorControlOdo();
      if ((moveRightFinish) && (moveLeftFinish) )
      {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)
        { //wait until the 2 motors completly stop
          setNextState(STATE_PERI_OBSTACLE_ROLL, RIGHT);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OBSTACLE_REV in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_ROLL, RIGHT);
      }

      break;

    case STATE_PERI_OBSTACLE_ROLL:
      motorControlOdo();
      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0) { //wait until the 2 motors completly stop
          setNextState(STATE_PERI_OBSTACLE_FORW, 0);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OBSTACLE_ROLL in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_FORW, RIGHT);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_PERI_OBSTACLE_FORW:
      //forward
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)) {
        setNextState(STATE_PERI_OBSTACLE_AVOID, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OBSTACLE_FORW in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_AVOID, RIGHT);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_PERI_OBSTACLE_AVOID:
      //circle arc
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft)) {
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OBSTACLE_AVOID in time ");
        }
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_REVERSE:
      motorControlOdo();

      //if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {

      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (moveLeftFinish) ) {
          if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_ROLL, rollDir);
          }
        }
        else
        {
          if ((moveRightFinish) && (odometryLeft <= stateEndOdometryLeft) ) {
            if (motorRightPWMCurr == 0 ) { //wait until the right motor completly stop because rotation is inverted
              setNextState(STATE_ROLL, rollDir);
            }
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t reverse in time ");
        }
        setNextState(STATE_ROLL, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_ROLL:
      motorControlOdo();

      //if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
      if (rollDir == RIGHT) {
        if ((moveRightFinish) && (odometryLeft >= stateEndOdometryLeft) ) {
          if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_FORWARD_ODO, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (moveLeftFinish) ) {
          if (motorLeftPWMCurr == 0 ) {
            setNextState(STATE_FORWARD_ODO, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll in time ");
        }
        setNextState(STATE_FORWARD_ODO, rollDir);
      }

      break;


    case STATE_ROLL_TONEXTTAG:
      motorControlOdo();

      if ((moveRightFinish) && (odometryLeft >= stateEndOdometryLeft) ) {
        if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

          setNextState(STATE_PERI_FIND, rollDir);
        }
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll in time ");
        }
        setNextState(STATE_PERI_FIND, rollDir);//if the motor can't rech the odocible in slope
      }

      break;

    case STATE_ROLL1_TO_NEWAREA:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0) { //wait until the left motor completly stop because rotation is inverted
          setNextState(STATE_DRIVE1_TO_NEWAREA, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll in time ");
        }
        setNextState(STATE_DRIVE1_TO_NEWAREA, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROLL2_TO_NEWAREA:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
          setNextState(STATE_DRIVE2_TO_NEWAREA, rollDir);
        }
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll in time ");
        }
        setNextState(STATE_DRIVE2_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_DRIVE1_TO_NEWAREA:
      motorControlOdo();
      if (currDistToDrive >= newtagDistance1) { // time to brake
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t DRIVE1_TO_NEWAREA in time ");
        }
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_DRIVE2_TO_NEWAREA:
      motorControlOdo();
      if (currDistToDrive >= newtagDistance2) { // time to brake
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t DRIVE2_TO_NEWAREA in time ");
        }
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      break;



    case STATE_STOP_TO_NEWAREA:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (stateLast == STATE_DRIVE1_TO_NEWAREA) {  //2 possibility
            setNextState(STATE_ROLL2_TO_NEWAREA, rollDir);
          }
          else {
            setNextState(STATE_WAIT_FOR_SIG2, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t  stop ON BUMPER in time ");
        }
        if (stateLast == STATE_DRIVE1_TO_NEWAREA) {
          setNextState(STATE_ROLL2_TO_NEWAREA, rollDir);
        }
        else {
          setNextState(STATE_WAIT_FOR_SIG2, rollDir);
        }
      }


      break;

    case STATE_WAIT_FOR_SIG2:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (millis() >= nextTimeReadSmoothPeriMag) {
            nextTimeReadSmoothPeriMag = millis() + 1000;
            smoothPeriMag = perimeter.getSmoothMagnitude(0);
            ShowMessage("SmoothMagnitude =  ");
            ShowMessageln(smoothPeriMag);
            if ((perimeterInsideLeft) && (smoothPeriMag > 250)) //check if signal here and inside need a big value to be sure it is not only noise
            {
              if (areaToGo == 1) {
                statusCurr = BACK_TO_STATION; //if we are in the area1 it is to go to station
                periFindDriveHeading = imu.ypr.yaw;
              }
              else
              {
                areaInMowing = areaToGo;
                statusCurr = TRACK_TO_START;
              }
              if (RaspberryPIUse) MyRpi.SendStatusToPi();
              setNextState(STATE_PERI_FIND, rollDir);
              return;
            }
          }
        }
      }
      if (millis() > (stateStartTime + 180000)) {  //wait the signal for 3 minutes
        ShowMessageln ("Warning can t find the signal for area2 ");
        setNextState(STATE_ERROR, rollDir);
      }

      break;


    case STATE_TEST_COMPASS:
      motorControlOdo();

      YawActualDeg = (imu.ypr.yaw / PI * 180);

      if ((imu.distance180(YawActualDeg, yawToFind)) < 30) { //reduce speed to be sure stop
        PwmLeftSpeed = SpeedOdoMin / 2;
        PwmRightSpeed = -SpeedOdoMin / 2;
      }
      else {
        PwmLeftSpeed = SpeedOdoMin;
        PwmRightSpeed = -SpeedOdoMin;
      }


      if ((YawActualDeg >= yawToFind - 1) && (YawActualDeg <= yawToFind + 1))  {
        ShowMessage(" OdometryLeft ");
        ShowMessage(odometryLeft);
        ShowMessage(" OdometryRight ");
        ShowMessage(odometryRight);
        ShowMessage(" Find YAW ****************************************  ");
        ShowMessageln((imu.ypr.yaw / PI * 180));
        setNextState(STATE_OFF, rollDir);

      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestCompass in time ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_CALIB_MOTOR_SPEED:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
          ShowMessageln("Calibration finish ");
          ShowMessage("Real State Duration : ");
          Tempovar = millis() - stateStartTime;
          ShowMessageln(Tempovar);
          ShowMessage("Compute Max State Duration : ");
          ShowMessageln(MaxOdoStateDuration);
          motorTickPerSecond = 1000 * stateEndOdometryRight / Tempovar;
          //bber400
          float motorRpmAvg;
          motorRpmAvg = 60000 * (stateEndOdometryRight / odometryTicksPerRevolution) / Tempovar;
          ShowMessage(" motorTickPerSecond : ");
          ShowMessageln(motorTickPerSecond);
          ShowMessage(" Average RPM : ");
          ShowMessageln(motorRpmAvg);
          setNextState(STATE_OFF, 0);
          motorSpeedMaxRpm = int(motorRpmAvg); //limit to 80% to have enought PWM
          saveUserSettings();
          return;
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_TEST_MOTOR:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
          ShowMessageln("Test finish ");
          ShowMessage("Real State Duration : ");
          ShowMessageln(millis() - stateStartTime);
          ShowMessage("Compute Max State Duration : ");
          ShowMessageln(MaxOdoStateDuration);
          setNextState(STATE_OFF, 0);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }
      break;




    case STATE_ROLL_TO_FIND_YAW:
      boolean finish_4rev;
      finish_4rev = false;
      motorControlOdo();

      //bber400
      if ((imuUse) && (millis() >= nextTimeImuLoop)) {
        nextTimeImuLoop = millis() + 50;
        imu.run();
      }
      //it's ok
      if (CompassUse) {
        if ((yawToFind - 2 < (imu.comYaw / PI * 180)) && (yawToFind + 2 > (imu.comYaw / PI * 180)))  { //at +-2 degres
          findedYaw = (imu.comYaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
          return;
        }
      }
      else //without compass
      {
        if ((yawToFind - 2 < (imu.ypr.yaw / PI * 180)) && (yawToFind + 2 > (imu.ypr.yaw / PI * 180)))  { //at +-2 degres
          findedYaw = (imu.ypr.yaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
          return;
        }
      }


      //it's not ok
      if ((actualRollDirToCalibrate == RIGHT) && ((odometryRight <= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft))) finish_4rev = true;
      if ((actualRollDirToCalibrate == LEFT) && ((odometryRight >= stateEndOdometryRight) || (odometryLeft <= stateEndOdometryLeft))) finish_4rev = true;
      if (millis() > (stateStartTime + MaxOdoStateDuration + 6000)) finish_4rev = true;
      if (finish_4rev == true) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll to find yaw The Compass is certainly not calibrate correctly ");
          ShowMessageln ("Continue to mow in random mode without compass ");
        }
        if (stopMotorDuringCalib) motorMowEnable = true;//restart the mow motor
        endTimeCalibration = millis();
        compassYawMedian.clear();
        accelGyroYawMedian.clear();
        mowPatternCurr = MOW_RANDOM;
        findedYaw = yawToFind;
        nextTimeToDmpAutoCalibration = millis() + 21600 * 1000; //do not try to calibration for the next 6 hours
        setBeeper(0, 0, 0, 0, 0);
        if (perimeterInsideLeft) setNextState(STATE_ACCEL_FRWRD, rollDir);
        else setNextState(STATE_PERI_OUT_REV, rollDir);
        return;
      }


      break;

    //not use actually
    case STATE_PERI_ROLL:
      // perimeter find  roll
      if (millis() >= stateEndTime) setNextState(STATE_PERI_FIND, 0);
      motorControl();
      break;


    //not use actually
    case STATE_PERI_REV:  //obstacle in perifind
      // perimeter tracking reverse
      //bb
      ShowMessageln(odometryRight);

      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))  setNextState(STATE_PERI_ROLL, rollDir);
      motorControlOdo();


      break;

    case STATE_PERI_FIND:
      // find perimeter
      if (!perimeterInsideLeft) {
        ShowMessageln("Not inside so start to track the wire");
        setNextState(STATE_PERI_STOP_TOTRACK, 0);
        return;
      }

      checkSonar();
      checkBumpersPerimeter();
      checkCurrent();
      motorControlOdo();
      break;

    case STATE_PERI_TRACK:
      // track perimeter
      checkCurrent();  // use to detect obstacle
      checkBumpersPerimeter();  // use to detect voltage station
      checkSonarPeriTrack();
      if (statusCurr == BACK_TO_STATION) {
        checkStuckOnIsland();
      }
      if (ActualSpeedPeriPWM != MaxSpeedperiPwm) {  // RFID tag can reduce speed ,so need a reset
        if (totalDistDrive > whereToResetSpeed) {
          ShowMessage("Distance OK, time to reset the initial Speed : ");
          ShowMessageln(ActualSpeedPeriPWM);
          ActualSpeedPeriPWM = MaxSpeedperiPwm;
        }
      }
      //********************************* if start by timer
      if (statusCurr == TRACK_TO_START) {
        //bber11
        //areaToGo need to be use here to avoid start mowing before reach the rfid tag in area1
        // if ((areaToGo == areaInMowing) && (startByTimer) && (totalDistDrive > whereToStart * 100)) {
        //bber35

        if ((areaToGo == areaInMowing) && (totalDistDrive >= whereToStart * 100)) {
          startByTimer = false;
          ShowMessage("Distance OK, time to start mowing into new area ");
          ShowMessageln(areaInMowing);
          areaToGo = 1; //after mowing the mower need to back to station
          ActualSpeedPeriPWM = MaxSpeedperiPwm;
          setNextState(STATE_PERI_STOP_TOROLL, rollDir);
          return;
        }
      }
      if (read2Coil) {
        motorControlPerimeter2Coil();
      }
      else {
        motorControlPerimeter();
      }

      break;

    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor) {
        if (chgVoltage > 5.0) {
          if (batVoltage < startChargingIfBelow) { //read the battery voltage immediatly before it increase
            setNextState(STATE_STATION_CHARGING, 0);
            return;
          }
          else
          {
            if (millis() - stateStartTime > 10000) checkTimer(); //only check timer after 10 second to avoid restart before charging and check non stop after but real only 60 sec
          }
        }
        else
        {
          ShowMessageln("We are in station but ChargeVoltage is lost ??? ");
          setNextState(STATE_OFF, 0);
          return;
        }
      }
      else {
        if (millis() - stateStartTime > 10000) checkTimer(); //only check timer after 10 second to avoid restart before charging
      }

      break;

    case STATE_STATION_CHARGING:
      // waiting until charging completed
      if (batMonitor) {

        if (chgVoltage <= 2 ) { //something is wrong in the charger or bad contact
          ShowMessageln("Station Voltage was lost while charging");
          setNextState(STATE_OFF, 0);
          return;
        }


        if ((chgCurrent < batFullCurrent) && (millis() - stateStartTime > 3000))   //end of charge by current
        {
          if ((autoResetActive) && (millis() - stateStartTime > 3600000)) { // only reboot if the mower is charging for more 1 hour
            ShowMessageln("End of charge by batfullcurrent Time to Restart PI and Due");
            autoReboot(); // 05/11/22 not ok with teensy pcb because power is down when watchdog reset
          }
          if (!timerUse) {  //power off everything at the end of charging if the timer is not active
            ShowMessageln("End of charge and no timer so PowerOff");
            powerOff_pcb(); // to stop immediatly the PCB
          }
          setNextState(STATE_STATION, 0);
          return;

        }
        if (millis() - stateStartTime > chargingTimeout)   //end of charge by timeout
        {
          ShowMessageln("End of charging duration check the batfullCurrent to try to stop before");
          if (autoResetActive) {
            ShowMessageln("Time to Restart PI and Due");
            autoReboot(); // 05/11/22 not ok with teensy pcb because power is down when watchdog reset
          }
          if (!timerUse) {  //power off everything at the end of charging if the timer is not active
            ShowMessageln("End of charge and no timer so PowerOff");
            powerOff_pcb();
          }

          setNextState(STATE_STATION, 0);
          return;
        }
      }


      break;

    case STATE_STOP_ON_BUMPER:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {

        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (statusCurr == BACK_TO_STATION) {
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
          }
          else {
            setNextState(STATE_BUMPER_REV, rollDir);
          }
          //bber300 return ??
          return;
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t  stop ON BUMPER in time ");
        }
        setNextState(STATE_BUMPER_REV, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_PERI_OUT_STOP:
      motorControlOdo();
      checkCurrent();
      checkBumpers();
      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted

          //if ((DistPeriOutRev == 0) && (mowPatternCurr != MOW_LANES)) {
          if (mowPatternCurr != MOW_LANES) { // do not reverse in random mowing mode
            setNextState(STATE_PERI_OUT_ROLL, rollDir);
          }
          else
          {
            //DistPeriOutRev = DistPeriOutStop + 10; // into by lane mowing a reverse is mandatory to avoid mower change to random mode imediatelly
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t peri out stop in time ");
        }
        //if ((DistPeriOutRev == 0) && (mowPatternCurr != MOW_LANES)) {
        if (mowPatternCurr != MOW_LANES) { // do not reverse in random mowing mode
          setNextState(STATE_PERI_OUT_ROLL, rollDir);
        }
        else
        {
          //DistPeriOutRev = DistPeriOutStop + 10;
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }
      }
      break;




    case STATE_ENDLANE_STOP:
      motorControlOdo();
      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (rollDir == RIGHT) {
          if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) { //wait until the 2 motor completly stop because need precision
            setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
          }
        }
        else
        {
          if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) {
            setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
          }
        }
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t end lane in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);//if the motor can't reach the odocible in slope
      }

      break;

    case STATE_SONAR_TRIG:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        setBeeper(0, 0, 0, 0, 0);

        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          //bber10
          if (stateLast == STATE_PERI_FIND) {
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
          }
          else {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
          return;

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t sonar trig in time ");
        }
        if (stateCurr == STATE_PERI_FIND) {
          setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
        }
        else {
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }
        return;
      }
      checkCurrent();
      checkBumpers();
      break;


    case STATE_STOP_TO_FIND_YAW:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (laneUseNr == 1) yawToFind = yawSet1 ;
          if (laneUseNr == 2) yawToFind = yawSet2 ;
          if (laneUseNr == 3) yawToFind = yawSet3 ;
          if (CompassUse) {
            setNextState(STATE_ROLL_TO_FIND_YAW, rollDir);
          }
          else
          {
            findedYaw = (imu.ypr.yaw / PI * 180);
            setNextState(STATE_STOP_CALIBRATE, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t peri out stop in time ");
        }
        if (laneUseNr == 1) yawToFind = yawSet1 ;
        if (laneUseNr == 2) yawToFind = yawSet2 ;
        if (laneUseNr == 3) yawToFind = yawSet3 ;
        if (CompassUse) {
          setNextState(STATE_ROLL_TO_FIND_YAW, rollDir);//if the motor can't rech the odocible in slope
        }
        else
        {
          findedYaw = (imu.ypr.yaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
        }
      }
      break;

    case STATE_PERI_STOP_TOROLL:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {

        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (statusCurr == TRACK_TO_START) setNextState(STATE_STATION_ROLL, rollDir);
          else setNextState(STATE_ROLL_TONEXTTAG, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t stop to track in time ");
        }
        if (statusCurr == TRACK_TO_START) setNextState(STATE_STATION_ROLL, rollDir);
        else setNextState(STATE_ROLL_TONEXTTAG, rollDir);
      }
      break;

    case STATE_PERI_STOP_TO_FAST_START:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_ROLL_TONEXTTAG, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t stop to track in time ");
        }
        setNextState(STATE_ROLL_TONEXTTAG, rollDir);

      }
      break;

    case STATE_PERI_STOP_TO_NEWAREA:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_ROLL1_TO_NEWAREA, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t stop  in time ");
        }

        setNextState(STATE_ROLL1_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_PERI_STOP_TOTRACK:
      motorControlOdo();
      //if ((moveRightFinish) && (moveLeftFinish) ) {
      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
        }

      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t stop to track in time ");
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_AUTO_CALIBRATE:
      setBeeper(2000, 300, 300, 2000, 0);//beep for 2 sec
      if (millis() > nextTimeAddYawMedian) {  // compute a median of accelGyro and Compass  yaw
        compassYawMedian.add(imu.comYaw);
        accelGyroYawMedian.add(imu.ypr.yaw);
        nextTimeAddYawMedian = millis() + 70;  // the value are read each 70ms
      }
      if (accelGyroYawMedian.getCount() > 56) { //we have the value of 4 secondes try to verify if the drift is less than x deg/sec
        ShowMessageln("4 sec of read value, verify if the drift is stop");
        if  (abs(accelGyroYawMedian.getHighest() - accelGyroYawMedian.getLowest()) < 4 * maxDriftPerSecond * PI / 180) { //drift is OK restart mowing
          if (CompassUse) {
            imu.CompassGyroOffset = distancePI( scalePI(accelGyroYawMedian.getMedian() -  imu.CompassGyroOffset), compassYawMedian.getMedian()); //change the Gyro offset according to Compass Yaw
          }
          ShowMessageln("Drift is OK");
          setBeeper(0, 0, 0, 0, 0); //stop sound immediatly

          if (stopMotorDuringCalib) motorMowEnable = true;//restart the mow motor
          if (perimeterInsideLeft) {
            setNextState(STATE_ACCEL_FRWRD, rollDir); //if not outside continue in forward
          }
          else
          {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
          return;
        }
        else {   //not OK try to wait 4 secondes more
          ShowMessageln("Drift not Stop wait again 4 sec");
          compassYawMedian.clear();
          accelGyroYawMedian.clear();
        }

      }
      if (millis() > endTimeCalibration) { //we have wait enought and the result is not OK start to mow in random mode or make a total calibration
        mowPatternCurr = MOW_RANDOM;
        if (stopMotorDuringCalib) motorMowEnable = true;//stop the mow motor
        ShowMessageln("WAIT to stop Drift of GYRO : is not OK mowing Drift too important");
        nextTimeToDmpAutoCalibration = millis() + delayBetweenTwoDmpAutocalib * 1000;
        setBeeper(0, 0, 0, 0, 0);
        if (perimeterInsideLeft) {
          if (mowPatternCurr == MOW_LANES) {  //change the rolldir now because again when new forward_odo only in lane mowing
            // if (rollDir == 0) rollDir = 1;
            // else rollDir = 0;
          }
          setNextState(STATE_ACCEL_FRWRD, rollDir);
        }
        else
        {
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }


      }

      break;













    case STATE_STOP_CALIBRATE:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (perimeter.isInside(0)) {  // v417 if calibration begin out of perimeter mower can't continue and stop with outside error
            setNextState(STATE_AUTO_CALIBRATE, rollDir);
          }
          else
          {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t  stop to calibrate in time ");
        }
        if (perimeter.isInside(0)) { //if calibration begin out of perimeter mower can't continue and stop with outside error
          setNextState(STATE_AUTO_CALIBRATE, rollDir);
        }
        else
        {
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }
      }
      break;















    case STATE_STOP_BEFORE_SPIRALE:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_ROTATE_RIGHT_360, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning cant stop before spirale in time");
        }
        setNextState(STATE_ROTATE_RIGHT_360, rollDir);    //if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROTATE_RIGHT_360:
      motorControlOdo();
      checkCurrent();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_MOW_SPIRALE, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning cant rotate right 360 in time ");
        }
        setNextState(STATE_MOW_SPIRALE, rollDir);
      }

      break;
    case STATE_NEXT_SPIRE:
      motorControlOdo();
      checkCurrent();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        setNextState(STATE_MOW_SPIRALE, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t  stop before next spire in time ");
        }
        setNextState(STATE_MOW_SPIRALE, rollDir);//if the motor can't rech the odocible in slope
      }

      break;
    case STATE_MOW_SPIRALE:
      motorControlOdo();
      checkCurrent();
      checkBumpers();
      checkSonar();

      //checkLawn();
      checkTimeout();

      //*************************************end of the spirale ***********************************************
      if ((spiraleNbTurn >= 8) || (!highGrassDetect)) {
        spiraleNbTurn = 0;
        highGrassDetect = false;
        setNextState(STATE_STOP_ON_BUMPER, RIGHT); //stop the spirale or setNextState(STATE_PERI_OUT_FORW, rollDir)
        return;
      }
      //********************************************************************************************
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        if (!perimeterInsideLeft) {
          setNextState(STATE_STOP_ON_BUMPER, rollDir);
        }
        else
        {
          setNextState(STATE_NEXT_SPIRE, rollDir);
        }
        return;
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t MOW_SPIRALE in time ");
        }
        setNextState(STATE_NEXT_SPIRE, rollDir);//if the motor can't rech the odocible in slope
      }


      break;


    case STATE_BUMPER_REV:
      motorControlOdo();
      /*
            ShowMessage("REV L/R : ");
            ShowMessage(odometryLeft);
            ShowMessage(" / ");
            ShowMessageln(odometryRight);
            delay(50);
      */

      if (mowPatternCurr == MOW_LANES) {  //  *************************LANE***************************************
        if ((moveRightFinish) && (moveLeftFinish) ) {
          if (rollDir == RIGHT) {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) { //wait until the 2 motor completly stop because need precision
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
          else
          {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) {
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
        }
      }
      else


      { //  *************************RANDOM***************************************

        if (rollDir == RIGHT) {
          if ((odometryRight <= stateEndOdometryRight) && (moveLeftFinish) ) {

            if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
        else
        {
          if ((moveRightFinish) && (odometryLeft <= stateEndOdometryLeft) ) {

            if (motorRightPWMCurr == 0 ) { //wait until the right motor completly stop because rotation is inverted
              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
      }



      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t bumper rev in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);//if the motor can't rech the odocible in slope
      }

      break;

    case STATE_PERI_OUT_REV:
      motorControlOdo();
      /*
            ShowMessage("REV L/R : ");
            ShowMessage(odometryLeft);
            ShowMessage(" / ");
            ShowMessageln(odometryRight);
            delay(50);
      */

      if (mowPatternCurr == MOW_LANES) {  //  *************************LANE***************************************
        if ((moveRightFinish) && (moveLeftFinish) ) {
          if (rollDir == RIGHT) {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) { //wait until the 2 motor completly stop because need precision
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
          else
          {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) {
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
        }
      }
      else


      { //  *************************RANDOM***************************************

        if (rollDir == RIGHT) {
          if ((odometryRight <= stateEndOdometryRight) && (moveLeftFinish) ) {

            if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
        else
        {
          if ((moveRightFinish) && (odometryLeft <= stateEndOdometryLeft) ) {

            if (motorRightPWMCurr == 0 ) { //wait until the right motor completly stop because rotation is inverted
              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
      }


      /*
        { //  *************************RANDOM***************************************
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (rollDir == RIGHT) {
            if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
          else
          {
            if (motorRightPWMCurr == 0 ) { //wait until the right motor completly stop because rotation is inverted
              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
        }
      */
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t peri out rev in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);//if the motor can't rech the odocible in slope
      }

      break;

    case STATE_PERI_OUT_ROLL:
      motorControlOdo();
      /*
        ShowMessage("left E/A : ");
        ShowMessage(stateEndOdometryLeft);
        ShowMessage(" / ");
        ShowMessageln(odometryLeft);
        ShowMessage("right E/A : ");
        ShowMessage(stateEndOdometryRight);
        ShowMessage(" / ");
        ShowMessage(odometryRight);
        ShowMessage(" rolldir : ");
        ShowMessageln(rollDir);
        delay(100);
      */
      if (rollDir == RIGHT) {
        if ((moveRightFinish) && (odometryLeft >= stateEndOdometryLeft) ) {  //no brake on left wheel
          //if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
          if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (moveLeftFinish) ) {
          // if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {

          if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t peri out roll in time ");
        }
        setNextState(STATE_PERI_OUT_FORW, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_PERI_OUT_ROLL_TOINSIDE:
      checkBumpers();
      motorControlOdo();
      //bber17
      if (RollToInsideQty >= 10) {
        ShowMessageln("ERROR Mower is lost out the wire and can't find the signal. Roll to inside occur more than 10 Time");
        setNextState(STATE_ERROR, rollDir);
        return;
      }


      if ((moveRightFinish) && (moveLeftFinish) ) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          if (!perimeterInsideLeft) setNextState(STATE_WAIT_AND_REPEAT, rollDir);//again until find the inside
          else setNextState(STATE_PERI_OUT_FORW, rollDir);
        }
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t Roll to inside in time ");
        }
        if (!perimeterInsideLeft) setNextState(STATE_WAIT_AND_REPEAT, rollDir);//again until find the inside
        else setNextState(STATE_PERI_OUT_FORW, rollDir);
      }
      break;

    case STATE_PERI_OUT_ROLL_TOTRACK:
      motorControlOdo();

      if (perimeterInsideLeft) {
        setNextState(STATE_PERI_OUT_STOP_ROLL_TOTRACK, 0);
        return;
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t find perimeter Wire while PERI_OUT_ROLL_TOTRACK in time ");
        }
        rollToTrackQty = rollToTrackQty + 1;
        if (rollToTrackQty >= 6) {
          ShowMessageln ("Warning can t find again perimeter Wire after 6 rev");
          setNextState(STATE_ERROR, 0);
          return;
        }
        if (!perimeterInsideLeft) setNextState(STATE_WAIT_AND_REPEAT, 0);//again until find the inside
        else setNextState(STATE_PERI_OUT_STOP_ROLL_TOTRACK, 0);
      }
      break;

    case STATE_PERI_OUT_STOP_ROLL_TOTRACK:
      motorControlOdo();

      if (perimeterInsideLeft) {

        if ((moveRightFinish) && (moveLeftFinish) ) {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) {
            lastTimeForgetWire = millis(); //avoid motor reverse on tracking startup
            //bber300

            setNextState(STATE_PERI_TRACK, 0);
            return;
          }
        }
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OUT_STOP_ROLL_TOTRACK in time ");
        }
        if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOTRACK, 0);//again until find the inside
        else setNextState(STATE_PERI_TRACK, 0);
      }
      break;

    case STATE_PERI_OUT_LANE_ROLL1:
      motorControlOdo();
      checkCurrent();
      checkBumpers();


      if ((moveRightFinish) && (moveLeftFinish))
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the left motor completly stop because rotation is inverted
          if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
          else setNextState(STATE_NEXT_LANE_FORW, rollDir);
        }
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t Roll1 by lane in time ");
        }
        setNextState(STATE_NEXT_LANE_FORW, rollDir);//if the motor can't reach the odocible in slope
      }
      break;

    case STATE_NEXT_LANE_FORW:
      motorControlOdo();
      checkCurrent();
      checkBumpers();

      //bber14
      //if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
      if (!perimeterInsideLeft) {
        setNextState(STATE_PERI_OUT_STOP, rollDir);
        return;
      }

      if ((moveRightFinish) && (moveLeftFinish)) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) {
          setNextState(STATE_PERI_OUT_LANE_ROLL2, rollDir);

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t reach next lane in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL2, rollDir);//if the motor can't reach the odocible in slope for example

      }

      break;

    case STATE_PERI_OUT_LANE_ROLL2:
      motorControlOdo();
      checkCurrent();
      checkBumpers();

      if (rollDir == RIGHT) {
        if ((moveRightFinish) && (moveLeftFinish))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
            if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_FORWARD_ODO, rollDir);// forward odo to straight line
            rollDir = LEFT;//invert the next rotate
          }
        }
      }

      else
      {
        if ((moveRightFinish) && (moveLeftFinish))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
            if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_FORWARD_ODO, rollDir);
            rollDir = RIGHT;// invert the next rotate
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessageln ("Warning can t make the roll2 in time ");
        }
        if (rollDir == RIGHT) {
          if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
          else setNextState(STATE_FORWARD_ODO, rollDir);// forward odo to straight line
          rollDir = LEFT;//invert the next rotate
        }
        else
        {
          if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
          else setNextState(STATE_FORWARD_ODO, rollDir);
          rollDir = RIGHT;// invert the next rotate
        }
      }
      break;




    case STATE_PERI_OUT_FORW:
      motorControlOdo();
      if (!perimeterInsideLeft) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
      if ((millis() > (stateStartTime + MaxOdoStateDuration)) || (odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        setNextState(STATE_FORWARD_ODO, rollDir);

      }
      break;

    case STATE_STATION_CHECK:
      //check motor sense to stop imediatly if blocking station (no spring on charging contact)
      if ((motorLeftPower >= 0.8 * motorPowerMax) || (motorLeftPower >= 0.8 * motorPowerMax)) {
        //stop immediatly
        ShowMessageln ("Station check detect overload on motor ");
        motorLeftPWMCurr = motorRightPWMCurr = 0;
        motorLeftRpmCurr = motorRightRpmCurr = 0 ;
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
        setMotorPWM(0, 0);
        setNextState(STATE_OFF, rollDir);// we are
        return;

      }
      // check for charging voltage here after detect station
      if ((moveRightFinish) && (moveLeftFinish)) //move some CM to be sure the contact is OK
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          //need to adapt if station is traversante
          // if (millis() >= delayToReadVoltageStation) { //wait 0.5 sec after all stop and before read voltage
          //bber300
          if (powerboard_I2c_line_Ok) chgVoltage = ChargeIna226.readBusVoltage() ;
          if (chgVoltage > 5.0)  {
            ShowMessageln ("Charge Voltage detected ");
            setNextState(STATE_STATION, rollDir);// we are into the station
            return;
          }
          else {
            ShowMessageln ("No Voltage detected so certainly Obstacle ");
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);// not into the station so avoid obstacle
            return;
          }
          //}
        }
      }
      //if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
      if (millis() > (stateStartTime + 500)) {//16/10/22 limit to 500ms
        if (developerActive) {
          ShowMessageln ("Warning can t make the station check in time ");
        }
        // if (millis() >= delayToReadVoltageStation) { //wait 0.5 sec after all stop and before read voltage
        //bber300
        if (powerboard_I2c_line_Ok) chgVoltage = ChargeIna226.readBusVoltage() ;
        if (chgVoltage > 5.0)  {
          ShowMessageln ("Charge Voltage detected ");
          setNextState(STATE_STATION, rollDir);// we are into the station
          return;
        }
        else {
          ShowMessageln ("No Voltage detected so certainly Obstacle ");
          setNextState(STATE_PERI_OBSTACLE_REV, rollDir);// not into the station so avoid obstacle
          return;
        }
        //}
      }
      motorControlOdo();
      break;


    case STATE_START_FROM_STATION:
      //adjust imu to station heading
      if ((imuUse) && (millis() >= nextTimeImuLoop)) {
        nextTimeImuLoop = millis() + 50;
        StartReadAt = millis();
        imu.run();
        EndReadAt = millis();
        ReadDuration = EndReadAt - StartReadAt;
        if ( ReadDuration > 30) {
          ShowMessage("Error reading imu too long duration : ");
          ShowMessageln(ReadDuration);
          ShowMessageln ("IMU and RFID are DEACTIVATE Mow in safe mode");
          imuUse = false;
          rfidUse = false;
          addErrorCounter(ERR_IMU_COMM);
        }
        else
        {
          if (!CompassUse) { //set the yaw heading to station heading before mower leave station if compass is not use
            imu.CompassGyroOffset = imu.CompassGyroOffset + scalePI((stationHeading / 180 * PI) - imu.ypr.yaw);
            imu.run();
          }
        }
      }

      if (millis() > (stateStartTime + MaxStateDuration)) {
        if (imuUse) {
          ShowMessage("Imu Heading is reset to Station Heading : ");
          ShowMessageln(imu.ypr.yaw * 180 / PI);
        }
        setNextState(STATE_STATION_REV, 1);
      }

      break;



    case STATE_STATION_REV:

      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) ) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          setNextState(STATE_STATION_ROLL, 1);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessage ("Warning station rev not in time Max Compute duration in ms :");
        }
        setNextState(STATE_STATION_ROLL, 1);//if the motor can't reach the odocible in slope
      }
      break;

    case STATE_STATION_ROLL:
      motorControlOdo();

      if ((moveRightFinish) && (moveLeftFinish) )
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          smoothPeriMag = perimeter.getSmoothMagnitude(0);

          if ((perimeterInsideLeft) && (smoothPeriMag > 250)) //check if signal here and inside need a big value to be sure it is not only noise
          {
            ShowMessage("SIGNAL OK SmoothMagnitude =  ");
            ShowMessageln(smoothPeriMag);
            motorMowEnable = true;
            setNextState(STATE_STATION_FORW, rollDir);
            return;
          }
          else {
            ShowMessage("ERROR No SIGNAL SmoothMagnitude =  ");
            ShowMessageln(smoothPeriMag);
            setNextState(STATE_ERROR, 0);
            return;
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessageln ("Warning can t make the station roll in time ");
        }
        smoothPeriMag = perimeter.getSmoothMagnitude(0);

        if ((perimeterInsideLeft) && (smoothPeriMag > 250)) //check if signal here and inside need a big value to be sure it is not only noise
        {
          setNextState(STATE_STATION_FORW, rollDir);
        }
        else {
          ShowMessage("ERROR No SIGNAL SmoothMagnitude =  ");
          ShowMessageln(smoothPeriMag);
          setNextState(STATE_ERROR, 0);
        }
      }
      break;

    case STATE_STATION_FORW:
      // forward (charge station)
      //disabble the sonar during 10 seconds

      nextTimeCheckSonar = millis() + 10000;  //Do not check the sonar during 30 second  to avoid detect the station


      //justChangeLaneDir=false;
      motorControlOdo();

      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft))
      {
        if ((whereToStart != 0) && (startByTimer)) { //if ((whereToStart != 0) make a circle arround the station if not start immediatly
          setNextState(STATE_PERI_OBSTACLE_AVOID, rollDir);
        }
        else
        {
          //020919 to check but never call and not sure it's ok
          statusCurr = NORMAL_MOWING;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_FORWARD_ODO, rollDir);
        }

      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessageln ("Warning can t make the station forw in time ");
        }
        if ((whereToStart != 0) && (startByTimer)) {
          setNextState(STATE_PERI_OBSTACLE_AVOID, rollDir);
        }
        else  setNextState(STATE_FORWARD_ODO, rollDir);
      }
      break;

    case STATE_WAIT_AND_REPEAT:
      if (millis() > (stateStartTime + 500)) setNextState(stateLast, rollDir);//1000
      break;

    case STATE_WAIT_COVER:
      if (millis() > (stateStartTime + 10000)) {
        ShowMessageln("Cover not closed after 10 secondes ???????");
        setNextState(STATE_OFF, rollDir);
      }

      if (coverIsClosed) {
        setBeeper(0, 0, 0, 0, 0);
        motorMowEnable = true;
        setNextState(STATE_ACCEL_FRWRD, rollDir);//1000
      }
      break;

    //bber50
    case STATE_ACCEL_FRWRD:

      motorControlOdo();
      if (!perimeterInsideLeft) {
        ShowMessageln("Try to start at other location : We are not inside perimeter");
        setNextState(STATE_OFF, rollDir);
        return;
      }

      if ((LEFT_MOTOR_DRIVER == 1) || (LEFT_MOTOR_DRIVER == 4)) { //brussless driver have it's own acceleration
        imuDirPID.reset();
        motorRightPID.reset();
        motorLeftPID.reset();
        setNextState(STATE_FORWARD_ODO, rollDir);

        return;
      }

      if ((millis() > (stateStartTime + MaxOdoStateDuration)) || (odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        imuDirPID.reset();
        motorRightPID.reset();
        motorLeftPID.reset();
        setNextState(STATE_FORWARD_ODO, rollDir);
      }

      break;


  } // end switch

  bumperRight = false;
  bumperLeft = false;
  bumperRearRight = false;
  bumperRearLeft = false;

  loopsPerSecCounter++;
  wdt.feed();

  /*
    StartReadAt = millis();
    distance_find = sensor.readRangeSingleMillimeters();
    EndReadAt = millis();
    ReadDuration = EndReadAt - StartReadAt;
    ShowMessage("Dist :    ");
    ShowMessage(distance_find);
    ShowMessage("         Read Duration in ms ");
    ShowMessageln(ReadDuration);
  */

}
