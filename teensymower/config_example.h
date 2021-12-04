// Configure the Options below and finally compile and upload it to the Teensy 4.1
// Rename the file 'config_example.h' into 'config.h'

// code version
#define VER "1.0-Teensyber GY-521"

// ------ Compass/IMU
//#define COMPASS_IS HMC5883L
#define COMPASS_IS QMC5883L

// SD new did not know if it is in Code
//  should the mower turn off if IMU is tilt over? (1 = yes, 0 = no)
#define TILT_DETECTION 1

// ------ Motor Driver
#define BTS7960
//#define L298N
//#define BRUSHLESS //JYQD-V7.3E3 or similar

// ------- baudrates---------------------------------

// ------ used serial ports for console, Bluetooth, ESP32 -----------------------------

// we use 'SerialUSB' for 'Console' so the Raspberry PI receive all data
// we use 'Serial' for 'Console' so the PC receive all data
//#define Console SerialUSB
#define Console Serial
#define CONSOLE_BAUDRATE 115200 // baudrate used for Raspberry PI console and PC

#define track_ClockWise false

#define Enable_DueWatchdog false
//#define Enable_DueWatchdog false

#define autoBylaneToRandom true

#define RaspberryPIPort Serial // The PI is connected over USB cable

//#define Bluetooth Serial1        // Ardumower default OK for ESP32 or HC05
//#define BLUETOOTH_BAUDRATE 19200 // baudrate used for communication with Bluetooth module (Ardumower default: 19200)

#define GpsPort Serial3 // GPS do not forget workarround if PCB1.3 use

// ------- ultrasonic
#define NO_ECHO 0 //??

// ---- choose only one perimeter signal code ----
#define SIGCODE_1 // Ardumower default perimeter signal
//#define SIGCODE_2  // Ardumower alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal

#define motorAccel 1500 // motor wheel acceleration - only functional when odometry is not in use (warning: do not set too low)
                        // bb
#define motorLeftChange 500
#define motorRightChange 500
#define motorOdoAccel 1500         // Time for accel from 0 to 100% in ms
#define motorSpeedMaxRpm 24        // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
#define motorSpeedMaxPwm 190       // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
#define motorPowerMax 23           // motor wheel max power (Watt)
#define motorSenseRightScale 1.870 // normal is 1.536 motor right sense scale (mA=(ADC-zero)/scale)
#define motorSenseLeftScale 1.650  // normal is 1.536 motor left sense scale  (mA=(ADC-zero)/scale)
#define motorPowerIgnoreTime 2000  // time to ignore motor power when start to avoid read the peack on motor start (ms)
#define motorZeroSettleTime 2000   // defaut 3000 how long (ms) to wait for motors to settle at zero speed
#define motorRollDegMax 100        // max. roll Deg
#define motorRollDegMin 20         // min. roll Deg

#define motorForwTimeMax 80000    // not use max. forward time (ms) / timeout
#define motorBiDirSpeedRatio1 0.3 // bidir mow pattern speed ratio 1
#define motorBiDirSpeedRatio 0.92 // bidir mow pattern speed ratio 2
#define motorLeftPID_Kp 1.0      // motor wheel PID controller
#define motorLeftPID_Ki 0.4
#define motorLeftPID_Kd 0.0
#define motorRightSwapDir 0 // inverse right motor direction?
#define motorLeftSwapDir 0  // inverse left motor direction?

#define motorRightOffsetFwd 0  // percent offset in PWM use for the 2 wheels motor have the same speed a the same PWM
#define motorRightOffsetRev 0  // use the 1 ml ODO test to find good value the 2 wheels need to stop at the same time
#define motorTickPerSecond 200 // use to compute the maxodostate duration and computed on the calibration motor

#define UseAccelLeft 1
#define UseBrakeLeft 1
#define UseAccelRight 1
#define UseBrakeRight 1
#define AngleRotate 100
#define SpeedOdoMin 50
#define SpeedOdoMax 140
#define odoLeftRightCorrection true // left-right correction for straight lines used in manual mode
#define autoAdjustSlopeSpeed true   // adjust the speed on slope to have same speed on uphill and downhill

// ------ mower motor -------------------------------
#define motorMowAccel 1000      // motor mower acceleration (warning: do not set too low) 2000 seems to fit best considerating start time and power consumption
#define motorMowSpeedMaxPwm 200 // motor mower max PWM
#define motorMowSpeedMinPwm 100 // motor mower minimum PWM (only for cutter modulation)
#define motorMowPowerMax 18.0   // motor mower max power (Watt)

#define motorMowSenseScale 1.536 // motor mower sense scale (mA=(ADC-zero)/scale)
#define motorMowPID_Kp 0.005    // motor mower RPM PID controller
#define motorMowPID_Ki 0.01
#define motorMowPID_Kd 0.01
//  ------ bumper -----------------------------------
#define bumperUse 0 // has bumpers?                                                                         Dropsensor - Kontakt 0-Ã–ffner - 1-SchlieÃŸer betÃ¤tigt gegen GND
                    // ------ rain ------------------------------------
#define rainUse 0   // use rain sensor?

// ------ DHT22Use ------------------------------------
#define DHT22Use 0        // use DHT22 sensor?
#define maxTemperature 55 // max temp before switch off
                          // bber35
                          //  ------ RFID ------------------------------------
#define rfidUse 0;        // use rfid
#define newtagRotAngle1 -90
#define newtagRotAngle2 0
#define newtagDistance1 10
#define newtagDistance2 0

// ------ sonar ------------------------------------
#define sonarUse 0 // use ultra sonic sensor? (WARNING: robot will slow down, if enabled but not connected!)
#define sonarLeftUse 1
#define sonarRightUse 1
#define sonarTriggerBelow 87  // ultrasonic sensor trigger distance in cm (0=off)
#define sonarToFrontDist 30   // ultrasonic sensor distance to front mower in cm
#define sonarLikeBumper false // ultrasonic reduce speed vs bumper like

// ------ perimeter ---------------------------------
#define perimeterUse 0              // use perimeter?
#define perimeterTriggerMinSmag 200 // perimeter minimum smag to use on big area
//#define perimeterOutRollTimeMax 2000    // free
//#define perimeterOutRollTimeMin 750     // free
#define perimeterOutRevTime 2200    // free
#define perimeterTrackRollTime 1500 // roll time during perimeter tracking
#define perimeterTrackRevTime 2200  // reverse time during perimeter tracking
#define DistPeriOutRev 40           // reverse distance when reach the perimeter in cm
#define DistPeriObstacleRev 30      // reverse distance when hit obstacle while tracking in cm
#define DistPeriOutForw 60          // distance to accell
#define DistPeriOutStop 15          // slowing distance after crossover the wire
#define DistPeriObstacleForw 25     // distance while arc circle in peri obstacle avoid
#define perimeterPID_Kp 16.5       // perimeter PID controller
#define perimeterPID_Ki 8
#define perimeterPID_Kd 0
#define trackingPerimeterTransitionTimeOut 1500
#define trackingErrorTimeOut 10000
#define trakBlockInnerWheel true
// bb
#define MaxSpeedperiPwm 180                // speed max in PWM while perimeter tracking
#define ActualSpeedPeriPWM MaxSpeedperiPwm // speed in PWM while perimeter tracking
//#define  timeToResetSpeedPeri 0  // if millis() > at this var the speed is set to max value
#define RollTimeFor45Deg 1000      // time while roll in peri obstacle avoid if no Odometry
#define circleTimeForObstacle 4000 // time while arc circle in peri obstacle avoid if no Odometry
#define DistPeriObstacleAvoid 100  // distance while arc circle in peri obstacle avoid
#define perimeterMagMaxValue 2000  // Maximum value return when near the perimeter wire (use for tracking and slowing when near wire
//#define  perimeter.read2Coil false
#define areaToGo 1 // initialise the areatogo to the station area
// ------  IMU (compass/accel/gyro) ----------------------
#define imuUse 1               // use IMU?
#define CompassUse 0           // activate compass?
#define stopMotorDuringCalib 0 // correct direction by compass?
#define imuDirPID_Kp 4.4      // direction PID controller
#define imuDirPID_Ki 3.3
#define imuDirPID_Kd 0.0
#define imuRollPID_Kp 0.8 // roll PID controller
#define imuRollPID_Ki 21
#define imuRollPID_Kd 0
// bb
#define yawSet1 45
#define yawOppositeLane1RollRight -125
#define yawOppositeLane1RollLeft -135
#define yawSet2 90
#define yawOppositeLane2RollRight -92
#define yawOppositeLane2RollLeft -88
#define yawSet3 135
#define yawOppositeLane3RollRight -47
#define yawOppositeLane3RollLeft -42
#define laneUseNr 2
#define maxDriftPerSecond 0.05          // limit the stop time if small drift
#define maxDurationDmpAutocalib 60      // in sec
#define delayBetweenTwoDmpAutocalib 360 // in sec
#define yawCiblePos 90
#define yawCibleNeg -90
#define DistBetweenLane 38
#define maxLenghtByLane 9 // distance to run in bylane before simulate a wire detection
#define justChangeLaneDir true
#define mowPatternCurr MOW_LANES
#define compassRollSpeedCoeff 40 // speed used when the mower search the compass yaw it's percent of motorSpeedMaxRpm ,Avoid to roll to fast for a correct detection
// ------ battery -------------------------------------
#define batMonitor false          // monitor battery and charge voltage?
#define batGoHomeIfBelow 24.3     // drive home voltage (Volt)
#define batSwitchOffIfBelow 23    // switch off battery if below voltage (Volt)
#define batSwitchOffIfIdle 300    // switch off battery if idle (minutes)
#define batFactor 10.88           // depend of the resistor divisor on board R12 and R13
#define batChgFactor 10.89        // depend of the resistor divisor on board R9 and R10
#define batFull 29.4              // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
#define batChargingCurrentMax 2   // maximum current your charger can devliver
#define batFullCurrent 0.1        // current flowing when battery is fully charged
#define startChargingIfBelow 25.0 // start charging if battery Voltage is below
#define chargingTimeout 25200000  // safety timer for charging (ms)  7 hrs
#define chgSenseZero 511          // charge current sense zero point
#define batSenseFactor 1.11       // charge current conversion factor   - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
#define chgSense 185.0            // mV/A empfindlichkeit des Ladestromsensors in mV/A (FÃ¼r ACS712 5A = 185)
#define chgChange 0               // Messwertumkehr von - nach +         1 oder 0
#define chgNull 2                 // Nullduchgang abziehen (1 oder 2)
// ------  charging station ---------------------------
#define stationRevDist 50   // charge station reverse 50 cm
#define stationRollAngle 45 // charge station roll after reverse
#define stationForwDist 30  // charge station accel distance cm
#define stationCheckDist 2  // charge station  check distance to be sure voltage is OK cm
#define UseBumperDock true  // bumper is pressed when docking or not
#define dockingSpeed 60     // speed docking is (percent of maxspeed)
#define autoResetActive 0   // after charging reboot or not

// ------ odometry ------------------------------------
#define odometryUse 1                   // use odometry?
#define odometryTicksPerRevolution 1010 // encoder ticks per one full resolution
#define odometryTicksPerCm 12.9         // encoder ticks per cm
#define odometryWheelBaseCm 43          // wheel-to-wheel distance (cm)

// ----- other -----------------------------------------
#define buttonUse 1               // has digital ON/OFF button?
#define RaspberryPIUse false      // a raspberryPi is connected to USBNative port
#define mowPatternDurationMax 120 // in minutes

// ----- user-defined switch ---------------------------
#define userSwitch1 0 // user-defined switch 1 (default value)
#define userSwitch2 0 // user-defined switch 2 (default value)
#define userSwitch3 0 // user-defined switch 3 (default value)
// ----- timer -----------------------------------------
#define timerUse 0 // use RTC and timer?

// ------ mower stats-------------------------------------------
#define statsOverride false // if set to true mower stats are overwritten - be careful
#define statsMowTimeMinutesTotal 300
#define statsBatteryChargingCounterTotal 10     // 11
#define statsBatteryChargingCapacityTotal 10000 // 30000
// -----------configuration end-------------------------------------

// ------ Pins, no changes needed---------------------------------------
// ------ Motor Left
#define pinMotorLeftEnable 5 // EN motors enable
#define pinMotorLeftPWM 7    // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 6    // M1_IN2 left motor Dir pin
// ------ Motor Right
#define pinMotorRightEnable 2 // EN motors enable
#define pinMotorRightPWM 4    // M2_IN1 right motor PWM pin
#define pinMotorRightDir 3    // M2_IN2 right motor Dir pin
// ------ Cutter Motor
#define pinMotorMowEnable 8 // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowPWM 10   // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 9    // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
// ------ Bumper
#define pinBumperLeft 35 // bumper pins
#define pinBumperRight 36
// ------ Sonar
#define pinSonarRightTrigger 29 // BBER10
#define pinSonarRightEcho A13
#define pinSonarLeftTrigger 28 // BBER10
#define pinSonarLeftEcho A12
// ------ Perimeter
#define pinPerimeterRight A8 // perimeter
#define pinPerimeterLeft A9
// ------ Buzzer
#define pinBuzzer 37 // Buzzer
// ------ Button
#define pinButton 38 // digital ON/OFF button
// ------ Battery Switsch
#define pinBatterySwitch 33 // battery-OFF switch
// ------ Charging Pin
#define pinChargeEnable 34 // charge relay
// ------ Hall Sensors
#define pinOdometryLeft 12  // left odometry sensor
#define pinOdometryRight 11 // right odometry sensor
// ------ Rain Sensor
#define pinRain 39 // rain sensor
// ------ Free Pins
#define pinUserOut1 13
#define pinUserOut2 32
#define pinUserOut3 A16
// IMU (compass/gyro/accel): I2C  (SCL, SDA)