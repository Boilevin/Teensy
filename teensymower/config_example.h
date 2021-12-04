// Configure the Options below and finally compile and upload it to the Teensy 4.1

// code version
#define VER "1.0-Teensyber GY-521"

// ------ Compass
//#define COMPASS_IS HMC5883L
#define COMPASS_IS QMC5883L

// ------ Motor Driver
#define BTS7960
//#define L298N
//#define BRUSHLESS

// ------ No Changes needed---------------------------------------
// ------ pins---------------------------------------
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
#define pinOdometryLeft 12 // left odometry sensor
#define pinOdometryRight 11 // right odometry sensor
// ------ Rain Sensor
#define pinRain 39 // rain sensor
// ------ Free Pins
#define pinUserOut1 13 
#define pinUserOut2 32 
#define pinUserOut3 A16 
// IMU (compass/gyro/accel): I2C  (SCL, SDA)

// ------- baudrates---------------------------------

// ------ used serial ports for console, Bluetooth, ESP8266 -----------------------------

// we use 'SerialUSB' for 'Console' so the Raspberry PI receive all data
// we use 'Serial' for 'Console' so the PC receive all data
#define Console Serial
#define CONSOLE_BAUDRATE 115200 // baudrate used for Raspberry PI console

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