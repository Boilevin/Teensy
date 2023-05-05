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

// drivers (motor driver, sensor drivers, etc.)

#ifndef DRIVERS_H
#define DRIVERS_H
#include <Arduino.h>

// ---------- date time --------------------------------------

extern const char *dayOfWeek[];

struct timehm_t {
  byte hour;
  byte minute;  
};

typedef struct timehm_t timehm_t;

struct date_t {
  byte dayOfWeek;
  byte day;
  byte month;
  short year;  
};

typedef struct date_t date_t;

struct datetime_t {
  timehm_t time;
  date_t date;
};

typedef struct datetime_t datetime_t;

// ---------- timers --------------------------------------
struct ttimer_t {
  boolean active;
  timehm_t startTime;
  timehm_t stopTime;
  byte daysOfWeek;
  int startDistance;
  byte startArea;
  byte startMowPattern;
  byte startNrLane;
  boolean startRollDir;
  byte startLaneMaxlengh;
  int rfidBeacon;
  
};

typedef struct ttimer_t ttimer_t;


// ---- other ----------------------------------


// returns sign of variable (-1, 0, +1)
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// ---------- driver functions ----------------------------------

//int freeRam();
  
// print helpers
void StreamPrint_progmem(Print &out,PGM_P format,...);
//#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)
#define Streamprint(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)
//String verToString(int v);

// time helpers
void minutes2time(int minutes, timehm_t &time);
int time2minutes(timehm_t time);
String time2str(timehm_t time);
String date2str(date_t date);

String versionToStr(byte v[]);

// rescale to -PI..+PI
double scalePI(double v);

// computes minimum distance between x radiant (current-value) and w radiant (set-value)
double distancePI(double x, double w);

// motor drivers
void setPwmFrequency(int pin, int divisor);
void setZSX11HV1(int pinDir, int pinPWM, int pinBrake, int speed, boolean brake);
void setZSX12HV1(int pinDir, int pinPWM, int pinBrake, int speed, boolean brake);

void setL298N(int pinDir, int pinPWM, int pinEnable, int speed);
void setRomeoMotor(int pinDir, int pinPWM, int speed);
void setMC33926(int pinDir, int pinPWM, int speed);
void setBTS7960(int pinDir ,int pinPWM ,int pinEnable , int speed);
// Returns the day of week (0=Sunday, 6=Saturday) for a given date
int getDayOfWeek(int month, int day, int year, int CalendarSystem);
unsigned long hstol(String recv);
#endif 
