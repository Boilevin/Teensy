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
#include "drivers.h"

const char *dayOfWeek[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[128], *ptr;

  strncpy( formatString, format, sizeof(formatString) ); // copy in from program mem

  // null terminate - leave char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString) - 2 ] = '\0';
  ptr = &formatString[ strlen(formatString) + 1 ]; // our result buffer...
  va_list args;
  va_start (args, format);
  vsnprintf(ptr, sizeof(formatString) - 1 - strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString) - 1 ] = '\0';
  out.print(ptr);
}

// rescale to -PI..+PI
double scalePI(double v)
{
  double d = v;
  while (d < 0) d += 2 * PI;
  while (d >= 2 * PI) d -= 2 * PI;
  if (d >= PI) return (-2 * PI + d);
  else if (d < -PI) return (2 * PI + d);
  else return d;
}

// computes minimum distance between x radiant (current-value) and w radiant (set-value)
double distancePI(double x, double w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  double d = scalePI(w - x);
  if (d < -PI) d = d + 2 * PI;
  else if (d > PI) d = d - 2 * PI;
  return d;
}

int time2minutes(timehm_t time) {
  return (time.hour * 60 + time.minute);
}

void minutes2time(int minutes, timehm_t &time) {
  time.hour   = minutes / 60;
  time.minute = minutes % 60;
}

String time2str(timehm_t time) {
  String s = String(time.hour / 10);
  s += (time.hour % 10);
  s += ":";
  s += (time.minute / 10);
  s += (time.minute % 10);
  return s;
}

String date2str(date_t date) {
  String s = dayOfWeek[date.dayOfWeek];
  s += " ";
  s += date.day / 10;
  s += date.day % 10;
  s += ".";
  s += date.month / 10;
  s += date.month % 10;
  s += ".";
  s += date.year;
  return s;
}

// brushless ZSX11HV1 motor driver
//NEED A SHUNT ON DRIVER J1 NEAR BIG CAPACITOR AND POTI TO full counter clock wise
//5V --> NOT CONNECTED
//0-5v --> NOT CONNECTED
//GND <-- TO J8 GND
//DIR <-- Direction TO J8 PIN DIR
//BRAKE <-- Brake TO J8 PIN EN
//STOP <-- NOT CONNECTED
//G --> NOT CONNECTED
//P --> SPEED TO J8 PIN PWM
//V --> NOT CONNECTED
//S --> ODO TO J1 ODO 
//G --> NOT CONNECTED

void setZSX11HV1(int pinDir, int pinPWM, int pinBrake, int speed, boolean brake) {
  //setZSX11HV1(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value, useMotorDriveBrake);
  if (speed < 0) {
    digitalWrite(pinBrake, LOW) ;
    digitalWrite(pinDir, LOW) ;
    analogWrite(pinPWM, ((byte)abs(speed)));
  }
  if (speed > 0) {
    digitalWrite(pinBrake, LOW) ;
    digitalWrite(pinDir, HIGH) ;
    analogWrite(pinPWM, ((byte)abs(speed)));
  }
  if (speed == 0) {
    analogWrite(pinPWM, 0);
    digitalWrite(pinDir, HIGH) ;
    if (brake) {
      digitalWrite(pinBrake, HIGH) ; // active the brake function of the motor driver  
    }
     
  }
}

// brushless ZSX12HV1 motor driver

void setZSX12HV1(int pinDir, int pinPWM, int pinBrake, int speed, boolean brake) {
  //setZSX11HV1(pinMotorRightDir, pinMotorRightPWM, pinMotorRightEnable, value, useMotorDriveBrake);
  if (speed < 0) {
    digitalWrite(pinBrake, HIGH) ;
    digitalWrite(pinDir, LOW) ;
    analogWrite(pinPWM, ((byte)abs(speed)));
  }
  if (speed > 0) {
    digitalWrite(pinBrake, HIGH) ;
    digitalWrite(pinDir, HIGH) ;
    analogWrite(pinPWM, ((byte)abs(speed)));
  }
  if (speed == 0) {
    analogWrite(pinPWM, 0);
    digitalWrite(pinDir, HIGH) ;
    if (brake) {
      digitalWrite(pinBrake, LOW) ; // active the brake function of the motor driver  
    }
     
  }
}



// L298N motor driver
// IN2/C(10)/PinPWM   IN1/D(12)/PinDir
// H                  L     Forward
// L                  H     Reverse
// pinDir  connected to IN1
// pinEnable  connected to IN2
// pinPWM  connected to ENA

void setL298N(int pinDir, int pinPWM , int pinEnable, int speed) {
  if (speed < 0) {
    digitalWrite(pinDir, HIGH) ;
    digitalWrite(pinEnable, HIGH) ;
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW) ;
    digitalWrite(pinEnable, HIGH) ;
    analogWrite(pinPWM, speed);
  }
}

// DFRobot Romeo All in one V1.1 motor driver
// D5/D6 PinPWM       D4/D7 PinDir
// H                  L     Forward
// H                  H     Reverse
void setRomeoMotor(int pinDir, int pinPWM, int speed) {
  if (speed < 0) {
    //digitalWrite(pinDir, HIGH) ;
    //PinMan.analogWrite(pinPWM, abs(speed));
  } else {
    //digitalWrite(pinDir, LOW) ;
    //PinMan.analogWrite(pinPWM, abs(speed));
  }
}



//bber1
// BTS7960 motor driver
// Dir pin is used for reverse PWM signal
// Pwm pin is used for forward PWM signal
// 2 EN_R an L are shunt to only one enable possibility ,so never send PWM on both pinDIR and PinPWM at same time
//VCC --> 3.3V
//GND --> GND
//R_IS --> NC
//L_IS --> NC
//R_EN --> pinMotorRightEnable
//L_EN --> pinMotorRightEnable
//RPWM --> pinMotorRightPWM
//LPWM --> pinMotorRightDir
//setBTS7960(pinMotorLeftDir, pinMotorLeftPWM, pinMotorLeftEnable, value); break;//
void setBTS7960(int pinDir , int pinPWM , int pinEnable , int speed) {
  if (speed > 0) {
    digitalWrite(pinEnable, HIGH) ;
    analogWrite(pinPWM, 0);
    analogWrite(pinDir, abs(speed));
  }
  if (speed < 0) {
    digitalWrite(pinEnable, HIGH) ;
    analogWrite(pinDir, 0);
    analogWrite(pinPWM, abs(speed));
  }
  if (speed == 0) {
    digitalWrite(pinEnable, LOW) ;
    analogWrite(pinPWM, 0);
    analogWrite(pinDir, 0);
  }
}

// MC33926 motor driver
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse
void setMC33926(int pinDir, int pinPWM, int speed) {
  if (speed < 0) {
    //digitalWrite(pinDir, HIGH) ;
    //PinMan.analogWrite(pinPWM, 255-((byte)abs(speed)));
  } else {
    //digitalWrite(pinDir, LOW) ;
    //PinMan.analogWrite(pinPWM, ((byte)speed));
  }
}

// ---- sensor drivers --------------------------------------------------------------










// Returns the day of week (0=Sunday, 6=Saturday) for a given date
int getDayOfWeek(int month, int day, int year, int CalendarSystem)
{
  // CalendarSystem = 1 for Gregorian Calendar, 0 for Julian Calendar
  if (month < 3)
  {
    month = month + 12;
    year = year - 1;
  }
  return (
           day
           + (2 * month)
           + int(6 * (month + 1) / 10)
           + year
           + int(year / 4)
           - int(year / 100)
           + int(year / 400)
           + CalendarSystem
         ) % 7;
}



unsigned long hstol(String recv) {
  //Serial.println(recv);
  char c[recv.length() + 1];
  unsigned long resultat;
  recv.toCharArray(c, recv.length() + 1);
  resultat = strtol(c, NULL, 16);
  //Serial.println(resultat);
  //Serial.println(String(resultat, HEX));
  return resultat;

}
