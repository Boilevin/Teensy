#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "robot.h"
#include "drivers.h"

class Mower : public Robot
{
public:
  Mower();
  virtual void setup(void);
  // virtual void resetMotorFault();

  virtual void setActuator(char type, int value);
};

extern Mower robot;

#endif
