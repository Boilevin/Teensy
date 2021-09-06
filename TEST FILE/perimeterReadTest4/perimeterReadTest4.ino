#include "perimeter.h"
PerimeterClass perimeter;
int nextTimeShow = 0;
int ShowEach = 1000;
#define pinBatterySwitch 33  

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);
  perimeter.begin(A8, A9);
 // perimeter.speedTest();
}

void loop() {
  // put your main code here, to run repeatedly:
 // printConsole();
  /*
  if (millis() >= nextTimeShow) {

    nextTimeShow = nextTimeShow + ShowEach;
    Serial.println(perimeter.getMagnitude(1));
    //Serial.print("  ");

    //Serial.println(perimeter.getMagnitude(1));
  }
*/
Serial.println(perimeter.getMagnitude(0));
delay(10);
}
void printConsole() {
  
  /*
  Serial.print("min ");
  Serial.print(perimeter.getSignalMin(0));
  Serial.print("\t");
  Serial.print("max ");
  Serial.print(perimeter.getSignalMax(0));
  Serial.print("\t");
  Serial.print("avg ");
  Serial.print(perimeter.getSignalAvg(0));
  Serial.print("\t\t");
  */
  Serial.print("mag ");
  Serial.print((int)perimeter.getMagnitude(0));
  //Serial.print(",");
  //Serial.print((int)perimeter.getMagnitude(1));
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("smag ");
  Serial.print((int)perimeter.getSmoothMagnitude(0));
  Serial.print("\t");
  Serial.print("qty ");
  Serial.print(perimeter.getFilterQuality(0));
  /*
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("in ");
  Serial.print((int)perimeter.isInside(1));
  Serial.print("\t");
  Serial.print("on ");
  Serial.print((int)(!perimeter.signalTimedOut(1)));
  Serial.print("\t");
 */
  Serial.println();
  delay(100);
}
