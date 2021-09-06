/*Test code for Wemos D1 R1
   Vin to 5V dc/dc out
   GND to GND dc/dc out
   D10 to pinbatteryswith PCB

   SDA to SDA0 PCB
   SCL to SCL0 PCB
   3.3V to 3.3V PCB
   GND to GND PCNB
*/
#include "INA226.h"
#include "Wire.h"

INA226 inaBat(0x40);
int motorIn1 = D2;
int motorIn2 = D3;
int motorEna = D11;



void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("ok");
  Serial.println("Start ESP8266 INA226");
  Wire.begin();
  if (!inaBat.begin() )
  {
    Serial.println("could not connect. Fix and Reboot");
  }
  inaBat.setMaxCurrentShunt(1, 0.002);



  pinMode(BUILTIN_LED, OUTPUT);

  pinMode(D10, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEna, OUTPUT);
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);

  analogWrite(motorEna, 0);

  digitalWrite(D10, HIGH);
  Serial.println("Start PCB");
  analogWriteFreq(1000);


  /*
    for (int i = 0; i < 40; i++)
    {
     Serial.print(inaBat.getBusVoltage(), 3);
     Serial.print("          ");
     Serial.print(inaBat.getShuntVoltage_mV(), 3);
     Serial.print("          ");
     Serial.print(inaBat.getCurrent_mA(), 3);
     Serial.print("          ");
     Serial.print(inaBat.getPower_mW(), 3);
     Serial.println();
     delay(500);
    }
  */


}

void loop() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  Serial.println("Accel FW");
  for (int i = 500; i < 1024; i=i+1)
  {
    Serial.println(i);
    analogWrite(motorEna, i);
    delay(1);
  }
delay(5000);
  Serial.println("Brake FW");
  for (int i = 500; i < 1024; i=i+1)
  {
    Serial.println(1024-i);
    analogWrite(motorEna, 1024-i);
    delay(1);
  }
  analogWrite(motorEna, 0);
Serial.println("Change Dir");
  
delay(1000);
digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  Serial.println("Accel BW");
  for (int i = 500; i < 1024; i=i+1)
  {
    Serial.println(i);
    analogWrite(motorEna, i);
    delay(1);
  }
delay(5000);
  Serial.println("Brake BW");
  for (int i = 500; i < 1024; i=i+1)
  {
    Serial.println(1024-i);
    analogWrite(motorEna, 1024-i);
    delay(1);
  }
 analogWrite(motorEna, 0);
delay(1000);
Serial.println("Change Dir");
}
