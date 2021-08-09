/*Test code for Wemos D1 R1
 * Vin to 5V dc/dc out
 * GND to GND dc/dc out
 * D10 to pinbatteryswith PCB
 * D11 to pinCharge PCB
 * SDA to SDA0 PCB
 * SCL to SCL0 PCB
 * 3.3V to 3.3V PCB
 * GND to GND PCNB
*/
#include "INA226.h"
#include "Wire.h"

INA226 INA(0x40);


void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(1000);
  Serial.println("ok");
  Serial.println("Start ESP8266 INA226");
  Wire.begin();
  if (!INA.begin() )
  {
    Serial.println("could not connect. Fix and Reboot");
  }
  INA.setMaxCurrentShunt(1, 0.002);


  
  pinMode(BUILTIN_LED, OUTPUT);
 
  pinMode(D10, OUTPUT);
  pinMode(D11, OUTPUT);
  pinMode(D12, OUTPUT);
  
  digitalWrite(D10, HIGH);
  digitalWrite(D11, LOW);
  //digitalWrite(D12, HIGH);
  
  Serial.println("Start PCB");

  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUILTIN_LED, LOW);
    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
  }

  
  Serial.println("Start Charge");
  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(D11, HIGH);
  for (int i = 0; i < 40; i++)
  {
    Serial.print(INA.getBusVoltage(), 3);
    Serial.print("          ");
    Serial.print(INA.getShuntVoltage_mV(), 3);
    Serial.print("          ");
    Serial.print(INA.getCurrent_mA(), 3);
    Serial.print("          ");
    Serial.print(INA.getPower_mW(), 3);
    Serial.println();
    delay(500);
  }
  

  Serial.println("Stop Charge");
  digitalWrite(BUILTIN_LED, HIGH);
  digitalWrite(D11, LOW);
  for (int i = 0; i < 40; i++)
  {
    Serial.print(INA.getBusVoltage(), 3);
    Serial.print("          ");
    Serial.print(INA.getShuntVoltage_mV(), 3);
    Serial.print("          ");
    Serial.print(INA.getCurrent_mA(), 3);
    Serial.print("          ");
    Serial.print(INA.getPower_mW(), 3);
    Serial.println();
    delay(500);
  }
  
  Serial.println("Stop PCB");
  digitalWrite(D10, LOW);
  


  
}

void loop() {
}
