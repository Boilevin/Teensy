//passerelle for ESP32
//Possible use of ESP32 to pfod communication

//WIFI:
//you don't have wifi in all the garden:
//into config.h uncomment #define MODE_AP and comment #define MODE_STA
//pfod setting :  IP 192.168.1.4 port 8881 -> mower is the Access point pfod and perimeter sender connect to the mower

//you have wifi in all the garden:
//into config.h comment #define MODE_AP and uncomment #define MODE_STA
//set your ssid and password acoording to your WIFI router
//pfod setting : IP 10.0.0.122 (or change to correct IPgroup of your router) port 8881 -> mower and sender are connected to your home wifi router


//Bluetooth:
//limitation no automatic start and stop sender on multiple mowing area

//See config.h for more parameter

//TO RFID BOARD
// ESP-32    <--> PN5180 pin mapping:
// Vin       <--> 5V  if esp32 powered using USB YOU NEED TO SEND THE 5V TO PN5180
// 3.3V      <--> 3.3V
// GND       <--> GND
// SCLK, GPIO18   --> SCLK
// MISO, GPIO19  <--  MISO
// MOSI, GPIO23   --> MOSI
// SS, GPIO12     --> NSS (=Not SS -> active LOW)
// BUSY, GPIO13   <--  BUSY
// Reset, GPIO14  --> RST
//

//TO PCB1.3
// ESP-32    <--> pcb1.3 pin mapping:
// Vin       <--> 5v ON BT CONNECTOR
// GND       <--> GND ON BT CONNECTOR
// RX2       <--> TX ON BT CONNECTOR
// TX2       <--> RX ON BT CONNECTOR


#include "PN5180.h"
#include "PN5180ISO15693.h"

#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <HTTPClient.h>

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif


#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiClient TheClient;
WiFiServer TheServeur(8881);
#endif


uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;

char line_receive[256];
byte mon_index = 0;
String entete = "";


/*
  #include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
  BluetoothSerial ESP_BT; //Object for Bluetooth
*/

PN5180ISO15693 nfc(12, 13, 14);
uint8_t lastUid[8];



void esp32_Sender() {
  //line_receive is a char array separate by ,
  //example to stop sender on ip 15  "#SENDER,10.0.0.15,A0
  //example to start sender on ip 15  "#SENDER,10.0.0.15,A1
  HTTPClient http;


  char val1[20], val2[20], val3[10];
  sscanf(line_receive, "%[^,],%[^,],%[^,]", val1, val2, val3);
  //sscanf(line_receive, "%s %s %s", val1, val2, val3);


  String serverPath = "http://" + String(val2) + "/" + String(val3);
  Serial.println(serverPath);
  Serial.println(millis());

  http.begin(serverPath.c_str());
  //  http.begin("http://10.0.0.150:80/A1"); // URL
  //int httpCode = http.GET(); //we don't need the feedback smoothmag value is used to check if sender is OK
  http.GET();
  Serial.println(millis());

  // for (uint8_t posit = 0; posit < mon_index; posit++) {



  // }

}

void esp32_Mqtt() {
  //inData is a char array separate by ,
  //example to stop sender on ip 15  "#MQTT,10.0.0.15,A0

}



void setup() {
  delay(500);
  Serial.begin(115200);
  Serial2.begin(19200);
  if (debug) Serial.println("\n\n ESP32 BT and WiFi serial bridge V1.00");



  //


#ifdef MODE_AP
  if (debug) Serial.println("Open ESP Access Point mode");
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
  if (debug) Serial.println("ESP Access Point mode started at 192.168.4.1 on port 8881");
#endif


#ifdef MODE_STA
  if (debug) Serial.println("Start ESP32 Station mode");
  // STATION mode (ESP connects to router and use config.h IP and pass value)


  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, netmask);
  WiFi.begin(ssid, pw);
  if (debug) Serial.print("try to Connect to your Wireless network: ");
  if (debug) Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug) Serial.print(".");
    ///need to count and stop wifi scanning if more than 20 secondes
  }
  if (debug) Serial.println("\nWiFi connected IP fixe see config.h");
  if (debug) Serial.println("\nUse port 8881");
#endif

#ifdef BLUETOOTH
  if (debug) Serial.println("Start Bluetooth Server");
  SerialBT.begin("Teensy2"); //Bluetooth device name
#endif


#ifdef PROTOCOL_TCP


  if (debug) Serial.println("Starting Serveur on port 8881");
  TheServeur.begin(); // start TCP server
  TheServeur.setNoDelay(true);

#endif

  // esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power






  /*
    Serial.begin(115200);
    //BT seial for Pfod init
    Serial2.begin(19200);
    Serial.println("Start the Bluetooth");
    ESP_BT.begin("Teensy");
    Serial.println("Bluetooth Ready");
  */









  // rfid reader init
  Serial.println("Begin the RFID");
  nfc.begin();
  Serial.println("Reset the RFID");
  nfc.reset();
  Serial.println("Init the RFID");
  nfc.setupRF();
  Serial.println("RFID READER READY");
}

void loop() {

  uint8_t thisUid[8];
  // Try to read a tag ID (or "get inventory" in ISO15693-speak)
  ISO15693ErrorCode rc = nfc.getInventory(thisUid);
  // If the result code was that a card had been read
  if (rc == ISO15693_EC_OK) {
    // If this is the same ID as we read last frame
    if (memcmp(thisUid, lastUid, 8) == 0) {
      // Nothing to do - move on to the next reader
    }
    // If it's a different ID
    else {
      /*
        //Serial.print(F("New Card Detected"));
        //Serial.print(F("... "));
        Serial2.print("{RFID"); // pfod start message with {
        for (int j = 0; j <=3; j++) {
        Serial2.print(thisUid[j], HEX);
        }
        Serial2.println("}"); //pfod stop message with }
        // Update the array that keeps track of most recent ID
      */
      memcpy(lastUid, thisUid, sizeof(lastUid[0]) * 8);
    }
  }
  // If a card cannot be read
  else {
    // Test if we previously knew about a card (in which case it's just been removed
    // The most significant (last) byte of a valid UID should always be 0xE0. e.g. E007C4A509C247A8
    if (lastUid[7] == 0xE0) {
      /*
        Serial.print("Card ");
        for (int j=0; j<sizeof(lastUid); j++) {
        Serial.print(lastUid[j], HEX);
        }
        Serial.print(" removed from Reader ");
        Serial.println();
      */
      Serial2.print("{RFID"); // pfod start message with {
      /*
        //i dont use all the byte number
        for (int j = 0; j < sizeof(thisUid); j++) {
        Serial2.print(thisUid[8-j], HEX);
        }
      */
      for (int j = 0; j <= 2; j++) {
        Serial2.print(lastUid[j], HEX);
      }
      Serial2.println("}"); //pfod stop message with }
      // Update the array that keeps track of most recent ID
      // Update the array that keeps track of last known ID
      memset(lastUid, 0, sizeof(lastUid[0]) * 8);
    }

#ifdef DEBUG
    Serial.print(F("Error when reading : "));
    Serial.println(nfc.strerror(rc));
#endif
  }
  /*
    // BT passthrou for
    while (ESP_BT.available()) {
      Serial2.write(ESP_BT.read());
    }
    while (Serial2.available()) {
      ESP_BT.write(Serial2.read());
    }
  */

#ifdef BLUETOOTH
  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }
#endif

#ifdef PROTOCOL_TCP

  if (TheServeur.hasClient())
  {
    //find free/disconnected spot
    if (!TheClient || !TheClient.connected()) {
      if (TheClient) TheClient.stop();
      TheClient = TheServeur.available();
      if (debug) Serial.println("New WIFI client ");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TheServeur.available();
    TmpserverClient.stop();
  }

#endif

  if (Serial2 != NULL)
  {
    if (TheClient)
    {
      while (TheClient.available())
      {
        WIFIbuf[inWiFI] = TheClient.read(); // read char from client
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      Serial2.write(WIFIbuf, inWiFI); // now send to UART(2):
      inWiFI = 0;
    }
    if (Serial2.available())
    {
      while (Serial2.available())
      {
        WIFIbuf[inWiFI] = Serial2.read(); // read char from UART(2)
        char aChar = WIFIbuf[inWiFI];
        if (inWiFI < bufferSize - 1) inWiFI++;

        if (aChar == '\n')
        {
          // End of record detected. Time to parse and check for non pfod sentence
          Serial.println(line_receive);
          if (strncmp(line_receive, "#SENDER", 7) == 0) {
            esp32_Sender();
          }
          if (strncmp(line_receive, "#MQTT", 5) == 0) {
            esp32_Mqtt();
          }
          mon_index = 0;
          line_receive[mon_index] = NULL;
        }
        else
        {
          line_receive[mon_index] = aChar;
          mon_index++;
          line_receive[mon_index] = '\0'; // Keep the string NULL terminated
        }









      }
      if (TheClient)
        TheClient.write(WIFIbuf, inWiFI);

#ifdef BLUETOOTH
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);
#endif
      inWiFI = 0;
    }

  }
}
