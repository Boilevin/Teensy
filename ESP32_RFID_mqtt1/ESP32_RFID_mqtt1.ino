//SEE config.h for all the setting

//Hardware wiring :

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

//TO PCB1.3 if used :
// ESP-32    <--> pcb1.3 pin mapping:
// Vin       <--> 5v ON BT CONNECTOR
// GND       <--> GND ON BT CONNECTOR
// RX2       <--> TX ON BT CONNECTOR
// TX2       <--> RX ON BT CONNECTOR


#include "PN5180.h"
#include "PN5180ISO15693.h"

#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "PubSubClient.h"

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif



#include <WiFiClient.h>
WiFiClient pfodClient;
WiFiClient mqttClient;
PubSubClient client(mqttClient);

WiFiServer TheServeur(8881);



uint8_t BTbuf[my_bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[my_bufferSize];
uint16_t inWiFI = 0;

char msg[20];
unsigned long next_test_connection;
long lastMsg = 0;

char line_receive[256];
byte mon_index = 0;
String entete = "";


/*
  #include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
  BluetoothSerial ESP_BT; //Object for Bluetooth
*/

PN5180ISO15693 nfc(12, 13, 14);
uint8_t lastUid[8];

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mqtt received: ");
  Serial.println(topic);
  String payloadString = "";
  // Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + String(((char)payload[i]));
  }
  if (debug) Serial.println(payloadString);
  if (payloadString == "START") {
    Serial2.println("{ra}");

  }
  if (payloadString == "STOP") {
    Serial2.println("{ro}");
  }
  if (payloadString == "HOME") {
    Serial2.println("{rh}");
  }
  //for test on motor max power
  if (payloadString == "ON") {
    Serial2.println("{ya0`1}");
    Serial2.println("{ya1`2}");
    Serial2.println("{ya2`1}");
    Serial2.println("{ya3`220}");
    Serial2.println("{ya4`3}");
    Serial2.println("{ya5`80}");

  }
  if (payloadString == "OFF") {
    Serial2.println("{ya0`2}");
    Serial2.println("{ya1`3}");
    Serial2.println("{ya2`0}");
    Serial2.println("{ya3`440}");
    Serial2.println("{ya4`1}");
    Serial2.println("{ya5`10}");

    
  }
  //end test

  if (payloadString == "STARTTIMER") {

    Serial2.println("{ya0`1}");
    Serial2.println("{ya1`3}");
    Serial2.println("{ya2`0}");
    Serial2.println("{ya3`440}");
    Serial2.println("{ya4`1}");
    Serial2.println("{ya5`10}");

    Serial2.println("{rv}");
  }

  //payload : STARTTIMER;1;1;0;25;2;50 it's to start from station STARTTIMER;MowPattern;LaneNr;rollDir;WhereStart;areToGo;LaneLenght
  /*
    if(str(responsetable[0]) == "STARTTIMER"):
                  send_var_message('w','mowPatternCurr',''+str(responsetable[1])+'','laneUseNr',''+str(responsetable[2])+'','rollDir',''+str(responsetable[3])+'','0','0','0')
                  send_var_message('w','whereToStart',''+str(responsetable[4])+'','areaToGo',''+str(responsetable[5])+'','actualLenghtByLane',''+str(responsetable[6])+'','0','0','0')
                  send_pfo_message('rv','1','2','3','4','5','6',)


    sendSlider("ya0", F("mowPatternCurr"), robot->mowPatternCurr, "", 1, 3, 0);
    sendSlider("ya1", F("laneUseNr"), robot->laneUseNr, "", 1, 3, 0);
    sendSlider("ya2", F("rollDir"), robot->rollDir, "", 1, 1, 0);
    sendSlider("ya3", F("whereToStart"), robot->whereToStart, "", 1,9999, 0);
    sendSlider("ya4", F("areaToGo"), robot->areaToGo, "", 1, 3, 0);
    sendSlider("ya5", F("actualLenghtByLane"), robot->actualLenghtByLane, "", 1, 255, 0);
  */


}

void mqttconnect() {

  if (!client.connected()) {
    if (debug) Serial.print("MQTT connecting ...");
    /* client ID */
    //String clientId = "admin";
    /* connect now */

    //if (client.connect(clientId.c_str())) {
    if (client.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      if (debug) Serial.println("connected");
      //const char* cmd_msg = "/COMMAND/#";
      char outMessage[strlen(mower_name) + strlen(mqtt_subscribeTopic1)];
      sprintf(outMessage, "%s%s", mower_name, mqtt_subscribeTopic1);
      if (debug) Serial.print("Subscribe to : ");
      if (debug) Serial.println(outMessage);
      client.subscribe(outMessage);

    } else {
      if (debug) Serial.print("mqtt failed, status code =");
      if (debug) Serial.print(client.state());
      if (debug) Serial.println("try again in 5 seconds");

    }
  }
}

void esp32_Sender() {
  //line_receive is a char array separate by ,
  //example to stop sender on ip 15  "#SENDER,10.0.0.15,A0
  //example to start sender on ip 15  "#SENDER,10.0.0.15,A1
  HTTPClient http;
  char val1[10], val2[20], val3[10];
  sscanf(line_receive, "%[^,],%[^,],%[^,]", val1, val2, val3);
  String serverPath = "http://" + String(val2) + "/" + String(val3);
  http.begin(serverPath.c_str());
  http.GET();
}

void esp32_Mqtt_sta() {
  //receive from mower msgid,status,state,temp,battery,idle
  //message separation
  char val1[6], val2[20], val3[20], val4[20], val5[20], val6[20];
  sscanf(line_receive, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", val1, val2, val3, val4, val5, val6);

  //status
  char outTopic1[strlen(mower_name) + strlen(mqtt_statusTopic)];
  sprintf(outTopic1, "%s%s", mower_name, mqtt_statusTopic);
  client.publish(outTopic1, val2);
  //state
  char outTopic2[strlen(mower_name) + strlen(mqtt_stateTopic)];
  sprintf(outTopic2, "%s%s", mower_name, mqtt_stateTopic);
  client.publish(outTopic2, val3);
  //temp
  char outTopic3[strlen(mower_name) + strlen(mqtt_tempTopic)];
  sprintf(outTopic3, "%s%s", mower_name, mqtt_tempTopic);
  client.publish(outTopic3, val4);
  //battery
  char outTopic4[strlen(mower_name) + strlen(mqtt_batteryTopic)];
  sprintf(outTopic4, "%s%s", mower_name, mqtt_batteryTopic);
  client.publish(outTopic4, val5);
  //idle
  char outTopic5[strlen(mower_name) + strlen(mqtt_idleTopic)];
  sprintf(outTopic5, "%s%s", mower_name, mqtt_idleTopic);
  client.publish(outTopic5, val6);
}



void setup() {
  delay(500);
  Serial.begin(115200);
  Serial2.begin(19200);
  if (debug) Serial.println("\n\n ESP32 BT and WiFi serial bridge");

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


  //********************************WIFI code*************************************
  if (debug) Serial.println("Starting Serveur on port 8881");
  TheServeur.begin(); // start TCP server
  TheServeur.setNoDelay(true);


  //********************************MQTT code*************************************
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(receivedCallback);

  //********************************RFID code*************************************
  if (rfid_board_IsPluged) {
    // rfid reader init
    if (debug) Serial.println("Begin the RFID");
    nfc.begin();
    if (debug) Serial.println("Reset the RFID");
    nfc.reset();
    if (debug) Serial.println("Init the RFID");
    nfc.setupRF();
    if (debug) Serial.println("RFID READER READY");
  }
  else
  {
    if (debug) Serial.println("RFID READER NOT USE");
  }

}

void loop() {
  //********************************MQTT code*************************************
  if ((useMqtt) && (!client.connected()) && (millis() > next_test_connection))
  {
    next_test_connection = millis() + 5000;
    mqttconnect();
  }
  //********************************RFID code*************************************
  if (rfid_board_IsPluged)
  {
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
        memcpy(lastUid, thisUid, sizeof(lastUid[0]) * 8);
      }
    }
    // If a card cannot be read
    else {
      // Test if we previously knew about a card (in which case it's just been removed
      // The most significant (last) byte of a valid UID should always be 0xE0. e.g. E007C4A509C247A8
      if (lastUid[7] == 0xE0) {

        Serial2.print("{RFID"); // pfod start message with {

        for (int j = 0; j <= 2; j++) {
          Serial2.print(lastUid[j], HEX);
        }
        Serial2.println("}"); //pfod stop message with }
        // Update the array that keeps track of most recent ID
        // Update the array that keeps track of last known ID
        memset(lastUid, 0, sizeof(lastUid[0]) * 8);
      }

    }
  }

#ifdef BLUETOOTH
  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < my_bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }
#endif

  if (TheServeur.hasClient())
  {
    //find free/disconnected spot
    if (!pfodClient || !pfodClient.connected()) {
      if (pfodClient) pfodClient.stop();
      pfodClient = TheServeur.available();
      if (debug) Serial.println("New WIFI client ");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TheServeur.available();
    TmpserverClient.stop();
  }


  //data coming from pfod
  if (Serial2 != NULL)
  {
    if (pfodClient)
    {
      while (pfodClient.available())
      {
        WIFIbuf[inWiFI] = pfodClient.read(); // read char from client
        if (inWiFI < my_bufferSize - 1) inWiFI++;
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
        if (inWiFI < my_bufferSize - 1) inWiFI++;

        if (aChar == '\n')
        {
          // End of record detected. Time to parse and check for non pfod sentence
          //here data coming from mqtt or pfod over wifi
          if (debug) Serial.println(line_receive);
          if (strncmp(line_receive, "$SENDER", 7) == 0) {
            esp32_Sender();
          }
          if (strncmp(line_receive, "#RMSTA", 6) == 0) {
            esp32_Mqtt_sta();
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
      if (pfodClient)
        pfodClient.write(WIFIbuf, inWiFI);

#ifdef BLUETOOTH
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);
#endif
      inWiFI = 0;
    }

  }
  //mqtt refresh
  client.loop();

}
