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
  // Serial.print("Mqtt received: ");
  // Serial.println(topic);
  String payloadString = "";
  // Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + String(((char)payload[i]));
  }
  // Serial.println(payloadString);
  if (payloadString == "START") {
    Serial2.println("{ra}");
    
  }
  if (payloadString == "STOP") {
    Serial2.println("{ro}");
  }
  if (payloadString == "HOME") {
    Serial2.println("{rh}");
  }
  if (payloadString == "STARTTIMER") {
    //Serial2.println("{a02`160}");  //a02 is the motor power max pfod slider a ` is use for int 160 and ~ for string
    
    Serial2.println("{rv}");
  }
/*
 if(str(responsetable[0]) == "STARTTIMER"):
                send_var_message('w','mowPatternCurr',''+str(responsetable[1])+'','laneUseNr',''+str(responsetable[2])+'','rollDir',''+str(responsetable[3])+'','0','0','0')
                send_var_message('w','whereToStart',''+str(responsetable[4])+'','areaToGo',''+str(responsetable[5])+'','actualLenghtByLane',''+str(responsetable[6])+'','0','0','0')
                send_pfo_message('rv','1','2','3','4','5','6',)

*/

  
}

void mqttconnect() {

  if (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "admin";
    /* connect now */
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("Rl1000/COMMAND/#");
    } else {
      Serial.print("mqtt failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");

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
  char val1[6], val2[20], val3[20], val4[20], val5[20], val6[20];
  sscanf(line_receive, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", val1, val2, val3, val4, val5, val6);

  client.publish("Rl1000/Status", val2);
  client.publish("Rl1000/State", val3);
  client.publish("Rl1000/Temp", val4);
  client.publish("Rl1000/Battery", val5);
  client.publish("Rl1000/Idle", val6);





}
void esp32_Mqtt_stu() {
  char val1[6], val2[20], val3[10];
  sscanf(line_receive, "%[^,],%[^,],%[^,]", val1, val2, val3);
  client.publish("Rl1000/Status", val2);

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
  client.setServer(mqtt_server, 1883);
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
    Serial.println(client.connected());
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
          Serial.println(line_receive);
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
