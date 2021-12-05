
#define BLUETOOTH
//Select one of the 2 possibility AP (access point) or STA (station)
//#define MODE_AP // phone connects directly to ESP32 inside the mower 
#define MODE_STA // ESP32 and phone connects to wifi routeur
#define my_bufferSize 1024
#define rfid_board_IsPluged true
#define useMqtt true
#define VERSION "1.20"

const char* mqtt_server = "10.0.0.8";
bool debug = true;

#ifdef MODE_STA
// For standard mode:
//you need to set a fix IP and gateway according to your routeur value
// ssid and password according your routeur
const char *ssid = "your ssid";  // You will connect your phone to this Access Point
const char *pw = "your pass"; // and this is the password
//for pfod use this IP and port 8881
IPAddress ip(10, 0, 0, 123); //
IPAddress gateway(10, 0, 0, 1); //
IPAddress netmask(255, 255, 255, 0);
#endif


#ifdef MODE_AP
// For AP mode:
// Don't forget to connect your WIFI phone to the AP :

const char *ssid = "Teensy2000";  // You will connect your phone to this Access Point
const char *pw = ""; // no password
//for pfod use this IP and port 8881
IPAddress ip(192, 168, 4, 1); //
IPAddress gateway(192, 168, 4, 0); //
IPAddress netmask(255, 255, 255, 0);
#endif
