// *******************************************************************************************************************************
// START userdefined data
// *******************************************************************************************************************************

//uncomment following line, if you want Serial log printing
//#define SERIALPRINT

#define uS_TO_S_FACTOR 1000000ULL //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  3600       //Time ESP32 will go to sleep (in seconds) 

//Select DHT type on the module - supported are DHT11, DHT12, DHT22
//#define DHT_TYPE DHT11
#define DHT_TYPE DHT12
//#define DHT_TYPE DHT22

// define your SSID's
char const *ssidArr[] = { "Vosport", "Volera"};
char const *passArr[] = { "pass1", "pass3" };


// uncomment if you want to use DHCP 
#define STATICIP
#ifdef STATICIP
    IPAddress DeviceIP(192, 168, 0, 190);
    IPAddress GatewayIP(192, 168, 0, 1);
    IPAddress Subnet(255, 255, 255, 0);
#endif

//ADVISE -> USE DIRECT IP ADDRESS OF BROKER, NOT A DNS NAME.
const char broker[] = "192.168.0.251";
int        port     = 1883;
const char mqttuser[] = ""; //add eventual mqtt username
const char mqttpass[] = ""; //add eventual mqtt password

// *******************************************************************************************************************************
// END userdefined data
// *******************************************************************************************************************************
