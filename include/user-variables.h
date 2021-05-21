// *******************************************************************************************************************************
// START userdefined data
// *******************************************************************************************************************************

//uncomment following line, if you want Serial log printing
//#define SERIALPRINT

#define uS_TO_S_FACTOR 1000000ULL //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 3600        //Time ESP32 will go to sleep (in seconds)

//If you define PERTOPIC, the firmware will give each sensor value it's own topic..
//MQTT Topics will look like
//- plant/<mac>/temperature
//- plant/<mac>/lumen
//- plant/<mac>/etc..
//If it's NOT defined , we will send all info into 1 topic as json.
//- plant/<mac>
//#define MQTT_PERTOPIC

//Select DHT type on the module - supported are DHT11, DHT12, DHT22
#define DHT_TYPE DHT11
//#define DHT_TYPE DHT12
//#define DHT_TYPE DHT22

#define DHT_FOUND
// Set to true if you have a DHT sensor on the board, and false if not

// define your SSID's
char const *ssidArr[] = {"bukazoid"};
char const *passArr[] = {"memento_17"};

// uncomment if you want to use DHCP
//#define STATICIP
#ifdef STATICIP
IPAddress DeviceIP(192, 168, 0, 191);
IPAddress GatewayIP(192, 168, 0, 1);
IPAddress Subnet(255, 255, 255, 0);
#endif

//ADVISE -> USE DIRECT IP ADDRESS OF BROKER, NOT A DNS NAME.
const char broker[] = "192.168.112.71";
int port = 1883;
const char mqttuser[] = "frozen"; //add eventual mqtt username
const char mqttpass[] = "buka83"; //add eventual mqtt password


const int WIFI_CONNECT_TIMEOUT_MS = 2500;// to not drain all battery on single login. Never seen more then 2100, if happends some value will be lost, so be it
// *******************************************************************************************************************************
// END userdefined data
// *******************************************************************************************************************************
