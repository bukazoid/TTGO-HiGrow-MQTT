#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <PubSubClient.h>
//#include <ESP.h>
#include <Adafruit_BME280.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>

#include "user-variables.h"

#ifdef SERIALPRINT
#define SP(x) Serial.print(x)
#define SPLN(x) Serial.println(x)
#else
//#define SP(x)      do { if (0) Serial.print(x); } while (0)
//#define SPLN(x)    do { if (0) Serial.println(x); } while (0)
#define SP(x)
#define SPLN(x)

#endif /* DEBUG */

const char *version = "1.0.6";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Reboot counters
//XXX: store ip same way to speed up wifi
RTC_DATA_ATTR int bootCount = 0;
int bmeFound = 1;

// Collected data
int INVALID_VALUE = -1111; // unreal for most cases
float lux = INVALID_VALUE;
float temp = INVALID_VALUE;
float pressure = INVALID_VALUE;
float humidity = INVALID_VALUE;
uint16_t soil = INVALID_VALUE;
uint32_t salt = INVALID_VALUE;
float bat = INVALID_VALUE;
char macAddr[13]; //= "AABBCCDDEEFF"; and zero at the end

const int led = 13;

#define I2C_SDA 25
#define I2C_SCL 26
#define DHT_PIN 16
#define BAT_ADC 33
#define SALT_PIN 34
#define SOIL_PIN 32
#define BOOT_PIN 0
#define POWER_CTRL 4
#define USER_BUTTON 35
#define DS18B20_PIN 21 //18b20 data pin

BH1750 lightMeter(0x23); //0x23
Adafruit_BME280 bmp;     //0x77

#ifdef DHT_FOUND
DHT dht(DHT_PIN, DHT_TYPE); // NOT APPLICABLE ON MY DEVICE
#endif

bool bme_found = false;

// ***************************
// GOTO SLEEP......
// ***************************
void goToDeepSleep()
{
  SP("Going to sleep... ");
  SP(TIME_TO_SLEEP);
  SPLN(" seconds");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();

  // Turn off the BH1750
  digitalWrite(POWER_CTRL, 0);

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}

int connectToNetwork()
{
  int counter = 0;
#ifdef STATICIP
  WiFi.config(DeviceIP, GatewayIP, Subnet);
#endif
  WiFi.mode(WIFI_STA);

  //WiFi.begin(ssid, password);

  bool breakLoop = false;
  int ssidCount = sizeof(ssidArr) / sizeof(ssidArr[0]);
  SP("Number of registerd SSID: ");
  SPLN(ssidCount);

  for (int i = 0; i < ssidCount; i++)
  {
    //ssid = ssidArr2[i];
    //password = passArr[i].c_str();
    SP("SSID name: ");
    SPLN(ssidArr[i]);

    while (WiFi.status() != WL_CONNECTED)
    {
      // wifi down, reconnect here
      WiFi.begin(ssidArr[i], passArr[i]);

      while (WiFi.status() != WL_CONNECTED)
      {
        delay(100);
        SP(".");
        counter++;
        if (counter >= WIFI_CONNECT_TIMEOUT_MS / 100) // just keep terminal from scrolling sideways
        {
          breakLoop = true;
          break;
        }
      }
      if (breakLoop)
      {
        breakLoop = false;
        break;
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    goToDeepSleep();
  }

  return counter;
}

// READ SALT
uint32_t readSalt()
{
  uint8_t samples = 120;
  uint32_t humi = 0;
  uint16_t array[120];

  for (int i = 0; i < samples; i++)
  {
    array[i] = analogRead(SALT_PIN);
    delay(2);
  }
  std::sort(array, array + samples);
  for (int i = 0; i < samples; i++)
  {
    if (i == 0 || i == samples - 1)
      continue;
    humi += array[i];
  }
  humi /= samples - 2;
  return humi;
}

// *** READ SOIL
uint16_t readSoil()
{
  uint16_t soil = analogRead(SOIL_PIN);
  return soil;
}

// *** READ BATTERY
float readBattery()
{
  int vref = 1100;
  uint16_t volt = analogRead(BAT_ADC);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

// *** SEND FLOAT TO MQTT
// Dest : the location
void sendfloat(const char *dest, float value)
{
  char topic[100];
  char stringvalue[40];

  // convert the float value to string
  sprintf(stringvalue, "%f", value);
  // send to plant/<macaddres>/<dest>
  strcpy(topic, "plant/");
  //strcpy(topic, "tele/");
  strcat(topic, macAddr);
  strcat(topic, dest);

  if (!mqttClient.publish(topic, stringvalue, true))
  {
    goToDeepSleep();
  }
}

void sendbuffer(const char *dest, char *buffer)
{
  char topic[100];
  topic[0] = 0;
  strcpy(topic, "plant/");
  //strcpy(topic, "tele/");
  strcat(topic, macAddr);
  //strcat(topic, dest);
  // send to plant/<macaddres>/<dest>
  SPLN("TOPIC:");
  SPLN(topic);
  if (!mqttClient.publish(topic, buffer, true))
  {
    goToDeepSleep();
  }
}

void setup()
{

  Serial.begin(115200);

  SPLN("Entered Setup...");

  //Start WiFi
  int wifiCount = connectToNetwork();

  SPLN("Connected to network");
  SPLN(WiFi.macAddress());
  SPLN(WiFi.localIP());

  //Store mac address, we use this as part of the mqtt topic
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(macAddr, "%2X%2X%2X%2X%2X%2X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

#ifdef DHT_FOUND
  dht.begin(); // N/A in my version
#endif

  Wire.begin(I2C_SDA, I2C_SCL); // wire can not be initialized at beginng, the bus is busy

  //! Sensor power control pin , use deteced must set high
  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(POWER_CTRL, 1);
  adc_power_on();

  delay(10);

  // sometimes it would be better to call wakeup https://forum.mysensors.org/topic/9051/sleep-mode-for-bmp280
  // write8(BMP280_REGISTER_CONTROL, 0x3F);
  // if (bootCount == 0)
  // {
  if (!bmp.begin())
  {
    bme_found = false;
    bmeFound = 0;
  }
  else
  {
    SPLN("init BME");
    bmeFound = 1;
    bme_found = true;
    bmp.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
  }

  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  lightMeter.readLightLevel(); // 1st read seems to return 0 always
  delay(10);

  // DHT not available on our boards.
#ifdef DHT_FOUND
  delay(2000);
  temp = dht.readTemperature(); // Read temperature as Fahrenheit then dht.readTemperature(true)
  delay(100);
  humidity = dht.readHumidity();
#endif

  soil = readSoil();
  SP("soil ");
  SPLN(soil);
  int soil_min = 1500;
  int soil_max = 3330; // if we accept +- couple percents

  int diff = soil_max - soil_min;
  int top = soil_max - soil;
  if (top < 0)
  {
    top = 0;
  }

  int soilPercent = 100 * top / diff;

  salt = readSalt();
  SP("salt ");
  SPLN(salt);

  bat = readBattery();
  SP("battery ");
  SPLN(bat);

  lux = lightMeter.readLightLevel();
  SP("lux 2 ");
  SPLN(lux);

  if ((bme_found) && bmp.takeForcedMeasurement())
  {
    temp = bmp.readTemperature();
    SP("temp ");
    SPLN(temp);

    pressure = (bmp.readPressure() / 100.0F);
    pressure *= 0.75; // to get it in mmHg
    SP("pressure ");
    SPLN(pressure);

    humidity = bmp.readHumidity();
    SP("humidity ");
    SPLN(pressure);

    //https://forum.mysensors.org/topic/9051/sleep-mode-for-bmp280
    //bmp.sleep();
  };

  SPLN(F("Publishing results.."));

  //Connect MQTT client
  mqttClient.setServer(broker, 1883);
  if (!mqttClient.connect(broker, mqttuser, mqttpass))
  {
    SP("MQTT connection failed! Error code = ");
    SPLN(mqttClient.state());
    goToDeepSleep();
  }

  SPLN("Connected to the MQTT broker!");
  //meta push

#ifdef MQTT_PERTOPIC
  sendfloat("/temperature", temp);
  sendfloat("/pressure", pressure);
  sendfloat("/lumen", lux);
  sendfloat("/battery", bat);

  char soilbuf[10];
  sprintf(soilbuf, "%d", soil);
  sendbuffer("/soil", soilbuf);

  char saltbuf[10];
  sprintf(saltbuf, "%d", salt);
  sendbuffer("/salt", saltbuf);

  //Setup a meta data

  StaticJsonDocument<1024> doc;
  JsonObject root = doc.to<JsonObject>();
  root["mac"] = WiFi.macAddress();
  root["ip"] = WiFi.localIP().toString();
  root["bootcount"] = bootCount;
  root["firmware"] = version;
  root["ticks"] = xthal_get_ccount;
  char jsonbuffer[1024];
  serializeJson(doc, jsonbuffer);
  sendbuffer("/meta", jsonbuffer);
#else
  StaticJsonDocument<1024> doc;
  JsonObject root = doc.to<JsonObject>();
  if (bme_found == 1)
  {
    root["pressure"] = pressure;
  }
  root["lux"] = lux;
  root["batt"] = bat;
  root["soil"] = soil;
  root["soilPercent"] = soilPercent;
  root["salt"] = salt;
  root["mac"] = WiFi.macAddress();
  root["ip"] = WiFi.localIP().toString();
  root["bootcount"] = bootCount;
  root["firmware"] = version;
  root["ticks"] = xthal_get_ccount();
  root["worktime"] = millis();
  root["wifiCount"] = wifiCount;
  if (!isnan(temp))
  {
    root["temperature"] = temp;
  }
  if (!isnan(humidity))
  {
    root["humidity"] = humidity;
  }
  root["bme"] = bme_found;

  char jsonbuffer[1024];
  serializeJson(doc, jsonbuffer);

  sendbuffer("", jsonbuffer);
#endif
  SPLN("buffer sent");
  //Increment boot number and print it every reboot
  ++bootCount;

  //Go to sleep now
  delay(10);
  goToDeepSleep();
}

void loop()
{
}
