#include <stdio.h>

//display
#include <TM1637Display.h>
#define CLK 13                //D7                         //pins for display
#define DIO 12                //d6
TM1637Display display(CLK, DIO);

#include "Arduino.h"
#include "mhz19.h"                                         // include main library
#include "SoftwareSerial.h"


#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
const int DELAY = 3000;
const int INTERVAL = 60000;
const int INTERVAL_DISP = 2000;      //display change interval
const int STARTUP_DELAY = 500;
boolean flag = false;
byte flagg=0;
float tempC = 0;
int co2 = 0;


#include <Adafruit_NeoPixel.h>

// Which pin for LED
#define PIN            D3
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);





Adafruit_BME280 bme;


#define PIN_RX  D2
#define PIN_TX  D1

#define MQTT_HOST   "192.168.0.15"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "/homeassistant/office/sensors/co2"
#define MQTT_TOPIC1 "/homeassistant/office/sensors/temp"
#define MQTT_TOPIC2 "/homeassistant/office/sensors/pressure"
#define MQTT_TOPIC3 "/homeassistant/office/sensors/humidity"
#define MQTT_USER "ha1"
#define MQTT_PASS "ha1"


SoftwareSerial sensor(PIN_RX, PIN_TX);
WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//Time


#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200, 60000);



static char esp_id[16];

static bool exchange_command(uint8_t cmd, uint8_t data[], unsigned int timeout)
{
  // create command buffer
  uint8_t buf[9];
  int len = prepare_tx(cmd, data, buf, sizeof(buf));

  // send the command
  sensor.write(buf, len);

  // wait for response
  unsigned long start = millis();
  while ((millis() - start) < timeout) {
    if (sensor.available() > 0) {
      uint8_t b = sensor.read();
      if (process_rx(b, cmd, data)) {
        return true;
      }
    }
    yield();
  }

  return false;
}

static bool read_temp_co2(int *co2, int *temp)
{
  uint8_t data[] = { 0, 0, 0, 0, 0, 0 };
  bool result = exchange_command(0x86, data, 3000);
  if (result) {
    *co2 = (data[0] << 8) + data[1];
    *temp = data[2] - 40;
#if 1
    char raw[32];
    sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3],
            data[4], data[5]);
    Serial.println(raw);
#endif
  }
  return result;
}

static bool mqtt_send(const char *topic, const char *value, bool retained)
{
  bool result = false;
  if (!mqttClient.connected()) {
    Serial.print("Connecting MQTT...");
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    result = mqttClient.connect(esp_id, MQTT_USER, MQTT_PASS, topic, 0, retained, "offline");
    Serial.println(result ? "OK" : "FAIL");
  }
  if (mqttClient.connected()) {
    Serial.print("Publishing ");
    Serial.print(value);
    Serial.print(" to ");
    Serial.print(topic);
    Serial.print("...");
    result = mqttClient.publish(topic, value, retained);
    Serial.println(result ? "OK" : "FAIL");
  }
  return result;
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  display.setBrightness(0x0f);
  display.showNumberDec(0, true);


  Serial.println("MHZ19 ESP reader\n");

  sprintf(esp_id, "%08X", ESP.getChipId());
  Serial.print("ESP ID: ");
  Serial.println(esp_id);

  sensor.begin(9600);

  Serial.println("Starting WIFI manager ...");
  wifiManager.setConfigPortalTimeout(120);
  wifiManager.autoConnect("ESP-MHZ19");

  //time
  timeClient.begin();
    //time
      timeClient.update();



  Wire.begin(D5, D4);
  // Wire.setClock(100000);
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
    {
      yield();
      delay(DELAY);
    }
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

}

void loop()
{
  static unsigned long last_sent = 0;
  static unsigned long last_shown = 0;
  int temp;
  unsigned long m = millis();
  if ((m - last_sent) >= INTERVAL) {
    bme.takeForcedMeasurement();
    tempC = bme.readTemperature();
    int humidity = bme.readHumidity();
    int pressure = bme.readPressure() / 100;
    if (read_temp_co2(&co2, &temp)) {
      Serial.print("CO2:");
      Serial.println(co2, DEC);
      Serial.print("tempC:");
      Serial.println(tempC, DEC);
      Serial.print (pressure);
      Serial.println(" hPa");
      Serial.print (humidity);
      Serial.println ( " %");
      // send over MQTT
      char message[16], message1[16], message2[16], message3[16];
      snprintf(message, sizeof(message), "%d", co2);
      snprintf(message1, sizeof(message1), "%10.1f", tempC);
      snprintf(message2, sizeof(message2), "%d", pressure);
      snprintf(message3, sizeof(message3), "%d", humidity);


      if (!mqtt_send(MQTT_TOPIC, message, true) & !mqtt_send(MQTT_TOPIC1, message1, true) & !mqtt_send(MQTT_TOPIC2, message2, true) & !mqtt_send(MQTT_TOPIC3, message3, true)) {
        Serial.println("Restarting ESP...");
        ESP.restart();
      }
      if (co2 < 800)
      {
        pixels.setPixelColor(0, pixels.Color(0, 150, 0));
      }
      else if (co2 < 1200)
      {
        pixels.setPixelColor(0, pixels.Color(150, 150, 0));
      }
      else
      {
        pixels.setPixelColor(0, pixels.Color(150, 000, 0));
      }


      pixels.show();




    }
    last_sent = m;
  }
  //Show data on display
  m = millis();
  if ((m - last_shown) >= INTERVAL_DISP) {
    if (flagg<=2) {
      if (flagg==0) {display.clear();}
      display.showNumberDecEx(timeClient.getHours(), 0x40, true, 2, 0);
            display.showNumberDecEx(timeClient.getMinutes(), 0, true, 2, 2);
      flagg++;
    }
    else if (flagg==3) {
      display.clear();
      //   void showNumberDecEx(int num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
      display.showNumberDecEx(tempC, 0, false, 2, 0);
      display.showNumberHexEx(0x0c, 0, false, 1, 3);       // Expect: C
      flagg++;
    }
    else  {
      display.clear();
      display.showNumberDecEx(co2, 0, false); // no zero's
             
      flagg=0;
    }
    
          Serial.print(timeClient.getHours());
          Serial.print (":");
          Serial.println(timeClient.getMinutes());


    last_shown = m;

  }

  mqttClient.loop();
}
