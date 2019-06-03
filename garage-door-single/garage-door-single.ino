//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "garage-single" // Enter your MQTT device
#define MQTT_SSL_PORT 8883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define TEMP_UPDATE_INTERVAL_SEC 6
#define DOOR_UPDATE_INTERVAL_MS 5000
#define MOTION_UPDATE_INTERVAL_MS 250
#define MOTION_TIMER_SEC 60
#define DOOR_OPEN_TIME_SEC 15
#define RELAY_DELAY 600
#define LIGHT_ON_THRESHOLD 800
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.30"
#define ENABLE_TEMP_MONITOR 1

/****************************** MQTT TOPICS (change these topics as you wish)  ***************************************/
#define MQTT_TEMPERATURE_PUB "sensor/garage-single/temperature"
#define MQTT_MOTION_PUB "sensor/garage-single/motion"
#define MQTT_TIMER_PUB "sensor/garage-single/timer"
#define MQTT_VERSION_PUB "sensor/garage-single/version"
#define MQTT_COMPILE_PUB "sensor/garage-single/compile"
#define MQTT_HEARTBEAT_PUB "sensor/garage-single/heartbeat"
#define MQTT_DOOR_POSITION_TEXT_TOPIC "sensor/garage-single/positiontext"
#define MQTT_DOOR_BUTTON_TOPIC "sensor/garage-single/action"
#define MQTT_DOOR_POSITION_TOPIC "sensor/garage-single/position"
#define MQTT_LIGHT_TOPIC "sensor/garage-single/light-state"
#define MQTT_LIGHT_BUTTON_TOPIC "sensor/garage-single/light-action"
#define MQTT_LIGHT_INTENSITY_TOPIC "sensor/garage-single/light-intensity"
#define MQTT_DAYLIGHT_TOPIC "sensor/daylight"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_GARAGE_SUB "sensor/garage-single/#"
#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_DAYLIGHT_SUB "sensor/daylight/#"

//Define the pins
// D1 and D2 - SDA/SCL
#define DOOR_RELAY_PIN 14      // D5
#define LIGHT_RELAY_PIN 15     // D8
#define DOOR_OPEN_PIN 13       // D7
#define DOOR_CLOSE_PIN 12      // D6
#define WATCHDOG_PIN 0         // D3
#define MOTION_PIN 16          // D0
#define LIGHT_DETECTION_PIN A0

#define RELAY_ON 0
#define RELAY_OFF 1
 
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

#include <ESP8266SSDP.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file
#include "certificates.h" // Place certificates for mqtt in this file 

int analogValue = 0;

int timerCounter = 0;

Ticker ticker_fw, tickerDoorState;
Ticker tickerMotionCheck, tickerMotionTimer;
bool readyForFwUpdate = false;

WiFiClientSecure espClient;
PubSubClient client(espClient);

#include "common.h"

#ifdef ENABLE_TEMP_MONITOR
Ticker tickerTemp;
Adafruit_BMP280 bme;

float f,f2;
float temp_offset = 0;

bool readyForTempUpdate = false;
#endif

bool readyForDoorUpdate = false;
bool readyForMotionUpdate = false;
bool killTimer = false;
bool doorPositionUpdate = false;
bool daylightStatus = true;
  
 
//Setup Variables
String switch1;
String daylight;
String strTopic;
String strPayload;
char* door_state = "UNDEFINED";
int doorStateCount = 0;
char* last_state = "";
 
void setup() {
  //Set Relay(output) and Door(input) pins
  pinMode(DOOR_RELAY_PIN, OUTPUT);
  digitalWrite(DOOR_RELAY_PIN, LOW);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(LIGHT_RELAY_PIN, LOW);
  pinMode(DOOR_OPEN_PIN, INPUT_PULLUP);
  pinMode(DOOR_CLOSE_PIN, INPUT_PULLUP);
  pinMode(WATCHDOG_PIN, OUTPUT);
  digitalWrite(WATCHDOG_PIN, LOW);
  pinMode(MOTION_PIN, INPUT);
 
  Serial.begin(115200);
  resetWatchdog();

#ifdef ENABLE_TEMP_MONITOR
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
#endif

  setup_wifi();

  client.setServer(MQTT_SERVER, MQTT_SSL_PORT); //1883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port 
  client.setCallback(callback); //callback is the function that gets called for a topic sub


#ifdef ENABLE_TEMP_MONITOR
  tickerTemp.attach_ms(TEMP_UPDATE_INTERVAL_SEC * 1000, tempTickerFunc);
#endif

  tickerDoorState.attach_ms(DOOR_UPDATE_INTERVAL_MS, doorStateTickerFunc);
  tickerMotionCheck.attach_ms(MOTION_UPDATE_INTERVAL_MS, motionCheckTickerFunc);

  checkForUpdates();
}

void loop() {
  char strCh[10];
  String str;
  
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

#ifdef ENABLE_TEMP_MONITOR
  if(readyForTempUpdate) {
      readyForTempUpdate = false;
      // Read temperature as Celcius
      f = bme.readTemperature();
      f = (f * 9 / 5) + 32;

      // Check if any reads failed and exit early (to try again).
      if (isnan(f)) {
        Serial.println("Failed to read from DHT sensor!");
        //h=t=f=-1;
        f=f2;
      }
      else { //add offsets, if any
        f = f + temp_offset;
        f2=f;
        Serial.print("Temperature: ");
        Serial.print(f,1);
        Serial.print(" *F\n");
        str = String(f,1);
        str.toCharArray(strCh,9);
        client.publish(MQTT_TEMPERATURE_PUB, strCh, true);

      }
  }
#endif

  if(readyForDoorUpdate) {
    readyForDoorUpdate = false;
    checkDoorState();
    checkLightState();
  }

  if(killTimer) {
    killTimer = false;
    tickerMotionTimer.detach();
    toggleOpenerLight();
    timerCounter = 0;
    client.publish(MQTT_TIMER_PUB, "Light Timer killed");
  }

  if(readyForMotionUpdate) {
    readyForMotionUpdate = false;
    if ((daylightStatus == false) && (timerCounter == 0)) {
      if (digitalRead(MOTION_PIN) == 1) {
        Serial.println("Motion Detected");
        client.publish(MQTT_MOTION_PUB, "motion", true);
        if(timerCounter == 0) {
          client.publish(MQTT_TIMER_PUB, "Light Timer Started");
          timerCounter = MOTION_TIMER_SEC;
          tickerMotionTimer.attach_ms(1000, motionTimerFunc);
          toggleOpenerLight();
        }
        else {
          timerCounter = MOTION_TIMER_SEC;
        }
      }
    }
  }
  
  client.loop(); //the mqtt function that processes MQTT messages
}
 
void callback(char* topic, byte* payload, unsigned int length) {
  //if the 'garage/button' topic has a payload "OPEN", then 'click' the relay
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if (strTopic == MQTT_DOOR_BUTTON_TOPIC) {
    switch1 = String((char*)payload);
    Serial.println(switch1);
    if (switch1 == "OPEN") {
      //'click' the relay
      Serial.println("ON");
      digitalWrite(DOOR_RELAY_PIN, HIGH);
      my_delay(RELAY_DELAY);
      digitalWrite(DOOR_RELAY_PIN, LOW);
    }
  }
  if (strTopic == MQTT_LIGHT_BUTTON_TOPIC) {
    switch1 = String((char*)payload);
    Serial.println(switch1);
    if ((switch1 == "ON") || (switch1 == "OFF")) {
      //'click' the relay
      Serial.println("ON");
      digitalWrite(LIGHT_RELAY_PIN, HIGH);
      my_delay(RELAY_DELAY);
      digitalWrite(LIGHT_RELAY_PIN, LOW);
    }
  }  
  if (strTopic == MQTT_HEARTBEAT_TOPIC) {
    Serial.println("Heartbeat received");
    resetWatchdog();
    client.publish(MQTT_HEARTBEAT_PUB, "Heartbeat Received");
  }
  if (strTopic == MQTT_DAYLIGHT_TOPIC) {
    Serial.println("Daylight Status Received");
    daylight = String((char*)payload);
    if (daylight == "TRUE") {
      daylightStatus = true;
      client.publish(MQTT_HEARTBEAT_PUB, "Sunrise Mode");
    }
    else {
      daylightStatus = false;
      client.publish(MQTT_HEARTBEAT_PUB, "Sunset Mode");
    }
  }
}

void checkDoorState() {
  //Checks if the door state has changed, and MQTT pub the change
  last_state = door_state; //get previous state of door
  doorStateCount++;
  String door_position;
  if (digitalRead(DOOR_CLOSE_PIN) == 0) {
    door_state = "0";
    doorPositionUpdate = true;
    doorStateCount = 0; 
    door_position = "Closed";
  }
  else if (digitalRead(DOOR_OPEN_PIN) == 0) {
    door_state = "100";
    doorPositionUpdate = true;
    doorStateCount = 0;
    door_position = "Open"; 
  }  
  else if (doorStateCount > DOOR_OPEN_TIME_SEC) {
    door_state = "50";
    doorStateCount = 0;
    doorPositionUpdate = true;
    door_position = "Partially Open"; 
  }

  if (doorPositionUpdate) {
    doorPositionUpdate = false;
    client.publish(MQTT_DOOR_POSITION_TOPIC, door_state, true);
    client.publish(MQTT_DOOR_POSITION_TEXT_TOPIC, door_position.c_str(), true);
    Serial.print("Door Position: ");
    Serial.println(door_state);
  }
}

void checkLightState() {
  analogValue = analogRead(LIGHT_DETECTION_PIN);
  Serial.print("Analog Value: ");
  Serial.println(analogValue);
  if (analogValue > LIGHT_ON_THRESHOLD) {
    client.publish(MQTT_LIGHT_TOPIC, "ON", true);
    Serial.println("Light: ON");
  }
  else {
    client.publish(MQTT_LIGHT_TOPIC, "OFF", true);    
    Serial.println("Light: OFF");
  }
  client.publish(MQTT_LIGHT_INTENSITY_TOPIC, String(analogValue).c_str(), true);
}
 
#ifdef ENABLE_TEMP_MONITOR
// Temperature update ticker
void tempTickerFunc() {
  readyForTempUpdate = true;
}
#endif

// Door update ticker
void doorStateTickerFunc() {
  readyForDoorUpdate = true;
}

void motionCheckTickerFunc() {
  readyForMotionUpdate = true;
}

void motionTimerFunc() {
  timerCounter--;
  if(timerCounter == 0) {
    killTimer = true;
  }
  Serial.println(timerCounter);
  client.publish(MQTT_TIMER_PUB, String(timerCounter).c_str());
}

void toggleOpenerLight() {
  Serial.println("Motion Toggle ON");
  digitalWrite(LIGHT_RELAY_PIN, HIGH);
  my_delay(RELAY_DELAY);
  digitalWrite(LIGHT_RELAY_PIN, LOW);  
}
