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
#define DOOR_UPDATE_INTERVAL_MS 500
#define MOTION_UPDATE_INTERVAL_MS 50
#define DOOR_OPEN_TIME_SEC 15
#define RELAY_DELAY 600
#define LIGHT_ON_THRESHOLD 75
#define FIRMWARE_VERSION "-2.01"
#define ENABLE_TEMP_MONITOR 1

/****************************** MQTT TOPICS (change these topics as you wish)  ***************************************/
#define MQTT_TIMER_PUB "sensor/garage-double/timer"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_GARAGE_SUB "sensor/garage-single/#"
#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_UPDATE_REQUEST "update"
#define MQTT_DISCOVERY_BINARY_SENSOR_PREFIX  "homeassistant/binary_sensor/"
#define MQTT_DISCOVERY_SENSOR_PREFIX  "homeassistant/sensor/"
#define MQTT_DISCOVERY_LIGHT_PREFIX  "homeassistant/light/"
#define MQTT_DISCOVERY_COVER_PREFIX  "homeassistant/cover/"
#define HA_TELEMETRY                         "ha"

#define TEMPERATURE "garage_single_temperature"
#define TEMPERATURE_NAME "Garage Single Temperature"
#define MOTION "garage_single_motion"
#define MOTION_NAME "Garage Single Door Motion"
#define LIGHT "garage_single_light"
#define LIGHT_NAME "Garage Single Light"
#define COVER "garage_single_door"
#define COVER_NAME "Garage Single Door"

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

Ticker ticker_fw, tickerDoorState;
Ticker tickerMotionCheck, tickerMotionTimer;
bool readyForFwUpdate = false;

WiFiClientSecure espClient;
PubSubClient client(espClient);

#include "common.h"

int analogValue = 0;
int timerCounter = 0;

#ifdef ENABLE_TEMP_MONITOR
Ticker tickerTemp;
Adafruit_BMP280 bme;

float f,f2;
float temp_offset = -0.5;

bool readyForTempUpdate = false;
#endif

bool readyForDoorUpdate = false;
bool readyForMotionUpdate = false;
bool killTimer = false;
bool doorPositionUpdate = false;
bool daylightStatus = true;
bool registered = false;
bool motion = false;
bool lastMotion = false;

String switch1;
String strTopic;
String strPayload;
int doorStateCount = 0;
String doorState = "";
String lastDoorState = "";
String lightState = "";
String lastLightState = "";

String lightStateTopic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + LIGHT + "/state";
String ligthCommandTopic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + LIGHT + "/command";
String coverCommandTopic = String(MQTT_DISCOVERY_COVER_PREFIX) + COVER + "/command";

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

  IPAddress result;
  int err = WiFi.hostByName(MQTT_SERVER, result) ;
  if(err == 1){
        Serial.print("MQTT Server IP address: ");
        Serial.println(result);
        MQTTServerIP = result.toString();
  } else {
        Serial.print("Error code: ");
        Serial.println(err);
  }  

  client.setBufferSize(512);  
  client.setServer(MQTT_SERVER, MQTT_SSL_PORT); //8883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port
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
    client.subscribe(MQTT_GARAGE_SUB);
    client.subscribe(MQTT_HEARTBEAT_SUB);
    client.subscribe(ligthCommandTopic.c_str());
    client.subscribe(coverCommandTopic.c_str());
  }

  if (readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

#ifdef ENABLE_TEMP_MONITOR
  if (readyForTempUpdate) {
    readyForTempUpdate = false;
    // Read temperature as Celcius
    f = bme.readTemperature();
    f = (f * 9 / 5) + 32;

    // Check if any reads failed and exit early (to try again).
    if (isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      f = f2;
    }
    else { //add offsets, if any
      f = f + temp_offset;
      f2 = f;
      Serial.print("Temperature: ");
      Serial.print(f, 1);
      Serial.print(" *F\n");
      str = String(f, 1);
      str.toCharArray(strCh, 9);
      updateSensor(TEMPERATURE, strCh);
    }
  }
#endif

  if (readyForDoorUpdate) {
    readyForDoorUpdate = false;
    checkDoorState();
    checkLightState();
  }

  if (killTimer) {
    killTimer = false;
    tickerMotionTimer.detach();
    toggleOpenerLight();
    timerCounter = 0;
    client.publish(MQTT_TIMER_PUB, "Light Timer killed");
  }

  if (readyForMotionUpdate) {
    readyForMotionUpdate = false;
    
    if (digitalRead(MOTION_PIN) == HIGH) {
      motion = true;
    }
    else {
      motion = false;
    }

    if (motion != lastMotion) {
      lastMotion = motion;
      if (motion == true) {
        updateBinarySensor(MOTION, "ON");
      }
      else {
        updateBinarySensor(MOTION, "OFF");
      }
    }
  }  

  client.loop(); //the mqtt function that processes MQTT messages
  if (! registered) {
    registerTelemetry();
    updateTelemetry("Unknown");
    createSensors(TEMPERATURE, TEMPERATURE_NAME, "temperature", "°F");
    createBinarySensors(MOTION, MOTION_NAME, "motion");
    createLight(LIGHT, LIGHT_NAME);
    createCover(COVER, COVER_NAME);
    registered = true;
  }

}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  String payload;

  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
 
  //if the 'garage/button' topic has a payload "OPEN", then 'click' the relay
  p_payload[p_length] = '\0';
  strTopic = String((char*)p_topic);
  if (strTopic == coverCommandTopic) {
    switch1 = String((char*)p_payload);
    if (switch1 == "open") {
      //'click' the relay
      digitalWrite(DOOR_RELAY_PIN, HIGH);
      my_delay(RELAY_DELAY);
      digitalWrite(DOOR_RELAY_PIN, LOW);
    }
  }
  if (strTopic == ligthCommandTopic) {
    switch1 = String((char*)p_payload);
    if ((switch1 == "ON") || (switch1 == "OFF")) {
      //'click' the relay
      digitalWrite(LIGHT_RELAY_PIN, HIGH);
      my_delay(RELAY_DELAY);
      digitalWrite(LIGHT_RELAY_PIN, LOW);
    }
  }
  if (strTopic == MQTT_HEARTBEAT_TOPIC) {
    resetWatchdog();
    updateTelemetry(String((char*)p_payload));
    if (payload.equals(String(MQTT_UPDATE_REQUEST))) {
      checkForUpdates();
    }    
  }
}

void checkDoorState() {
  //Checks if the door state has changed, and MQTT pub the change
  doorStateCount++;
  String door_position;
  if (digitalRead(DOOR_CLOSE_PIN) == 0) {
    doorState = "0";
    doorStateCount = 0;
  }
  else if (digitalRead(DOOR_OPEN_PIN) == 0) {
    doorState = "100";
    doorStateCount = 0;
  }
  else if (doorStateCount > DOOR_OPEN_TIME_SEC) {
    doorState = "50";
    doorStateCount = 0;
  }
  if(doorState != lastDoorState) {
    updateCover(COVER, doorState);
    lastDoorState = doorState;
  }
}

void checkLightState() {
  analogValue = analogRead(LIGHT_DETECTION_PIN);
  if (analogValue > LIGHT_ON_THRESHOLD) {
    lightState = "ON";
  }
  else {
    lightState = "OFF";
  }
  if (lightState != lastLightState) {
    updateLight(LIGHT, lightState);
    lastLightState = lightState;
  }
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
  if (timerCounter == 0) {
    killTimer = true;
  }
  client.publish(MQTT_TIMER_PUB, String(timerCounter).c_str());
}

void toggleOpenerLight() {
  digitalWrite(LIGHT_RELAY_PIN, HIGH);
  my_delay(RELAY_DELAY);
  digitalWrite(LIGHT_RELAY_PIN, LOW);
}

void createSensors(String sensor, String sensor_name, String device_class, String unit) {
  String topic = String(MQTT_DISCOVERY_SENSOR_PREFIX) + sensor + "/config";
  String message = String("{\"name\": \"") + sensor_name +
                   String("\", \"unique_id\": \"") + sensor + getUUID() +
                   String("\", \"unit_of_measurement\": \"") + unit +
                   String("\", \"state_topic\": \"") + String(MQTT_DISCOVERY_SENSOR_PREFIX) + sensor +
                   String("/state\", \"device_class\": \"" + device_class + "\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateSensor(String sensor, String state) {
  String topic = String(MQTT_DISCOVERY_SENSOR_PREFIX) + sensor + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void createBinarySensors(String sensor, String sensor_name, String device_class) {
  String topic = String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor + "/config";
  String message = String("{\"name\": \"") + sensor_name +
                   String("\", \"unique_id\": \"") + sensor + getUUID() +
                   String("\", \"state_topic\": \"") + String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor +
                   String("/state\", \"device_class\": \"" + device_class + "\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateBinarySensor(String sensor, String state) {
  String topic = String(MQTT_DISCOVERY_BINARY_SENSOR_PREFIX) + sensor + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void createLight(String light, String light_name) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/config";
  String message = String("{\"name\": \"") + light_name +
                   String("\", \"unique_id\": \"") + light + getUUID() +
                   String("\", \"optimistic\": \"false") +
                   String("\", \"command_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/command\", \"state_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/state\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateLight(String light, String state) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void createCover(String cover, String cover_name) {
  String topic = String(MQTT_DISCOVERY_COVER_PREFIX) + cover + "/config";
  String message = String("{\"name\": \"") + cover_name +
                   String("\", \"retain\": \"false") +
                   String("\", \"unique_id\": \"") + cover + getUUID() +
                   String("\", \"optimistic\": \"false") +
                   String("\", \"payload_close\": \"open") +
                   String("\", \"payload_stop\": \"open") +
                   String("\", \"payload_open\": \"open") +
                   String("\", \"device_class\": \"garage") +
                   String("\", \"command_topic\": \"") + String(MQTT_DISCOVERY_COVER_PREFIX) + cover +
                   String("/command\", \"position_topic\": \"") + String(MQTT_DISCOVERY_COVER_PREFIX) + cover +
                   String("/position\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateCover(String cover, String coverPosition) {
  String topic = String(MQTT_DISCOVERY_COVER_PREFIX) + cover + "/position";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(coverPosition);
  client.publish(topic.c_str(), coverPosition.c_str(), true);

}
