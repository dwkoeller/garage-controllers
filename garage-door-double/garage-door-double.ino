#include <ESP8266SSDP.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file
 
//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "garage-double" // Enter your MQTT device
#define MQTT_PORT 1883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define TEMP_UPDATE_INTERVAL_SEC 6
#define WATCHDOG_UPDATE_INTERVAL_SEC 1
#define WATCHDOG_RESET_INTERVAL_SEC 30
#define DOOR_UPDATE_INTERVAL_MS 1000
#define DOOR_OPEN_TIME_SEC 15
#define RELAY_DELAY 600
#define LIGHT_ON_THRESHOLD 900
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.15"
//#define ENABLE_TEMP_MONITOR 1

/****************************** MQTT TOPICS (change these topics as you wish)  ***************************************/
//#define MQTT_TEMPERATURE_PUB "sensor/garage-double/temperature"
#define MQTT_DOOR_POSITION_TEXT_TOPIC "sensor/garage-double/positiontext"
#define MQTT_DOOR_BUTTON_TOPIC "sensor/garage-double/action"
#define MQTT_DOOR_POSITION_TOPIC "sensor/garage-double/position"
#define MQTT_LIGHT_TOPIC "sensor/garage-double/light-state"
#define MQTT_LIGHT_BUTTON_TOPIC "sensor/garage-double/light-action"
#define MQTT_LIGHT_INTENSITY_TOPIC "sensor/garage-double/light-intensity"
#define MQTT_VERSION_PUB "sensor/garage-double/version"
#define MQTT_COMPILE_PUB "sensor/garage-double/compile"
#define MQTT_GARAGE_SUB "sensor/garage-double/#"

//Define the pins
#define DOOR_RELAY_PIN 14      // D5
#define LIGHT_RELAY_PIN 15     // D8
#define DOOR_OPEN_PIN 13       // D7
#define DOOR_CLOSE_PIN 12      // D6
#define LIGHT_DETECTION_PIN A0

#define RELAY_ON 0
#define RELAY_OFF 1
 
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

int analogValue = 0;

volatile int watchDogCount = 0;

// Create event timers to update NTP, temperature and invert OLED disply
Ticker ticker_fw, ticker_watchdog, tickerDoorState;

#ifdef ENABLE_TEMP_MONITOR
Ticker ticker_temp;
Adafruit_BMP280 bme; // I2C

float f,f2;//Added %2 for error correction
float temp_offset = 0;

bool readyForTempUpdate = false;
#endif

bool readyForFwUpdate = false;
bool readyForDoorUpdate = false;

bool doorPositionUpdate = false;
  
WiFiClient espClient;
 
//Initialize MQTT
PubSubClient client(espClient);
 
//Setup Variables
String switch1;
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
 
  Serial.begin(115200);

#ifdef ENABLE_TEMP_MONITOR
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
#endif

  setup_wifi();

  client.setServer(MQTT_SERVER, MQTT_PORT); //1883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port 
  client.setCallback(callback); //callback is the function that gets called for a topic sub


#ifdef ENABLE_TEMP_MONITOR
  ticker_temp.attach_ms(TEMP_UPDATE_INTERVAL_SEC * 1000, tempTicker); // Run a 1 second interval Ticker
#endif

  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker); // Run a 24 hour interval Ticker
  ticker_watchdog.attach_ms(WATCHDOG_UPDATE_INTERVAL_SEC * 1000, watchdogTicker); // Run a 24 hour interval Ticker
  tickerDoorState.attach_ms(DOOR_UPDATE_INTERVAL_MS, doorTicker); // Run a 24 hour interval Ticker

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
  
  client.loop(); //the mqtt function that processes MQTT messages
  watchDogCount = 0;
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
}

void setup_wifi() {
  int count = 0;
  my_delay(50);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MQTT_DEVICE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    my_delay(250);
    Serial.print(".");
    count++;
    if(count > 50) {
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      wifiManager.autoConnect();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
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
 
void reconnect() {
  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  Serial.print("Attempting MQTT connection...");
  if (client.connect(MQTT_DEVICE, MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("connected");
    client.subscribe(MQTT_GARAGE_SUB);
    String firmwareVer = String("Firmware Version: ") + String(FIRMWARE_VERSION);
    String compileDate = String("Build Date: ") + String(compile_date);
    client.publish(MQTT_VERSION_PUB, firmwareVer.c_str(), true);
    client.publish(MQTT_COMPILE_PUB, compileDate.c_str(), true);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    my_delay(5000);
  }
}

#ifdef ENABLE_TEMP_MONITOR
// Temperature update ticker
void tempTicker() {
  readyForTempUpdate = true;
}
#endif

// FW update ticker
void fwTicker() {
  readyForFwUpdate = true;
}

// Watchdog update ticker
void watchdogTicker() {
  watchDogCount++;
  if(watchDogCount >= WATCHDOG_RESET_INTERVAL_SEC) {
    Serial.println("Reset system");
    ESP.restart();  
  }
}

// Door update ticker
void doorTicker() {
  readyForDoorUpdate = true;
}

String WiFi_macAddressOf(IPAddress aIp) {
  if (aIp == WiFi.localIP())
    return WiFi.macAddress();

  if (aIp == WiFi.softAPIP())
    return WiFi.softAPmacAddress();

  return String("00-00-00-00-00-00");
}

void checkForUpdates() {

  String clientMAC = WiFi_macAddressOf(espClient.localIP());

  Serial.print("MAC: ");
  Serial.println(clientMAC);
  clientMAC.replace(":", "-");
  String filename = clientMAC.substring(9);
  String firmware_URL = String(UPDATE_SERVER) + filename + String(FIRMWARE_VERSION);
  String current_firmware_version_URL = String(UPDATE_SERVER) + filename + String("-current_version");

  HTTPClient http;

  http.begin(current_firmware_version_URL);
  int httpCode = http.GET();
  
  if ( httpCode == 200 ) {

    String newFirmwareVersion = http.getString();
    newFirmwareVersion.trim();
    
    Serial.print( "Current firmware version: " );
    Serial.println( FIRMWARE_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFirmwareVersion );
    
    if(newFirmwareVersion.substring(1).toFloat() > String(FIRMWARE_VERSION).substring(1).toFloat()) {
      Serial.println( "Preparing to update" );
      String new_firmware_URL = String(UPDATE_SERVER) + filename + newFirmwareVersion + ".bin";
      Serial.println(new_firmware_URL);
      t_httpUpdate_return ret = ESPhttpUpdate.update( new_firmware_URL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
         break;
      }
    }
    else {
      Serial.println("Already on latest firmware");  
    }
  }
  else {
    Serial.print("GET RC: ");
    Serial.println(httpCode);
  }
}

void my_delay(unsigned long ms) {
  uint32_t start = micros();

  while (ms > 0) {
    yield();
    while ( ms > 0 && (micros() - start) >= 1000) {
      ms--;
      start += 1000;
    }
  }
}

