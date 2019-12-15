#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>
#include "config.h"

// PINOUT
// DS18B20
#define TS_PIN D4
// RELAYS
#define PUMP_PIN D1
#define WALL_PIN D2

//RELAY STATES
#define ON LOW
#define OFF HIGH


// INDEXES
const uint8_t STUDIO = 0;
const uint8_t BATHROOM = 1;
const uint8_t KIDSROOM = 2;
const uint8_t BEDROOM = 3;

const uint8_t ROOM_PINS[4] = {
  D5,           // STUDIO_PIN,
  D6,           // BATHROOM_PIN,
  D7,           // KIDSROOM_PIN,
  D8            // BEDROOM_PIN
};

const char roomNames[4][14] = {
  "studio",
  "bathroom",
  "kidsroom",
  "bedroom"
};

// VARIABLES
float interval = 20000;
float hysteresis = 0.3;

float desiredTemps[4] = {
  21.5, // STUDIO
  21.5, // BATHROOM
  21.5, // KIDSROOM
  21.5, // BEDROOM
};

// PRIVATE
long previousTime = 0;
// ONEWIRE TEMP SENSORS IDS

const uint8_t (SENSOR_IDS[4])[8] = {
  {0x28, 0xFF, 0x76, 0x65, 0x71, 0x15, 0x02, 0x43},  // STUDIO
  {0x28, 0xFF, 0x91, 0xA3, 0x73, 0x15, 0x02, 0xB7},  // BATHROOM
  {0x28, 0xFF, 0xD2, 0x57, 0x71, 0x15, 0x01, 0xC2},  // KIDSROOM
  {0x28, 0xFF, 0x0D, 0x82, 0x73, 0x15, 0x02, 0xDE}   // BEDROOM
};


// NETWORK CREDENTIALS
const char* LAN_SSID = WIFI_SSID;
const char* LAN_PASSWORD = WIFI_PASSWORD;

// MQTT
const char MQTT_BROKER_IP[] = MQTT_IP;
const int MQTT_BROKER_PORT = MQTT_PORT;
// TOPICS
const char startTopic[] = "started";
// READ
const char roomTempTopics[4][14] = {
  "temp/studio",
  "temp/bathroom",
  "temp/kidsroom",
  "temp/bedroom"
};

const char powerTopic[] = "power";
const char variablesTopic[] = "variables";
// SET
const char theWallTopic[] = "thewall";
// COMMON
const char confirmTopic[] = "set/confirmation";
const char setHysteresisTopic[] = "set/hysteresis";
const char setIntervalTopic[] = "set/interval";
// ROOMS TEMP
const char setRoomTempTopics[4][14] = {
  "set/studio",
  "set/bathroom",
  "set/kidsroom",
  "set/bedroom"
};

OneWire oneWire(TS_PIN);
DallasTemperature oneWireSensors(&oneWire);

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println();

  setupRelayPins();
  demo();
  oneWireSensors.begin();

  WiFi.begin(LAN_SSID, LAN_PASSWORD);
  int attempts = 1;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000 * attempts);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    mqttConnect();
    sendMQTTMessage(startTopic, "pump-controller");
  }
}

void setupRelayPins() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(WALL_PIN, OUTPUT);
  pinMode (ROOM_PINS[STUDIO], OUTPUT);
  pinMode (ROOM_PINS[BATHROOM], OUTPUT);
  pinMode (ROOM_PINS[KIDSROOM], OUTPUT);
  pinMode (ROOM_PINS[BEDROOM], OUTPUT);

  digitalWrite(PUMP_PIN, OFF);
  digitalWrite(WALL_PIN, OFF);
  digitalWrite(ROOM_PINS[STUDIO], OFF);
  digitalWrite(ROOM_PINS[BATHROOM], OFF);
  digitalWrite(ROOM_PINS[KIDSROOM], OFF);
  digitalWrite(ROOM_PINS[BEDROOM], OFF);
}

void demo() {
  delay(2000);
  digitalWrite(PUMP_PIN, ON);
  digitalWrite(ROOM_PINS[STUDIO], ON);
  delay(500);
  digitalWrite(ROOM_PINS[STUDIO], OFF);
  digitalWrite(ROOM_PINS[BATHROOM], ON);
  delay(500);
  digitalWrite(ROOM_PINS[BATHROOM], OFF);
  digitalWrite(ROOM_PINS[KIDSROOM], ON);
  delay(500);
  digitalWrite(ROOM_PINS[KIDSROOM], OFF);
  digitalWrite(ROOM_PINS[BEDROOM], ON);
  delay(500);
  digitalWrite(ROOM_PINS[BEDROOM], OFF);
  digitalWrite(WALL_PIN, ON);
  delay(500);
  digitalWrite(PUMP_PIN, OFF);
  digitalWrite(WALL_PIN, OFF);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {
    previousTime = currentTime;

    verifyMqttConnection();
    heatingControl();
    sendPowerMessage();
    sendVariablesMessage();
  }
  listenMqtt();
}

boolean areAllPinsOff() {
  boolean areAllRoomsOff = true;
  for (uint8_t i = 0; i < 4; i++) {
    areAllRoomsOff = areAllRoomsOff && digitalRead(ROOM_PINS[i]) == OFF;
  }
  return areAllRoomsOff && OFF == digitalRead(WALL_PIN);
}

void heatingControl() {
  oneWireSensors.requestTemperatures();

  float currentRoomTemps[4];
  for (uint8_t i = 0; i < 4; i++) {
    currentRoomTemps[i] = oneWireSensors.getTempC(SENSOR_IDS[i]);
    controlRoomTemp(currentRoomTemps[i], i);
  }

  sendCurrentTemperatures(currentRoomTemps);

  if (areAllPinsOff()) {
    turnOff(PUMP_PIN);
  } else {
    turnOn(PUMP_PIN);
  }
}

void sendCurrentTemperatures(float currentRoomTemps[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    sendMQTTMessage(roomTempTopics[i], String(currentRoomTemps[i]));
  }
}

void controlRoomTemp(float roomTemp, uint8_t i) {
  if (roomTemp < desiredTemps[i]) {
    turnOn(ROOM_PINS[i]);
  }

  if (roomTemp >= desiredTemps[i] + hysteresis) {
    turnOff(ROOM_PINS[i]);
  }
}

void sendPowerMessage() {
  String powerMessage = "pump=" + String(isOn(PUMP_PIN)) + ";";
  powerMessage += "wall=" + String(isOn(WALL_PIN)) + ";";
  for (uint8_t i = 0; i < 4; i++) {
    powerMessage += String(roomNames[i]) + "=" + String(isOn(ROOM_PINS[i])) + ";";
  }
  sendMQTTMessage(powerTopic, powerMessage);
}

void sendVariablesMessage() {
  String response = "interval=" + String(interval) + ";";
  response += "hysteresis=" + String(hysteresis) + ";";
  for (uint8_t i = 0; i < 4; i++) {
    response += String(roomNames[i]) + "=" + String(desiredTemps[i]) + ";";
  }
  sendMQTTMessage(variablesTopic, response);
}

boolean isOn(uint8_t pin) {
  return digitalRead(pin) == ON;
}

void turnOn(uint8_t pin) {
  if (!isOn(pin)) {
    digitalWrite(pin, ON);
  }
}

void turnOff(uint8_t pin) {
  if (isOn(pin)) {
    digitalWrite(pin, OFF);
  }
}

void mqttConnect() {
  if (!mqttClient.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  } else {
    mqttClient.subscribe("set/#");
    mqttClient.subscribe("thewall");
  }
}

void listenMqtt() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    int srtLength = mqttClient.messageTopic().length() + 1;
    char recievedTopic[srtLength];
    mqttClient.messageTopic().toCharArray(recievedTopic, srtLength);

    String payload = "";
    while (mqttClient.available()) {
      payload += (char)mqttClient.read();
    }

    for (uint8_t i = 0; i < 4; i++) {
      if (isEqual(recievedTopic, setRoomTempTopics[i])) {
        desiredTemps[i] = payload.toFloat();
        sendMQTTMessage(confirmTopic, String(roomNames[i]) + "=" + String(desiredTemps[i]));
      }
    }

    if (isEqual(recievedTopic, setHysteresisTopic)) {
      hysteresis = payload.toFloat();
      sendMQTTMessage(confirmTopic, "hysteresis=" + String(hysteresis));
    }

    if (isEqual(recievedTopic, setIntervalTopic)) {
      interval = payload.toInt();
      sendMQTTMessage(confirmTopic, "interval=" + String(interval));
    }

    if (isEqual(recievedTopic, theWallTopic)) {
      if (payload == "on") {
        turnOn(WALL_PIN);
      } else {
        turnOff(WALL_PIN);
      }
    }
  }
}

void verifyMqttConnection() {
  if (!mqttClient.connected()) {
    mqttConnect();
  }
}

boolean isEqual(char recievedTopic[], const char topic[]) {
  return strcmp(topic, recievedTopic) == 0;
}

void sendMQTTMessage(const char outTopic[], String payload) {
  if (mqttClient.connected()) {
    mqttClient.beginMessage(outTopic, payload.length(), false, 1, false);
    mqttClient.print(payload);
    mqttClient.endMessage();
  } else {
    Serial.println("disconnected from MQTT broker :(");
  }
}
