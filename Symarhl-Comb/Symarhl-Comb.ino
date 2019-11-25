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
#define STUDIO_PIN D5
#define BATHROOM_PIN D6
#define KIDSROOM_PIN D7
#define BEDROOM_PIN D8

//RELAY STATES
#define ON LOW
#define OFF HIGH

// VARIABLES
float interval = 20000;
float studioTemp = 21;
float bathroomTemp = 21;
float kidsroomTemp = 21;
float bedroomTemp = 21;
float hysteresis = 0.3;
// PRIVATE
long previousTime = 0;
// ONEWIRE TEMP SENSORS IDS
const uint8_t STUDIO_ID[8] = {0x28, 0xFF, 0x76, 0x65, 0x71, 0x15, 0x02, 0x43};
const uint8_t BATHROOM_ID[8] = {0x28, 0xFF, 0x91, 0xA3, 0x73, 0x15, 0x02, 0xB7};
const uint8_t KIDSROOM_ID[8] = {0x28, 0xFF, 0xD2, 0x57, 0x71, 0x15, 0x01, 0xC2};
const uint8_t BEDROOM_ID[8] = {0x28, 0xFF, 0x0D, 0x82, 0x73, 0x15, 0x02, 0xDE};

// NETWORK CREDENTIALS
const char* LAN_SSID = WIFI_SSID;
const char* LAN_PASSWORD = WIFI_PASSWORD;

// MQTT
const char MQTT_BROKER_IP[] = MQTT_IP;
const int MQTT_BROKER_PORT = MQTT_PORT;
// TOPICS
const char startTopic[] = "started";
// READ
const char studioTempTopic[] = "temp/studio";
const char bathroomTempTopic[] = "temp/bathroom";
const char kidsroomTempTopic[] = "temp/kidsroom";
const char bedroomTempTopic[] = "temp/bedroom";
const char powerTopic[] = "power";
const char variablesTopic[] = "variables";
// REQUEST
const char getVariablesTopic[] = "get/variables";
// SET
// COMMON
const char confirmTopic[] = "set/confirmation";
const char setHysteresisTopic[] = "set/hysteresis";
const char setIntervalTopic[] = "set/interval";
// ROOMS TEMP
const char setStudioTempTopic[] = "set/studio";
const char setBathroomTempTopic[] = "set/bathroom";
const char setKidsroomTempTopic[] = "set/kidsroom";
const char setBedroomTempTopic[] = "set/bedroom";

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

  if (WiFi.status() != WL_CONNECTED) {
    mqttConnect();
    sendMQTTMessage(startTopic, "pump-controller");
  }

}

void setupRelayPins() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(WALL_PIN, OUTPUT);
  pinMode (STUDIO_PIN, OUTPUT);
  pinMode (BATHROOM_PIN, OUTPUT);
  pinMode (KIDSROOM_PIN, OUTPUT);
  pinMode (BEDROOM_PIN, OUTPUT);

  digitalWrite(PUMP_PIN, OFF);
  digitalWrite(WALL_PIN, OFF);
  digitalWrite(STUDIO_PIN, OFF);
  digitalWrite(BATHROOM_PIN, OFF);
  digitalWrite(KIDSROOM_PIN, OFF);
  digitalWrite(BEDROOM_PIN, OFF);
}

void demo() {
  delay(2000);
  digitalWrite(PUMP_PIN, ON);
  digitalWrite(STUDIO_PIN, ON);
  delay(500);
  digitalWrite(STUDIO_PIN, OFF);
  digitalWrite(BATHROOM_PIN, ON);
  delay(500);
  digitalWrite(BATHROOM_PIN, OFF);
  digitalWrite(KIDSROOM_PIN, ON);
  delay(500);
  digitalWrite(KIDSROOM_PIN, OFF);
  digitalWrite(BEDROOM_PIN, ON);
  delay(500);
  digitalWrite(BEDROOM_PIN, OFF);
  digitalWrite(WALL_PIN, ON);
  delay(500);
  digitalWrite(PUMP_PIN, OFF);
  digitalWrite(WALL_PIN, OFF);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {
    previousTime = currentTime;
    heatingControl();
    verifyMqttConnection();
  }
  listenMqtt();
}

boolean areAllPinsOff() {
  return OFF == digitalRead(STUDIO_PIN) && OFF == digitalRead(BATHROOM_PIN) && OFF == digitalRead(KIDSROOM_PIN) && OFF == digitalRead(BEDROOM_PIN) && OFF == digitalRead(WALL_PIN);
}

void heatingControl() {
  oneWireSensors.requestTemperatures();

  controlRoomTemp(STUDIO_ID, studioTempTopic, STUDIO_PIN, studioTemp);
  controlRoomTemp(BATHROOM_ID, bathroomTempTopic, BATHROOM_PIN, bathroomTemp);
  controlRoomTemp(KIDSROOM_ID, kidsroomTempTopic, KIDSROOM_PIN, kidsroomTemp);
  controlRoomTemp(BEDROOM_ID, bedroomTempTopic, BEDROOM_PIN, bedroomTemp);


  if (areAllPinsOff()) {
    turnOff(PUMP_PIN);
  } else {
    turnOn(PUMP_PIN);
  }

  sendPowerMessage();
}

void controlRoomTemp(const uint8_t roomId[], const char outTopic[], uint8_t pin, float minTemp) {
  float roomTemp = oneWireSensors.getTempC(roomId);
  if (roomTemp < minTemp) {
    turnOn(pin);
  }

  if (roomTemp >= minTemp + hysteresis) {
    turnOff(pin);
  }
  sendMQTTMessage(outTopic, String(roomTemp));
}

void sendPowerMessage() {
  String powerMessage = "pump=" + String(isOn(PUMP_PIN)) + ";";
  powerMessage += "wall=" + String(isOn(WALL_PIN)) + ";";
  powerMessage += "studio=" + String(isOn(STUDIO_PIN)) + ";";
  powerMessage += "bathroom=" + String(isOn(BATHROOM_PIN)) + ";";
  powerMessage += "kidsroom=" + String(isOn(KIDSROOM_PIN)) + ";";
  powerMessage += "bedroom=" + String(isOn(BEDROOM_PIN));

  sendMQTTMessage(powerTopic, powerMessage);
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
    mqttClient.subscribe("get/#");
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

    if (isEqual(recievedTopic, setStudioTempTopic)) {
      studioTemp = payload.toFloat();
      sendMQTTMessage(confirmTopic, String(setStudioTempTopic) + "=" + String(studioTemp));
    }

    if (isEqual(recievedTopic, setBathroomTempTopic)) {
      bathroomTemp = payload.toFloat();
      sendMQTTMessage(confirmTopic, String(setBathroomTempTopic) + "=" + String(bathroomTemp));
    }

    if (isEqual(recievedTopic, setKidsroomTempTopic)) {
      kidsroomTemp = payload.toFloat();
      sendMQTTMessage(confirmTopic, String(setKidsroomTempTopic) + "=" + String(kidsroomTemp));
    }

    if (isEqual(recievedTopic, setBedroomTempTopic)) {
      bedroomTemp = payload.toFloat();
      sendMQTTMessage(confirmTopic, String(setBedroomTempTopic) + "=" + String(bedroomTemp));
    }

    if (isEqual(recievedTopic, setHysteresisTopic)) {
      hysteresis = payload.toFloat();
      sendMQTTMessage(confirmTopic, String(setHysteresisTopic) + "=" + String(hysteresis));
    }

    if (isEqual(recievedTopic, setIntervalTopic)) {
      interval = payload.toInt();
      sendMQTTMessage(confirmTopic, String(setIntervalTopic) + "=" + String(interval));
    }

    if (isEqual(recievedTopic, getVariablesTopic)) {
      String response = String(setStudioTempTopic) + "=" + String(studioTemp) + ";";
      response += String(setBathroomTempTopic) + "=" + String(bathroomTemp) + ";";
      response += String(setKidsroomTempTopic) + "=" + String(kidsroomTemp) + ";";
      response += String(setBedroomTempTopic) + "=" + String(bedroomTemp) + ";";
      response += String(setHysteresisTopic) + "=" + String(hysteresis) + ";";
      response += String(setIntervalTopic) + "=" + String(interval);

      sendMQTTMessage(variablesTopic, response);
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
