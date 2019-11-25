#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>
#include "config.h"

// PINOUT
// DS18B20
#define TS_PIN D4
// RELAYS

// VARIABLES
float interval = 20000;
// PRIVATE
long previousTime = 0;

// NETWORK CREDENTIALS
const char* LAN_SSID = WIFI_SSID;
const char* LAN_PASSWORD = WIFI_PASSWORD;

// MQTT
const char MQTT_BROKER_IP[] = MQTT_IP;
const int MQTT_BROKER_PORT = MQTT_PORT;
// TOPICS
const char startTopic[] = "started";
const char outdoorTempTopic[] = "temp/outdoor";
const char setIntervalTopic[] = "set/heater-interval";
const char confirmTopic[] = "set/heater-confirmation";

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

  oneWireSensors.begin();

  WiFi.begin(LAN_SSID, LAN_PASSWORD);
  int attempts = 1;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000 * attempts);
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    mqttConnect();
    sendMQTTMessage(startTopic, "heater-controller");
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {
    previousTime = currentTime;
    readOutdoorTemp();
    verifyMqttConnection();
  }
  listenMqtt();
}

void readOutdoorTemp() {
  oneWireSensors.requestTemperatures();
  float outdoorTemp = oneWireSensors.getTempCByIndex(0);
  sendMQTTMessage(outdoorTempTopic, String(outdoorTemp));
}

void mqttConnect() {
  if (!mqttClient.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  } else {
    mqttClient.subscribe("set/#");
    // mqttClient.subscribe("get/#");
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

    if (isEqual(recievedTopic, setIntervalTopic)) {
      interval = payload.toInt();
      sendMQTTMessage(confirmTopic, String(setIntervalTopic) + "=" + String(interval));
    }
  }
}
