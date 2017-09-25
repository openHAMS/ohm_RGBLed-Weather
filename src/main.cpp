#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <stdlib.h>
#include <ArduinoLog.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <Task.h>
#include "taskMqttConnect.hpp"
// include sensors
#include "taskBmp180.hpp"
#include "taskTsl2561.hpp"

//#include "Color.hpp"
//#include "RGBLed.hpp"


#define ALTITUDE   210  // Altitude via GPS - Miskolc, Diósgyőr

// LED pins
#define LED_RED     12
#define LED_GREEN   14
#define LED_BLUE    13

// Mqtt addresses
//     [home]/[room]/[passiveFunction]/[device]/[property]
//     [home]/[room]/[ activeFunction]/[ type ]/[location]
//         home/rcr/sensors/bmp180/pressure
//         home/rcr/sensors/bmp180/temperature
//         home/rcr/lights/rgbled/desk
char atmAddr    [] = "home/rcr/sensors/bmp180/pressure";
char tempAddr   [] = "home/rcr/sensors/bmp180/temperature";
char lightAddr  [] = "home/rcr/sensors/tsl2561/light";
char ledAddr    [] = "home/rcr/lights/rgbled/desk";

char deviceAddr [] = "status/freya";
char clientID   [] = "freya";

IPAddress mqttIP = IPAddress(192, 168, 1, 200);
uint16_t mqttPort = 1883;

AsyncMqttClient mqttClient;
//RGBLed led = RGBLed(LED_RED, LED_GREEN, LED_BLUE);

TaskManager taskManager;
// Callbacks
void sendData(long a, float t);
void sendMqttMessage(const char* address, char* message);
// Tasks
TaskReadWeather taskReadWeather(atmAddr, tempAddr, sendMqttMessage, ALTITUDE, MsToTaskTime(1000));
TaskReadLight taskReadLight(lightAddr, sendMqttMessage, MsToTaskTime(1000));
TaskMqttConnect taskMqttConnect(&mqttClient, MsToTaskTime(2500));




void sendMqttMessage(const char* address, char* message)
{
    if (mqttClient.connected())
    {
        mqttClient.publish(address, 1, true, message);
        Log.trace("[MQTT] Sending: %s --> %s", address, message);
    }
}

void onMqttConnect(bool sessionPresent)
{
    Log.notice("[MQTT] Connected!");
    uint16_t packetIdSub = mqttClient.subscribe(ledAddr, 1);
    Log.verbose("[MQTT] Subscribing at QoS 1, packetId: %d", packetIdSub);
    mqttClient.publish(deviceAddr, 1, true, "1");
    taskManager.StopTask(&taskMqttConnect);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Log.warning("[MQTT] Disconnected...");
    taskManager.StartTask(&taskMqttConnect);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Log.notice("[MQTT] Subscribed.");
    Log.trace ("       packetId: %d, qos: %d", packetId, qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Log.notice("[MQTT] Unsubscribed.");
    Log.trace ("       packetId: %d", packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
    /*
    #if DEBUG == 1
        Serial.println("[MQTT] Message received");
        Serial.print("  payload: ");
        Serial.println(payload);
        Serial.print("  color: ");
        Color color = ColorParse(payload);
        Serial.print(color.R);
        Serial.print(":");
        Serial.print(color.G);
        Serial.print(":");
        Serial.print(color.B);
        Serial.print(":");
        Serial.println(color.A);
        Serial.print("  topic: ");
        Serial.println(topic);
        Serial.print("  qos: ");
        Serial.println(properties.qos);
        Serial.print("  dup: ");
        Serial.println(properties.dup);
        Serial.print("  retain: ");
        Serial.println(properties.retain);
        Serial.print("  len: ");
        Serial.println(len);
        Serial.print("  index: ");
        Serial.println(index);
        Serial.print("  total: ");
        Serial.println(total);
    #endif
    if (topic == ledAddr) {
        led.setColor(ColorParse(payload));
    }
    */
}

void onMqttPublish(uint16_t packetId)
{
    Log.verbose("[MQTT] Message Sent, packetId: ", packetId);
}

void setup()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Log.begin(LOG_LEVEL_NOTICE, &Serial);

    Log.notice("[WiFi] Connecting...");
    WiFiManager wifiManager;
    wifiManager.autoConnect();

    Log.notice("[WiFi] Connected!");

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(mqttIP, mqttPort);
    mqttClient.setKeepAlive(10).setWill(deviceAddr, 1, true, "0", 0).setClientId(clientID);

    taskManager.Setup();
    taskManager.StartTask(&taskMqttConnect);
    taskManager.StartTask(&taskReadWeather);
    taskManager.StartTask(&taskReadLight);
}

void loop()
{
    taskManager.Loop();
}
