#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <stdlib.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <Task.h>
#include "taskBmp180.hpp"
#include "taskMqttConnect.hpp"

#include "Color.hpp"
#include "RGBLed.hpp"


#define DEBUG        0

#define ALTITUDE   210  // Altitude via GPS - Miskolc, Diósgyőr

// LED pins
#define LED_RED     12
#define LED_GREEN   14
#define LED_BLUE    13

// [home]/[room]/[passiveFunction]/[device]/[property]
// [home]/[room]/[ activeFunction]/[ type ]/[location]
//      home/rcr/sensors/bmp180/pressure
//      home/rcr/sensors/bmp180/temperature
//      home/rcr/lights/rgbled/desk

// String room = "home/rcr";
// String location = "/desk";
// char* asd;
// String rgbledAddrset     = room + "/lights/rgbled" + location + "/set";
// String rgbledAddrstatus  = room + "/lights/rgbled" + location + "/status";
// String bmp180MQTTatm     = room + "/sensors" + "/bmp180" + "/pressure";
// String bmp180MQTTtemp    = room + "/sensors" + "/bmp180" + "/temperature";

//mqttConfig qweqwe = mqttConfig("home/rcr", "/desk");

AsyncMqttClient mqttClient;
RGBLed led = RGBLed(LED_RED, LED_GREEN, LED_BLUE);

TaskManager taskManager;
// Callbacks
void sendData(long a, float t);
// Tasks
TaskReadWeather taskReadWeather(sendData, ALTITUDE, MsToTaskTime(1000));
TaskMqttConnect taskMqttConnect(&mqttClient, MsToTaskTime(2500));

// Mqtt addresses
char atmAddr    [] = "home/rcr/sensors/bmp180/pressure";
char tempAddr   [] = "home/rcr/sensors/bmp180/temperature";
char ledAddr    [] = "home/rcr/lights/rgbled/desk";

char deviceAddr [] = "status/huzzah1";
char clientID   [] = "huzzah1";
IPAddress mqttIP = IPAddress(192, 168, 1, 200);
uint16_t mqttPort = 1883;

void sendData(long a, float t)
{
    char atm [8];
    char temp[8];
    String(a / 100.0).toCharArray(atm, sizeof(atm));
    String(t).toCharArray(temp, sizeof(temp));
    mqttClient.publish(atmAddr, 1, true, atm);
    mqttClient.publish(tempAddr, 1, true, temp);
    #if DEBUG == 1
        Serial.print("[DEBUG]->sendData: atm: ");
        Serial.print(atm);
        Serial.print(", temp: ");
        Serial.print(temp);
        Serial.println();
    #endif
}


void onMqttConnect(bool sessionPresent)
{
    taskManager.StopTask(&taskMqttConnect);

    Serial.println("[MQTT] Connected!");
    uint16_t packetIdSub = mqttClient.subscribe(ledAddr, 1);
    #if DEBUG == 1
        Serial.print("  [DEBUG] Subscribing at QoS 1, packetId: ");
        Serial.print(packetIdSub);
        Serial.println();
    #endif
    mqttClient.publish(deviceAddr, 1, true, "1");

    taskManager.StartTask(&taskReadWeather);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("[MQTT] Disconnected...");
    taskManager.StartTask(&taskMqttConnect);
    taskManager.StopTask(&taskReadWeather);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("[MQTT] Subscribed.");
    #if DEBUG == 1
        Serial.print("  [DEBUG] packetId: " );
        Serial.print(packetId);
        Serial.print(", qos: ");
        Serial.print(qos);
        Serial.println();
    #endif
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Serial.println("[MQTT] Unsubscribed.");
    #if DEBUG == 1
        Serial.println("  [DEBUG] packetId: ");
        Serial.print(packetId);
        Serial.println();
    #endif
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
    #if DEBUG == 1
        Serial.print("  [MQTT] Message Sent, packetId: ");
        Serial.print(packetId);
        Serial.println();
    #endif
}

void setup()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    Serial.begin(115200);
    Serial.println();
    Serial.println();

    Serial.println("[WiFi] Connecting...");
    WiFiManager wifiManager;
    wifiManager.autoConnect();

    Serial.println("[WiFi] Connected!");

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
}

void loop()
{
    taskManager.Loop();
}
