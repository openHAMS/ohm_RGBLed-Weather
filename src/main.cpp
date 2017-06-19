#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <stdlib.h>

#include <Task.h>
#include <Sodaq_BMP085.h>
#include "taskBmp180.hpp"

#include "Color.hpp"
#include "RGBLed.hpp"


#define WIFI_SSID	"<SSID>"
#define WIFI_PASS	"<PASS>"
#define DEBUG        0

#define ALTITUDE   210 // Altitude via GPS - Miskolc, Diósgyőr

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
void sendTemp(long a, float t);
TaskReadWeather taskReadWeather(sendTemp, ALTITUDE, MsToTaskTime(1000));

char temp[6];
char atm[6];

void sendTemp(long a, float t)
{
	String(a).toCharArray(atm, sizeof(atm));
	String(t).toCharArray(temp, sizeof(temp));
	mqttClient.publish("rcr/rcr/desk/sensors/bmp180/pressure"   , 1, true, atm );
	mqttClient.publish("rcr/rcr/desk/sensors/bmp180/temperature", 1, true, temp);
    #if DEBUG == 1
        Serial.println("[DEBUG]->sendData: atm: " + String(atm) + ", temp: " + String(temp));
    #endif
}


void onMqttConnect()
{
	uint16_t packetIdSub = mqttClient.subscribe("rcr/rcr/desk/rgbled/set", 1);
    Serial.println("[MQTT] Connected!");
    #if DEBUG == 1
        Serial.println("  [DEBUG] Subscribing at QoS 1, packetId: ");
        Serial.println(packetIdSub);
    #endif
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
	Serial.println("** Disconnected from the broker **");
	Serial.println("Reconnecting to MQTT...");
	mqttClient.connect();
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("[MQTT] Subscribed.");
    #if DEBUG == 1
        Serial.println("  [DEBUG] packetId: " + packetId + ", qos: " + qos);
    #endif
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Serial.println("[MQTT] Unsubscribed.");
    #if DEBUG == 1
        Serial.println("  [DEBUG] packetId: " + packetId);
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
        Serial.println("[MQTT] Message Sent, packetId: " + packetId);
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
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	Serial.print("Connecting to Wi-Fi");
	WiFi.begin(WIFI_SSID, WIFI_PASS);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}

	Serial.println(" OK");

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.onSubscribe(onMqttSubscribe);
	mqttClient.onUnsubscribe(onMqttUnsubscribe);
	mqttClient.onMessage(onMqttMessage);
	mqttClient.onPublish(onMqttPublish);
	mqttClient.setServer(IPAddress(192, 168, 1, 111), 1883);
	mqttClient.setKeepAlive(5).setWill("topic/online", 2, true, "no").setCredentials("username", "password").setClientId("myDevice");
	Serial.print("Connecting to MQTT...");
	mqttClient.connect();
	Serial.println("Connected!");
	taskManager.StartTask(&taskReadWeather);
}

void loop()
{
	taskManager.Loop();
}
