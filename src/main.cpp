#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <stdlib.h>

#include <Task.h>
#include <Sodaq_BMP085.h>
#include "taskBMP180.hpp"

#include "Color.hpp"
#include "RGBLed.hpp"

#define DEBUG		0

#define WIFI_SSID	"<SSID>"
#define WIFI_PASS	"<PASS>"

#define ALTITUDE	210 // Altitude via GPS - Miskolc, Diósgyőr

#define LED_RED		12
#define LED_GREEN	14
#define LED_BLUE	13


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
}


void onMqttConnect()
{
	Serial.println("** Connected to the broker **");
//TODO: [home]/[room]/[device]/[function]/[node]/[property]
	// [home]/[room]/[device]/[node]/[property]
	uint16_t packetIdSub = mqttClient.subscribe("rcr/rcr/desk/rgbled/set", 1);
	Serial.print("Subscribing at QoS 1, packetId: ");
	Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
	Serial.println("** Disconnected from the broker **");
	Serial.println("Reconnecting to MQTT...");
	mqttClient.connect();
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
	Serial.println("** Subscribe acknowledged **");
	Serial.print("  packetId: ");
	Serial.println(packetId);
	Serial.print("  qos: ");
	Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
	Serial.println("** Unsubscribe acknowledged **");
	Serial.print("  packetId: ");
	Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
	led.setColor(ColorParse(payload));
	mqttClient.publish("rcr/rcr/desk/rgbled/status", 1, true, payload);
}

void onMqttPublish(uint16_t packetId)
{
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
