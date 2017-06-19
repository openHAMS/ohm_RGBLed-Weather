#include <Arduino.h>
#include <Task.h>

#include <AsyncMqttClient.h>

class TaskMqttConnect : public Task
{
public:
    TaskMqttConnect(AsyncMqttClient* mqttClient, uint32_t timeInterval):
        Task(timeInterval)
    {
        _mqttClient = mqttClient;
    }

private:
    AsyncMqttClient* _mqttClient;
    int counter = 0;

    virtual bool OnStart()
    {
        counter = 0;
        #if DEBUG == 1
            Serial.println("[MQTT-RE] task started...");
        #endif
    }

    virtual void OnStop()
    {
        #if DEBUG == 1
            Serial.println("[MQTT-RE] task stopped.");
        #endif
    }

    virtual void OnUpdate(uint32_t deltaTime)
    {
        Serial.println("[MQTT] Connecting... (" + String(counter) + ")");
        counter++;
        _mqttClient->connect();
    }
};
