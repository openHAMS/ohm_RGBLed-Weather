#include <Arduino.h>
#include <Task.h>
#include <AsyncMqttClient.h>
#include <ArduinoLog.h>


class TaskMqttConnect : public Task
{
public:
    TaskMqttConnect(AsyncMqttClient* mqttClient, uint32_t timeInterval):
        Task(timeInterval),
        _mqttClient(mqttClient)
    { }

private:
    AsyncMqttClient* _mqttClient;

    virtual bool OnStart()
    {
        Log.trace("[MQTT] Connecting task initiated.");
        return true;
    }

    virtual void OnStop()
    {
        Log.verbose("[MQTT] Connecting task stopped");
    }

    virtual void OnUpdate(uint32_t deltaTime)
    {
        Log.verbose("[MQTT] Connecting...");
        _mqttClient->connect();
    }
};
