#include <Arduino.h>
#include <Task.h>
#include <Wire.h>
#include <Sodaq_BMP085.h>

class TaskReadWeather : public Task
{
public:
    typedef void(*action)(long a, float t);
    TaskReadWeather(action function, unsigned int alt, uint32_t timeInterval):
        Task(timeInterval),
        _ALTITUDE(alt),
        _callback(function)
    {
        _sensor.begin();
    }

private:
    const action _callback;
    const unsigned int _ALTITUDE;
    Sodaq_BMP085 _sensor;

    int counter;

    virtual bool OnStart()
    {
        #if DEBUG == 1
            Serial.println("[SENSOR] task started...");
        #endif
    }

    virtual void OnStop()
    {
        #if DEBUG == 1
            Serial.println("[SENSOR] task stopped.");
        #endif
    }

    virtual void OnUpdate(uint32_t deltaTime)
    {
        #if DEBUG == 1
            Serial.print("[SENSOR] update ");
            Serial.print(counter);
            Serial.println();
        #endif
        counter++;
        float t = _sensor.readTemperature();
        long a = _sensor.readPressure(_ALTITUDE);
        _callback(a, t);
    }
};
