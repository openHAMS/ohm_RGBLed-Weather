#include <Arduino.h>
#include "taskBmp180.hpp"
#include <ExpFilter.h>

TaskReadWeather::TaskReadWeather(action function, uint16_t alt, uint32_t timeInterval):
    Task(timeInterval),
    ALTITUDE(alt),
    atmFilter(100000L, 10),
    tempFilter(25, 10),
    callback(function)
{
    sensor.begin();
}

bool TaskReadWeather::OnStart()
{
    #if DEBUG == 1
        counter = 0;
        Serial.println("[SENSOR] task started...");
    #endif
    return true;
}

void TaskReadWeather::OnStop()
{
    #if DEBUG == 1
        Serial.println("[SENSOR] task stopped.");
    #endif
}

void TaskReadWeather::OnUpdate(uint32_t deltaTime)
{
    long ar = sensor.readPressure(ALTITUDE);
    float tr = sensor.readTemperature();
    long a;
    float t;
    if (filterInited)
    {
        a = atmFilter.Filter(ar);
        t = tempFilter.Filter(tr);
    }
    else
    {
        a = atmFilter.SetValue(ar);
        t = tempFilter.SetValue(tr);
        filterInited = true;
    }

    #if DEBUG == 1
        Serial.print("[SENSOR] update ");
        Serial.print(counter);
        Serial.println();
        counter++;
        Serial.print(ar);
        Serial.print(":");
        Serial.print(a);
        Serial.print(" ");
        Serial.print(tr);
        Serial.print(":");
        Serial.print(t);
        Serial.println(),
    #endif

    callback(a, t);
}
