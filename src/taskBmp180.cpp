#include <Arduino.h>
#include "taskBmp180.hpp"
#include <ExpFilter.h>
#include <ArduinoLog.h>

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
    Log.notice("[SENSOR] task started...");
    return true;
}

void TaskReadWeather::OnStop()
{
    Log.notice("[SENSOR] task stopped.");
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

    Log.verbose("[BMP180] atm: %l:%l temp: %D:%D ", ar, a, tr, t);
    callback(a, t);
}
