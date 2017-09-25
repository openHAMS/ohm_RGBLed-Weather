#include <Arduino.h>
#include "taskBmp180.hpp"
#include <ExpFilter.h>
#include <ArduinoLog.h>

TaskReadWeather::TaskReadWeather(char* atmAddr, char* tempAddr, action function, uint16_t alt, uint32_t timeInterval):
    Task(timeInterval),
    ATM_ADDRESS(atmAddr),
    TEMP_ADDRESS(tempAddr),
    ALTITUDE(alt),
    atmFilter(1000, 5),
    tempFilter(25, 5),
    callback(function)
{ }

bool TaskReadWeather::OnStart()
{
    if(!sensor.begin())
    {
        Log.error("[BMP180] ERROR sensor not found");
        return false;
    }
    // read initial pressure
    sensors_event_t event;
    sensor.getEvent(&event);
    // set initial filter value
    atmFilter.SetValue(sensor.seaLevelForAltitude(ALTITUDE, event.pressure));
    // get temperature event
    sensor.getTempEvent(&event);
    tempFilter.SetValue(event.temperature);
    Log.verbose("[BMP180] atm: %D:%D temp: %D:%D ",
        sensor.seaLevelForAltitude(ALTITUDE, event.pressure), atmFilter.Current(),
        event.temperature, tempFilter.Current());

    // start successful
    Log.notice("[BMP180] task started...");
    return true;
}

void TaskReadWeather::OnStop()
{
    Log.notice("[BMP180] task stopped.");
}

void TaskReadWeather::OnUpdate(uint32_t deltaTime)
{
    // read sensor
    sensors_event_t event;
    // pressure
    sensor.getEvent(&event);
    if (event.pressure)
    {
        atmFilter.Filter(sensor.seaLevelForAltitude(ALTITUDE, event.pressure));
    }
    // temperature
    sensor.getTempEvent(&event);
    if (event.temperature)
    {
        tempFilter.Filter(event.temperature);
    }

    Log.verbose("[BMP180] atm: %D temp: %D ", atmFilter.Current(), tempFilter.Current());

    char atm[8];
    String(atmFilter.Current()).toCharArray(atm, sizeof(atm));
    char temp[8];
    String(tempFilter.Current()).toCharArray(temp, sizeof(temp));

    callback(ATM_ADDRESS, atm);
    callback(TEMP_ADDRESS, temp);
}
