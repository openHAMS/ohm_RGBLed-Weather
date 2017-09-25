#include <Arduino.h>
#include "taskTsl2561.hpp"
#include <ExpFilter.h>
#include <ArduinoLog.h>

TaskReadLight::TaskReadLight(char* addr, action function, uint32_t timeInterval):
    Task(timeInterval),
    ADDRESS(addr),
    lightFilter(25, 75),
    callback(function)
{ }

bool TaskReadLight::OnStart()
{
    if(!sensor.begin())
    {
        Log.error("[TSL2561] ERROR sensor not found");
        return false;
    }
    // setup config
    sensor.enableAutoRange(true);
    sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
    // read initial value
    sensors_event_t event;
    sensor.getEvent(&event);
    // set initial filter value
    lightFilter.SetValue(event.light);
    Log.verbose("[TSL2561] %D:%D", event.light, lightFilter.Current());
    // start successful
    Log.notice("[TSL2561] task started...");
    return true;
}

void TaskReadLight::OnStop()
{
    Log.notice("[TSL2561] task stopped.");
}

void TaskReadLight::OnUpdate(uint32_t deltaTime)
{
    // read sensor
    sensors_event_t event;
    sensor.getEvent(&event);
    // if reading ok, else it is oversaturated
    if (event.light)
    {
        lightFilter.Filter(event.light);
    }
    else
    {
        lightFilter.SetValue(event.light);
    }
    Log.verbose("[TSL2561] %D:%D", event.light, lightFilter.Current());
    // convert to string
    char light[8];
    String(lightFilter.Current()).toCharArray(light, sizeof(light));
    // send data
    callback(ADDRESS, light);
}
