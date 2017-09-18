#include <Arduino.h>
#include "taskTsl2561.hpp"
#include <ExpFilter.h>
#include <ArduinoLog.h>

TaskReadLight::TaskReadLight(char* addr, action function, uint32_t timeInterval):
    Task(timeInterval),
    address(addr),
    lightFilter(25, 10),
    callback(function)
{
    sensor.begin();
}

bool TaskReadLight::OnStart()
{
    if(!sensor.begin())
    {
        Log.error("[TSL2561] ERROR sensor not found");
        return false;
    }
    // setup config
    sensor.enableAutoRange(true);
    sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
    // read initial value
    sensors_event_t event;
    sensor.getEvent(&event);
    lightFilter.SetValue(event.light);
    // start successful
    return true;
}

void TaskReadLight::OnStop()
{

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
    Log.verbose("[TSL2561] %D:%D", event.light, lightFilter.Current());
    // convert to string
    char light[8];
    String(lightFilter.Current()).toCharArray(light, sizeof(light));
    // send data
    callback(address, light);
}
