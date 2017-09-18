#ifndef TASKTSL2561_HPP
#define TASKTSL2561_HPP

#include <Task.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ExpFilter.h>

class TaskReadLight : public Task
{
public:
    typedef void(*action)(const char* address, char* message);
    TaskReadLight(char* addr, action function, uint32_t timeInterval);

private:
    const action callback;
    //bool filterInited = false;
    const char* address;
    Adafruit_TSL2561_Unified sensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
    ExpFilter<float> lightFilter;

    virtual bool OnStart();
    virtual void OnStop();
    virtual void OnUpdate(uint32_t deltaTime);
};

#endif /* TASKTSL2561_HPP */
