#ifndef TASKBMP180_HPP
#define TASKBMP180_HPP

#include <Task.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP180_U.h>
#include <ExpFilter.h>

class TaskReadWeather : public Task
{
public:
    typedef void(*action)(const char* address, char* message);
    TaskReadWeather(char* atmAddr, char* tempAddr, action function, uint16_t alt, uint32_t timeInterval);

private:
    const action callback;
    const char* ATM_ADDRESS;
    const char* TEMP_ADDRESS;
    const uint16_t ALTITUDE;
    Adafruit_BMP180_Unified sensor = Adafruit_BMP180_Unified(10085);
    ExpFilter<float> atmFilter;
    ExpFilter<float> tempFilter;

    virtual bool OnStart();
    virtual void OnStop();
    virtual void OnUpdate(uint32_t deltaTime);
};

#endif /* TASKBMP180_HPP */
