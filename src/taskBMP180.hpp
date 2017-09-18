#ifndef TASKBMP180_HPP
#define TASKBMP180_HPP

#include <Task.h>
#include <Wire.h>
#include <Sodaq_BMP085.h>
#include <ExpFilter.h>

class TaskReadWeather : public Task
{
public:
    typedef void(*action)(long a, float t);
    TaskReadWeather(action function, uint16_t alt, uint32_t timeInterval);

private:
    const action callback;
    const uint16_t ALTITUDE;
    bool filterInited = false;
    Sodaq_BMP085 sensor;
    ExpFilter<long> atmFilter;
    ExpFilter<float> tempFilter;

    virtual bool OnStart();
    virtual void OnStop();
    virtual void OnUpdate(uint32_t deltaTime);
};

#endif /* TASKBMP180_HPP */
