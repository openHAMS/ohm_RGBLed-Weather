#include <Arduino.h>
#include <Task.h>
#include <Wire.h>
#include <Sodaq_BMP085.h>

class TaskReadWeather : public Task
{
	public:
		typedef void(*action)(long a, float t);
		TaskReadWeather(action function, unsigned int alt, uint32_t timeInterval) : // pass any custom arguments you need
			Task(timeInterval),
			altitude(alt),
			_callback(function)
		{ };

	private:
		// put member variables here that are scoped to this object
		const action _callback;
		const unsigned int altitude;
		Sodaq_BMP085 sensor;

		//bool ledOn;
		//const uint8_t ledPin = 5; // const means can't change other than in constructor

		virtual bool OnStart() // optional
		{
			/*
			// put code here that will be run when the task starts
			ledOn = false;
			pinMode(ledPin, OUTPUT);
			return true;
			*/
			sensor.begin();
		}

		virtual void OnStop() // optional
		{
			/*
			// put code here that will be run when the task stops
			ledOn = false;
			digitalWrite(ledPin, LOW);	// turn the LED off by making the voltage LOW
			*/
		}

		virtual void OnUpdate(uint32_t deltaTime)
		{
			/*
			if (ledOn)
			{
				digitalWrite(ledPin, LOW);	// turn the LED off by making the voltage LOW
			}
			else
			{
				digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
			}

			ledOn = !ledOn; // toggle led state
			*/
			float t = sensor.readTemperature();
			long a = sensor.readPressure(altitude);
			_callback(a, t);
		}
};
