# OHM: RGBLed + Weather
An ESP8266 RGB led controller and weather publisher module for openHAMS

##Dependencies
The following are required to use this:
- Adafruit HUZZAH Breakout or other compatible ESP8266
- Power supply
- RGB LED strip
- BMP085/BMP180
- 3 MOSFETs for each color
- openHAMS server
Circuits are available for Fritzing in the repo. 

##Features
- change LED color via `rgbled/set`
- confirmation of color change via `rgbled/status`

##Thanks
- [ESP8266 core for Arduino](https://github.com/esp8266/Arduino)
- [Async MQTT client for ESP8266](https://github.com/marvinroger/async-mqtt-client) by [marvinroger](https://github.com/marvinroger)
- [Task](https://github.com/Makuna/Task) by [Makuna](https://github.com/Makuna)
- [Sodaq's Arduino library for BMP085/BMP180](https://github.com/SodaqMoja/Sodaq_BMP085) by [SodaqMoja](https://github.com/SodaqMoja)
