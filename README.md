##Control an RC car with ESP8266 over WiFi

This repository contains code to control an RC car over WiFi, the work is based on [indrekots's esp8266 rc car controller](https://github.com/indrekots/esp8266-rc-car-controller). 

For building details,[here might have what you need](https://articles.oostore.com/ghost/#/editor/post/5dd6abd6eada5b0001ef3a87).

##Requirements - Hardware

* NodeMCU(ESP8266)
* Motor Driver Board L298N
* Optical Encoder Sensors
* Battery Pack
* RC Car Kit
* Cables

##Requirements - Software

* Python 3
* Arduino IDE

##Usage

Build the RC Car Kit.
Connect battery pack with L298N with (VCC-VCC/GND-GND)
Connect L298N with NodeMCU(5V-VCCIN/GND-GND/IN1-GPIO13/IN1-GPIO12/IN1-GPIO14/IN1-GPIO16)
Connect Optical Encoder Sensors with NodeMCU
Connect OLED Screen with NodeMCU
Open `esp8266-rc-car` with Arduino IDE, change WiFi SSID and password. 

##Change Points

```cpp
const char* ssid = "SSID"; //Enter your wifi network SSID
const char* password = "PASSWORD"; //Enter your wifi network password
```

##Remote Control UI
Open your terminal and run the python script with

```bash
python wifi_controller_gui.py --host=<IP address of the car>
```

Press W, A, S, D keys or drage the circle in the middle of UI to control your RC car.

!![Interface](https://github.com/angelamonster/esp8266-rc-car-controller/blob/master/resources/interface.png)
