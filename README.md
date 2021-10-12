# MotionXi platformio project

## Description

**MotionXi** is a motion capture device based on the M5StickC microcontroller. It sends euler angle data via Wifi / OSC to a host computer.
The **Platformio** project includes a wifi manager which allows you to define the wifi network used as well as the ip address and the port of the host computer.
All used libraries are static included in the lib directory. Simple clone repository to start...


|  |   |   |
|:---:|:---:| :---:|
| ![alt text](https://cdn.platformio.org/images/platformio-logo-xs.fd6e881d.png "Platformio Logo ")  |  | **[Platformio](https://www.platformio.org "Platformio")**  |
| ![alt text](https://m5stack.oss-cn-shenzhen.aliyuncs.com/image/icon/LOGO.jpg "M5Stack Logo Logo ")     | |   **[M5Stack](https://m5stack.com/ "M5Stack")** |
|  | |    |

**Arduino IDE** users rename MotionX1.cpp to MotionXi.ino and finde name + version of the used libraries in the description.
#include the header file(s) for the static library using quotation marks. In a typical solution, the path starts with ../<library project name>.
Make sure that ESP32 boards are installed in the boardsmanager, use **ESP32 pico kit** as selected board.

### Hardeware
In the case folder you will find .stl files for a case conversion which has space for a 350mAh battery and a switch.

* **[350mAh 3.7V LIPO](https://www.berrybase.de/strom/batterien-akkus/industrieakkus/lp-552035-lithium-polymer/lipo-akku-3-7v-350mah-mit-2-pin-jst-stecker?c=363 "berrybase.de")**
* **[BB-SWITCH-3P](https://www.berrybase.de/bauelemente/schalter-taster/mikroschalter-taster/3-poliger-schiebeschalter-f-252-r-breadboards "berrybase.de")**
* **[M5StickC ENV Hat (DHT12, BMP280, BMM150)](https://shop.m5stack.com/products/m5stickc-env-hat?_pos=3&_sid=d5a80659b&_ss=r&variant=17266544214106)** <--- NEW in this version
magnetic Sensor BMM150 for less drift

**Switching machine operation:** Press power switch for two seconds to turn it on, and press and hold for six seconds to turn it off.

* to enter **AP-Portal mode** keep **BUTTON A** pressed on start to make AP-Portal blocking
* **AP Portal IP-Adress:** 192.168.4.1
* to **reset device data** press and hold **BUTTON B** for 5 seconds

### AP-Portal
|   |   |   |
|:---|---:|---:|
|Host-IP  | ip Adress of the Host computer |
|send Port    | reciving Port of the Host System|
| recive Port | sending Port of the Host System|
|devID  | device ID |
|||

### OSC Topics
|   |   |   |
|:---|---:|---:|
|/_device ID_/roll  | roll angle |
|/_device ID_/pitch    | pitch angle|
| /_device ID_/yaw | yaw angle|
|||

### Button functions

* Button A changes the brightness of the display
* Button B inverts display color
* Button A long press - turns OFF/ON animation
* Button B long press resets device data

* Button A pressed 5 sec : Enter calibration Mode

follow the text on the display

first phase: hold the device steady for a few seconds

second phase: move, flip and rotate the device for a few seconds


Hardware based on **M5StickC**:
![alt text](https://m5stack.oss-cn-shenzhen.aliyuncs.com/image/m5-docs_content/core/m5stickc_01.png "M5StickC")


* **For the Detailed documentation of M5StickC, please click [here](https://docs.m5stack.com/#/en/core/m5stickc)**

* **In order to buy M5StickC, please click [here](https://m5stack-store.myshopify.com/collections/m5-core/products/stick-c)**


