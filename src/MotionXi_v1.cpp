/*--------------------------------------------------------------------------------
The MIT License (MIT)

Copyright (c) 2021 artdanion

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
--------------------------------------------------------------------------------*/

#include "SPIFFS.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <Wire.h>
#include <WiFiUdp.h>

#include <driver/rtc_io.h> // from ESP-IDF
#include <M5StickC.h>      // https://github.com/m5stack/M5StickC        version = 0.2.0
#include <WiFiManager.h>   // https://github.com/tzapu/WiFiManager       version = 2.0.3-alpha
#include <ArduinoJson.h>   // https://github.com/bblanchon/ArduinoJson   version = 6.17.3
#include <OSCBundle.h>     // https://github.com/CNMAT/OSC               version = 1.3.5
#include <OSCMessage.h>    // https://github.com/CNMAT/OSC               version = 1.3.5
#include <bmm150.h>        // bmm150 support added from  https://github.com/edumo/M5StickC_Imu-9-axis
#include <bmm150_defs.h>

WiFiUDP Udp; // UDP instance
WiFiManager wifiManager;

bmm150_mag_data value_offset;
BMM150 bmm = BMM150();

//define your default values here, if there are different values in config.json, they are overwritten.
char HostIP[20];
char sendPort[6];
char recivePort[6];
char devID[10];
char PortalName[20];

const int buttonPin = 0;
bool buttonState = 0;

//flag for saving data / erase data
bool shouldSaveConfig = true;

unsigned char ip[] = {0};
short ipAddress[4];
uint32_t chipId = 0;

float yaw = 0;
float roll = 0;
float pitch = 0;

float temp = 0;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float magX = 0;
float magY = 0;
float magZ = 0;

float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];
float magOffset[3];
float magmax[3];
float magmin[3];

uint8_t setup_flag = 1;
uint8_t action_flag = 1;

//Mahony fix updated_balac_pid
float power = 0;
float dt, preTime;
float loopfreq;
float myQ0 = 1.0, myQ1 = 0.0, myQ2 = 0.0, myQ3 = 0.0;

float myq0, myq1, myq2, myq3;                      // quaternion of sensor frame relative to auxiliary frame
float myintegralFBx, myintegralFBy, myintegralFBz; // integral error terms scaled by Ki
float invSampleFreq;
char anglesComputed;

boolean withMagnet = false;

#define WOM_ATTACH_ISR
volatile uint32_t g_wom_count = 0;
volatile uint32_t g_wom_last_millis = 0;

void IRAM_ATTR mpu6886_wake_on_motion_isr(void)
{
  g_wom_count++;
  g_wom_last_millis = millis();
}

void saveConfigCallback();
void loadParameters();
void saveParameters();
float getBatteryLevel(float voltage);
void draw_battery_percent();
void drawSend(char *HostIP, char *sendPort);
void cubeloop();
void vectRotXYZ(double angle, int axe);
void extractIpAddress(char *sourceString, short *ipAddress);
void myGetAhrsData(float samplefrequency);
void calibrate(uint32_t timeout);
void calibrate6886();
void calibrate_waiting(uint32_t timeout);
void applycalibration();
void initData();
void myMahonyAHRSupdateIMU9Axis(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *q0, float *q1, float *q2, float *q3, float samplefrequency);
void myMahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *q0, float *q1, float *q2, float *q3, float samplefrequency);

#define M_PI 3.141592653
#define grey 0x65DB
#define TFT_GREY 0x5AEB
#define WAKE_REASON_BUF_LEN 100
#define NSTARS 512

#define twoKpDef (2.0f * 0.6f)   // 2 * proportional gain
#define twoKiDef (2.0f * 0.006f) // 2 * integral gain

// screensaver options in seconds
#define SCSAVERTIME 30
#define DEEPSLEEPTIME 100

uint8_t sx[NSTARS] = {};
uint8_t sy[NSTARS] = {};
uint8_t sz[NSTARS] = {};

uint8_t za, zb, zc, zx;

// Fast 0-255 random number generator from http://eternityforest.com/Projects/rng.php:
inline uint8_t __attribute__((always_inline)) rng()
{
  zx++;
  za = (za ^ zc ^ zx);
  zb = (zb + za);
  zc = (zc + ((zb >> 1) ^ za));
  return zc;
}

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

int16_t ax, ay, az, gx, gy, gz;
double MMPI = 32768 * 3.141592653;
int bright[5] = {7, 8, 9, 10, 12};
static const float levels[] = {4.13, 4.06, 3.98, 3.92, 3.87, 3.82, 3.79, 3.77, 3.74, 3.68, 3.45, 3.00};
bool screenSaver = false;
bool returnScreen = false;

int framecount = 0;
int ispeed = 10000;
double bouncespeed = 100;
int brightnessLevel = 1;
int pressed = false;
bool inv = false;
bool animation = true;

long int timeLast = -100, period = 1;

// Overall scale and perspective distance
uint8_t sZ = 16, scale = 8, scaleMax = 12;

// screen center
uint8_t centerX = 24;
uint8_t centerY = 24;

// Initialize cube point arrays
double C1[] = {1, 1, 1};
double C2[] = {1, 1, -1};
double C3[] = {1, -1, 1};
double C4[] = {1, -1, -1};
double C5[] = {-1, 1, 1};
double C6[] = {-1, 1, -1};
double C7[] = {-1, -1, 1};
double C8[] = {-1, -1, -1};

// Initialize cube points coords
uint8_t P1[] = {0, 0};
uint8_t P2[] = {0, 0};
uint8_t P3[] = {0, 0};
uint8_t P4[] = {0, 0};
uint8_t P5[] = {0, 0};
uint8_t P6[] = {0, 0};
uint8_t P7[] = {0, 0};
uint8_t P8[] = {0, 0};

RTC_DATA_ATTR int bootCount = 0;

void setup()
{

  pinMode(39, INPUT_PULLUP);
  pinMode(37, INPUT_PULLUP);

  //wifiManager.resetSettings();
  //SPIFFS.format();

#ifdef WOM_ATTACH_ISR
  delay(100);
  pinMode(GPIO_NUM_35, INPUT);
  delay(100);
  // set up ISR to trigger on GPIO35
  attachInterrupt(GPIO_NUM_35, mpu6886_wake_on_motion_isr, FALLING);
#endif

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  M5.begin();

  Wire.begin(0, 26);
  Serial.begin(115200);

  M5.Mpu6886.Init(); // basic init
  //M5.Mpu6886.enableWakeOnMotion(M5.Mpu6886.AFS_16G, 10);

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM)
  {
    Serial.println("Chip ID can not read!");
    withMagnet = false;
  }
  else
  {
    Serial.println("Initialize done!");
    withMagnet = true;
  }

  Serial.print("\n\rCalibrate done..");
  initData();

  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  snprintf(PortalName, sizeof(PortalName), "MotionXi_%d", chipId);

  WiFi.setHostname(PortalName);
  Serial.println(PortalName);

  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setSwapBytes(true);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

  M5.Axp.EnableCoulombcounter();
  M5.Axp.ScreenBreath(bright[1]);

  za = random(256);
  zb = random(256);
  zc = random(256);
  zx = random(256);

  Serial.println("mounting FS...");
  SPIFFS.begin(true);

  loadParameters();

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_devID("devID", "Device ID", devID, 5);
  WiFiManagerParameter custom_HostIP("HostIP", "Host IP 192.168.6.42", HostIP, 20);
  WiFiManagerParameter custom_sendPort("sendPort", "Send Port 8888", sendPort, 6);
  WiFiManagerParameter custom_recivePort("recivePort", "Recive Port 9999", recivePort, 6);

  //add all your parameters here
  wifiManager.addParameter(&custom_devID);
  wifiManager.addParameter(&custom_HostIP);
  wifiManager.addParameter(&custom_sendPort);
  wifiManager.addParameter(&custom_recivePort);

  M5.update();

  wifiManager.setShowInfoErase(false);

  if (M5.BtnA.isPressed())
    wifiManager.setConfigPortalBlocking(true);
  else
    wifiManager.setConfigPortalBlocking(false);

  M5.Lcd.init();

  M5.Axp.ScreenBreath(10);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 6, 2);
  M5.Lcd.print("Connect to SSID:");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(3, 24, 2);
  M5.Lcd.print(PortalName);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 41, 2);
  M5.Lcd.print("Password: ");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.print("EnterThis");
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 58, 2);
  M5.Lcd.print("http://192.168.4.1");

  if (!wifiManager.autoConnect(PortalName, "EnterThis"))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...");

  //read updated parameters
  strcpy(devID, custom_devID.getValue());
  strcpy(HostIP, custom_HostIP.getValue());
  strcpy(sendPort, custom_sendPort.getValue());
  strcpy(recivePort, custom_recivePort.getValue());

  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    saveParameters();
  }

  Serial.println("local ip  ");
  Serial.print(WiFi.localIP());
  Serial.println();

  extractIpAddress(HostIP, &ipAddress[0]);

  const unsigned int localPort = atoi(recivePort); // local port to listen for OSC packets
  Udp.begin(localPort);

  Serial.println("Done");

  drawSend(HostIP, sendPort);
}

void loop()
{

  bool short_press, long_press;
  M5.Axp.GetPressIRQ(&short_press, &long_press);

  if (short_press || long_press)
  {
    M5.Axp.ClearPressIRQ(short_press, long_press);
  }

  IPAddress ip(WiFi.localIP());                                            // Device IP
  IPAddress outIp(ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]); // remote IP of the connected Computer
  unsigned int outPort = atoi(sendPort);                                   // Port send to

  OSCBundle bndl;

  char oscRoll[25] = "/";
  char oscPitch[25] = "/";
  char oscYaw[25] = "/";

  strcat(oscRoll, devID);
  strcat(oscRoll, "/roll");

  strcat(oscPitch, devID);
  strcat(oscPitch, "/pitch");

  strcat(oscYaw, devID);
  strcat(oscYaw, "/yaw");

  //M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);

  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  loopfreq = 1 / dt;

  myGetAhrsData(loopfreq);

  float pitch = asin(-2 * myQ1 * myQ3 + 2 * myQ0 * myQ2);                                                    // pitch
  float roll = atan2(2 * myQ2 * myQ3 + 2 * myQ0 * myQ1, -2 * myQ1 * myQ1 - 2 * myQ2 * myQ2 + 1);             // roll
  float yaw = atan2(2 * (myQ1 * myQ2 + myQ0 * myQ3), myQ0 * myQ0 + myQ1 * myQ1 - myQ2 * myQ2 - myQ3 * myQ3); //yaw

  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //  8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  yaw -= 4.36; // 4° 22' for Linz / Austria
  roll *= RAD_TO_DEG;

  bndl.add(oscRoll).add(roll);
  bndl.add(oscPitch).add(pitch);
  bndl.add(oscYaw).add(yaw);

  /* Quad send
  char oscqX[25] = "/";
  char oscqY[25] = "/";
  char oscqZ[25] = "/";
  char oscqW[25] = "/";

  strcat(oscqX, devID);
  strcat(oscqX, "/qX");

  strcat(oscqY, devID);
  strcat(oscqY, "/qY");

  strcat(oscqZ, devID);
  strcat(oscqZ, "/qZ");

  strcat(oscqW, devID);
  strcat(oscqW, "/qW");

  bndl.add(oscqX).add(myQ0);
  bndl.add(oscqY).add(myQ1);
  bndl.add(oscqZ).add(myQ2);
  bndl.add(oscqW).add(myQ3);
  */

  unsigned long t0 = micros();
  uint32_t since_last_wom_millis = millis() - g_wom_last_millis;
  uint8_t spawnDepthVariation = 255;

  Udp.beginPacket(outIp, outPort);
  bndl.send(Udp);  // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  bndl.empty();    // empty the bundle to free room for a new one

  //Serial.printf("%f    %f    %f \r\n", pitch, roll, yaw);

  if (since_last_wom_millis > SCSAVERTIME * 1000)
  {
    if (since_last_wom_millis > DEEPSLEEPTIME * 1000)
    {

      // disable all wakeup sources
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

      // enable waking up on pin 35 (from IMU)
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, LOW);

      //Go to sleep now
      Serial.println("Going to sleep now");
      M5.Axp.SetSleep(); // conveniently turn off screen, etc.
      delay(100);
      esp_deep_sleep_start();
    }

    if (!screenSaver)
    {
      M5.Lcd.setRotation(2);
      M5.Lcd.fillScreen(TFT_BLACK);
      screenSaver = true;
    }
    for (int i = 0; i < NSTARS; ++i)
    {
      if (sz[i] <= 1)
      {
        sx[i] = 160 - 120 + rng();
        sy[i] = rng();
        sz[i] = spawnDepthVariation--;
      }
      else
      {
        int old_screen_x = ((int)sx[i] - 160) * 256 / sz[i] + 40;
        int old_screen_y = ((int)sy[i] - 120) * 256 / sz[i] + 80;

        M5.Lcd.drawPixel(old_screen_x, old_screen_y, TFT_BLACK);

        sz[i] -= 2;
        if (sz[i] > 1)
        {
          int screen_x = ((int)sx[i] - 160) * 256 / sz[i] + 40;
          int screen_y = ((int)sy[i] - 120) * 256 / sz[i] + 80;

          if (screen_x >= 0 && screen_y >= 0 && screen_x < 80 && screen_y < 160)
          {
            uint8_t r, g, b;
            r = g = b = 255 - sz[i];
            M5.Lcd.drawPixel(screen_x, screen_y, M5.Lcd.color565(r, g, b));
          }
          else
            sz[i] = 0; // Out of screen, die.
        }
      }
    }
  }
  else

  {
    if (screenSaver)
    {
      M5.Lcd.setRotation(3);
      drawSend(HostIP, sendPort);
      screenSaver = false;
    }
    draw_battery_percent();

    M5.update();

    if (digitalRead(37) == 0)
    {
      if (pressed == 0)
      {
        pressed = 1;
        brightnessLevel++;
        if (brightnessLevel > 4)
          brightnessLevel = 0;
        M5.Axp.ScreenBreath(bright[brightnessLevel]);
        M5.Lcd.fillRect(128, 68, 160, 80, TFT_BLACK);
        drawSend(HostIP, sendPort);
      }
    }
    else
    {
      pressed = 0;
    }

    int temp = (int)(M5.Axp.GetTempInAXP192() + .5);
    M5.Lcd.setTextColor(grey, TFT_BLACK);
    M5.Lcd.drawString(String(temp) + " C  ", 3, 3);
    M5.Lcd.drawString(String(int(loopfreq)) + " FPS  ", 114, 3);

    for (int i = 0; i < brightnessLevel + 1; i++)
      M5.Lcd.fillRect(128 + (i * 5), 68, 3, 8, grey);

    if (M5.BtnB.wasPressed())
    {
      M5.Lcd.invertDisplay(inv);
      inv = !inv;
      drawSend(HostIP, sendPort);
    }

    if (short_press)
      ESP.restart();

    if (M5.BtnB.pressedFor(5000))
    {
      wifiManager.resetSettings();
      wifiManager.erase(true);
      wifiManager.setConfigPortalBlocking(true);
      ESP.restart();
    }

    if (M5.BtnA.pressedFor(2000))
    {
      animation = !animation;
    }

    if (M5.BtnA.pressedFor(5000))
    {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("begin calibration IMU in 3 seconds, dont move");
      delay(3000);
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.print("Calibrating");
      calibrate6886();
      if (withMagnet)
      {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("begin calibration compass in 3 seconds, move");
        delay(3000);
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.print("Flip + rotate core calibration");
        calibrate(10000);
        delay(100);
      }
    }

    if (animation)
    {
      cubeloop();
    }
  }
}

void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void loadParameters()
{
  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        StaticJsonDocument<140> doc;

        DeserializationError error = deserializeJson(doc, buf.get());

        if (error)
        {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }

        const char *devIDjsn = doc["devID"];           // "MotXi1"
        const char *HostIPjsn = doc["HostIP"];         // "192.168.0.8"
        const char *sendPortjsn = doc["sendPort"];     // "9999"
        const char *recivePortjsn = doc["recivePort"]; // "5555"

        strcpy(devID, devIDjsn);
        strcpy(HostIP, HostIPjsn);
        strcpy(sendPort, sendPortjsn);
        strcpy(recivePort, recivePortjsn);

        serializeJson(doc, Serial);

        Serial.println(devID);
        Serial.println(HostIP);
        Serial.println(sendPort);
        Serial.println(recivePort);

        Serial.println("\nparsed json");
        configFile.close();
      }
    }
    else
    {
      Serial.println("no Json Data!!");
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    SPIFFS.format();
  }
}

void saveParameters()
{

  StaticJsonDocument<140> doc;

  doc["devID"] = devID;
  doc["HostIP"] = HostIP;
  doc["sendPort"] = sendPort;
  doc["recivePort"] = recivePort;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }
  serializeJson(doc, configFile);
  delay(10);
  configFile.close();
  Serial.println("Saved Data to FS");
  serializeJson(doc, Serial);
  shouldSaveConfig = false;
}

void cubeloop()
{

  period = (millis() - timeLast);
  timeLast = millis();
  // precalc
  double MMPI_TIME = MMPI * period;
  double interpolation = cos(++framecount / MMPI * ispeed);

  scale = ceil((interpolation * 3) + (scaleMax - 3));

  //M5.MPU6886.getGyroData(&gyroX, &gyroY, &gyroZ);
  //M5.MPU6886.getAccelData(&accX, &accY, &accZ);

  int axCube = int(accX * 1000);
  int ayCube = int(accY * 1000);
  int azCube = int(accZ * 1000);

  int gxCube = int(gyroX * 1000);
  int gyCube = int(gyroY * 1000);
  int gzCube = int(gyroZ * 1000);

  if (abs(gxCube) < 20)
  {
    gxCube = (400 * interpolation) + 600;
  }
  if (abs(gyCube) < 15)
  {
    gyCube = (400 * interpolation) + 600;
  }
  if (abs(gzCube) < 30)
  {
    gzCube = (400 * interpolation) + 600;
  }

  tftSprite.createSprite(48, 50);
  tftSprite.setRotation(3);

  // scale angles down, rotate
  vectRotXYZ((double)-gyCube / MMPI_TIME, 1); // X
  vectRotXYZ((double)-gxCube / MMPI_TIME, 2); // Y
  vectRotXYZ((double)-gzCube / MMPI_TIME, 3); // Z

  // calculate each point coords
  P1[0] = centerX + scale / (1 + C1[2] / sZ) * C1[0];
  P1[1] = centerY + scale / (1 + C1[2] / sZ) * C1[1];
  P2[0] = centerX + scale / (1 + C2[2] / sZ) * C2[0];
  P2[1] = centerY + scale / (1 + C2[2] / sZ) * C2[1];
  P3[0] = centerX + scale / (1 + C3[2] / sZ) * C3[0];
  P3[1] = centerY + scale / (1 + C3[2] / sZ) * C3[1];
  P4[0] = centerX + scale / (1 + C4[2] / sZ) * C4[0];
  P4[1] = centerY + scale / (1 + C4[2] / sZ) * C4[1];
  P5[0] = centerX + scale / (1 + C5[2] / sZ) * C5[0];
  P5[1] = centerY + scale / (1 + C5[2] / sZ) * C5[1];
  P6[0] = centerX + scale / (1 + C6[2] / sZ) * C6[0];
  P6[1] = centerY + scale / (1 + C6[2] / sZ) * C6[1];
  P7[0] = centerX + scale / (1 + C7[2] / sZ) * C7[0];
  P7[1] = centerY + scale / (1 + C7[2] / sZ) * C7[1];
  P8[0] = centerX + scale / (1 + C8[2] / sZ) * C8[0];
  P8[1] = centerY + scale / (1 + C8[2] / sZ) * C8[1];

  if (!inv)
  {
    tftSprite.fillSprite(BLACK);
    tftSprite.drawLine(P1[0], P1[1], P2[0], P2[1], TFT_BLUE); //1-2
    tftSprite.drawLine(P1[0], P1[1], P3[0], P3[1], TFT_BLUE); //1-3
    tftSprite.drawLine(P1[0], P1[1], P5[0], P5[1], TFT_BLUE); //1-5
    tftSprite.drawLine(P2[0], P2[1], P4[0], P4[1], TFT_BLUE); //2-4
    tftSprite.drawLine(P2[0], P2[1], P6[0], P6[1], TFT_BLUE); //2-6
    tftSprite.drawLine(P3[0], P3[1], P4[0], P4[1], TFT_BLUE); //3-4
    tftSprite.drawLine(P3[0], P3[1], P7[0], P7[1], TFT_BLUE); //3-7
    tftSprite.drawLine(P4[0], P4[1], P8[0], P8[1], TFT_BLUE); //4-8
    tftSprite.drawLine(P5[0], P5[1], P6[0], P6[1], TFT_BLUE); //5-6
    tftSprite.drawLine(P5[0], P5[1], P7[0], P7[1], TFT_BLUE); //5-7
    tftSprite.drawLine(P6[0], P6[1], P8[0], P8[1], TFT_BLUE); //6-8
    tftSprite.drawLine(P7[0], P7[1], P8[0], P8[1], TFT_BLUE); //7-8

    // draw cross on each cube face
    tftSprite.drawLine(P1[0], P1[1], P4[0], P4[1], TFT_BLUE); //1-4
    tftSprite.drawLine(P2[0], P2[1], P3[0], P3[1], TFT_BLUE); //2-3
    tftSprite.drawLine(P1[0], P1[1], P6[0], P6[1], TFT_BLUE); //1-6
    tftSprite.drawLine(P2[0], P2[1], P5[0], P5[1], TFT_BLUE); //2-5
    tftSprite.drawLine(P2[0], P2[1], P8[0], P8[1], TFT_BLUE); //2-8
    tftSprite.drawLine(P6[0], P6[1], P4[0], P4[1], TFT_BLUE); //6-4
    tftSprite.drawLine(P4[0], P4[1], P7[0], P7[1], TFT_BLUE); //4-7
    tftSprite.drawLine(P3[0], P3[1], P8[0], P8[1], TFT_BLUE); //3-8
    tftSprite.drawLine(P1[0], P1[1], P7[0], P7[1], TFT_BLUE); //1-7
    tftSprite.drawLine(P3[0], P3[1], P5[0], P5[1], TFT_BLUE); //3-5
    tftSprite.drawLine(P5[0], P5[1], P8[0], P8[1], TFT_BLUE); //5-8
    tftSprite.drawLine(P7[0], P7[1], P6[0], P6[1], TFT_BLUE); //7-6
  }
  else
  {
    tftSprite.fillSprite(BLACK);
    tftSprite.drawLine(P1[0], P1[1], P2[0], P2[1], TFT_GREY); //1-2
    tftSprite.drawLine(P1[0], P1[1], P3[0], P3[1], TFT_GREY); //1-3
    tftSprite.drawLine(P1[0], P1[1], P5[0], P5[1], TFT_GREY); //1-5
    tftSprite.drawLine(P2[0], P2[1], P4[0], P4[1], TFT_GREY); //2-4
    tftSprite.drawLine(P2[0], P2[1], P6[0], P6[1], TFT_GREY); //2-6
    tftSprite.drawLine(P3[0], P3[1], P4[0], P4[1], TFT_GREY); //3-4
    tftSprite.drawLine(P3[0], P3[1], P7[0], P7[1], TFT_GREY); //3-7
    tftSprite.drawLine(P4[0], P4[1], P8[0], P8[1], TFT_GREY); //4-8
    tftSprite.drawLine(P5[0], P5[1], P6[0], P6[1], TFT_GREY); //5-6
    tftSprite.drawLine(P5[0], P5[1], P7[0], P7[1], TFT_GREY); //5-7
    tftSprite.drawLine(P6[0], P6[1], P8[0], P8[1], TFT_GREY); //6-8
    tftSprite.drawLine(P7[0], P7[1], P8[0], P8[1], TFT_GREY); //7-8

    // draw cross on each cube face
    tftSprite.drawLine(P1[0], P1[1], P4[0], P4[1], TFT_GREY); //1-4
    tftSprite.drawLine(P2[0], P2[1], P3[0], P3[1], TFT_GREY); //2-3
    tftSprite.drawLine(P1[0], P1[1], P6[0], P6[1], TFT_GREY); //1-6
    tftSprite.drawLine(P2[0], P2[1], P5[0], P5[1], TFT_GREY); //2-5
    tftSprite.drawLine(P2[0], P2[1], P8[0], P8[1], TFT_GREY); //2-8
    tftSprite.drawLine(P6[0], P6[1], P4[0], P4[1], TFT_GREY); //6-4
    tftSprite.drawLine(P4[0], P4[1], P7[0], P7[1], TFT_GREY); //4-7
    tftSprite.drawLine(P3[0], P3[1], P8[0], P8[1], TFT_GREY); //3-8
    tftSprite.drawLine(P1[0], P1[1], P7[0], P7[1], TFT_GREY); //1-7
    tftSprite.drawLine(P3[0], P3[1], P5[0], P5[1], TFT_GREY); //3-5
    tftSprite.drawLine(P5[0], P5[1], P8[0], P8[1], TFT_GREY); //5-8
    tftSprite.drawLine(P7[0], P7[1], P6[0], P6[1], TFT_GREY); //7-6
  }
  tftSprite.pushSprite(113, 13);
  tftSprite.deleteSprite();
}

void vectRotXYZ(double angle, int axe)
{
  int8_t m1;      // coords polarity
  uint8_t i1, i2; // coords index

  switch (axe)
  {
  case 1:   // X
    i1 = 1; // y
    i2 = 2; // z
    m1 = -1;
    break;
  case 2:   // Y
    i1 = 0; // x
    i2 = 2; // z
    m1 = 1;
    break;
  case 3:   // Z
    i1 = 0; // x
    i2 = 1; // y
    m1 = 1;
    break;
  }

  double t1 = C1[i1];
  double t2 = C1[i2];
  C1[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C1[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C2[i1];
  t2 = C2[i2];
  C2[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C2[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C3[i1];
  t2 = C3[i2];
  C3[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C3[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C4[i1];
  t2 = C4[i2];
  C4[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C4[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C5[i1];
  t2 = C5[i2];
  C5[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C5[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C6[i1];
  t2 = C6[i2];
  C6[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C6[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C7[i1];
  t2 = C7[i2];
  C7[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C7[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);

  t1 = C8[i1];
  t2 = C8[i2];
  C8[i1] = t1 * cos(angle) + (m1 * t2) * sin(angle);
  C8[i2] = (-m1 * t1) * sin(angle) + t2 * cos(angle);
}

float getBatteryLevel(float voltage)
{
  float level = 1;
  if (voltage >= levels[0])
  {
    level = 1;
  }
  else if (voltage >= levels[1])
  {
    level = 0.9;
    level += 0.1 * (voltage - levels[1]) / (levels[0] - levels[1]);
  }
  else if (voltage >= levels[2])
  {
    level = 0.8;
    level += 0.1 * (voltage - levels[2]) / (levels[1] - levels[2]);
  }
  else if (voltage >= levels[3])
  {
    level = 0.7;
    level += 0.1 * (voltage - levels[3]) / (levels[2] - levels[3]);
  }
  else if (voltage >= levels[4])
  {
    level = 0.6;
    level += 0.1 * (voltage - levels[4]) / (levels[3] - levels[4]);
  }
  else if (voltage >= levels[5])
  {
    level = 0.5;
    level += 0.1 * (voltage - levels[5]) / (levels[4] - levels[5]);
  }
  else if (voltage >= levels[6])
  {
    level = 0.4;
    level += 0.1 * (voltage - levels[6]) / (levels[5] - levels[6]);
  }
  else if (voltage >= levels[7])
  {
    level = 0.3;
    level += 0.1 * (voltage - levels[7]) / (levels[6] - levels[7]);
  }
  else if (voltage >= levels[8])
  {
    level = 0.2;
    level += 0.1 * (voltage - levels[8]) / (levels[7] - levels[8]);
  }
  else if (voltage >= levels[9])
  {
    level = 0.1;
    level += 0.1 * (voltage - levels[9]) / (levels[8] - levels[9]);
  }
  else if (voltage >= levels[10])
  {
    level = 0.05;
    level += 0.05 * (voltage - levels[10]) / (levels[9] - levels[10]);
  }
  else if (voltage >= levels[11])
  {
    level = 0.00;
    level += 0.05 * (voltage - levels[11]) / (levels[10] - levels[11]);
  }
  else
  {
    level = 0.00;
  }
  return level;
}

void draw_battery_percent()
{
  float battery_percent = getBatteryLevel(M5.Axp.GetBatVoltage());
  M5.Lcd.setTextColor(0xAA00FF00, TFT_BLACK);
  M5.Lcd.setCursor(59, 2, 1);
  M5.Lcd.print(battery_percent * 100, 1);
  M5.Lcd.print("%");
}

void drawSend(char *HostIp, char *sendPort)
{
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 18, 2);
  M5.Lcd.println("send to:");
  M5.Lcd.setCursor(3, 34, 2);
  if (!inv)
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  else
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println(HostIp);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 50, 2);
  M5.Lcd.print("Port: ");
  if (!inv)
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  else
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.print(sendPort);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0, 1);
}

void extractIpAddress(char *sourceString, short *ipAddress)
{
  short len = 0;
  char oct[4] = {0}, cnt = 0, cnt1 = 0, i, buf[5];

  len = strlen(sourceString);
  for (i = 0; i < len; i++)
  {
    if (sourceString[i] != '.')
    {
      buf[cnt++] = sourceString[i];
    }
    if (sourceString[i] == '.' || i == len - 1)
    {
      buf[cnt] = '\0';
      cnt = 0;
      oct[cnt1++] = atoi(buf);
    }
  }
  ipAddress[0] = oct[0];
  ipAddress[1] = oct[1];
  ipAddress[2] = oct[2];
  ipAddress[3] = oct[3];
}

void myGetAhrsData(float samplefrequency)
{
  applycalibration();

  bmm150_mag_data value;
  bmm.read_mag_data();

  magX = bmm.raw_mag_data.raw_datax - value_offset.x;
  magY = bmm.raw_mag_data.raw_datay - value_offset.y;
  magZ = bmm.raw_mag_data.raw_dataz - value_offset.z;

  if (withMagnet)
    myMahonyAHRSupdateIMU9Axis(gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, acc[0], acc[1], acc[2], -magX, magY, -magZ, &myQ0, &myQ1, &myQ2, &myQ3, samplefrequency);
  else
    myMahonyAHRSupdateIMU(gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, acc[0], acc[1], acc[2], &myQ0, &myQ1, &myQ2, &myQ3, samplefrequency);
}

void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);
    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);
    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);
    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    Serial.print(".");
    delay(10);
  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
}

// *MPU6886 Calibration
void calibrate6886()
{
  float gyroSum[3];
  float accSum[3];
  int counter = 2500;
  for (int i = 0; i < counter; i++)
  {
    M5.MPU6886.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
    M5.MPU6886.getAccelData(&acc[0], &acc[1], &acc[2]);
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    delay(2);
  }
  gyroOffset[0] = gyroSum[0] / counter;
  gyroOffset[1] = gyroSum[1] / counter;
  gyroOffset[2] = gyroSum[2] / counter;
  accOffset[0] = accSum[0] / counter;
  accOffset[1] = accSum[1] / counter;
  accOffset[2] = (accSum[2] / counter) - 1.0; //Gravitational Acceleration 1G, assuming that the M5 button is facing upward
}

void calibrate_waiting(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    if (digitalRead(M5_BUTTON_HOME) == LOW)
    {
      setup_flag = 1;
      while (digitalRead(M5_BUTTON_HOME) == LOW)
        ;
      break;
    }
    delay(100);
  }
}

void applycalibration()
{

  M5.MPU6886.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.MPU6886.getAccelData(&acc[0], &acc[1], &acc[2]);

  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  //acc[0] -= accOffset[0];
  //acc[1] -= accOffset[1];
  //acc[2] -= accOffset[2];

  //fake magnetometer data cuz MPU6886 doesn't come with BMM 150 chip
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
}

void initData()
{
  twoKp = twoKpDef; // 2 * proportional gain (Kp)
  twoKi = twoKiDef; // 2 * integral gain (Ki)
  myq0 = 1.0f;
  myq1 = 0.0f;
  myq2 = 0.0f;
  myq3 = 0.0f;
  myintegralFBx = 0.0f;
  myintegralFBy = 0.0f;
  myintegralFBz = 0.0f;
  anglesComputed = 0;
}

void myMahonyAHRSupdateIMU9Axis(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *q0, float *q1, float *q2, float *q3, float samplefrequency)
{
  static float myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;

  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
  {
    //updateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  invSampleFreq = (1.0f / samplefrequency);

  // Convert gyroscope degrees/sec to radians/sec
  //gx *= 0.0174533f;
  //gy *= 0.0174533f;
  //gz *= 0.0174533f;

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    //mx = mx+ 360.0 / 720.0;
    //my = my+ 360.0 / 720.0;
    //mz = mz+ 360.0 / 720.0;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = myq0 * myq0;
    q0q1 = myq0 * myq1;
    q0q2 = myq0 * myq2;
    q0q3 = myq0 * myq3;
    q1q1 = myq1 * myq1;
    q1q2 = myq1 * myq2;
    q1q3 = myq1 * myq3;
    q2q2 = myq2 * myq2;
    q2q3 = myq2 * myq3;
    q3q3 = myq3 * myq3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f)
    {
      // integral error scaled by Ki
      myintegralFBx += twoKi * halfex * invSampleFreq;
      myintegralFBy += twoKi * halfey * invSampleFreq;
      myintegralFBz += twoKi * halfez * invSampleFreq;
      gx += myintegralFBx; // apply integral feedback
      gy += myintegralFBy;
      gz += myintegralFBz;
    }
    else
    {
      myintegralFBx = 0.0f; // prevent integral windup
      myintegralFBy = 0.0f;
      myintegralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += 2.0 * halfex;
    gy += 2.0 * halfey;
    gz += 2.0 * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * invSampleFreq); // pre-multiply common factors
  gy *= (0.5f * invSampleFreq);
  gz *= (0.5f * invSampleFreq);
  qa = myq0;
  qb = myq1;
  qc = myq2;
  myq0 += (-qb * gx - qc * gy - myq3 * gz);
  myq1 += (qa * gx + qc * gz - myq3 * gy);
  myq2 += (qa * gy - qb * gz + myq3 * gx);
  myq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(myq0 * myq0 + myq1 * myq1 + myq2 * myq2 + myq3 * myq3);
  myq0 *= recipNorm;
  myq1 *= recipNorm;
  myq2 *= recipNorm;
  myq3 *= recipNorm;
  anglesComputed = 0;

  *q0 = myq0;
  *q1 = myq1;
  *q2 = myq2;
  *q3 = myq3;
}

/* This is a modified version of the original MahonyAHRSupdateIMU algorithm (which can be found in M5StickC/src/utility/MahonyAHRS.cpp) */
void myMahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *q0, float *q1, float *q2, float *q3, float samplefrequency)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  //static float myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
  //myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = myq1 * myq3 - myq0 * myq2;
    halfvy = myq0 * myq1 + myq2 * myq3;
    halfvz = myq0 * myq0 - 0.5f + myq3 * myq3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply proportional feedback
    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / samplefrequency)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = myq0;
  qb = myq1;
  qc = myq2;
  myq0 += (-qb * gx - qc * gy - myq3 * gz);
  myq1 += (qa * gx + qc * gz - myq3 * gy);
  myq2 += (qa * gy - qb * gz + myq3 * gx);
  myq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(myq0 * myq0 + myq1 * myq1 + myq2 * myq2 + myq3 * myq3);
  myq0 *= recipNorm;
  myq1 *= recipNorm;
  myq2 *= recipNorm;
  myq3 *= recipNorm;

  *q0 = myq0;
  *q1 = myq1;
  *q2 = myq2;
  *q3 = myq3;
}