#include <M5StickC.h>
#include <driver/rtc_io.h> // from ESP-IDF
#include "SPIFFS.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <OSCBundle.h>
#include <OSCMessage.h>
#include <Wire.h>
#include <WiFiUdp.h>

WiFiUDP Udp;                                // UDP instance
WiFiManager wifiManager;

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

#define WOM_ATTACH_ISR
volatile uint32_t g_wom_count = 0;
volatile uint32_t g_wom_last_millis = 0;

void IRAM_ATTR mpu6886_wake_on_motion_isr(void) {
  g_wom_count++;
  g_wom_last_millis = millis();
}

float getBatteryLevel(float voltage);
void draw_battery_percent();
void cubeloop();
void drawSend(char *HostIP, char *sendPort);
void vectRotXYZ(double angle, int axe);
void saveConfigCallback ();
void extractIpAddress(char *sourceString, short *ipAddress);
void loadParameters();
void saveParameters();

#define M_PI 3.141592653
#define grey 0x65DB
#define TFT_GREY 0x5AEB
#define WAKE_REASON_BUF_LEN 100
#define NSTARS 512

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
  zc = (zc + ((zb >> 1)^za));
  return zc;
}

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

float accX = 0;
float accY = 0;
float accZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float temp = 0;

int16_t  ax, ay, az, gx, gy, gz;
double MMPI = 32768 * M_PI;
int bright[5] = {7, 8, 9, 10, 12};
static const float levels[] = {4.13, 4.06, 3.98, 3.92, 3.87, 3.82, 3.79, 3.77, 3.74, 3.68, 3.45, 3.00};
bool screenSaver = false;
bool returnScreen = false;

long int timeLast = -100, period = 1;

// Overall scale and perspective distance
uint8_t sZ = 16, scale = 8, scaleMax = 12;

// screen center
uint8_t centerX = 24;
uint8_t centerY = 24;

// Initialize cube point arrays
double C1[] = {  1,  1,  1 };
double C2[] = {  1,  1, -1 };
double C3[] = {  1, -1,  1 };
double C4[] = {  1, -1, -1 };
double C5[] = { -1,  1,  1 };
double C6[] = { -1,  1, -1 };
double C7[] = { -1, -1,  1 };
double C8[] = { -1, -1, -1 };

// Initialize cube points coords
uint8_t P1[] = { 0, 0 };
uint8_t P2[] = { 0, 0 };
uint8_t P3[] = { 0, 0 };
uint8_t P4[] = { 0, 0 };
uint8_t P5[] = { 0, 0 };
uint8_t P6[] = { 0, 0 };
uint8_t P7[] = { 0, 0 };
uint8_t P8[] = { 0, 0 };

RTC_DATA_ATTR int bootCount = 0;

void setup() {

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
  M5.Mpu6886.Init(); // basic init
  M5.Mpu6886.enableWakeOnMotion(M5.Mpu6886.AFS_16G, 10);

  snprintf(PortalName, sizeof(PortalName), "MotionXi_%d", (uint16_t)ESP.getEfuseMac());

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
  SPIFFS.begin (true);

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

  if(M5.BtnA.isPressed())
    wifiManager.setConfigPortalBlocking(true);
  else
    wifiManager.setConfigPortalBlocking(false);

  M5.Lcd.init();

  M5.Axp.ScreenBreath(10);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 10, 2);
  M5.Lcd.print("Connect to SSID:");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(3, 28, 2);
  M5.Lcd.print(PortalName);
  M5.Lcd.setTextColor(TFT_GREY, TFT_BLACK);
  M5.Lcd.setCursor(3, 45, 2);
  M5.Lcd.print("Password: ");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.print("EnterThis");

  if (!wifiManager.autoConnect(PortalName, "EnterThis")) {
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

  if (shouldSaveConfig) {
    Serial.println("saving config");
    saveParameters();
  }

  Serial.println("local ip  ");
  Serial.print(WiFi.localIP());
  Serial.println();

  extractIpAddress(HostIP, &ipAddress[0]);

  const unsigned int localPort = int(recivePort);                                   // local port to listen for OSC packets
  Udp.begin(localPort);

  Serial.println("Done");

  drawSend(HostIP, sendPort);
}

int framecount = 0;
int ispeed = 10000; // interpolation speed
double bouncespeed = 100;
double lastspeedpot;
int c = 1;
int pres = false;
bool inv = false;

void loop() {

  bool short_press, long_press;
  M5.Axp.GetPressIRQ(&short_press, &long_press);

  if (short_press || long_press) {
    M5.Axp.ClearPressIRQ(short_press, long_press);
  }

  IPAddress ip(WiFi.localIP());                                               // Device IP
  IPAddress outIp(ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);    // remote IP of the connected Computer
  unsigned int outPort = atoi(sendPort);

  OSCBundle bndl;

  char oscRoll[20] = "/";
  char oscPitch[20] = "/";
  char oscYaw[20] = "/";

  strcat(oscRoll, devID);
  strcat(oscRoll, "/roll");

  strcat(oscPitch, devID);
  strcat(oscPitch, "/pitch");

  strcat(oscYaw, devID);
  strcat(oscYaw, "/yaw");

  M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);

  //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
  bndl.add(oscRoll).add(roll);
  bndl.add(oscPitch).add(pitch);
  bndl.add(oscYaw).add(yaw);

  Udp.beginPacket(outIp, outPort);
  bndl.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one

  unsigned long t0 = micros();
  uint32_t since_last_wom_millis = millis() - g_wom_last_millis;
  uint8_t spawnDepthVariation = 255;

  if (since_last_wom_millis > SCSAVERTIME * 1000 )
  {
    if (since_last_wom_millis > DEEPSLEEPTIME * 1000 ) {

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

    if (!screenSaver) {
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

  } else

  {
    if (screenSaver) {
      M5.Lcd.setRotation(3);
      drawSend(HostIP, sendPort);
      screenSaver = false;
    }
    draw_battery_percent();

    M5.update();

    if (digitalRead(37) == 0)
    {
      if (pres == 0)
      {
        pres = 1;
        c++;
        if (c > 4)
          c = 0;
        M5.Axp.ScreenBreath(bright[c]);
        M5.Lcd.fillRect(128, 68, 160, 80, TFT_BLACK);
      }
    } else {
      pres = 0;
    }

    int temp = (int)(M5.Axp.GetTempInAXP192() + .5);
    M5.Lcd.setTextColor(grey, TFT_BLACK);
    M5.Lcd.drawString(String( temp ) + " C  ", 3, 3);
    M5.Lcd.drawString(String( M5.Axp.GetBatVoltage() ) + " V  ", 114, 3);

    for (int i = 0; i < c + 1; i++)
      M5.Lcd.fillRect( 128 + (i * 5), 68, 3, 8, grey);

    if (M5.BtnB.wasPressed()) {
      M5.Lcd.invertDisplay(inv);
      drawSend(HostIP, sendPort);
      inv = !inv;
    }

    if (short_press)
      ESP.restart();

    if (M5.BtnB.pressedFor(5000)) {
      wifiManager.resetSettings();
      wifiManager.erase(true);
      wifiManager.setConfigPortalBlocking(true);
      ESP.restart();
    }

    cubeloop();
  }
}

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void loadParameters() {
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        StaticJsonDocument<140> doc;

        DeserializationError error = deserializeJson(doc, buf.get());

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }

        const char* devIDjsn = doc["devID"]; // "MotionXi1"
        const char* HostIPjsn = doc["HostIP"]; // "192.168.0.8"
        const char* sendPortjsn = doc["sendPort"]; // "15608"
        const char* recivePortjsn = doc["recivePort"]; // "5555"

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
    } else {
      Serial.println("no Json Data!!");
    }
  } else {
    Serial.println("failed to mount FS");
    SPIFFS.format();
  }
}

void saveParameters() {

  StaticJsonDocument<140> doc;

  doc["devID"] = devID;
  doc["HostIP"] = HostIP;
  doc["sendPort"] = sendPort;
  doc["recivePort"] = recivePort;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }
  serializeJson(doc, configFile);
  delay(10);
  configFile.close();
  Serial.println("Saved Data to FS");
  serializeJson(doc, Serial);
  shouldSaveConfig = false;
}

void cubeloop() {

  period = (millis() - timeLast);
  timeLast = millis();
  // precalc
  double MMPI_TIME = MMPI * period;
  double interpolation = cos(++framecount / MMPI * ispeed);

  scale = ceil( (interpolation * 3) + (scaleMax - 3) );

  M5.MPU6886.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.MPU6886.getAccelData(&accX, &accY, &accZ);

  ax = int(accX * 10000),
  ay = int(accY * 10000);
  az = int(accZ * 10000);

  gx = int(gyroX * 1000);
  gy = int(gyroY * 1000);
  gz = int(gyroZ * 1000);

  if (abs(gx) < 20) {
    gx = (400 * interpolation) + 600;
  }
  if (abs(gy) < 15) {
    gy = (400 * interpolation) + 600;
  }
  if (abs(gz) < 30) {
    gz = (400 * interpolation) + 600;
  }

  tftSprite.createSprite(48, 50);
  tftSprite.setRotation(3);

  // scale angles down, rotate
  vectRotXYZ((double) - gy / MMPI_TIME, 1); // X
  vectRotXYZ((double) - gx / MMPI_TIME, 2); // Y
  vectRotXYZ((double) - gz / MMPI_TIME, 3); // Z

  // calculate each point coords
  P1[0] = centerX + scale / (1 + C1[2] / sZ) * C1[0]; P1[1] = centerY + scale / (1 + C1[2] / sZ) * C1[1];
  P2[0] = centerX + scale / (1 + C2[2] / sZ) * C2[0]; P2[1] = centerY + scale / (1 + C2[2] / sZ) * C2[1];
  P3[0] = centerX + scale / (1 + C3[2] / sZ) * C3[0]; P3[1] = centerY + scale / (1 + C3[2] / sZ) * C3[1];
  P4[0] = centerX + scale / (1 + C4[2] / sZ) * C4[0]; P4[1] = centerY + scale / (1 + C4[2] / sZ) * C4[1];
  P5[0] = centerX + scale / (1 + C5[2] / sZ) * C5[0]; P5[1] = centerY + scale / (1 + C5[2] / sZ) * C5[1];
  P6[0] = centerX + scale / (1 + C6[2] / sZ) * C6[0]; P6[1] = centerY + scale / (1 + C6[2] / sZ) * C6[1];
  P7[0] = centerX + scale / (1 + C7[2] / sZ) * C7[0]; P7[1] = centerY + scale / (1 + C7[2] / sZ) * C7[1];
  P8[0] = centerX + scale / (1 + C8[2] / sZ) * C8[0]; P8[1] = centerY + scale / (1 + C8[2] / sZ) * C8[1];

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


void vectRotXYZ(double angle, int axe) {
  int8_t m1; // coords polarity
  uint8_t i1, i2; // coords index

  switch (axe) {
    case 1: // X
      i1 = 1; // y
      i2 = 2; // z
      m1 = -1;
      break;
    case 2: // Y
      i1 = 0; // x
      i2 = 2; // z
      m1 = 1;
      break;
    case 3: // Z
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

float getBatteryLevel(float voltage) {
  float level = 1;
  if (voltage >= levels[0]) {
    level = 1;
  } else if (voltage >= levels[1]) {
    level = 0.9;
    level += 0.1 * (voltage - levels[1]) / (levels[0] - levels[1]);
  } else if (voltage >= levels[2]) {
    level = 0.8;
    level += 0.1 * (voltage - levels[2]) / (levels[1] - levels[2]);
  } else if (voltage >= levels[3]) {
    level = 0.7;
    level += 0.1 * (voltage - levels[3]) / (levels[2] - levels[3]);
  } else if (voltage >= levels[4]) {
    level = 0.6;
    level += 0.1 * (voltage - levels[4]) / (levels[3] - levels[4]);
  } else if (voltage >= levels[5]) {
    level = 0.5;
    level += 0.1 * (voltage - levels[5]) / (levels[4] - levels[5]);
  } else if (voltage >= levels[6]) {
    level = 0.4;
    level += 0.1 * (voltage - levels[6]) / (levels[5] - levels[6]);
  } else if (voltage >= levels[7]) {
    level = 0.3;
    level += 0.1 * (voltage - levels[7]) / (levels[6] - levels[7]);
  } else if (voltage >= levels[8]) {
    level = 0.2;
    level += 0.1 * (voltage - levels[8]) / (levels[7] - levels[8]);
  } else if (voltage >= levels[9]) {
    level = 0.1;
    level += 0.1 * (voltage - levels[9]) / (levels[8] - levels[9]);
  } else if (voltage >= levels[10]) {
    level = 0.05;
    level += 0.05 * (voltage - levels[10]) / (levels[9] - levels[10]);
  } else if (voltage >= levels[11]) {
    level = 0.00;
    level += 0.05 * (voltage - levels[11]) / (levels[10] - levels[11]);
  } else {
    level = 0.00;
  }
  return level;
}

void draw_battery_percent() {
  float battery_percent = getBatteryLevel(M5.Axp.GetBatVoltage());
  M5.Lcd.setTextColor(0xAA00FF00, TFT_BLACK);
  M5.Lcd.setCursor(59, 2, 1);
  M5.Lcd.print(battery_percent * 100, 1);
  M5.Lcd.print("%");
}

void drawSend(char *HostIp, char *sendPort) {
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
    if (sourceString[i] != '.') {
      buf[cnt++] = sourceString[i];
    }
    if (sourceString[i] == '.' || i == len - 1) {
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
