#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#include <Servo.h>
#include <Wire.h>

#include <Ticker.h>

Ticker Pwm;

const uint8_t robotNum = 1;

#define ROOT_HTML "<html><head></head><body><p>HELLO FROM ESP #%d<\p><a href=\"/update\">update</a><br><a href=\"/chipdata\">chipdata</a></body></html>"

/*
   WiFi setting
*/
IPAddress ip(192, 168, 4, 10 + robotNum);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 4, 1);

const char *ssid = "YumeKobo_RS_2019";
const char *password = "yumekobors2019";
unsigned int port = 10000;
uint8_t buf[4];

WiFiUDP Udp;

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer Updater;

/*
   wii remote status
*/
uint16_t acc_y = 0;
bool btn_A = false;
bool btn_B = false;
bool btn_m = false;
bool btn_p = false;
bool btn_h = false;
bool btn_1 = false;
bool btn_2 = false;
bool btn_u = false;
bool btn_d = false;
bool btn_l = false;
bool btn_r = false;

/*
   micro sw status
*/
bool sw1 = false;
bool sw2 = false;

/*
   pin assignment
*/
#define PILOT  16 //pilot lamp pin
#define BUZZER 14 //buzzer pin

#define SERVO1 12  //servo1 pin
#define SERVO2 13  //servo2 pin

#define SDA    4  //I2C SDA pin
#define SCL    5  //I2C SCL pin

/*
   7 segment led setting
*/
#define SVN_SEG_n 0b00000000
#define SVN_SEG_0 0b01110111
#define SVN_SEG_1 0b00100001
#define SVN_SEG_2 0b01101110
#define SVN_SEG_3 0b01101011
#define SVN_SEG_4 0b00111001
//#define SVN_SEG_5 0b00000000
//#define SVN_SEG_6 0b00000000
//#define SVN_SEG_7 0b00000000
//#define SVN_SEG_8 0b00000000
//#define SVN_SEG_9 0b00000000

/*
   motor setting
*/
bool pwm_flag1 = true;
uint8_t pwm_count1 = 0;
uint8_t pwm_value1 = 0;
#define M1_STOP 0b00000000
#define M1_FORW 0b10000000
#define M1_BACK 0b01000000

bool pwm_flag2 = true;
uint8_t pwm_count2 = 0;
uint8_t pwm_value2 = 0;
#define M2_STOP 0b00000000
#define M2_FORW 0b00100000
#define M2_BACK 0b00010000

bool pwm_flag3 = true;
uint8_t pwm_count3 = 0;
uint8_t pwm_value3 = 0;
#define M3_STOP 0b00000000
#define M3_FORW 0b00001000
#define M3_BACK 0b00000100

/*
   servo setting
*/
#define SERVO1_START 155
#define SERVO1_RAD1 50
#define SERVO1_RAD2 155
#define SERVO2_START 0
#define SERVO2_RAD1 0
#define SERVO2_RAD2 130

/*
   I2C setting
*/
//I2C address
#define MCP_ADDR 0x20 //MCP23017 I2C address

//register value
#define IODIRA          0b00000000
#define IODIRB          0b00000000
#define IPOLA           0b00000000
#define IPOLB           0b00000000
#define GPPUA           0b00000000
#define GPPUB           0b00000000
#define GPINTENA        0b00000000
#define GPINTENB        0b00000000
#define DEFVALA         0b00000000
#define DEFVALB         0b00000001
#define INTCONA         0b00000000
#define INTCONB         0b00000001
#define IOCON           0b00000010

//register address
#define IODIRA_ADDR     0x00
#define IODIRB_ADDR     0x01
#define IPOLA_ADDR      0x02
#define IPOLB_ADDR      0x03
#define GPPUA_ADDR      0x0C
#define GPPUB_ADDR      0x0D
#define GPIOA_ADDR      0x12
#define GPIOB_ADDR      0x13
#define OLATA_ADDR      0x14
#define OLATB_ADDR      0x15
#define GPINTENA_ADDR   0x04
#define GPINTENB_ADDR   0x05
#define DEFVALA_ADDR    0x06
#define DEFVALB_ADDR    0x07
#define INTCONA_ADDR    0x08
#define INTCONB_ADDR    0x09
#define INTFA_ADDR      0x0E
#define INTFB_ADDR      0x0F
#define INTCAPA_ADDR    0x10
#define INTCAPB_ADDR    0x11
#define IOCON_ADDR      0x0A

//IO pin number
#define GPA         0
#define GPA0        0
#define GPA1        1
#define GPA2        2
#define GPA3        3
#define GPA4        4
#define GPA5        5
#define GPA6        6
#define GPA7        7
#define GPB         1
#define GPB0        0
#define GPB1        1
#define GPB2        2
#define GPB3        3
#define GPB4        4
#define GPB5        5
#define GPB6        6
#define GPB7        7

/*
   other setting
*/
#define WIFI_CONNECT_TIMEOUT 40

Servo servo1, servo2;

void motorStop() {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(GPIOA_ADDR);
  Wire.write(M1_STOP | M2_STOP | M3_STOP);
  Wire.endTransmission();
}

void makeWave(uint8_t *count, bool *flag, uint8_t value) {
  if (*count == 20) {
    *flag = true;
    count = 0;
  } else if (*count == constrain(value, 0, 20)) {
    *flag = false;
  }
  *count++;
}

void pwm() {
  //makeWave(&pwm_count1, &pwm_flag1, pwm_value1);
  //makeWave(&pwm_count2, &pwm_flag2, pwm_value2);

  if (pwm_count1 == 20) {
    pwm_flag1 = true;
    pwm_count1 = 0;
  } else if (pwm_count1 == constrain(pwm_value1, 0, 20)) {
    pwm_flag1 = false;
  }
  pwm_count1++;

  if (pwm_count2 == 20) {
    pwm_flag2 = true;
    pwm_count2 = 0;
  } else if (pwm_count2 == constrain(pwm_value2, 0, 20)) {
    pwm_flag2 = false;
  }
  pwm_count2++;
}

void showChipData(char *chipData) {
  char resetReason[32];
  ESP.getResetReason().toCharArray(resetReason, 30);

  char coreVersion[12];
  ESP.getCoreVersion().toCharArray(coreVersion, 30);

  char sketchMD5[128];
  ESP.getSketchMD5().toCharArray(sketchMD5, 30);

  sprintf(chipData,
          "<html>"
          "<p>"
          "reset reason: %s, free heap: %d, heap fragmentation: %d %%, max free block size %d"
          "<\p>"
          "<p>"
          "chip ID: %d, core version: %d, SDK version: %s, CPU freq: %d MHz, sketch size: %d"
          "<\p>"
          "<p>"
          "free sketch space: %d, sketch MD5: %s, flash chip Id: %d, flash chip size(viewd SDK): %d byte"
          "<\p>"
          "<p>"
          "flash chip size(real one): %d byte, flash chip speed: %d, cycle count: %lu"
          "<\p>"
          "<\html>"
          , resetReason, ESP.getFreeHeap(), ESP.getHeapFragmentation(), ESP.getMaxFreeBlockSize()
          , ESP.getChipId(), coreVersion, ESP.getSdkVersion(), ESP.getCpuFreqMHz(), ESP.getSketchSize()
          , ESP.getFreeSketchSpace(), sketchMD5, ESP.getFlashChipId(), ESP.getFlashChipSize()
          , ESP.getFlashChipRealSize(), ESP.getFlashChipSpeed(), ESP.getCycleCount());
}

void delay_alarm(uint16_t delay_time) {
  delay_time /= 4;

  digitalWrite(PILOT, LOW);
  digitalWrite(BUZZER, HIGH);

  delay(delay_time);

  digitalWrite(PILOT, HIGH);

  delay(delay_time);

  digitalWrite(PILOT, LOW);
  digitalWrite(BUZZER, LOW);

  delay(delay_time);

  digitalWrite(PILOT, HIGH);
}

void connectWiFi(const char* ssid , const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.config(ip, gateway, subnet, dns); //static_ip

  delay(100);

  WiFi.begin(ssid, password);

  uint8_t count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (count == WIFI_CONNECT_TIMEOUT) {
      digitalWrite(PILOT, HIGH);
      digitalWrite(BUZZER, HIGH);

      for (uint8_t i; i < 5; i++) delay_alarm(200);

      ESP.restart();
    }

    count++;

    delay_alarm(500);
  }

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    for (uint8_t i; i < 5; i++) delay_alarm(200);
    delay(4000);
    ESP.restart();
  }
}

void udpSend(char *byteData) {
  if (Udp.beginPacket(gateway, port + 10000)) {
    Udp.write(byteData);
    Udp.endPacket();
  }
}

void getCtrStatus() {
  if (Udp.parsePacket()) {
    uint16_t btn_val = 0;
    Udp.read(buf, 4);

    acc_y = ((uint16_t)buf[3] << 8);
    acc_y += buf[2];
    btn_val += ((uint16_t)buf[1] << 8);
    btn_val += buf[0];

    btn_2 =  btn_val & 0x0001;
    btn_1 = (btn_val & 0x0002) >>  1;
    btn_B = (btn_val & 0x0004) >>  2;
    btn_A = (btn_val & 0x0008) >>  3;
    btn_m = (btn_val & 0x0010) >>  4;
    btn_h = (btn_val & 0x0080) >>  7;
    btn_d = (btn_val & 0x0100) >>  8;
    btn_u = (btn_val & 0x0200) >>  9;
    btn_r = (btn_val & 0x0400) >> 10;
    btn_l = (btn_val & 0x0800) >> 11;
    btn_p = (btn_val & 0x1000) >> 12;

    /*
        char btn_status[255];
        sprintf(btn_status, "acc_y: % 05d, A: % d, B: % d, -: % d, +: % d, HOME: % d, 1: % d, 2: % d, u: % d, d: % d, l: % d, r: % d\n", acc_y, btn_A, btn_B, btn_m, btn_p, btn_h, btn_1, btn_2, btn_u, btn_d, btn_l, btn_r);
        udpSend(btn_status);
    */
  }
}

void report(char *str) {
  const uint8_t bufSize = 32;
  char buf[bufSize];

  snprintf(buf, bufSize, "[ESP#%d] %s", robotNum, str);

  udpSend(buf);
}

void reportVcc() {
  static uint8_t lowVccCount = 0;

  const uint16_t interval = 1000;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (analogRead(A0) * 5.4 < 4000) ++lowVccCount;
  }

  if (lowVccCount == 10) {
    motorStop();

    char str[20];
    sprintf(str, "warn: Vcc LOW %f mV", analogRead(A0) * 5.4);
    report(str);

    lowVccCount = 0;
  }
}

void serverSetup() {
  Updater.setup(&server, "yume", "kobo");
  server.begin();

  server.on("/", []() {
    char html[256];
    sprintf(html, ROOT_HTML, robotNum);
    server.send(200, "text/html", html);
  });

  server.on("/chipdata", []() {
    char chipData[1024];
    showChipData(chipData);
    server.send(200, "text / html", chipData);
  });
}

void setup() {
  delay(500);

  pinMode(PILOT, OUTPUT);
  digitalWrite(PILOT, HIGH);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

  servo1.attach(SERVO1);
  servo1.write(SERVO1_START);

  servo2.attach(SERVO2);
  servo2.write(SERVO2_START);

  Wire.begin(SDA, SCL);

  Wire.beginTransmission(MCP_ADDR);
  Wire.write(IODIRA_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MCP_ADDR);
  Wire.write(IODIRB_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  motorStop();

  Pwm.attach_ms(1, pwm);

  connectWiFi(ssid , password);

  Udp.begin(port);

  serverSetup();

  delay(200);

  report("info: start udp");
}

void loop() {
  motorStop();

  byte dataA = M1_STOP | M2_STOP | M3_STOP;
  byte dataB = SVN_SEG_n;

  byte old_dataA = ~dataA; //old_dataA とdataAを異なる値にするため
  byte old_dataB = ~dataB;

  switch (robotNum) {
    case 1:
      dataB = SVN_SEG_1;
      break;
    case 2:
      dataB = SVN_SEG_2;
      break;
    case 3:
      dataB = SVN_SEG_3;
      break;
    case 4:
      dataB = SVN_SEG_4;
      break;
  }

  while (true) {
    while ((WiFi.status() != WL_CONNECTED)) {
      motorStop();
      connectWiFi(ssid , password);
    }
    server.handleClient();

    static uint8_t rad1 = SERVO1_START;
    static uint8_t rad2 = SERVO2_START;

    getCtrStatus();

    /*
        Wire.beginTransmission(MCP_ADDR);
        Wire.write(GPIOA_ADDR); // address PORT B
        Wire.endTransmission();
        Wire.requestFrom(MCP_ADDR, 1); // request one byte of data
        byte gpaRead = Wire.read(); // store incoming byte into "input"
        sw1 = (gpaRead & 0x03) >> 1;
        sw2 =  gpaRead & 0x01;
    */


    /*
        if (btn_u) {
          dataA = (pwm_flag1) ? (M1_FORW | M2_FORW | M3_STOP) : (M1_STOP | M2_STOP | M3_STOP);
        } else if (btn_d) {
          dataA = (pwm_flag1) ? (M1_BACK | M2_BACK | M3_STOP) : (M1_STOP | M2_STOP | M3_STOP);
        } else if (btn_l) {
          dataA = (pwm_flag1) ? (M1_FORW | M2_BACK | M3_STOP) : (M1_STOP | M2_STOP | M3_STOP);
        } else if (btn_r) {
          dataA = (pwm_flag1) ? (M1_BACK | M2_FORW | M3_STOP) : (M1_STOP | M2_STOP | M3_STOP);
        } else {
          dataA = M1_STOP | M2_STOP | M3_STOP;
        }
    */

    int8_t Direction  = constrain(map(acc_y, 380, 580, -20, 20), -20, 20);
    if (Direction >= 0) {
      pwm_value1 = 20;
      pwm_value2 = 20 - Direction;
    } else {
      pwm_value1 = 20 + Direction;
      pwm_value2 = 20;
    }

    dataA = M1_STOP | M2_STOP | M3_STOP;

    if (btn_2) {
      dataA = ((pwm_flag1) ? M1_FORW : M1_STOP) | ((pwm_flag2) ? M2_FORW : M2_STOP);
    } else if (btn_1) {
      dataA = ((pwm_flag1) ? M1_BACK : M1_STOP) | ((pwm_flag2) ? M2_BACK : M2_STOP);
    }

    if (btn_l) {
      rad1 = SERVO1_RAD1;
    } else {
      rad1 = SERVO1_RAD2;
    }

    if (btn_u) {
      rad1 = SERVO1_RAD1;
    } else {
      rad1 = SERVO1_RAD2;
    }

    if (btn_r) {
      rad2 = SERVO2_RAD1;
    } else {
      rad2 = SERVO2_RAD2;
    }

    if (btn_h) {
      report("HELLO");
    }

    if (dataA != old_dataA) {
      Wire.beginTransmission(MCP_ADDR);
      Wire.write(GPIOA_ADDR);
      Wire.write(dataA);
      Wire.endTransmission();
    }
    old_dataA = dataA;

    if (dataB != old_dataB) {
      Wire.beginTransmission(MCP_ADDR);
      Wire.write(GPIOB_ADDR);
      Wire.write(dataB);
      Wire.endTransmission();
    }
    old_dataB = dataB;

    servo1.write(rad1);
    servo2.write(rad2);

    reportVcc();
  }
}
