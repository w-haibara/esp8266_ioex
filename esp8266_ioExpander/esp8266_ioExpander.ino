#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#include <Servo.h>
#include <Wire.h>

/*
   WiFi setting
*/
IPAddress ip(192, 168, 4, 11);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 4, 1);

const char *ssid = "YumeKobo_RS_2019";
const char *password = "yumekobors2019";
unsigned int port = 10000;
uint8_t buf[4];

WiFiUDP Udp;

ESP8266WebServer Server(80);
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
#define BUZZER 13 //buzzer pin

#define SERVO1 12  //servo1 pin
#define SERVO2 14  //servo2 pin

#define SDA    4  //I2C SDA pin
#define SCL    5  //I2C SCL pin

/*
   7 segment led setting
*/
#define SVN_SEG_n 0x00
#define SVN_SEG_0 0x7D
#define SVN_SEG_1 0x60
#define SVN_SEG_2 0x1E
#define SVN_SEG_3 0x7A
#define SVN_SEG_4 0x63
#define SVN_SEG_5 0x5B
#define SVN_SEG_6 0x5F
#define SVN_SEG_7 0x71
#define SVN_SEG_8 0xFF
#define SVN_SEG_9 0x7B

/*
   motor setting
*/
#define M1_STOP 0b11000000
#define M1_FORW 0b10000000
#define M1_BACK 0b01000000
#define M2_STOP 0b00110000
#define M2_FORW 0b00100000
#define M2_BACK 0b00010000

/*
   motor setting
*/
#define SERVO1_1 120
#define SERVO1_2 60
#define SERVO2_1 120
#define SERVO2_2 60

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

      delay_alarm(200);

      ESP.restart();
    }

    count++;

    delay_alarm(500);
  }

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay_alarm(200);
    delay(5000);
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

    acc_y += ((uint16_t)buf[3] << 8);
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
       char str[255];
       sprintf(str, "acc_y: %05d, A: %d, B: %d, -: %d, +: %d, HOME: %d, 1: %d, 2: %d, u: %d, d: %d, l: %d, r: %d\n", acc_y, btn_A, btn_B, btn_m, btn_p, btn_h, btn_1, btn_2, btn_u, btn_d, btn_l, btn_r);
       udpSend(str);
    */
  }
}

void setup() {
  pinMode(PILOT, OUTPUT);
  digitalWrite(PILOT, HIGH);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

  //pinMode(1, FUNCTION_3);
  servo1.attach(SERVO1);
  servo1.write(120);

  //pinMode(3, FUNCTION_3);
  servo2.attach(SERVO2);
  servo2.write(120);

  Wire.begin(4, 5);

  Wire.beginTransmission(MCP_ADDR);
  Wire.write(IODIRA_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MCP_ADDR);
  Wire.write(IODIRB_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  connectWiFi(ssid , password);

  Updater.setup(&Server, "yume", "kobo");
  Server.begin();

  Udp.begin(port);

  delay(200);
}

void loop() {

  while (true) {
    while ((WiFi.status() != WL_CONNECTED)) {
      connectWiFi(ssid , password);
    }
    Server.handleClient();

    byte dataA = M1_STOP | M2_STOP;
    byte dataB = SVN_SEG_0;
    static uint8_t rad1 = 120;
    static uint8_t rad2 = 120;

    getCtrStatus();

    Wire.beginTransmission(MCP_ADDR);
    Wire.write(GPIOA_ADDR); // address PORT B
    Wire.endTransmission();
    Wire.requestFrom(MCP_ADDR, 1); // request one byte of data
    byte gpaRead = Wire.read(); // store incoming byte into "input"
    sw1 = (gpaRead & 0x03) >> 1;
    sw2 =  gpaRead & 0x01;

    if (btn_u) {
      dataA = M1_FORW | M2_FORW;
      dataB = SVN_SEG_1;
    } else if (btn_d) {
      dataA = M1_BACK | M2_BACK;
      dataB = SVN_SEG_2;
    } else if (btn_l) {
      dataA = M1_FORW | M2_BACK;
      dataB = SVN_SEG_3;
    } else if (btn_r) {
      dataA = M1_BACK | M2_FORW;
      dataB = SVN_SEG_4;
    }

    if (btn_1) {
      rad1 = 60;
      dataB = SVN_SEG_5;
    } else {
      rad1 = 120;
    }

    if (btn_2) {
      rad2 = 60;
      dataB = SVN_SEG_6;
    } else {
      rad2 = 120;
    }

    Wire.beginTransmission(MCP_ADDR);
    Wire.write(GPIOA_ADDR);
    Wire.write(dataA);
    Wire.endTransmission();

    Wire.beginTransmission(MCP_ADDR);
    Wire.write(GPIOB_ADDR);
    Wire.write(dataB);
    Wire.endTransmission();

    servo1.write(rad1);
    servo2.write(rad2);

    /*
        char str[255];
        sprintf(str, "");
        udpSend(str);
    */
  }
}
