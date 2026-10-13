#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

const char *ssid = "netGu";
const char *password = "12345678";

const int UDP_PORT = 5008;

// ---- ขามอเตอร์ (Cytron MDD3A) ----
const int IN1A = 4, IN1B = 5; // ล้อซ้าย
const int IN2A = 6, IN2B = 7; // ล้อขวา

// ---- LEDC channel ----
const int CH1A = 0, CH1B = 1, CH2A = 2, CH2B = 3;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;

// ---- ความเร็ว ----
int speedFwd = 200;
int speedTurn = 90;

// ---- ปรับสมดุลล้อ ----
float trim1 = 1.00;
float trim2 = 0.975;

// ---- failsafe ----
const unsigned long TIMEOUT = 400;
unsigned long lastCmd = 0;

WiFiUDP udp;
char lastState = 'x';

void driveMotor(int chA, int chB, int speed)
{
  speed = constrain(speed, -255, 255);
  if (speed >= 0)
  {
    ledcWrite(chA, speed);
    ledcWrite(chB, 0);
  }
  else
  {
    ledcWrite(chA, 0);
    ledcWrite(chB, -speed);
  }
}

void drive(int left, int right)
{
  driveMotor(CH1A, CH1B, (int)(left * trim1));
  driveMotor(CH2A, CH2B, (int)(right * trim2));
}

void stopAll() { drive(0, 0); }

void motion(char c)
{
  switch (c)
  {
  case 'w':
    drive(speedFwd, speedFwd);
    break;
  case 's':
    drive(-speedFwd, -speedFwd);
    break;
  case 'a':
    drive(speedTurn, -speedTurn);
    break;
  case 'd':
    drive(-speedTurn, speedTurn);
    break;
  default:
    stopAll();
    break;
  }
}

void applyCommand(char c)
{
  switch (c)
  {
  case 'w':
  case 's':
  case 'a':
  case 'd':
  case 'x':
    motion(c);
    lastState = c;
    lastCmd = millis();
    return;

  case '1':
    trim2 -= 0.02f;
    if (trim2 < 0.50f)
      trim2 = 0.50f;
    Serial.printf("trim2 = %.2f\n", trim2);
    motion(lastState);
    lastCmd = millis();
    return;

  case '2':
    trim2 += 0.02f;
    if (trim2 > 1.00f)
      trim2 = 1.00f;
    Serial.printf("trim2 = %.2f\n", trim2);
    motion(lastState);
    lastCmd = millis();
    return;

  case '3':
    speedTurn -= 10;
    if (speedTurn < 60)
      speedTurn = 60;
    Serial.printf("speedTurn = %d\n", speedTurn);
    motion(lastState);
    lastCmd = millis();
    return;

  case '4':
    speedTurn += 10;
    if (speedTurn > 255)
      speedTurn = 255;
    Serial.printf("speedTurn = %d\n", speedTurn);
    motion(lastState);
    lastCmd = millis();
    return;

  default:
    return;
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 40)
  {
    delay(500);
    t++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setSleep(false);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  ledcSetup(CH1A, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN1A, CH1A);
  ledcSetup(CH1B, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN1B, CH1B);
  ledcSetup(CH2A, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN2A, CH2A);
  ledcSetup(CH2B, PWM_FREQ, PWM_RES);
  ledcAttachPin(IN2B, CH2B);
  stopAll();

  connectWiFi();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  Serial.printf("trim2 = %.2f  speedTurn = %d\n", trim2, speedTurn);

  udp.begin(UDP_PORT);
  lastCmd = millis();
}

void loop()
{
  int n = udp.parsePacket();
  if (n > 0)
  {
    char buf[8];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0)
    {
      buf[len] = 0;
      applyCommand(buf[0]);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(buf[0]);
      udp.endPacket();
    }
  }

  if (millis() - lastCmd > TIMEOUT && lastState != 'x')
  {
    stopAll();
    lastState = 'x';
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    stopAll();
    lastState = 'x';
    connectWiFi();
    udp.begin(UDP_PORT);
  }
}