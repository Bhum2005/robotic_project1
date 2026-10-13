#include <Arduino.h>
#include <Wire.h>

// ---- แขนกล้อง: servo บน PCA9685 ----
const int I2C_SDA = 8; // ★ ค่า default ของ ESP32-S3 DevKit — แก้ถ้าต่อขาอื่น
const int I2C_SCL = 9;
const int PAN_CH = 7;  // ช่อง 7 = หันซ้าย/ขวา
const int TILT_CH = 4; // ช่อง 4 = ก้ม/เงย
const int SERVO_FREQ = 50;
const int PULSE_MIN = 500, PULSE_MAX = 2500; // us ที่ 0 และ 180 องศา
// ลิมิตเชิงกล — เปิดเต็ม 0-180 ตามที่ servo ทำได้
// ★ ถ้าแขนชนโครงหุ่นก่อนถึงสุด ให้บีบตรงนี้ แล้วแก้ใน server.py ให้ตรงกันด้วย
const int PAN_MIN = 0, PAN_MAX = 180, PAN_HOME = 90;
const int TILT_MIN = 0, TILT_MAX = 180, TILT_HOME = 90;

// ---- soft start: กันกระแสกระชากตอนเปิดเครื่องจนบั๊กตก ----
// ตอนบูต servo ยังไม่รู้ว่าตัวเองอยู่มุมไหน พอได้พัลส์แรกมันจะวิ่งไปหาตำแหน่ง
// ที่สั่งด้วยความเร็วสูงสุด สองตัวพร้อมกันกินได้ถึง 5-6A ชั่วขณะ
const unsigned long SERVO_BOOT_DELAY = 900; // ปล่อยให้ servo อ่อนตัว รอรางไฟนิ่ง
const unsigned long SERVO_STAGGER = 800;    // เว้นระยะก่อนปลุกตัวที่สอง
const int SERVO_SLEW_DEG = 3;               // ขยับได้ไม่เกินกี่องศาต่อรอบ 20ms
const unsigned long SERVO_TICK_MS = 20;     // (3° ต่อ 20ms = 150°/วินาที)
// เลื่อนเฟสพัลส์ของ tilt ไปครึ่งเฟรม พัลส์สองตัวจะไม่ทับกัน
// กระแสพีคบนรางเลยไม่บวกกัน — เป็นของที่ PCA9685 ทำได้เพราะเฟสแยกรายช่อง
const uint16_t PHASE_PAN = 0, PHASE_TILT = 2048;

// ---- แบ่งกระแสบนราง 6V: อย่าให้แปรงกับแขนพีคพร้อมกัน ----
// servo ตอนขยับกินหลักแอมป์ ส่วน TT motor ก็กินอีก ถ้าบั๊กจ่ายไม่ไหวรางจะตก
// แล้ว servo จะสั่นหาตำแหน่ง ยิ่งสั่นยิ่งกินไฟ กลายเป็นวงจรป้อนกลับ
// วิธีแก้ที่ไม่ต้องซื้ออะไร: หรี่แปรงลงชั่วคราวเฉพาะตอนแขนกำลังขยับ
const bool LOAD_SHARE = true;            // ปิดได้ถ้าบั๊กจ่ายไหวสบาย ๆ
const uint8_t TT_DUTY_ARM_MOVE = 110;    // duty แปรงตอนแขนขยับ
const unsigned long ARM_SETTLE_MS = 350; // แขนหยุดแล้วรออีกเท่านี้ค่อยคืนค่า

// ---- ขยับ servo ทีละตัว: ตัดกระแสพีคลงครึ่ง ----
// โยกสติ๊กเฉียงจะสั่งทั้ง pan และ tilt พร้อมกัน สอง servo ออกตัวชนกัน
// พีคเลยบวกกันเป็นสองเท่า (ติดขัดพร้อมกันได้ถึง ~4A บนราง 5V)
// แก้โดยให้ "ถือคิว" ทีละตัว ตัวที่ถือคิวจะขยับจนถึงเป้า แล้วปล่อยให้อีกตัว
// แต่ถ้าถือนานเกิน ARM_SLICE_MS แล้วอีกฝ่ายรออยู่ ก็บังคับสลับ
// (ไม่งั้นตอนโยกสติ๊กค้าง เป้าของ pan ขยับตลอด tilt จะไม่ได้คิวเลย)
const bool ARM_ONE_AT_A_TIME = true;
const unsigned long ARM_SLICE_MS = 250; // ที่ 150°/วิ = ได้ราว 37° ต่อคิว

// ---- ล้อขับเคลื่อน JGB (Cytron MDD3A) ----
const int IN1A = 15, IN1B = 16; // ล้อซ้าย
const int IN2A = 17, IN2B = 18; // ล้อขวา

// ---- TT motor (DRV8833) — STBY ผูก 3.3V ไว้แล้ว ----
const int TT_AIN1 = 5, TT_AIN2 = 4;
const int TT_BIN1 = 7, TT_BIN2 = 6;

// ---- LEDC channel ----
const int CH1A = 0, CH1B = 1, CH2A = 2, CH2B = 3;
const int CH_TA1 = 4, CH_TA2 = 5;
const int CH_TB1 = 6, CH_TB2 = 7;

const int PWM_FREQ = 20000;
const int PWM_RES = 8; // duty 0-255

// ================= PWM: รองรับ arduino-esp32 ทั้ง 2.x และ 3.x =================
// core 3.x ตัด ledcSetup()/ledcAttachPin() ออก เหลือ ledcAttach(pin,freq,res)
// และ ledcWrite() เปลี่ยนไปรับ "ขา" แทน "ช่อง" — ห่อไว้ที่เดียวจะได้ไม่ต้อง
// ล็อกเวอร์ชัน core และโค้ดข้างล่างเขียนแบบเดิมได้หมด
static uint8_t pwmPin[8]; // ช่อง -> ขา

static inline void pwmSetup(uint8_t ch, uint8_t pin)
{
  pwmPin[ch] = pin;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
#endif
}

static inline void pwmWrite(uint8_t ch, int duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwmPin[ch], duty);
#else
  ledcWrite(ch, duty);
#endif
}

// ---- ความเร็วล้อ ----
int speedFwd = 200;
const int FWD_MIN = 80, FWD_MAX = 255, FWD_STEP = 15;

// ---- ความเร็วเลี้ยว ----
int speedTurn = 60;
const int TURN_MIN = 20, TURN_MAX = 255, TURN_STEP = 5;

// ---- ปรับสมดุลล้อ ----
float trim1 = 1.00; // ล้อซ้าย
float trim2 = 0.96; // ล้อขวา

// ================= แก้ทิศทางล้อโดยไม่ต้องรื้อสาย =================
// แก้ตรงนี้ที่เดียว มีผลกับทุกทาง — #M จากสติ๊ก/เว็บ, ปุ่ม w a s d, trim
//
//   เดินหน้าถูก แต่เลี้ยวสลับข้าง     -> SWAP_LR = true
//   สั่งเดินหน้าแล้วหมุนอยู่กับที่     -> INVERT ข้างที่หมุนผิด เป็น true
//   สั่งเดินหน้าแล้วถอยหลังทั้งคู่     -> INVERT ทั้งสองข้าง เป็น true
// ★ ตอนนี้การสลับซ้าย/ขวาทำอยู่ที่ server.py (SWAP_LR = True) แล้ว
//   ตรงนี้จึงต้องเป็น false ไม่งั้นจะสลับสองรอบ = กลับไปผิดเหมือนเดิม
//   จะย้ายมาทำฝั่งนี้ก็ได้ แต่ต้องปิดของอีกฝั่งเสมอ ให้มีที่เดียวเท่านั้น
const bool SWAP_LR = false;  // ช่อง M1 ต่อกับล้อขวา / M2 ต่อกับล้อซ้าย
const bool INVERT_L = false; // กลับทิศล้อซ้าย
const bool INVERT_R = false; // กลับทิศล้อขวา

// ================= กันกระดก =================
const int RAMP_ACCEL = 12;               // ขั้นตอนเร่ง ต่อรอบ ramp
const int RAMP_DECEL = 25;               // ขั้นตอนชะลอ (ชะลอเร็วกว่าเร่งได้)
const unsigned long RAMP_MS = 20;        // คาบ ramp
const unsigned long REVERSE_DWELL = 180; // หยุดนิ่งก่อนกลับทิศ (ms)

int curL = 0, curR = 0; // ความเร็วจริงที่ส่งออก
int tgtL = 0, tgtR = 0; // ความเร็วเป้าหมาย
unsigned long lastRamp = 0;
unsigned long dwellUntil = 0; // ห้ามออกตัวจนกว่าจะถึงเวลานี้

// ================= jog mode =================
bool jogMode = true;              // true = กดครั้งเดียวเดินระยะสั้น
const unsigned long JOG_MS = 350; // ระยะเวลาเดินต่อการกด 1 ครั้ง
unsigned long jogUntil = 0;

// ================= TT motor =================
// *** ส่วนที่เพิ่มใหม่: ปรับความเร็วได้ ***
int ttDuty = 180;      // ความเร็วปัจจุบัน (0-255) เริ่มที่เพดานปลอดภัย = TT_MAX
const int TT_MIN = 80; // ต่ำกว่านี้มอเตอร์ไม่หมุน แค่หวีดและร้อน
// ★ ค่านี้ขึ้นกับแรงดันของราง — MX1508 จ่ายได้ 1.5A ต่อช่อง
//   ราง 6V -> ใช้ 180  (กระแสสตอล ~1.06 A/มอเตอร์ เหลือ margin 29%)
//   ราง 5V -> ใช้ 255  (กระแสสตอล ~1.25 A/มอเตอร์ เหลือ margin 17% และแปรงได้ไฟเต็ม 5V)
const int TT_MAX = 180;
const int TT_STEP = 15;
bool ttRunning = false;

// ---- failsafe ----
const unsigned long TIMEOUT = 400;
unsigned long lastCmd = 0;

char lastState = 'x';

// ---- ป้ายบอกรุ่นเฟิร์มแวร์ — ส่งตอนบูต และตอบเมื่อได้รับ '?' ----
// ใช้เช็กว่าแฟลชตัวใหม่ติดจริงไหม:  ตัวเก่าจะเงียบสนิทเมื่อส่ง '?'
const char *FW_BANNER = "# robot fw v5 | rx: #M,#T,#S,#H | servo soft-start + one-at-a-time | telemetry 14 fields";

// ================= PCA9685 (ตัวขับ servo) =================
// เขียนเองสั้น ๆ ไม่ต้องลงไลบรารีเพิ่ม และถ้าไม่มีบอร์ดต่ออยู่
// ทุกอย่างที่เหลือ (ล้อ/แปรง) ยังทำงานได้ตามปกติ
const uint8_t PCA_ADDR = 0x40;
const uint8_t PCA_MODE1 = 0x00, PCA_PRESCALE = 0xFE, PCA_LED0_ON_L = 0x06;
bool pcaOK = false;

void pcaWrite(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool pcaBegin(int freq)
{
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0)
    return false; // ไม่มีบอร์ดอยู่บนบัส I2C
  pcaWrite(PCA_MODE1, 0x00);
  delay(5);
  // prescale = round(25MHz / (4096 * freq)) - 1
  uint8_t pre = (uint8_t)(25000000.0 / (4096.0 * freq) + 0.5) - 1;
  pcaWrite(PCA_MODE1, 0x10); // ต้อง sleep ก่อนถึงจะตั้ง prescale ได้
  pcaWrite(PCA_PRESCALE, pre);
  pcaWrite(PCA_MODE1, 0x00);
  delay(5);
  pcaWrite(PCA_MODE1, 0xA0); // auto-increment + restart
  return true;
}

void pcaSet(uint8_t ch, uint16_t on, uint16_t off)
{
  if (!pcaOK)
    return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_LED0_ON_L + 4 * ch);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

// ปิดสัญญาณช่องนั้นสนิท (บิต full-OFF) — servo จะอ่อนตัว กินไฟเกือบศูนย์
void pcaOff(uint8_t ch)
{
  pcaSet(ch, 0, 0x1000);
}

// ================= แขนกล้อง =================
int panTgt = PAN_HOME, tiltTgt = TILT_HOME; // มุมที่สั่ง
int panDeg = PAN_HOME, tiltDeg = TILT_HOME; // มุมที่จ่ายออกจริง (ไล่เข้าหาเป้า)
bool panLive = false, tiltLive = false;     // ปล่อยพัลส์ให้ช่องนี้แล้วหรือยัง
uint8_t armBoot = 0;                        // ขั้นของ soft start
unsigned long armBootAt = 0, lastServoTick = 0;
unsigned long armMovedAt = 0;    // ครั้งล่าสุดที่ servo ขยับจริง
uint8_t armOwner = 0;            // ใครถือคิวขยับ: 0=ว่าง 1=pan 2=tilt
unsigned long armOwnerSince = 0; // ถือคิวมาตั้งแต่เมื่อไหร่
bool ttDimmed = false;           // ตอนนี้หรี่แปรงอยู่หรือเปล่า

bool armBusy()
{
  return LOAD_SHARE && armMovedAt && (millis() - armMovedAt < ARM_SETTLE_MS);
}

void servoWrite(uint8_t ch, int deg, uint16_t phase)
{
  long us = map(deg, 0, 180, PULSE_MIN, PULSE_MAX);
  uint16_t width = (uint16_t)(us * 4096L / (1000000L / SERVO_FREQ));
  pcaSet(ch, phase, (uint16_t)((phase + width) & 0x0FFF)); // ล้นเฟรมได้ ชิปรองรับ
}

// รับคำสั่ง = ตั้งเป้าหมายเท่านั้น การขยับจริงถูกจำกัดความเร็วใน updateArm()
void armSet(int p, int t)
{
  panTgt = constrain(p, PAN_MIN, PAN_MAX);
  tiltTgt = constrain(t, TILT_MIN, TILT_MAX);
}

int stepTo(int cur, int tgt, int step)
{
  if (cur < tgt)
    return min(cur + step, tgt);
  if (cur > tgt)
    return max(cur - step, tgt);
  return cur;
}

// ไล่มุมเข้าหาเป้าอย่างช้า ๆ ทุกทางที่สั่งเข้ามาจึงถูกจำกัดความเร็วเหมือนกันหมด
// ไม่ว่าจะมาจากสติ๊ก สไลเดอร์ในเว็บ หรือ #H
void updateArm()
{
  if (millis() - lastServoTick < SERVO_TICK_MS)
    return;
  lastServoTick = millis();

  bool panWants = panLive && panDeg != panTgt;
  bool tiltWants = tiltLive && tiltDeg != tiltTgt;

  if (ARM_ONE_AT_A_TIME)
  {
    // เจ้าของคิวถึงเป้าแล้ว -> ปล่อยคิว
    if ((armOwner == 1 && !panWants) || (armOwner == 2 && !tiltWants))
      armOwner = 0;

    // ถือคิวนานเกินสไลซ์ และอีกฝ่ายรออยู่ -> บังคับสลับ
    if (armOwner && millis() - armOwnerSince >= ARM_SLICE_MS)
    {
      if (armOwner == 1 && tiltWants)
      {
        armOwner = 2;
        armOwnerSince = millis();
      }
      else if (armOwner == 2 && panWants)
      {
        armOwner = 1;
        armOwnerSince = millis();
      }
    }

    // คิวว่าง -> ยกให้คนที่รออยู่ (pan ได้สิทธิ์ก่อน)
    if (!armOwner)
    {
      if (panWants)
      {
        armOwner = 1;
        armOwnerSince = millis();
      }
      else if (tiltWants)
      {
        armOwner = 2;
        armOwnerSince = millis();
      }
    }

    // เหลือแค่เจ้าของคิวที่ได้ขยับรอบนี้
    panWants = panWants && armOwner == 1;
    tiltWants = tiltWants && armOwner == 2;
  }

  if (panWants)
  {
    panDeg = stepTo(panDeg, panTgt, SERVO_SLEW_DEG);
    armMovedAt = millis();
    servoWrite(PAN_CH, panDeg, PHASE_PAN);
  }
  if (tiltWants)
  {
    tiltDeg = stepTo(tiltDeg, tiltTgt, SERVO_SLEW_DEG);
    armMovedAt = millis();
    servoWrite(TILT_CH, tiltDeg, PHASE_TILT);
  }
}

// soft start — ปลุก servo ทีละตัว ไม่บล็อก loop
void updateArmBoot()
{
  if (!pcaOK || armBoot >= 3)
    return;
  if (millis() - armBootAt < (armBoot == 0 ? SERVO_BOOT_DELAY : SERVO_STAGGER))
    return;
  armBootAt = millis();

  if (armBoot == 0)
  {
    panLive = true; // ปลุกตัวหัน
    servoWrite(PAN_CH, panDeg, PHASE_PAN);
  }
  else if (armBoot == 1)
  {
    tiltLive = true; // แล้วค่อยตัวก้มเงย
    servoWrite(TILT_CH, tiltDeg, PHASE_TILT);
  }
  else
    Serial.println("# แขนกล้องพร้อม (soft start เสร็จ)");
  armBoot++;
}

// ---- telemetry ----
const unsigned long TELEM_MS = 100; // รายงานสถานะทุก 100ms
unsigned long lastTelem = 0;

// ---- บัฟเฟอร์คำสั่งแบบบรรทัด (#M / #T / #S / #H) ----
char lineBuf[40];
uint8_t lineLen = 0;
bool inLine = false;

// ================= มอเตอร์พื้นฐาน =================
void driveMotor(int chA, int chB, int speed)
{
  speed = constrain(speed, -255, 255);
  if (speed >= 0)
  {
    pwmWrite(chA, speed);
    pwmWrite(chB, 0);
  }
  else
  {
    pwmWrite(chA, 0);
    pwmWrite(chB, -speed);
  }
}

// ส่งค่าจริงออกมอเตอร์ (เรียกจาก ramp เท่านั้น)
// curL/curR เป็น "ล้อซ้าย/ขวาตามตรรกะ" ส่วนจะออกช่องไหนของ MDD3A
// ให้ SWAP_LR / INVERT_* ข้างบนเป็นตัวตัดสิน
void applyWheels()
{
  int l = (int)(curL * trim1);
  int r = (int)(curR * trim2);
  if (INVERT_L)
    l = -l;
  if (INVERT_R)
    r = -r;

  if (SWAP_LR)
  {
    driveMotor(CH1A, CH1B, r); // ช่อง M1 -> ล้อขวา
    driveMotor(CH2A, CH2B, l); // ช่อง M2 -> ล้อซ้าย
  }
  else
  {
    driveMotor(CH1A, CH1B, l); // ช่อง M1 -> ล้อซ้าย
    driveMotor(CH2A, CH2B, r); // ช่อง M2 -> ล้อขวา
  }
}

// ================= ตั้งเป้าหมาย =================
void drive(int left, int right)
{
  tgtL = left;
  tgtR = right;
  // การกลับทิศจัดการใน updateRamp() (บังคับผ่าน 0 + dwell)
}

void stopAll()
{
  tgtL = 0;
  tgtR = 0;
}

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
  case 'a': // เลี้ยวซ้าย: ล้อซ้ายถอย ล้อขวาเดินหน้า
    drive(-speedTurn, speedTurn);
    break;
  case 'd': // เลี้ยวขวา: ล้อซ้ายเดินหน้า ล้อขวาถอย
    drive(speedTurn, -speedTurn);
    break;
  case 'M':
    break; // โหมดอนาล็อก — เป้าหมายถูกตั้งจาก #M แล้ว ห้ามทับ
  default:
    stopAll();
    break;
  }
}

// ================= ramp =================
int stepToward(int cur, int tgt)
{
  int step = (abs(tgt) > abs(cur)) ? RAMP_ACCEL : RAMP_DECEL;
  if (cur < tgt)
    return min(cur + step, tgt);
  if (cur > tgt)
    return max(cur - step, tgt);
  return cur;
}

void updateRamp()
{
  if (millis() - lastRamp < RAMP_MS)
    return;
  lastRamp = millis();

  // เป้าหมายสวนทางกับปัจจุบัน -> ต้องผ่าน 0 ก่อน
  int wantL = tgtL, wantR = tgtR;
  bool flipL = (tgtL > 0 && curL < 0) || (tgtL < 0 && curL > 0);
  bool flipR = (tgtR > 0 && curR < 0) || (tgtR < 0 && curR > 0);

  if (flipL || flipR)
  {
    wantL = 0;
    wantR = 0;
  }

  // อยู่ในช่วง dwell -> ค้างที่ 0
  if (curL == 0 && curR == 0)
  {
    if (flipL || flipR)
    {
      if (dwellUntil == 0)
        dwellUntil = millis() + REVERSE_DWELL;
    }
    if (dwellUntil && millis() < dwellUntil)
    {
      wantL = 0;
      wantR = 0;
    }
    else
      dwellUntil = 0;
  }

  int nL = stepToward(curL, wantL);
  int nR = stepToward(curR, wantR);

  if (nL != curL || nR != curR)
  {
    curL = nL;
    curR = nR;
    applyWheels();
  }
}

// ================= TT motor =================
void ttWrite(int duty)
{
  driveMotor(CH_TA1, CH_TA2, duty);
  driveMotor(CH_TB1, CH_TB2, duty);
}

// ส่งค่าปัจจุบันออกไป (เรียกทุกครั้งที่เปลี่ยนความเร็วหรือ on/off)
void ttApply()
{
  if (!ttRunning)
  {
    ttWrite(0);
    return;
  }
  ttDimmed = armBusy();
  ttWrite(ttDimmed ? min((int)ttDuty, (int)TT_DUTY_ARM_MOVE) : ttDuty);
}

void ttSet(bool on)
{
  ttRunning = on;
  ttApply();
}

// ================= telemetry =================
// S,<cmd>,<fwd>,<turn>,<ratio>,<trim1>,<trim2>,<tt>,<jog>,<ttDuty>,<curL>,<curR>,<pan>,<tilt>
void sendStatus(char c)
{
  int ratio = speedFwd ? (speedTurn * 100 / speedFwd) : 0;
  Serial.printf("S,%c,%d,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d\n",
                c, speedFwd, speedTurn, ratio,
                trim1, trim2, ttRunning ? 1 : 0, jogMode ? 1 : 0,
                ttDuty, curL, curR, panDeg, tiltDeg);
  lastTelem = millis();
}

// ================= คำสั่งแบบบรรทัด =================
// #M,<left>,<right>   ขับแบบอนาล็อก -255..255   (ทางหลักที่ Pi ใช้)
// #S,<pan>,<tilt>     ตั้งมุมแขนกล้องเป็นองศา
// #T,<duty>           ตั้งความเร็วแปรงตรง ๆ 0..255
// #H                  แขนกลับตำแหน่งกลาง
static bool parse2(char *s, int &a, int &b)
{
  char *c1 = strchr(s, ',');
  if (!c1)
    return false;
  char *c2 = strchr(c1 + 1, ',');
  if (!c2)
    return false;
  *c2 = '\0';
  a = atoi(c1 + 1);
  b = atoi(c2 + 1);
  return true;
}

void applyLine(char *s)
{
  int a, b;
  if (s[1] == 'M')
  {
    if (!parse2(s, a, b))
      return;
    int l = constrain(a, -255, 255);
    int r = constrain(b, -255, 255);
    drive(l, r);
    lastState = (l || r) ? 'M' : 'x';
    lastCmd = millis();
    jogUntil = 0; // สตรีมมาต่อเนื่องอยู่แล้ว ไม่ต้องใช้ jog timer
  }
  else if (s[1] == 'S') // แขนกล้อง — ไม่ผูกกับ failsafe ปล่อยมือแล้วค้างที่เดิม
  {
    if (!parse2(s, a, b))
      return;
    armSet(a, b);
  }
  else if (s[1] == 'H')
  {
    armSet(PAN_HOME, TILT_HOME);
    sendStatus('H');
  }
  else if (s[1] == 'T')
  {
    char *c1 = strchr(s, ',');
    if (!c1)
      return;
    ttDuty = constrain(atoi(c1 + 1), 0, TT_MAX);
    ttRunning = ttDuty > 0;
    ttApply();
    sendStatus('T');
  }
}

// ================= คำสั่ง =================
void applyCommand(char c)
{
  switch (c)
  {
  case 'w':
  case 's':
    motion(c);
    lastState = c;
    lastCmd = millis();
    if (jogMode)
      jogUntil = millis() + JOG_MS; // เดินแค่ช่วงสั้น
    break;

  case 'a':
  case 'd':
  case 'x':
    motion(c);
    lastState = c;
    lastCmd = millis();
    jogUntil = 0;
    break;

  case '?': // ถามรุ่นเฟิร์มแวร์ + สถานะแขน (Pi ถามตอนเปิดพอร์ต)
    Serial.println(FW_BANNER);
    Serial.println(pcaOK
                       ? (armBoot >= 3 ? "# แขนกล้องพร้อม"
                                       : "# แขนกล้องกำลัง soft start")
                       : "# ไม่พบ PCA9685 ที่ 0x40 — แขนกล้องไม่ทำงาน "
                         "(ตรวจสาย SDA/SCL และไฟเลี้ยง)");
    return;

  case 'j': // สลับ jog / continuous
    jogMode = !jogMode;
    jogUntil = 0;
    stopAll();
    lastState = 'x';
    break;

  case 'k':
    ttSet(!ttRunning);
    break;

  // *** เพิ่มใหม่: ปรับความเร็ว TT motor ***
  case '[':
    ttDuty = max(ttDuty - TT_STEP, TT_MIN);
    ttApply();
    break;

  case ']':
    ttDuty = min(ttDuty + TT_STEP, TT_MAX);
    ttApply();
    break;

  case '-':
    speedFwd = max(speedFwd - FWD_STEP, FWD_MIN);
    motion(lastState);
    lastCmd = millis();
    break;

  case '+':
    speedFwd = min(speedFwd + FWD_STEP, FWD_MAX);
    motion(lastState);
    lastCmd = millis();
    break;

  case '3':
    speedTurn = max(speedTurn - TURN_STEP, TURN_MIN);
    motion(lastState);
    lastCmd = millis();
    break;

  case '4':
    speedTurn = min(speedTurn + TURN_STEP, TURN_MAX);
    motion(lastState);
    lastCmd = millis();
    break;

  case '1':
    trim2 -= 0.02f;
    if (trim2 < 0.50f)
      trim2 = 0.50f;
    motion(lastState);
    lastCmd = millis();
    break;

  case '2':
    trim2 += 0.02f;
    if (trim2 > 1.00f)
      trim2 = 1.00f;
    motion(lastState);
    lastCmd = millis();
    break;

  default:
    return;
  }

  sendStatus(c);
}

// ================= setup =================
void setup()
{
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif

  pwmSetup(CH1A, IN1A);
  pwmSetup(CH1B, IN1B);
  pwmSetup(CH2A, IN2A);
  pwmSetup(CH2B, IN2B);

  pwmSetup(CH_TA1, TT_AIN1);
  pwmSetup(CH_TA2, TT_AIN2);
  pwmSetup(CH_TB1, TT_BIN1);
  pwmSetup(CH_TB2, TT_BIN2);

  curL = curR = tgtL = tgtR = 0;
  applyWheels();
  ttWrite(0);

  // ---- แขนกล้อง: เริ่มด้วยการ "ไม่จ่ายพัลส์เลย" ----
  // servo จะอ่อนตัวกินไฟเกือบศูนย์ แล้วค่อยปลุกทีละตัวใน updateArmBoot()
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  pcaOK = pcaBegin(SERVO_FREQ);
  if (pcaOK)
  {
    pcaOff(PAN_CH);
    pcaOff(TILT_CH);
    armBootAt = millis();
  }
  else
    Serial.println("# ไม่พบ PCA9685 ที่ 0x40 — แขนกล้องจะไม่ทำงาน "
                   "(ตรวจสาย SDA/SCL และไฟเลี้ยง) ส่วนล้อกับแปรงยังใช้ได้");

  Serial.println(FW_BANNER);
  lastCmd = millis();
}

// ================= loop =================
void loop()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '#') // เริ่มคำสั่งแบบบรรทัด
    {
      inLine = true;
      lineLen = 0;
      lineBuf[lineLen++] = c;
      continue;
    }
    if (inLine)
    {
      if (c == '\r' || c == '\n')
      {
        lineBuf[lineLen] = '\0';
        applyLine(lineBuf);
        inLine = false;
      }
      else if (lineLen < sizeof(lineBuf) - 1)
        lineBuf[lineLen++] = c;
      else
        inLine = false; // บรรทัดยาวผิดปกติ ทิ้ง
      continue;
    }

    if (c == '\r' || c == '\n')
      continue;
    applyCommand(c);
  }

  updateRamp();
  updateArmBoot();
  updateArm();

  // แขนเริ่มขยับ -> หรี่แปรง / แขนหยุดแล้ว -> คืนค่า
  if (ttRunning && ttDimmed != armBusy())
    ttApply();

  // รายงานสถานะสม่ำเสมอ ให้ฝั่ง Pi รู้ว่ายังมีชีวิต
  if (millis() - lastTelem >= TELEM_MS)
    sendStatus(lastState);

  // jog mode: หมดเวลาแล้วหยุดเอง
  if (jogMode && jogUntil && millis() > jogUntil)
  {
    stopAll();
    lastState = 'x';
    jogUntil = 0;
  }

  // failsafe: ขาดคำสั่ง -> หยุดล้อ
  if (millis() - lastCmd > TIMEOUT && lastState != 'x')
  {
    stopAll();
    lastState = 'x';
  }
}