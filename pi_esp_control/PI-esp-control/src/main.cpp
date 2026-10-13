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
const bool LOAD_SHARE = false;           // ปิดได้ถ้าบั๊กจ่ายไหวสบาย ๆ
const uint8_t TT_DUTY_ARM_MOVE = 255;    // duty แปรงตอนแขนขยับ
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
const int IN1A = 42, IN1B = 41; // ล้อซ้าย
const int IN2A = 40, IN2B = 39; // ล้อขวา

// ---- TT motor (DRV8833) — STBY ผูก 3.3V ไว้แล้ว ----
const int TT_AIN1 = 15, TT_AIN2 = 16;
const int TT_BIN1 = 17, TT_BIN2 = 18;

// ---- LEDC channel ----
const int CH1A = 0, CH1B = 1, CH2A = 2, CH2B = 3;
const int CH_TA1 = 4, CH_TA2 = 5;
const int CH_TB1 = 6, CH_TB2 = 7;

const int PWM_FREQ = 20000;   // ล้อ JGB
const int TT_PWM_FREQ = 1000; // แปรง
const int PWM_RES = 8;        // duty 0-255
const bool TT_SLOW_DECAY = true;

// ================= PWM: รองรับ arduino-esp32 ทั้ง 2.x และ 3.x =================
// core 3.x ตัด ledcSetup()/ledcAttachPin() ออก เหลือ ledcAttach(pin,freq,res)
// และ ledcWrite() เปลี่ยนไปรับ "ขา" แทน "ช่อง" — ห่อไว้ที่เดียวจะได้ไม่ต้อง
// ล็อกเวอร์ชัน core และโค้ดข้างล่างเขียนแบบเดิมได้หมด
static uint8_t pwmPin[8]; // ช่อง -> ขา

static inline void pwmSetup(uint8_t ch, uint8_t pin, int freq = PWM_FREQ)
{
  pwmPin[ch] = pin;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, PWM_RES);
#else
  ledcSetup(ch, freq, PWM_RES);
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
// ★★ ตั้งค่าเดียวพอ: แรงดันรางจริงที่จ่ายให้ MX1508 ★★
//    เพดาน duty จะคำนวณเองให้กระแสสตอลไม่เกินที่ไดรเวอร์รับไหว
const float TT_RAIL_V = 5.0;

// TT เกียร์ 1:48 สตอลที่ 6V = 1.5A  กระแสแปรผันตรงกับแรงดัน
// MX1508 จ่ายได้ 1.5A/ช่อง — กันไว้ 15% เหลือใช้จริง 1.275A
const float TT_STALL_6V = 1.5;
const float TT_DRIVER_MAX = 1.275;

// กระแสสตอลถ้าปล่อย duty เต็มบนรางนี้
const float TT_STALL_RAIL = TT_STALL_6V * TT_RAIL_V / 6.0;
// duty สูงสุดที่ยังไม่เกินเพดานไดรเวอร์ (ราง 5V -> 255 / ราง 6V -> 217)
const int TT_MAX = (TT_STALL_RAIL <= TT_DRIVER_MAX)
                       ? 255
                       : (int)(255.0 * TT_DRIVER_MAX / TT_STALL_RAIL + 0.5);

// ความเร็วแปรงตอนเปิดเครื่อง — ไม่เอาเพดานเต็มเพราะยังไม่รู้ว่าแปรง
// ติดขยะหรือกินลึกเกินไหม เริ่มกลาง ๆ แล้วกด + ขึ้นเองได้ถึง TT_MAX
const int TT_DUTY_START = 210;
// ยังเคารพเพดานปลอดภัยอยู่ — ถ้าวันหลังเปลี่ยนราง TT_RAIL_V เป็น 6V
// TT_MAX จะเหลือ 217 ส่วนนี้ก็ยังไม่เกิน แต่ถ้าตั้ง START สูงกว่าเพดานก็ถูกตัด
int ttDuty = (TT_DUTY_START < TT_MAX) ? TT_DUTY_START : TT_MAX;
const int TT_MIN = 80; // ต่ำกว่านี้มอเตอร์ไม่หมุน แค่หวีดและร้อน
const int TT_STEP = 15;

// ---- ทิศหมุนของแปรงแต่ละตัว ----
// แปรงซ้าย/ขวาต้องหมุนสวนทางกันถึงจะปัดขยะเข้ากลาง
// กด 'r' ตอนรันอยู่เพื่อวนลองทั้ง 4 แบบ ไม่ต้องแฟลชใหม่
// เจอแบบที่ใช่แล้วค่อยมาแก้ค่าเริ่มต้นตรงนี้ให้ตรงกัน
int ttDirA = 1, ttDirB = -1;
uint8_t ttDirMode = 2; // 0:++  1:-+  2:+-  3:--

// ---- ปรับสมดุลความเร็วแปรง ----
// TT เกียร์รุ่นนี้ความเร็วต่างกันเองได้ถึง +-20%
// กด ',' = หรี่ A   กด '.' = หรี่ B  (หรี่ได้ทีละตัว)
float ttTrimA = 1.00f, ttTrimB = 1.00f;
const float TT_TRIM_MIN = 0.50f, TT_TRIM_STEP = 0.05f;
bool ttRunning = false;

// ---- failsafe ----
const unsigned long TIMEOUT = 400;
unsigned long lastCmd = 0;

char lastState = 'x';

// ---- ป้ายบอกรุ่นเฟิร์มแวร์ — ส่งตอนบูต และตอบเมื่อได้รับ '?' ----
// ใช้เช็กว่าแฟลชตัวใหม่ติดจริงไหม:  ตัวเก่าจะเงียบสนิทเมื่อส่ง '?'
const char *FW_BANNER = "# robot fw v8 | rx: #M,#T,#S,#H | keys: k r , . p | tt slow-decay + dir/trim | telemetry 15 fields";

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
// แปรงใช้ slow decay — IN1 ค้าง HIGH, IN2 เป็น PWM กลับด้าน
//   IN2 LOW  = จ่ายไฟเข้ามอเตอร์
//   IN2 HIGH = ขดลวดถูกลัดวงจรเข้าหากัน กระแสวนต่อ ไม่ดับ
// ต่างจาก driveMotor() ที่ใช้กับล้อ ซึ่งปล่อยลอยฟรีตอน PWM ต่ำ (fast decay)
// duty บวก = หมุนหน้า, ลบ = หมุนกลับ
void driveMotorTT(int chA, int chB, int duty)
{
  duty = constrain(duty, -255, 255);
  if (duty == 0)
  {
    pwmWrite(chA, 0); // สั่งหยุด = ปล่อยลอยฟรี ไม่ใช่เบรกค้าง
    pwmWrite(chB, 0);
    return;
  }
  if (!TT_SLOW_DECAY)
  {
    driveMotor(chA, chB, duty);
    return;
  }
  if (duty > 0)
  {
    pwmWrite(chA, 255); // IN1 ค้าง HIGH
    pwmWrite(chB, 255 - duty);
  }
  else
  {
    pwmWrite(chB, 255); // IN2 ค้าง HIGH — กลับทิศ
    pwmWrite(chA, 255 + duty);
  }
}

void ttWrite(int duty)
{
  int a = (int)(duty * ttTrimA + 0.5f);
  int b = (int)(duty * ttTrimB + 0.5f);
  if (duty > 0) // อย่าหรี่จนต่ำกว่าจุดที่มอเตอร์ยังหมุนได้
  {
    if (a < TT_MIN)
      a = TT_MIN;
    if (b < TT_MIN)
      b = TT_MIN;
  }
  driveMotorTT(CH_TA1, CH_TA2, a * ttDirA);
  driveMotorTT(CH_TB1, CH_TB2, b * ttDirB);
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
// duty ที่ "จ่ายออกจริง" — ต่างจาก ttDuty ที่เป็นแค่ค่าที่ตั้งไว้
// ตอน LOAD_SHARE หรี่แปรงอยู่ สองค่านี้ไม่เท่ากัน ถ้ารายงานแต่ ttDuty
// หน้าเว็บจะขึ้น 255 ทั้งที่มอเตอร์ได้จริงแค่ 110 แล้วไล่ปัญหาไม่เจอ
int ttEffective()
{
  if (!ttRunning)
    return 0;
  return armBusy() ? min((int)ttDuty, (int)TT_DUTY_ARM_MOVE) : ttDuty;
}

// S,<cmd>,<fwd>,<turn>,<ratio>,<trim1>,<trim2>,<tt>,<jog>,<ttDuty>,<curL>,<curR>,<pan>,<tilt>,<ttOut>
void sendStatus(char c)
{
  int ratio = speedFwd ? (speedTurn * 100 / speedFwd) : 0;
  Serial.printf("S,%c,%d,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d\n",
                c, speedFwd, speedTurn, ratio,
                trim1, trim2, ttRunning ? 1 : 0, jogMode ? 1 : 0,
                ttDuty, curL, curR, panDeg, tiltDeg, ttEffective());
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

// ================= ทดสอบขาล้อทีละขา =================
// ไล่จ่ายไฟเต็มให้ขาล้อทีละขา ขาละ 700ms เพื่อแยกว่า
// "เฟิร์มแวร์ไม่ส่งสัญญาณ" หรือ "สายไม่ถึง/ไดรเวอร์ไม่ทำงาน"
// เอามิเตอร์จิ้มที่ขา IN ของ MDD3A ระหว่างทดสอบ ต้องเห็น ~3.3V ตอนถึงคิวขานั้น
// ★ บล็อก loop ราว 4 วินาที telemetry จะหยุดชั่วคราว เป็นเรื่องปกติ
void wheelPinTest()
{
  stopAll();
  curL = curR = 0;
  applyWheels();
  const char *names[4] = {"M1A ล้อซ้าย-เดินหน้า", "M1B ล้อซ้าย-ถอยหลัง",
                          "M2A ล้อขวา-เดินหน้า", "M2B ล้อขวา-ถอยหลัง"};
  const int chs[4] = {CH1A, CH1B, CH2A, CH2B};
  const int pins[4] = {IN1A, IN1B, IN2A, IN2B};
  Serial.println("# ---- ทดสอบขาล้อ: จ่ายเต็มทีละขา ขาละ 700ms ----");
  for (int i = 0; i < 4; i++)
  {
    Serial.printf("# -> %s   GPIO %d\n", names[i], pins[i]);
    pwmWrite(chs[i], 255);
    delay(700);
    pwmWrite(chs[i], 0);
    delay(250);
  }
  applyWheels();
  Serial.println("# ---- ทดสอบขาล้อจบ ----");
  lastCmd = millis();
  lastState = 'x';
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

  case 'r': // วนทิศหมุนแปรง 4 แบบ — หาว่าแบบไหนปัดขยะเข้ากลาง
    ttDirMode = (ttDirMode + 1) & 3;
    ttDirA = (ttDirMode & 1) ? -1 : 1;
    ttDirB = (ttDirMode & 2) ? -1 : 1;
    ttApply();
    Serial.printf("# ทิศแปรง: A=%s  B=%s\n",
                  ttDirA > 0 ? "หน้า" : "กลับ", ttDirB > 0 ? "หน้า" : "กลับ");
    break;

  case ',': // หรี่แปรง A (คืน B ให้เต็มก่อนถ้าเคยหรี่ไว้)
    if (ttTrimB < 1.00f)
      ttTrimB = (ttTrimB + TT_TRIM_STEP > 1.00f) ? 1.00f : ttTrimB + TT_TRIM_STEP;
    else if (ttTrimA - TT_TRIM_STEP >= TT_TRIM_MIN)
      ttTrimA -= TT_TRIM_STEP;
    ttApply();
    Serial.printf("# สมดุลแปรง: A=%.2f  B=%.2f\n", ttTrimA, ttTrimB);
    break;

  case '.': // หรี่แปรง B
    if (ttTrimA < 1.00f)
      ttTrimA = (ttTrimA + TT_TRIM_STEP > 1.00f) ? 1.00f : ttTrimA + TT_TRIM_STEP;
    else if (ttTrimB - TT_TRIM_STEP >= TT_TRIM_MIN)
      ttTrimB -= TT_TRIM_STEP;
    ttApply();
    Serial.printf("# สมดุลแปรง: A=%.2f  B=%.2f\n", ttTrimA, ttTrimB);
    break;

  case 'p': // ทดสอบขาล้อทีละขา
    wheelPinTest();
    return;

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

  pwmSetup(CH_TA1, TT_AIN1, TT_PWM_FREQ);
  pwmSetup(CH_TA2, TT_AIN2, TT_PWM_FREQ);
  pwmSetup(CH_TB1, TT_BIN1, TT_PWM_FREQ);
  pwmSetup(CH_TB2, TT_BIN2, TT_PWM_FREQ);

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