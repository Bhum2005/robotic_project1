#!/usr/bin/env python3
"""
robot_server.py — ระบบรวมของหุ่น (รันบน Raspberry Pi 5)

  จอย USB (GameSir โหมด Xbox) ─┐
                               ├─► drive loop 20Hz ─► USB serial ─► ESP32-S3
  หน้าเว็บ                     ─┘

  กล้อง Pi ─► MJPEG ─► หน้าเว็บเดียวกัน (สตรีม + โฟกัส + ซูม + ถ่ายภาพ)

ปุ่มบนจอย
    สติ๊กซ้าย     ขับเคลื่อน (เดินหน้า / ถอย / เลี้ยว)
    สติ๊กขวา      แขนกล้อง — ซ้ายขวา = หัน, ขึ้นลง = ก้ม/เงย
    A            เปิด/ปิดแปรงกวาด
    B            หยุดล้อ
    L            แตะ = โฟกัสอัตโนมัติ / กดค้าง = สแกนหาจุดคมสุด
    R            ถ่ายรูป
    ZL / ZR      ซูมออก / ซูมเข้า          (กดค้างได้)
    D-pad ขึ้น/ลง  ความเร็วการเคลื่อนที่  เพิ่ม / ลด   (กดค้างได้)
    D-pad ซ้าย/ขวา ความเร็วแปรง  ลด / เพิ่ม

usage:
    python3 robot_server.py --pad-device /dev/input/event8
    python3 robot_server.py --no-cam          ทดสอบส่วนควบคุมโดยไม่ใช้กล้อง
    python3 robot_server.py --no-pad          ไม่ใช้จอย (คุมผ่านเว็บอย่างเดียว)
    python3 robot_server.py --pad-raw         ดูรหัสปุ่ม/แกนของจอย
    python3 robot_server.py --pad-list        ดูรายชื่ออุปกรณ์ input

ติดตั้ง:  sudo apt install -y python3-serial python3-evdev python3-pil
เปิดเว็บ: http://<pi-ip>:9000/
"""
import argparse
import io
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial

# ---------------- ตั้งค่า ----------------
SERIAL_DEV = os.environ.get("ROBOT_SERIAL", "/dev/ttyACM0")
BAUD = 115200
HTTP_PORT = 9000
PHOTO_DIR = os.path.expanduser(os.environ.get("ROBOT_PHOTOS", "~/robot_photos"))
THUMB_W = 320

SEND_HZ = 20          # ความถี่ส่ง #M (ต้องถี่กว่า failsafe 400ms ของ ESP32)
HOLD_TIMEOUT = 0.6    # ไม่มีคำสั่งใหม่เกินนี้ = ปล่อยคันเร่ง
MAX_SPEED = 255
TURN_GAIN = 0.7
DEADZONE = 0.12
EXPO = 0.45

# ================= แก้ทิศทางล้อ (ไม่ต้องแฟลชเฟิร์มแวร์ใหม่) =================
# แก้ตรงนี้ที่เดียว มีผลทั้งสติ๊กในเว็บ จอย และปุ่มลูกศร เพราะทุกทางส่งผ่าน #M
#
#   เดินหน้าถูก แต่เลี้ยวสลับข้าง   -> SWAP_LR = True
#   สั่งเดินหน้าแล้วหมุนอยู่กับที่   -> INVERT ข้างที่หมุนผิด เป็น True
#   สั่งเดินหน้าแล้วถอยหลังทั้งคู่   -> INVERT ทั้งสองข้าง เป็น True
#
# สั่งทับตอนรันได้ด้วย  --swap / --no-swap / --invert-l / --invert-r
SWAP_LR = True      # ช่อง M1 ของ MDD3A ต่อกับล้อขวา / M2 ต่อกับล้อซ้าย
INVERT_L = False    # กลับทิศล้อซ้าย
INVERT_R = False    # กลับทิศล้อขวา


def to_board(l, r):
    """ล้อซ้าย/ขวาตามตรรกะ -> ค่าที่ส่งเข้าช่อง M1,M2 ของบอร์ด"""
    if INVERT_L:
        l = -l
    if INVERT_R:
        r = -r
    return (r, l) if SWAP_LR else (l, r)


def from_board(m1, m2):
    """ค่าที่บอร์ดรายงานกลับ (ช่อง M1,M2) -> ล้อซ้าย/ขวาตามตรรกะ"""
    l, r = (m2, m1) if SWAP_LR else (m1, m2)
    if INVERT_L:
        l = -l
    if INVERT_R:
        r = -r
    return l, r


# ความเร็วสูงสุด — จอย (ZL/ZR) และสไลเดอร์ในเว็บใช้ค่าตัวเดียวกัน
SPEED_MIN, SPEED_MAX, SPEED_STEP = 60, 255, 15
SPEED_DEFAULT = 200

# แมปจอย GameSir โหมด Xbox — ยืนยันด้วย --pad-raw แล้ว
# ★ ถ้าจอยรุ่นอื่นไม่ตรง รัน --pad-raw แล้วแก้ตรงนี้
PAD_MAP = {
    "drive_x": "ABS_X",        # สติ๊กซ้าย ซ้าย/ขวา -> เลี้ยว
    "drive_y": "ABS_Y",        # สติ๊กซ้าย ขึ้น/ลง  -> เดินหน้า/ถอย
    "arm_x": "ABS_RX",         # สติ๊กขวา ซ้าย/ขวา -> แขนหัน
    "arm_y": "ABS_RY",         # สติ๊กขวา ขึ้น/ลง  -> แขนก้ม/เงย
    "tt_toggle": "BTN_SOUTH",  # A -> เปิด/ปิดแปรง
    "estop": "BTN_EAST",       # B -> หยุดล้อ
    "focus": "BTN_TL",         # L -> โฟกัสกล้อง
    "shot": "BTN_TR",          # R -> ถ่ายรูป
    "zoom_out": "ABS_Z",       # ZL -> ซูมออก (ทริกเกอร์อนาล็อก)
    "zoom_in": "ABS_RZ",       # ZR -> ซูมเข้า
    "dpad_x": "ABS_HAT0X",     # D-pad ซ้าย/ขวา -> ความเร็วแปรง
    "dpad_y": "ABS_HAT0Y",     # D-pad ขึ้น/ลง  -> ความเร็วเคลื่อนที่
}
REPEAT_EVERY = 0.22    # กดค้างแล้วซ้ำทุกกี่วินาที
REPEAT_DELAY = 0.35    # หน่วงก่อนเริ่มซ้ำครั้งแรก
TRIG_ON = 0.40         # ทริกเกอร์เกิน 40% ของช่วงที่จอยรายงาน = ถือว่ากด
ZOOM_STEP = 0.25       # ซูมทีละกี่เท่าต่อการซ้ำหนึ่งครั้ง (ละเอียดขึ้น = ลื่นขึ้น)
ZOOM_REPEAT = 0.12     # ZL/ZR กดค้างแล้วซ้ำถี่กว่าปุ่มอื่น เพื่อให้ซูมลื่น

# ---------------- กล้อง ----------------
STREAM_SIZE = (1280, 720)   # ขนาดภาพสตรีม — ใหญ่กว่านี้ fps ตกและกินแบนด์วิดท์
STILL_FULLRES = True        # ถ่ายภาพนิ่งเต็มเซนเซอร์ (ไม่ใช่เฟรม 720p จากสตรีม)
STILL_SIZE = None           # None = เต็มเซนเซอร์  หรือกำหนดเอง เช่น (4056, 3040)
SHARPNESS = 2.0             # ความคมของ ISP (1.0 = ค่าเริ่มต้น, มากไปจะเห็นขอบเทียม)
AF_WINDOW = 0.40            # กรอบโฟกัส = กี่ส่วนของภาพตรงกลาง
FOCUS_HOLD = 0.45           # กดปุ่ม L ค้างนานเท่านี้ = สแกนแทน AF ธรรมดา
# ช่วงกวาดหาโฟกัส — LensPosition หน่วยเป็น 1/เมตร (10 = 10cm, 3.33 = 30cm)
SCAN_LO, SCAN_HI = 0.5, 10.0
SCAN_COARSE, SCAN_FINE = 14, 9
SCAN_MIN_CONTRAST = 0.15    # ความคมต้องต่างกันอย่างน้อยเท่านี้ ถึงจะเชื่อผลสแกน

# ---------------- แขนกล้อง (servo บน PCA9685 ผ่าน ESP32) ----------------
# ลิมิตต้องตรงกับที่ตั้งไว้ใน main.cpp — เฟิร์มแวร์ clamp ซ้ำอีกชั้นอยู่แล้ว
# ★ ต้องตรงกับที่ตั้งใน main.cpp — เฟิร์มแวร์ clamp ซ้ำอีกชั้น ถ้าสองฝั่งไม่ตรง
#   ค่าที่แคบกว่าจะชนะ แล้วจะงงว่าทำไมหมุนไม่สุด
PAN_MIN, PAN_MAX, PAN_HOME = 0, 180, 90      # ช่อง 7 — หันซ้าย/ขวา
TILT_MIN, TILT_MAX, TILT_HOME = 0, 180, 90   # ช่อง 4 — ก้ม/เงย
ARM_RATE = 70.0          # องศาต่อวินาที เมื่อโยกสติ๊กสุด
# กลับทิศ — มีผลกับ "การโยก" ทั้งจอยและสติ๊กในเว็บ (ทำที่ Arm.nudge ที่เดียว)
# ไม่มีผลกับสไลเดอร์ เพราะสไลเดอร์สั่งเป็นมุมสัมบูรณ์ ไม่ใช่ทิศ
ARM_INVERT_PAN = True    # ถ้าโยกขวาแล้วแขนหันซ้าย ให้เป็น True
ARM_INVERT_TILT = False  # ถ้าโยกขึ้นแล้วแขนก้มลง ให้เป็น True


# ================= ลิงก์ไป ESP32 =================
class RobotLink:
    """เปิด serial ตัวเดียว ใช้ร่วมกันทุก thread"""

    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.buf = b""
        # สถานะที่ ESP32 รายงานกลับ
        self.st = {"cmd": "x", "tt": 0, "tt_pwm": 0, "fwd": 0, "turn": 0,
                   "ratio": 0, "trim1": 0.0, "trim2": 0.0, "jog": 1,
                   "curL": 0, "curR": 0, "pan": PAN_HOME, "tilt": TILT_HOME}
        self.note = ""          # ข้อความล่าสุดที่ ESP32 ส่งมา (บรรทัดขึ้นต้นด้วย #)
        self.fields = 0         # จำนวนช่องใน telemetry — บอกว่าเป็นเฟิร์มแวร์รุ่นไหน
        self.rx = 0             # จำนวนบรรทัด telemetry ที่รับมาทั้งหมด
        self.rx_hz = 0.0        # อัตราต่อวินาที
        self._rx0 = 0
        self._rx_t = time.time()
        self.last_rx = 0.0
        self._next_try = 0.0     # เวลาที่จะลองเปิดพอร์ตใหม่
        self._err = ""           # กันพิมพ์ error ซ้ำ ๆ
        self._open()

    def _open(self):
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = self.port, self.baud, 0
            s.dtr = False          # กัน ESP32 รีเซ็ตตอนเปิดพอร์ต
            s.rts = False
            s.open()
            time.sleep(0.2)
            s.reset_input_buffer()
            self.ser = s
            self._err = ""
            print(f"serial: {self.port} connected")
            # ถามรุ่นเฟิร์มแวร์ + สถานะแขน เพราะข้อความตอนบูตของ ESP32
            # มักหลุดไปก่อนที่เราจะเปิดพอร์ตทัน
            s.write(b"?")
        except (OSError, serial.SerialException) as e:
            self.ser = None
            self._next_try = time.time() + 2.0
            if str(e) != self._err:
                self._err = str(e)
                print(f"serial: {e} — จะลองใหม่ทุก 2 วินาที")

    def send(self, text):
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(text.encode())
                return True
            except (OSError, serial.SerialException):
                self.ser = None
                self._next_try = time.time() + 2.0
                return False

    def drive(self, left, right):
        m1, m2 = to_board(int(left), int(right))
        return self.send(f"#M,{m1},{m2}\n")

    def pump(self):
        """อ่าน telemetry — เรียกจาก thread เดียวเท่านั้น"""
        if self.ser is None:
            # ห้าม sleep ตรงนี้ — จะทำให้ drive loop 20Hz สะดุดไปทั้งวินาที
            if time.time() >= self._next_try:
                self._open()
            return
        try:
            chunk = self.ser.read(256)
        except (OSError, serial.SerialException):
            self.ser = None
            self._next_try = time.time() + 2.0
            return
        if not chunk:
            return
        self.buf += chunk
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            self._parse(line.strip().decode(errors="ignore"))
        self.buf = self.buf[-256:]

    def _parse(self, line):
        # S,<cmd>,<fwd>,<turn>,<ratio>,<trim1>,<trim2>,<tt>,<jog>,<ttDuty>,<curL>,<curR>,<pan>,<tilt>
        if line.startswith("#"):
            # ข้อความจาก ESP32 เช่น ป้ายรุ่น / เตือนหา PCA9685 ไม่เจอ
            self.note = line.lstrip("# ").strip()
            print("esp32:", self.note)
            return
        if not line.startswith("S,"):
            return
        f = line.split(",")
        try:
            self.st["cmd"] = f[1][:1] or "x"
            self.st["fwd"] = int(f[2])
            self.st["turn"] = int(f[3])
            self.st["ratio"] = int(f[4])
            # trim ในเฟิร์มแวร์ผูกกับ "ช่อง" M1/M2 — แปลงเป็นซ้าย/ขวาตามตรรกะ
            t1, t2 = float(f[5]), float(f[6])
            self.st["trim1"], self.st["trim2"] = (t2, t1) if SWAP_LR else (t1, t2)
            self.st["tt"] = int(f[7])
            self.st["jog"] = int(f[8])
            if len(f) >= 10:
                self.st["tt_pwm"] = int(f[9])
            if len(f) >= 12:
                self.st["curL"], self.st["curR"] = from_board(int(f[10]),
                                                              int(f[11]))
            if len(f) >= 14:                 # เฟิร์มแวร์ v3 ขึ้นไป — มีแขนกล้อง
                self.st["pan"] = int(f[12])
                self.st["tilt"] = int(f[13])
            self.fields = len(f)        # 9 = เฟิร์มแวร์เก่า, 12 = ตัวใหม่
            self.rx += 1
            now = self.last_rx = time.time()
            dt = now - self._rx_t
            if dt >= 1.0:                       # อัตรา telemetry ต่อวินาที
                self.rx_hz = (self.rx - self._rx0) / dt
                self._rx0, self._rx_t = self.rx, now
        except (IndexError, ValueError):
            pass

    @property
    def online(self):
        return bool(self.last_rx) and (time.time() - self.last_rx) < 1.5


# ================= ตัวรวมคำสั่งขับ =================
class DriveMux:
    """รับเป้าหมายจากจอยหรือเว็บ ใครสั่งทีหลังชนะ แล้วยิงซ้ำให้ ESP32"""

    def __init__(self, link, stats=None):
        self.link = link
        self.stats = stats
        self.lock = threading.Lock()
        self.l = self.r = 0
        self.ax = self.ay = 0.0      # ทิศสติ๊ก -1..1 (ไว้วาดสติ๊กในเว็บ)
        self.ts = 0.0
        self.src = "-"
        self.speed = SPEED_DEFAULT   # ความเร็วสูงสุด ใช้ร่วมกันจอย+เว็บ
        self.arm = None              # ตั้งทีหลัง — loop นี้เป็นคนส่ง #S ให้ด้วย

    def set(self, left, right, src, ax=None, ay=None):
        left = max(-MAX_SPEED, min(MAX_SPEED, int(left)))
        right = max(-MAX_SPEED, min(MAX_SPEED, int(right)))
        with self.lock:
            self.l, self.r = left, right
            if ax is None:
                # เว็บส่งมาเป็นความเร็วล้อ -> ถอดกลับเป็นทิศสติ๊ก
                s = float(self.speed) or 1.0
                fwd = (left + right) / 2.0 / s
                turn = (left - right) / 2.0 / s
                ax = max(-1.0, min(1.0, turn / TURN_GAIN))
                ay = max(-1.0, min(1.0, -fwd))
            self.ax, self.ay = ax, ay
            self.ts = time.time()
            self.src = src

    def stop(self, src="-"):
        self.set(0, 0, src, 0.0, 0.0)

    def bump_speed(self, delta):
        """ZL / ZR บนจอย"""
        with self.lock:
            self.speed = max(SPEED_MIN, min(SPEED_MAX, self.speed + delta))
            return self.speed

    def set_speed(self, v):
        """สไลเดอร์ในเว็บ"""
        with self.lock:
            self.speed = max(SPEED_MIN, min(SPEED_MAX, int(v)))
            return self.speed

    def loop(self):
        period = 1.0 / SEND_HZ
        last = time.time()
        while True:
            now = time.time()
            with self.lock:
                if now - self.ts > HOLD_TIMEOUT:
                    self.l = self.r = 0
                    self.ax = self.ay = 0.0
                    self.src = "-"
                l, r = self.l, self.r
            self.link.drive(l, r)
            if self.arm:
                self.arm.flush()
            self.link.pump()
            if self.stats:
                self.stats.add_drive(now - last, bool(l or r))
                self.stats.sample_link(self.link.online)
            last = now
            time.sleep(period)


# ================= แขนกล้อง =================
class Arm:
    """เก็บมุมแขนไว้ฝั่ง Pi แล้วส่ง #S ให้ ESP32
    ไม่ผูกกับ failsafe — ปล่อยสติ๊กแล้วแขนค้างที่เดิม ไม่เด้งกลับกลาง"""

    def __init__(self, link):
        self.link = link
        self.lock = threading.Lock()
        self.pan = float(PAN_HOME)
        self.tilt = float(TILT_HOME)
        self.dirty = True
        self.last_send = 0.0

    def nudge(self, dpan, dtilt):
        """ขยับแบบต่อเนื่องตามแรงโยกสติ๊ก — กลับทิศที่นี่ที่เดียว
        ทั้งจอยและสติ๊กในเว็บเรียกผ่านทางนี้หมด เลยไม่มีทางกลับข้างกันเอง"""
        if ARM_INVERT_PAN:
            dpan = -dpan
        if ARM_INVERT_TILT:
            dtilt = -dtilt
        with self.lock:
            self.pan = min(PAN_MAX, max(PAN_MIN, self.pan + dpan))
            self.tilt = min(TILT_MAX, max(TILT_MIN, self.tilt + dtilt))
            self.dirty = True

    def set(self, pan, tilt):
        with self.lock:
            self.pan = min(PAN_MAX, max(PAN_MIN, float(pan)))
            self.tilt = min(TILT_MAX, max(TILT_MIN, float(tilt)))
            self.dirty = True
            return int(round(self.pan)), int(round(self.tilt))

    def home(self):
        return self.set(PAN_HOME, TILT_HOME)

    def snapshot(self):
        with self.lock:
            return int(round(self.pan)), int(round(self.tilt))

    def flush(self):
        """เรียกจาก drive loop 20Hz — ส่งเมื่อค่าเปลี่ยน และย้ำทุก 1 วินาที
        เผื่อ ESP32 รีบูตกลางทางจะได้กลับไปมุมเดิมเอง"""
        now = time.time()
        with self.lock:
            if not self.dirty and now - self.last_send < 1.0:
                return
            p, t = int(round(self.pan)), int(round(self.tilt))
            self.dirty = False
            self.last_send = now
        self.link.send(f"#S,{p},{t}\n")


# ================= ถ่ายภาพ =================
def take_photo(cam, gal, stats, src="เว็บ"):
    """ใช้ร่วมกันทั้งปุ่มชัตเตอร์ในเว็บและปุ่ม R บนจอย"""
    if not cam.ok:
        return None
    data = cam.grab()
    if data is None:
        return None
    name = gal.save(data)
    with stats.lock:
        stats.photos += 1
        stats.shot_name = name
        stats.shot_src = src
        stats.shot_n += 1
    print(f"photo: {name}  ({len(data)} bytes) [{src}]")
    return name, len(data)


# ================= สถิติการทำงาน =================
class Stats:
    def __init__(self):
        self.t0 = time.time()
        self.lock = threading.Lock()
        self.drive_ms = 0.0        # เวลาที่ล้อหมุนจริง
        self.photos = 0
        self.reconnects = 0
        self._frames0 = 0
        self._ft = time.time()
        self.fps = 0.0
        self.link_ok = 0
        self.link_n = 0
        self.pad_name = ""      # ว่าง = ยังไม่เจอจอย
        self.shot_name = ""     # ภาพล่าสุดที่ถ่าย — ใช้เด้งแจ้งเตือนในเว็บ
        self.shot_n = 0         # นับครั้ง เว็บดูตัวเลขนี้ว่าเปลี่ยนไหม
        self.shot_src = ""      # ถ่ายจากอะไร: "จอย" หรือ "เว็บ" 

    def add_drive(self, dt, moving):
        if moving:
            with self.lock:
                self.drive_ms += dt

    def sample_link(self, ok):
        with self.lock:
            self.link_n += 1
            if ok:
                self.link_ok += 1

    def tick_fps(self, frames):
        now = time.time()
        dt = now - self._ft
        if dt >= 1.0:
            self.fps = (frames - self._frames0) / dt
            self._frames0, self._ft = frames, now

    def snapshot(self):
        with self.lock:
            n, ok = self.link_n, self.link_ok
            return {
                "up": int(time.time() - self.t0),
                "driveSec": int(self.drive_ms),
                "photos": self.photos,
                "reconnects": self.reconnects,
                "fps": round(self.fps, 1),
                "linkPct": round(100 * ok / n) if n else 0,
                "pad": bool(self.pad_name),
                "padName": self.pad_name,
                "shotN": self.shot_n,
                "shotName": self.shot_name,
                "shotSrc": self.shot_src,
            }


def soc_health():
    """อุณหภูมิและสถานะ throttle ของ Pi"""
    t = None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            t = int(f.read().strip()) / 1000.0
    except OSError:
        pass
    thr = 0
    try:
        import subprocess
        out = subprocess.run(["vcgencmd", "get_throttled"],
                             capture_output=True, text=True, timeout=1).stdout
        thr = int(out.strip().split("=")[1], 16)
    except Exception:
        pass
    load = 0.0
    try:
        load = os.getloadavg()[0]
    except OSError:
        pass
    return {"temp": round(t, 1) if t else None, "throttled": thr,
            "load": round(load, 2)}


# ================= กล้อง =================
class Camera:
    """ห่อ picamera2 — ถ้าเปิดไม่ได้ ระบบควบคุมยังทำงานต่อได้"""

    def __init__(self):
        self.ok = False
        self.frame = None
        self.n = 0
        self.cond = threading.Condition()
        self.crop = None          # ScalerCrop ปัจจุบัน (เปลี่ยนตามซูม)
        self.state = {"zoom": 1.0, "lens": 3.33, "busy": False, "soft": 0.0,
                      "fom": 0, "fomMax": 1, "still": "", "scan": ""}

    # ---------------- เปิดกล้อง ----------------
    def start(self):
        from picamera2 import Picamera2
        from picamera2.encoders import JpegEncoder
        from picamera2.outputs import FileOutput
        from libcamera import controls, Transform

        self.controls = controls
        self.Transform = Transform
        outer = self

        class StreamOut(io.BufferedIOBase):
            def write(self, buf):
                with outer.cond:
                    outer.frame = buf
                    outer.n += 1
                    outer.cond.notify_all()

        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": STREAM_SIZE},
            transform=Transform(hflip=1, vflip=1))
        self.picam2.configure(cfg)
        self.FULL = self.picam2.camera_properties["ScalerCropMaximum"]
        self.crop = tuple(self.FULL)
        # เกินเท่านี้คือขยายพิกเซลล้วน ๆ ภาพจะเริ่มเบลอ
        self.state["soft"] = round(self.FULL[2] / float(STREAM_SIZE[0]), 1)
        # ขนาดภาพนิ่ง — ใช้เต็มเซนเซอร์ถ้าไม่ได้กำหนด
        self.still_size = STILL_SIZE or tuple(
            self.picam2.camera_properties.get("PixelArraySize", self.FULL[2:4]))

        self.picam2.start_recording(JpegEncoder(), FileOutput(StreamOut()))
        self.ok = True
        self.tune()
        self.set_lens(3.33)
        threading.Thread(target=self._meta_loop, daemon=True).start()
        print(f"camera: started  สตรีม {STREAM_SIZE[0]}x{STREAM_SIZE[1]} / "
              f"ภาพนิ่ง {self.still_size[0]}x{self.still_size[1]} / "
              f"ซูมคมถึง {self.state['soft']}x")

    def stop(self):
        if self.ok:
            self.ok = False
            try:
                self.picam2.stop_recording()
            except Exception:
                pass

    # ---------------- ปรับความคมของ ISP ----------------
    def tune(self):
        try:
            self.picam2.set_controls({
                "Sharpness": SHARPNESS,          # 1.0 = ค่าเริ่มต้น
                "NoiseReductionMode": 2,         # HighQuality — ลด noise ที่กลืนรายละเอียด
                "AfRange": self.controls.AfRangeEnum.Full,   # ให้ไล่ถึงระยะใกล้ด้วย
                "AfSpeed": self.controls.AfSpeedEnum.Normal,
            })
        except Exception as e:
            print("tune:", e)

    # ---------------- อ่านค่าความคมจาก ISP ----------------
    def _meta_loop(self):
        """FocusFoM คือค่าที่ ISP คำนวณความคมให้อยู่แล้ว เอามาใช้ฟรี
        ไม่ต้องถอด JPEG มาคำนวณเองให้เปลือง CPU"""
        while self.ok:
            try:
                md = self.picam2.capture_metadata()
                fom = int(md.get("FocusFoM", 0))
                self.state["fom"] = fom
                if fom > self.state["fomMax"]:
                    self.state["fomMax"] = fom
                if not self.state["busy"]:
                    lp = md.get("LensPosition")
                    if lp is not None:
                        self.state["lens"] = round(float(lp), 2)
            except Exception:
                time.sleep(0.5)
            time.sleep(0.12)

    def _fom(self, settle=2):
        """รอให้ภาพนิ่งแล้วอ่านค่าความคม"""
        best = 0
        for _ in range(settle):
            try:
                md = self.picam2.capture_metadata()
                best = max(best, int(md.get("FocusFoM", 0)))
            except Exception:
                pass
        return best

    # ---------------- โฟกัส ----------------
    def af_window(self):
        """กรอบโฟกัสกลางภาพ — บังคับให้ AF เกาะสิ่งที่เล็ง ไม่ใช่ฉากหลัง"""
        fx, fy, fw, fh = self.crop
        w, h = int(fw * AF_WINDOW), int(fh * AF_WINDOW)
        return [(fx + (fw - w) // 2, fy + (fh - h) // 2, w, h)]

    def set_lens(self, pos):
        pos = max(0.0, min(float(pos), 10.0))
        self.picam2.set_controls({
            "AfMode": self.controls.AfModeEnum.Manual,
            "LensPosition": pos})
        self.state["lens"] = pos
        return pos

    def af_once(self):
        """AF ของกล้องเอง แต่จำกัดให้มองเฉพาะกรอบกลางภาพ"""
        if self.state["busy"]:
            return
        self.state["busy"] = True

        def job():
            try:
                self.picam2.set_controls({
                    "AfMode": self.controls.AfModeEnum.Auto,
                    "AfRange": self.controls.AfRangeEnum.Full,
                    "AfMetering": self.controls.AfMeteringEnum.Windows,
                    "AfWindows": self.af_window()})
                time.sleep(0.15)
                self.picam2.autofocus_cycle(wait=True)
                md = self.picam2.capture_metadata()
                pos = md.get("LensPosition", self.state["lens"])
                self.picam2.set_controls({
                    "AfMode": self.controls.AfModeEnum.Manual,
                    "LensPosition": pos})
                self.state["lens"] = pos
            except Exception as e:
                print("af error:", e)
            finally:
                self.state["busy"] = False

        threading.Thread(target=job, daemon=True).start()

    def focus_scan(self):
        """กวาดระยะโฟกัสทีละขั้นแล้ววัดความคมจริง เลือกจุดที่คมที่สุด
        แม่นกว่า AF ปกติกับเป้าแบน ๆ คอนทราสต์ต่ำอย่างหน้าปัดมิเตอร์
        กวาดหยาบก่อน แล้วค่อยกวาดละเอียดรอบจุดที่ดีที่สุด"""
        if self.state["busy"]:
            return
        self.state["busy"] = True

        def sweep(lo, hi, steps):
            best, hi_fom, lo_fom = lo, -1, None
            for i in range(steps):
                pos = lo + (hi - lo) * i / max(1, steps - 1)
                self.set_lens(pos)
                f = self._fom()
                lo_fom = f if lo_fom is None else min(lo_fom, f)
                if f > hi_fom:
                    best, hi_fom = pos, f
            return best, hi_fom, lo_fom

        def job():
            start = self.state["lens"]
            try:
                self.picam2.set_controls({
                    "AfMode": self.controls.AfModeEnum.Manual})
                self.state["scan"] = "กวาดหยาบ"
                coarse, hi_fom, lo_fom = sweep(SCAN_LO, SCAN_HI, SCAN_COARSE)

                # ถ้าความคมแทบไม่ต่างกันเลยตลอดช่วง แปลว่าเป้าไม่มีลายให้จับ
                # (ผนังเปล่า มืดเกินไป เลนส์บัง) อย่าไปจอดมั่ว คืนค่าเดิมดีกว่า
                if hi_fom <= 0 or (hi_fom - lo_fom) < hi_fom * SCAN_MIN_CONTRAST:
                    self.set_lens(start)
                    self.state["scan"] = "เป้าไม่มีลายให้จับโฟกัส"
                    print(f"focus scan: คอนทราสต์ไม่พอ (fom {lo_fom}-{hi_fom}) "
                          f"คืนค่าเดิม {start:.2f}")
                    return

                span = (SCAN_HI - SCAN_LO) / (SCAN_COARSE - 1)
                self.state["scan"] = "กวาดละเอียด"
                fine, fom, _ = sweep(max(SCAN_LO, coarse - span),
                                     min(SCAN_HI, coarse + span), SCAN_FINE)
                self.set_lens(fine)
                self.state["scan"] = ""
                print(f"focus scan: lens={fine:.2f} fom={fom}")
            except Exception as e:
                self.state["scan"] = ""
                print("focus scan error:", e)
            finally:
                self.state["busy"] = False

        threading.Thread(target=job, daemon=True).start()

    # ---------------- ซูม ----------------
    def apply_zoom(self, z):
        z = max(1.0, min(float(z), 8.0))
        fx, fy, fw, fh = self.FULL
        w, h = int(fw / z), int(fh / z)
        self.crop = (fx + (fw - w) // 2, fy + (fh - h) // 2, w, h)
        self.picam2.set_controls({"ScalerCrop": self.crop})
        self.state["zoom"] = z
        return z

    # ---------------- ภาพนิ่งเต็มความละเอียด ----------------
    def still(self):
        """สตรีมเป็น 720p ซึ่งอ่านตัวเลขบนมิเตอร์ไม่ออก
        ตอนกดถ่ายจึงสลับไปโหมดภาพนิ่งเต็มเซนเซอร์ แล้วกลับมาสตรีมต่อ
        รักษาทั้งซูมและระยะโฟกัสไว้ข้ามการสลับโหมด"""
        cfg = self.picam2.create_still_configuration(
            main={"size": self.still_size},
            transform=self.Transform(hflip=1, vflip=1),
            controls={"ScalerCrop": self.crop,
                      "AfMode": self.controls.AfModeEnum.Manual,
                      "LensPosition": self.state["lens"]})
        buf = io.BytesIO()
        self.picam2.switch_mode_and_capture_file(cfg, buf, format="jpeg")
        return buf.getvalue()

    def grab(self):
        """คืนภาพเต็มความละเอียดถ้าทำได้ ถ้าไม่ได้ก็ใช้เฟรมจากสตรีมแทน
        เพื่อไม่ให้การถ่ายพังทั้งระบบถ้าสลับโหมดไม่สำเร็จ"""
        if STILL_FULLRES:
            try:
                data = self.still()
                self.state["still"] = f"{self.still_size[0]}x{self.still_size[1]}"
                return data
            except Exception as e:
                print("still capture failed -> ใช้เฟรมจากสตรีมแทน:", e)
                self.state["still"] = "สตรีม (สลับโหมดไม่สำเร็จ)"
        else:
            self.state["still"] = "สตรีม"
        with self.cond:
            self.cond.wait_for(lambda: self.frame is not None, timeout=3)
            return self.frame


# ================= จอย =================
def pad_shape(v):
    if abs(v) < DEADZONE:
        return 0.0
    s = (abs(v) - DEADZONE) / (1 - DEADZONE)
    s = (1 - EXPO) * s + EXPO * s * s * s
    return s if v > 0 else -s


def pad_mix(fwd, turn, speed):
    left, right = fwd + turn, fwd - turn
    peak = max(1.0, abs(left), abs(right))
    return int(left / peak * speed), int(right / peak * speed)


def pad_find(ecodes, InputDevice, list_devices):
    """เลือกตัวที่มีทั้งสติ๊กและปุ่ม A — กัน keyboard/mouse ปลอมที่ dongle สร้างขึ้น"""
    best = None
    for path in list_devices():
        try:
            d = InputDevice(path)
        except OSError:
            continue
        caps = d.capabilities()
        axes = [c for c, _ in caps.get(ecodes.EV_ABS, [])]
        keys = caps.get(ecodes.EV_KEY, [])
        if ecodes.ABS_X in axes and ecodes.BTN_SOUTH in keys:
            return d
        if axes and keys and best is None:
            best = d
    return best


def pad_thread(mux, link, stats, arm, cam, gal, device=None):
    """หาจอยใหม่เรื่อย ๆ — ถอดแล้วเสียบกลับก็ใช้ได้ต่อโดยไม่ต้องรีสตาร์ต"""
    try:
        from evdev import InputDevice, list_devices, ecodes
    except ImportError:
        print("pad: ไม่มี evdev (sudo apt install python3-evdev) — ข้ามจอย")
        return

    warned = False
    while True:
        try:
            dev = (InputDevice(device) if device
                   else pad_find(ecodes, InputDevice, list_devices))
        except OSError:
            dev = None
        if dev is None:
            if not warned:
                print("pad: ไม่เจอจอย — เปิดโหมด Xbox ด้วย A + Home "
                      "(คุมผ่านเว็บได้ตามปกติ)")
                warned = True
            stats.pad_name = ""
            time.sleep(2.0)
            continue

        print(f"pad: {dev.name} ({dev.path})")
        stats.pad_name = dev.name
        warned = False
        try:
            _pad_loop(dev, ecodes, mux, link, arm, cam, gal, stats)
        except OSError:
            print("pad: หลุด -> หยุดหุ่น")
        finally:
            mux.stop("pad")
            stats.pad_name = ""
        time.sleep(2.0)


def _pad_loop(dev, ecodes, mux, link, arm, cam, gal, stats):
    code = {k: getattr(ecodes, v) for k, v in PAD_MAP.items()}
    absinfo = {c: i for c, i in dev.capabilities().get(ecodes.EV_ABS, [])}
    axis = {code[k]: 0.0 for k in ("drive_x", "drive_y", "arm_x", "arm_y")}
    hat = {code["dpad_x"]: 0, code["dpad_y"]: 0}
    hat_next = {code["dpad_x"]: 0.0, code["dpad_y"]: 0.0}
    trig = {code["zoom_out"]: 0.0, code["zoom_in"]: 0.0}
    hold = {"t": 0.0, "fired": False}      # จับเวลากดค้างปุ่ม L
    trig_next = {code["zoom_out"]: 0.0, code["zoom_in"]: 0.0}
    period = 1.0 / SEND_HZ
    next_tick = 0.0
    active = False

    def norm(value, info):
        mid = (info.max + info.min) / 2
        half = (info.max - info.min) / 2
        return 0.0 if half == 0 else max(-1.0, min(1.0, (value - mid) / half))

    def bg(fn):
        """งานที่อาจใช้เวลา — อย่าให้บล็อก loop ของจอย"""
        threading.Thread(target=fn, daemon=True).start()

    def dpad(c, v):
        """ขึ้น/ลง = ความเร็วเคลื่อนที่   ซ้าย/ขวา = ความเร็วแปรง"""
        if c == code["dpad_y"]:
            mux.bump_speed(SPEED_STEP if v < 0 else -SPEED_STEP)
        else:
            link.send("]" if v > 0 else "[")

    while True:
        # ---- อ่าน event (ไม่บล็อก) ----
        while True:
            ev = dev.read_one()
            if ev is None:
                break

            if ev.type == ecodes.EV_ABS:
                if ev.code in axis:
                    info = absinfo.get(ev.code)
                    if info:
                        axis[ev.code] = norm(ev.value, info)
                elif ev.code in trig:
                    # ทริกเกอร์อนาล็อก — เทียบเป็นสัดส่วนของช่วงที่จอยบอก
                    # ไม่ใช่ค่าดิบ จอยรุ่นไหนช่วงเท่าไรก็ใช้ได้
                    info = absinfo.get(ev.code)
                    if info and info.max != info.min:
                        trig[ev.code] = (ev.value - info.min) / (info.max - info.min)

                elif ev.code in hat:
                    # D-pad ให้ค่า -1 / 0 / 1 — ทำทันทีตอนกด ไม่งั้นแตะสั้น ๆ
                    # จะหลุด เพราะ event กดกับปล่อยมาถึงในรอบอ่านเดียวกันได้
                    hat[ev.code] = ev.value
                    if ev.value:
                        dpad(ev.code, ev.value)
                        hat_next[ev.code] = time.time() + REPEAT_DELAY
                    else:
                        hat_next[ev.code] = 0.0

            elif ev.type == ecodes.EV_KEY and ev.code == code["focus"]:
                # L: แตะ = AF เร็ว / กดค้าง = สแกนหาจุดคมสุด (แม่นกว่าแต่ช้ากว่า)
                if ev.value == 1:
                    hold["t"] = time.time()
                    hold["fired"] = False
                elif ev.value == 0:
                    if not hold["fired"] and cam.ok:
                        cam.af_once()
                    hold["t"] = 0.0

            elif ev.type == ecodes.EV_KEY and ev.value == 1:
                if ev.code == code["tt_toggle"]:        # A
                    link.send("k")
                elif ev.code == code["estop"]:          # B
                    mux.stop("pad")
                    link.send("x")
                elif ev.code == code["shot"]:           # R -> ถ่ายรูป
                    bg(lambda: take_photo(cam, gal, stats, "จอย"))

        now = time.time()

        # ---- D-pad กดค้าง -> ซ้ำ (ครั้งแรกทำไปแล้วตอนรับ event) ----
        for c in hat:
            if hat[c] and hat_next[c] and now >= hat_next[c]:
                dpad(c, hat[c])
                hat_next[c] = now + REPEAT_EVERY

        # ---- L กดค้างครบเวลา -> สแกนหาจุดคมสุด ----
        if hold["t"] and not hold["fired"] and now - hold["t"] >= FOCUS_HOLD:
            hold["fired"] = True
            if cam.ok:
                cam.focus_scan()

        # ---- ZL / ZR ซูมออก / ซูมเข้า (กดค้างแล้วซ้ำ) ----
        for c, delta in ((code["zoom_out"], -ZOOM_STEP),
                         (code["zoom_in"], ZOOM_STEP)):
            if trig[c] > TRIG_ON:
                if now >= trig_next[c]:
                    if cam.ok:
                        cam.apply_zoom(cam.state["zoom"] + delta)
                    trig_next[c] = now + (ZOOM_REPEAT if trig_next[c]
                                          else REPEAT_DELAY)
            else:
                trig_next[c] = 0.0

        if now >= next_tick:
            next_tick = now + period

            # ---- สติ๊กซ้าย -> ล้อ ----
            nx = pad_shape(axis[code["drive_x"]])
            ny = pad_shape(axis[code["drive_y"]])
            l, r = pad_mix(-ny, nx * TURN_GAIN, mux.speed)  # โยกขึ้น = เดินหน้า
            if l or r:
                mux.set(l, r, "pad", nx, ny)   # ส่ง nx/ny ไปวาดสติ๊กในเว็บ
                active = True
            elif active:
                mux.stop("pad")                # ปล่อยสติ๊ก -> ส่งศูนย์ครั้งเดียว
                active = False

            # ---- สติ๊กขวา -> แขนกล้อง (คุมเป็นความเร็ว ไม่ใช่ตำแหน่ง) ----
            ax = pad_shape(axis[code["arm_x"]])
            ay = pad_shape(axis[code["arm_y"]])
            if ax or ay:
                step = ARM_RATE * period
                arm.nudge(ax * step, -ay * step)   # กลับทิศทำใน nudge() แล้ว
        time.sleep(0.005)


# ================= คลังภาพ =================
class Gallery:
    """เก็บภาพที่ถ่ายลงดิสก์ของ Pi พร้อมรูปย่อ"""

    def __init__(self, root, thumb_w=THUMB_W):
        self.root = root
        self.thumbs = os.path.join(root, ".thumbs")
        self.thumb_w = thumb_w
        os.makedirs(self.thumbs, exist_ok=True)
        try:
            from PIL import Image
            self.Image = Image
        except ImportError:
            self.Image = None
            print("gallery: ไม่มี Pillow — รูปย่อจะใช้ไฟล์เต็ม (pip install pillow)")

    @staticmethod
    def safe(name):
        """กัน path traversal — รับเฉพาะชื่อไฟล์ที่เราตั้งเอง"""
        base = os.path.basename(name)
        return base if base.endswith(".jpg") and "/" not in name else None

    def save(self, data):
        name = time.strftime("shot_%Y%m%d_%H%M%S.jpg")
        path = os.path.join(self.root, name)
        n = 1
        while os.path.exists(path):          # ถ่ายรัวในวินาทีเดียวกัน
            name = time.strftime(f"shot_%Y%m%d_%H%M%S_{n}.jpg")
            path = os.path.join(self.root, name)
            n += 1
        with open(path, "wb") as f:
            f.write(data)
        self._thumb(name, data)
        return name

    def _thumb(self, name, data):
        if self.Image is None:
            return
        try:
            im = self.Image.open(io.BytesIO(data))
            im.thumbnail((self.thumb_w, self.thumb_w))
            im.convert("RGB").save(os.path.join(self.thumbs, name),
                                   "JPEG", quality=78)
        except Exception as e:
            print("thumb error:", e)

    def list(self, limit=200):
        try:
            names = [n for n in os.listdir(self.root) if n.endswith(".jpg")]
        except OSError:
            return []
        names.sort(reverse=True)
        out = []
        for n in names[:limit]:
            try:
                st = os.stat(os.path.join(self.root, n))
            except OSError:
                continue
            out.append({"name": n, "size": st.st_size, "ts": int(st.st_mtime)})
        return out

    def read(self, name, thumb=False):
        name = self.safe(name)
        if not name:
            return None
        p = os.path.join(self.thumbs if thumb else self.root, name)
        if thumb and not os.path.exists(p):
            p = os.path.join(self.root, name)
        try:
            with open(p, "rb") as f:
                return f.read()
        except OSError:
            return None

    def delete(self, name):
        name = self.safe(name)
        if not name:
            return False
        ok = False
        for d in (self.root, self.thumbs):
            try:
                os.remove(os.path.join(d, name))
                ok = True
            except OSError:
                pass
        return ok

    def usage(self):
        total = 0
        count = 0
        try:
            for n in os.listdir(self.root):
                if n.endswith(".jpg"):
                    total += os.path.getsize(os.path.join(self.root, n))
                    count += 1
        except OSError:
            pass
        free = 0
        try:
            st = os.statvfs(self.root)
            free = st.f_bavail * st.f_frsize
        except OSError:
            pass
        return {"count": count, "bytes": total, "free": free}


# ================= หน้าเว็บ =================
PAGE = """<!DOCTYPE html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#05090C">
<title>Robot cockpit</title>
<style>
:root{
  --bg:#05090C; --panel:#111B23; --sink:#0C141A; --line:#22323F; --line2:#31485A;
  --ink:#D5E2EA; --dim:#8398A6; --faint:#55697A;
  --drive:#E8A33D; --optics:#4FC3D9; --halt:#C4453C; --live:#5FB97F; --warn:#D98A3D;
  --glass:rgba(6,11,15,.62); --edge:rgba(213,226,234,.18);
  --pad:calc(10px + env(safe-area-inset-left,0px));
  box-sizing:border-box;
}
*,*::before,*::after{box-sizing:inherit}
html,body{height:100%;overflow:hidden}
body{
  margin:0;background:var(--bg);color:var(--ink);
  font:400 15px/1.45 ui-sans-serif,system-ui,"Segoe UI",Roboto,"Noto Sans Thai",sans-serif;
  font-variant-numeric:tabular-nums;
  overscroll-behavior:none;-webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent;
}
button{font:inherit;color:inherit;background:none;border:0;cursor:pointer;
  touch-action:manipulation}
button:focus-visible{outline:2px solid var(--optics);outline-offset:2px}

/* ================= ภาพสดเต็มจอ ================= */
.feed{position:fixed;inset:0;background:#000;z-index:0}
/* แนวตั้ง: เห็นภาพครบดีกว่า / แนวนอน: 16:9 พอดีจออยู่แล้ว เลยให้เต็มพื้นที่ */
.feed img{width:100%;height:100%;object-fit:contain;display:block}
@media (orientation:landscape){.feed img{object-fit:cover}}
.feed.fill img{object-fit:cover}
.feed.fit img{object-fit:contain}
.feed.noimg img{opacity:.12}
/* แฟลชขาวตอนชัตเตอร์ลั่น — เห็นชัดแม้กดจากจอย ไม่ได้มองหน้าจออยู่ */
.flash{position:fixed;inset:0;z-index:9;background:#EAF2F7;opacity:0;
  pointer-events:none;transition:opacity .28s ease-out}
.flash.on{opacity:.75;transition:none}
.feed.noimg::after{content:"ยังไม่มีภาพจากกล้อง";position:absolute;inset:0;
  display:grid;place-items:center;color:var(--dim);font-size:14px;pointer-events:none}

/* ================= ชั้นลอย ================= */
/* ตัวครอบไม่รับคลิก ปล่อยให้เฉพาะปุ่มจริงรับ */
.layer{position:fixed;z-index:5;pointer-events:none}
.layer > *{pointer-events:auto}
.tl{top:calc(9px + env(safe-area-inset-top,0px));left:var(--pad);
    display:flex;flex-wrap:wrap;gap:6px;max-width:62vw}
.tr{top:calc(9px + env(safe-area-inset-top,0px));
    right:calc(10px + env(safe-area-inset-right,0px));display:flex;gap:6px}

/* ป้ายสถานะ */
.pill{display:inline-flex;align-items:center;gap:6px;padding:5px 10px;border-radius:999px;
  font-size:11.5px;color:var(--dim);background:var(--glass);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);
  border:1px solid var(--edge)}
.pill i{width:7px;height:7px;border-radius:50%;background:var(--halt);flex:none}
.pill.up{color:var(--ink);border-color:rgba(95,185,127,.5)}
.pill.up i{background:var(--live)}
.pill.warn{color:var(--drive);border-color:rgba(232,163,61,.55)}
.pill.warn i{background:var(--drive)}

/* ปุ่มผี */
.gb{padding:7px 11px;border:1px solid var(--edge);border-radius:9px;font-size:12px;
  background:var(--glass);backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);
  white-space:nowrap}
.gb:active{background:rgba(213,226,234,.18)}
.gb.on{border-color:var(--optics);color:var(--optics)}

/* ================= แถบกล้อง ขอบขวา ================= */
.cam{top:50%;right:calc(10px + env(safe-area-inset-right,0px));
  transform:translateY(-50%);display:flex;flex-direction:column;align-items:center;gap:8px}
.shutter{width:58px;height:58px;border-radius:50%;border:3px solid rgba(213,226,234,.7);
  background:var(--glass);padding:0;position:relative;
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.shutter::after{content:"";position:absolute;inset:6px;border-radius:50%;background:#EAF2F7;
  transition:background .12s}
.shutter:active::after{background:var(--optics)}
.shutter.busy::after{background:var(--drive)}
.zoomer{display:flex;flex-direction:column;align-items:center;gap:2px;padding:5px 7px;
  border:1px solid var(--edge);border-radius:11px;background:var(--glass);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.zoomer button{width:30px;height:28px;font-size:16px;line-height:1;border-radius:7px}
.zoomer button:active{background:rgba(213,226,234,.18)}
.zoomer span{font-size:11px;color:var(--dim)}

/* ================= สติ๊ก ================= */
.sticks{left:0;right:0;bottom:calc(10px + env(safe-area-inset-bottom,0px));
  display:flex;align-items:flex-end;gap:10px;padding:0 var(--pad)}
.sticks .mid{flex:1 1 auto;min-width:0;display:flex;flex-direction:column;
  align-items:center;gap:8px;pointer-events:none}
.sticks .mid > *{pointer-events:auto}

.stick{position:relative;flex:0 0 auto;
  width:clamp(104px,27vw,152px);height:clamp(104px,27vw,152px);border-radius:50%;
  background:radial-gradient(circle at 50% 42%,rgba(28,44,56,.8) 0%,rgba(8,14,19,.72) 74%);
  border:1px solid var(--edge);touch-action:none;
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.stick::before{content:"";position:absolute;inset:30%;border-radius:50%;
  border:1px dashed rgba(213,226,234,.14)}
.knob{position:absolute;left:50%;top:50%;width:38%;height:38%;
  transform:translate(0,0);margin:-19% 0 0 -19%;
  border-radius:50%;background:var(--panel);border:1px solid var(--line2);
  box-shadow:0 5px 16px rgba(0,0,0,.6);transition:background .12s,border-color .12s}
.stick.live .knob{background:#2A3B4A;border-color:var(--drive)}
.stick.pad .knob{background:#22392F;border-color:var(--live)}
.stick.arm.live .knob{background:#1F3640;border-color:var(--optics)}
/* ป้ายอยู่เหนือสติ๊ก — ถ้าไว้ใต้จะโดนขอบล่างของจอตัด */
.cap{position:absolute;left:0;right:0;top:-15px;text-align:center;
  font-size:10.5px;color:var(--faint);pointer-events:none;white-space:nowrap}
.stick.pad .cap{color:var(--live)}
.stick.arm .cap{color:var(--faint)}

/* มาตรวัดล้อ */
.meters{width:100%;max-width:280px;display:flex;flex-direction:column;gap:5px;
  padding:7px 9px;border:1px solid var(--edge);border-radius:11px;background:var(--glass);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.mrow{display:grid;grid-template-columns:26px 1fr 36px;gap:7px;align-items:center;
  font-size:11px;color:var(--dim)}
.track{position:relative;height:12px;border-radius:3px;background:rgba(6,11,15,.75);
  border:1px solid var(--line)}
.track::before{content:"";position:absolute;left:50%;top:0;bottom:0;width:1px;
  background:var(--line2)}
.fill{position:absolute;top:2px;bottom:2px;left:50%;width:0;background:var(--drive);
  border-radius:2px}
.ghost{position:absolute;top:0;bottom:0;left:50%;width:0;
  border:1px solid rgba(232,163,61,.55);border-radius:2px}
.mrow b{font-weight:400;color:var(--ink);text-align:right}
.halt{padding:9px 20px;border:1px solid var(--halt);border-radius:10px;color:#F0B4AF;
  font-size:14px;background:rgba(120,30,26,.6);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.halt:active{background:rgba(196,69,60,.75)}

/* ================= ข้อความชั่วคราว ================= */
.toast{left:50%;top:calc(52px + env(safe-area-inset-top,0px));transform:translateX(-50%);
  max-width:80vw}
.toast span{display:block;padding:7px 13px;border-radius:999px;font-size:12.5px;
  color:var(--optics);background:var(--glass);border:1px solid var(--edge);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);text-align:center}
.toast span:empty{display:none}
.alert{left:50%;bottom:calc(178px + env(safe-area-inset-bottom,0px));
  transform:translateX(-50%);max-width:min(560px,90vw)}
.alert span{display:none;padding:9px 14px;border-radius:10px;font-size:12.5px;
  color:#F0CB94;background:rgba(80,50,14,.82);border:1px solid var(--warn);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px)}
.alert span.show{display:block}

/* ================= แผงเลื่อนจากขวา ================= */
.sheet{position:fixed;top:0;bottom:0;right:0;width:min(430px,94vw);z-index:20;
  background:rgba(8,13,18,.94);backdrop-filter:blur(16px);
  -webkit-backdrop-filter:blur(16px);border-left:1px solid var(--line);
  transform:translateX(101%);transition:transform .22s ease;
  display:flex;flex-direction:column}
.sheet.on{transform:none}
.shead{display:flex;align-items:center;gap:10px;padding:13px 14px;
  padding-top:calc(13px + env(safe-area-inset-top,0px));border-bottom:1px solid var(--line)}
.shead .stabs{display:flex;gap:4px}
.shead .stabs button{padding:6px 11px;border-radius:8px;font-size:13px;color:var(--dim);
  border:1px solid transparent}
.shead .stabs button.on{color:var(--ink);border-color:var(--line2);background:var(--sink)}
.shead .sp{margin-left:auto}
.sbody{flex:1 1 auto;overflow-y:auto;padding:12px;display:flex;flex-direction:column;
  gap:11px;padding-bottom:calc(12px + env(safe-area-inset-bottom,0px))}
/* ห้ามให้ flex บีบแผงย่อยให้พอดีจอ ไม่งั้นเนื้อหาข้างในโดนตัด */
.sbody > *{flex:none}
.sheet .x{padding:6px 11px;border:1px solid var(--line2);border-radius:8px;font-size:13px}

/* ================= ชิ้นส่วนในแผง ================= */
.panel{border:1px solid var(--line);border-radius:11px;background:var(--panel);
  overflow:hidden}
.head{display:flex;align-items:center;gap:10px;padding:12px 13px;width:100%;text-align:left}
.head h3{margin:0;font-size:13.5px;font-weight:500}
.head .val{margin-left:auto;font-size:12.5px;color:var(--dim)}
.head .caret{color:var(--faint);transition:transform .18s}
.panel.open .caret{transform:rotate(90deg)}
.body{display:none;padding:0 13px 13px;flex-direction:column;gap:10px}
.panel.open .body{display:flex}
.seg{display:flex;gap:7px;flex-wrap:wrap}
.seg button{flex:1 1 auto;min-width:58px;padding:9px 5px;border:1px solid var(--line2);
  border-radius:8px;font-size:13px;background:var(--sink)}
.seg button:active{background:#1C2A35}
.seg button.on{border-color:var(--drive);color:var(--drive)}
.seg button.go{border-color:var(--optics);color:var(--optics)}
input[type=range]{width:100%;accent-color:var(--optics);background:transparent;margin:0}
.slab{display:flex;justify-content:space-between;gap:10px;font-size:12px;color:var(--dim)}
.slab span:last-child{color:var(--ink);text-align:right}

.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(98px,1fr));gap:8px}
.tile{border:1px solid var(--line);border-radius:10px;background:var(--panel);padding:10px 11px}
.tile span{display:block;font-size:11px;color:var(--dim);margin-bottom:3px}
.tile strong{font-weight:500;font-size:20px;line-height:1.1}
.tile small{font-size:11.5px;color:var(--faint);margin-left:3px}
.tile.warn{border-color:var(--warn)} .tile.warn strong{color:var(--warn)}
.tile.bad{border-color:var(--halt)} .tile.bad strong{color:var(--halt)}
.tile.good strong{color:var(--live)}
.tele{margin:0;padding:10px 11px;border:1px solid var(--line2);border-radius:9px;
  background:var(--sink);color:var(--ink);white-space:pre-wrap;word-break:break-word;
  font:12px/1.6 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
/* แถบวัดความคม — ยิ่งยาวยิ่งชัด ใช้เล็งโฟกัสด้วยมือได้แม่น */
.fbar{position:relative;height:16px;border-radius:4px;background:var(--sink);
  border:1px solid var(--line);overflow:hidden}
.fbar i{position:absolute;left:0;top:0;bottom:0;width:0;background:var(--optics);
  transition:width .12s linear}
.fbar b{position:absolute;top:0;bottom:0;width:2px;background:var(--live);
  left:0;transition:left .3s}
.keymap{display:flex;flex-direction:column;gap:6px}
.keymap div{display:flex;align-items:baseline;gap:9px;font-size:12.5px;color:var(--dim)}
.keymap kbd{flex:none;min-width:64px;padding:3px 7px;border:1px solid var(--line2);
  border-radius:6px;background:var(--sink);color:var(--ink);font:inherit;font-size:11.5px;
  text-align:center}

/* ================= อัลบั้ม ================= */
figure{margin:0}
.albumbar{display:flex;align-items:center;gap:9px}
.albumbar .n{font-size:12.5px;color:var(--dim)}
.albumbar .sp{margin-left:auto}
.albumbar button{padding:6px 11px;border:1px solid var(--line2);border-radius:8px;
  font-size:12.5px}
.albumbar button.on{border-color:var(--optics);color:var(--optics)}
.albumbar button.danger{border-color:var(--halt);color:var(--halt)}
.day h4{margin:2px 0 7px;font-size:12.5px;font-weight:500;color:var(--dim);
  display:flex;align-items:baseline;gap:8px}
.day h4 em{font-style:normal;font-size:11px;color:var(--faint)}
.shelf{display:grid;grid-template-columns:repeat(auto-fill,minmax(96px,1fr));gap:6px}
.card{position:relative;aspect-ratio:1;border-radius:8px;overflow:hidden;
  background:var(--sink);border:1px solid var(--line);cursor:pointer}
.card img{width:100%;height:100%;object-fit:cover;display:block;background:var(--sink)}
.card.miss::after{content:"ภาพเสีย";position:absolute;inset:0;display:grid;
  place-items:center;color:var(--faint);font-size:11px}
.card time{position:absolute;left:0;right:0;bottom:0;padding:14px 6px 4px;font-size:10.5px;
  color:#EAF2F7;background:linear-gradient(transparent,rgba(4,8,11,.82))}
.card .mark{position:absolute;top:6px;right:6px;width:20px;height:20px;border-radius:50%;
  border:1.5px solid rgba(255,255,255,.75);background:rgba(4,8,11,.35);display:none}
.picking .card .mark{display:block}
.card.sel{border-color:var(--optics)}
.card.sel img{opacity:.55}
.card.sel .mark{background:var(--optics);border-color:var(--optics)}
.card.sel .mark::after{content:"";position:absolute;left:6px;top:2px;width:5px;height:10px;
  border:solid #08121A;border-width:0 2px 2px 0;transform:rotate(45deg)}
.empty{padding:30px 16px;text-align:center;border:1px dashed var(--line2);border-radius:12px}
.empty b{display:block;font-weight:500;font-size:14.5px;margin-bottom:5px}
.empty span{font-size:12.5px;color:var(--dim)}

/* ================= ดูภาพเต็ม ================= */
.viewer{position:fixed;inset:0;background:#04070A;display:none;flex-direction:column;
  z-index:40;padding-top:env(safe-area-inset-top,0)}
.viewer.on{display:flex}
.vtop{display:flex;align-items:center;gap:10px;padding:11px 13px;font-size:12.5px;
  color:var(--dim)}
.vtop .sp{margin-left:auto}
.vtop button{padding:6px 11px;border:1px solid var(--line2);border-radius:7px;font-size:13px}
.vstage{flex:1 1 auto;min-height:0;position:relative;display:flex;align-items:center;
  justify-content:center;touch-action:pan-y}
.vstage img{max-width:100%;max-height:100%;object-fit:contain}
.arrow{position:absolute;top:50%;transform:translateY(-50%);width:42px;height:62px;
  border-radius:9px;background:rgba(14,22,29,.75);border:1px solid var(--line2);
  font-size:20px}
.arrow.l{left:10px} .arrow.r{right:10px}
.arrow[disabled]{opacity:.25}
.vbar{display:flex;gap:8px;padding:12px;
  padding-bottom:calc(12px + env(safe-area-inset-bottom,0))}
.vbar button{flex:1;padding:12px 6px;border:1px solid var(--line2);border-radius:9px;
  font-size:13.5px}
.vbar .del{border-color:var(--halt);color:var(--halt)}

/* แนวนอน — ย้ายแถบกล้องไปขอบบนกลาง ไม่งั้นชนสติ๊กแขนที่มุมขวาล่าง */
@media (orientation:landscape){
  .cam{top:calc(8px + env(safe-area-inset-top,0px));left:50%;right:auto;
    transform:translateX(-50%);flex-direction:row;align-items:center;gap:7px}
  .zoomer{flex-direction:row;align-items:center;padding:3px 6px;gap:4px}
  .zoomer button{width:28px;height:26px}
  .shutter{width:44px;height:44px;border-width:2px}
  .shutter::after{inset:5px}
  .tl{max-width:34vw}
}
/* จอเตี้ยมาก ๆ — ย่อของที่ไม่จำเป็นลง */
@media (max-height:430px){
  .meters{display:none}
}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>

<div class="feed" id="feed"><img id="v" src="/stream" alt="ภาพสดจากกล้องหุ่น"></div>
<div class="flash" id="flash"></div>

<!-- ---------- ขอบบนซ้าย: สถานะ ---------- -->
<div class="layer tl">
  <span class="pill" id="pLink"><i></i><span>กำลังเชื่อม</span></span>
  <span class="pill" id="pPad"><i></i><span>หาจอย…</span></span>
  <span class="pill" id="pBrush"><i></i><span>แปรงปิด</span></span>
  <span class="pill" id="pSpeed"><i></i><span>เร็ว 200</span></span>
</div>

<!-- ---------- ขอบบนขวา: เปิดแผง ---------- -->
<div class="layer tr">
  <button class="gb" id="bFit" title="สลับเต็มพื้นที่/พอดีภาพ">พอดีภาพ</button>
  <button class="gb" id="bDash">สถานะ</button>
  <button class="gb" id="bShots">ภาพ</button>
</div>

<!-- ---------- ขอบขวา: กล้อง ---------- -->
<div class="layer cam">
  <button class="gb" id="af">โฟกัส</button>
  <div class="zoomer">
    <button id="zIn" aria-label="ซูมเข้า">+</button>
    <span id="zLab">1.0x</span>
    <button id="zOut" aria-label="ซูมออก">−</button>
  </div>
  <button class="shutter" id="shutter" aria-label="ถ่ายรูป"></button>
  <button class="gb" id="bCam">ตั้งค่า</button>
</div>

<!-- ---------- ข้อความชั่วคราว ---------- -->
<div class="layer toast"><span id="note"></span></div>
<div class="layer alert"><span id="alert"></span></div>

<!-- ---------- ขอบล่าง: สติ๊ก + มาตรวัด ---------- -->
<div class="layer sticks">
  <div class="stick" id="stick">
    <div class="knob" id="knob"></div>
    <span class="cap" id="capDrive">ขับเคลื่อน</span>
  </div>

  <div class="mid">
    <div class="meters">
      <div class="mrow"><span>ซ้าย</span>
        <span class="track"><i class="ghost" id="gl"></i><i class="fill" id="fl"></i></span>
        <b id="nl">0</b></div>
      <div class="mrow"><span>ขวา</span>
        <span class="track"><i class="ghost" id="gr"></i><i class="fill" id="fr"></i></span>
        <b id="nr">0</b></div>
    </div>
    <button class="halt" id="halt">หยุด</button>
  </div>

  <div class="stick arm" id="armStick">
    <div class="knob" id="armKnob"></div>
    <span class="cap" id="capArm">แขน 90° / 90°</span>
  </div>
</div>

<!-- ---------- แผงตั้งค่ากล้อง + แขน ---------- -->
<aside class="sheet" id="shCam">
  <div class="shead">
    <div class="stabs">
      <button data-sheet="#shCam">กล้อง</button>
      <button data-sheet="#shDash">สถานะ</button>
      <button data-sheet="#shShots">ภาพ</button>
    </div>
    <span class="sp"></span>
    <button class="x" data-close>ปิด</button></div>
  <div class="sbody">
    <div class="panel open">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>โฟกัส</h3><span class="val" id="lval">30 cm</span></button>
      <div class="body">
        <input id="ls" type="range" min="0" max="10" step="0.05" value="3.33">
        <div class="slab"><span>ไกล</span><span>ใกล้</span></div>
        <div class="seg">
          <button data-l="0">∞</button><button data-l="1">1 m</button>
          <button data-l="2">50 cm</button><button data-l="3.33">30 cm</button>
          <button data-l="6.67">15 cm</button>
        </div>
        <div class="seg">
          <button class="go" id="af2">โฟกัสอัตโนมัติ</button>
          <button class="go" id="afscan">สแกนหาจุดคมสุด</button>
        </div>
        <div class="fbar"><i id="fFill"></i><b id="fPeak"></b></div>
        <div class="slab"><span>ความคมที่วัดได้</span><span id="fVal">–</span></div>
      </div>
    </div>

    <div class="panel open">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>ซูม</h3><span class="val" id="zLab2">1.0x</span></button>
      <div class="body">
        <div class="seg">
          <button data-z="1">1x</button><button data-z="1.5">1.5x</button>
          <button data-z="2">2x</button><button data-z="3">3x</button>
        </div>
        <div class="seg">
          <button data-z="4">4x</button><button data-z="5">5x</button>
          <button data-z="6">6x</button><button data-z="8">8x</button>
        </div>
        <input id="zSl" type="range" min="1" max="8" step="0.25" value="1">
        <div class="slab"><span>ลากปรับละเอียด</span><span id="zSoft">–</span></div>
      </div>
    </div>

    <div class="panel open">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>แขนกล้อง</h3><span class="val" id="armVal">90° / 90°</span></button>
      <div class="body">
        <div class="slab"><span>หันซ้าย/ขวา (ช่อง 7)</span><span id="panVal">90°</span></div>
        <input id="panSl" type="range" min="0" max="180" step="1" value="90">
        <div class="slab"><span>ก้ม/เงย (ช่อง 4)</span><span id="tiltVal">90°</span></div>
        <input id="tiltSl" type="range" min="0" max="180" step="1" value="90">
        <div class="seg"><button id="armHome">กลับตำแหน่งกลาง</button></div>
      </div>
    </div>

    <div class="panel">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>ปรับแต่งการขับ</h3><span class="val" id="tuneVal">เร็วสุด 200</span></button>
      <div class="body">
        <input id="sp" type="range" min="60" max="255" step="5" value="200">
        <div class="slab"><span>ความเร็วสูงสุด (D-pad ขึ้น/ลง)</span><span id="spv">200</span></div>
        <div class="seg">
          <button id="trimDn" onclick="cmd('1')">ช้าลง</button>
          <button id="trimUp" onclick="cmd('2')">เร็วขึ้น</button>
        </div>
        <div class="slab"><span>ชดเชยล้อ</span><span id="trimv">1.00 / 0.96</span></div>
        <div class="seg">
          <button id="btt">เปิด/ปิดแปรง</button>
          <button onclick="cmd('[')">แปรงช้าลง</button>
          <button onclick="cmd(']')">แปรงเร็วขึ้น</button>
        </div>
      </div>
    </div>
  </div>
</aside>

<!-- ---------- แผงสถานะ ---------- -->
<aside class="sheet" id="shDash">
  <div class="shead">
    <div class="stabs">
      <button data-sheet="#shCam">กล้อง</button>
      <button data-sheet="#shDash">สถานะ</button>
      <button data-sheet="#shShots">ภาพ</button>
    </div>
    <span class="sp"></span>
    <button class="x" data-close>ปิด</button></div>
  <div class="sbody">
    <div class="grid">
      <div class="tile" id="tTemp"><span>อุณหภูมิ Pi</span><strong id="dTemp">–</strong><small>°C</small></div>
      <div class="tile" id="tLink"><span>สัญญาณ ESP32</span><strong id="dLink">–</strong><small>%</small></div>
      <div class="tile" id="tPad"><span>จอย</span><strong id="dPad">–</strong></div>
      <div class="tile"><span>ภาพต่อวินาที</span><strong id="dFps">–</strong></div>
      <div class="tile"><span>โหลด CPU</span><strong id="dLoad">–</strong></div>
      <div class="tile"><span>เปิดมาแล้ว</span><strong id="dUp">–</strong></div>
      <div class="tile"><span>เวลาที่ขับ</span><strong id="dDrive">–</strong></div>
      <div class="tile"><span>ภาพที่เก็บ</span><strong id="dShots">–</strong></div>
      <div class="tile"><span>พื้นที่เหลือ</span><strong id="dFree">–</strong><small>GB</small></div>
    </div>

    <div class="panel open">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>สถานะจากหุ่น</h3><span class="val" id="dMoving">–</span></button>
      <div class="body">
        <pre class="tele" id="tele">รอข้อมูลจาก ESP32…</pre>
        <div class="slab"><span>คำสั่งที่ ESP32 รับล่าสุด</span><span id="dCmd">–</span></div>
        <div class="slab"><span>ล้อ สั่ง / จริง</span><span id="dWheel">0,0 / 0,0</span></div>
        <div class="slab"><span>ผู้ควบคุม</span><span id="dSrc">ว่าง</span></div>
        <div class="slab"><span>แขน สั่ง / จริง</span><span id="dArm">–</span></div>
          <div class="slab"><span>ข้อความจาก ESP32</span><span id="dNote">–</span></div>
          <div class="slab"><span>ขนาดภาพที่เซฟ</span><span id="dStill">–</span></div>
        <div class="slab"><span>แปรงกวาด</span><span id="dTT">–</span></div>
        <div class="slab"><span>telemetry ที่รับมา</span><span id="dRx">–</span></div>
        <div class="slab"><span>พอร์ต serial</span><span id="dSer">–</span></div>
        <div class="slab"><span>รุ่นจอยที่เจอ</span><span id="dPadName">–</span></div>
        <div class="slab"><span>พื้นที่ใช้ไป</span><span id="dUsed">–</span></div>
      </div>
    </div>

    <div class="panel">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>ค่าตั้งของหุ่น</h3><span class="val" id="dSetHead">–</span></button>
      <div class="body">
        <div class="slab"><span>ความเร็วสูงสุด — สติ๊ก/จอย</span><span id="dSpeed">–</span></div>
        <div class="slab"><span>ความเร็วฐาน w/s ในเฟิร์มแวร์</span><span id="dFwd">–</span></div>
        <div class="slab"><span>ความเร็วเลี้ยว a/d</span><span id="dTurn">–</span></div>
        <div class="slab"><span>ชดเชยล้อ trim ซ้าย/ขวา</span><span id="dTrim">–</span></div>
        <div class="slab"><span>duty แปรงกวาด</span><span id="dTTd">–</span></div>
        <div class="slab"><span>โหมด jog</span><span id="dJog">–</span></div>
        <div class="slab"><span>ทิศล้อที่ตั้งไว้</span><span id="dWiring">–</span></div>
      </div>
    </div>

    <div class="panel open">
      <button class="head" data-toggle><span class="caret">›</span><h3>ปุ่มบนจอย</h3></button>
      <div class="body"><div class="keymap">
        <div><kbd>สติ๊กซ้าย</kbd><span>ขับเคลื่อน — ขึ้นเดินหน้า ลงถอย ซ้ายขวาเลี้ยว</span></div>
        <div><kbd>สติ๊กขวา</kbd><span>แขนกล้อง — ซ้ายขวาหัน ขึ้นลงก้มเงย</span></div>
        <div><kbd>A</kbd><span>เปิด / ปิดแปรงกวาด</span></div>
        <div><kbd>B</kbd><span>หยุดล้อทันที</span></div>
        <div><kbd>L</kbd><span>แตะ = โฟกัสอัตโนมัติ · กดค้าง = สแกนหาจุดคมสุด</span></div>
        <div><kbd>R</kbd><span>ถ่ายรูป</span></div>
        <div><kbd>ZL / ZR</kbd><span>ซูมออก / ซูมเข้า (กดค้างได้)</span></div>
        <div><kbd>D-pad ↑↓</kbd><span>ความเร็วการเคลื่อนที่ เพิ่ม / ลด</span></div>
        <div><kbd>D-pad ←→</kbd><span>ความเร็วแปรง ลด / เพิ่ม</span></div>
      </div></div>
    </div>

    <div class="panel">
      <button class="head" data-toggle><span class="caret">›</span><h3>ที่อยู่หน้านี้</h3></button>
      <div class="body">
        <div class="slab"><span>ชื่อ</span><span id="aName">–</span></div>
        <div class="slab"><span>IP</span><span id="aIp">–</span></div>
      </div>
    </div>
  </div>
</aside>

<!-- ---------- แผงภาพที่ถ่าย ---------- -->
<aside class="sheet" id="shShots">
  <div class="shead">
    <div class="stabs">
      <button data-sheet="#shCam">กล้อง</button>
      <button data-sheet="#shDash">สถานะ</button>
      <button data-sheet="#shShots">ภาพ</button>
    </div>
    <span class="sp"></span>
    <button class="x" data-close>ปิด</button></div>
  <div class="sbody">
    <div class="albumbar">
      <span class="n" id="albumN">—</span>
      <span class="sp"></span>
      <button id="pick">เลือก</button>
      <button id="wipe" class="danger" hidden>ลบ 0</button>
      <button id="refresh">โหลดใหม่</button>
    </div>
    <div id="shelfWrap"></div>
  </div>
</aside>

<div class="viewer" id="viewer">
  <div class="vtop">
    <button id="vClose">ปิด</button>
    <span class="sp"></span>
    <span id="vMeta"></span>
  </div>
  <div class="vstage" id="vStage">
    <button class="arrow l" id="vPrev" aria-label="ก่อนหน้า">‹</button>
    <img id="vImg" alt="">
    <button class="arrow r" id="vNext" aria-label="ถัดไป">›</button>
  </div>
  <div class="vbar">
    <button id="vSave">บันทึกลงเครื่อง</button>
    <button class="del" id="vDel">ลบภาพนี้</button>
  </div>
</div>

<script>
const $=s=>document.querySelector(s);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const secs=s=>{const h=s/3600|0,m=(s%3600)/60|0;
  return h?`${h}:${String(m).padStart(2,'0')}`:`${m}:${String(s%60).padStart(2,'0')}`};
const gb=b=>(b/1073741824).toFixed(1);
const pad2=n=>String(n).padStart(2,'0');

for(const h of document.querySelectorAll('[data-toggle]'))
  h.onclick=()=>h.parentElement.classList.toggle('open');

/* ---------- แผงเลื่อน ---------- */
function sheet(id,on){
  for(const s of document.querySelectorAll('.sheet')) s.classList.remove('on');
  if(on!==false && id) $(id).classList.add('on');
  for(const b of document.querySelectorAll('[data-sheet]'))
    b.classList.toggle('on', on!==false && b.dataset.sheet===id);
  if(id==='#shShots' && on!==false) loadShelf();
}
for(const b of document.querySelectorAll('[data-sheet]'))
  b.onclick=()=>sheet(b.dataset.sheet);
for(const b of document.querySelectorAll('[data-close]'))
  b.onclick=()=>sheet(null,false);
$('#bCam').onclick=()=>sheet('#shCam');
$('#bDash').onclick=()=>sheet('#shDash');
$('#bShots').onclick=()=>sheet('#shShots');
/* ค่าเริ่มต้นมาจาก CSS ตามแนวจอ — ปุ่มนี้บังคับทับได้ */
function fitLabel(){
  const f=$('#feed');
  const fill=f.classList.contains('fill') ||
    (!f.classList.contains('fit') && matchMedia('(orientation:landscape)').matches);
  $('#bFit').textContent=fill?'เต็มพื้นที่':'พอดีภาพ';
  $('#bFit').classList.toggle('on',fill);
  return fill;
}
$('#bFit').onclick=()=>{
  const f=$('#feed'), toFill=!fitLabel();
  f.classList.toggle('fill',toFill);
  f.classList.toggle('fit',!toFill);
  fitLabel();
};
matchMedia('(orientation:landscape)').addEventListener('change',fitLabel);
fitLabel();

/* ---------- ข้อความชั่วคราว ---------- */
const note=$('#note');
let noteT=null;
function say(t,keep){
  note.textContent=t;
  if(noteT){clearTimeout(noteT);noteT=null;}
  if(!keep) noteT=setTimeout(()=>{note.textContent='';},4000);
}

/* ================= สติ๊กขับเคลื่อน ================= */
let vec={l:0,r:0}, tick=null, touching=false;
let maxSpeed=200;
const stick=$('#stick'), knob=$('#knob'), DEAD=0.10;
function sendVec(){ fetch(`/drive?l=${vec.l}&r=${vec.r}`).catch(()=>{}); }
function startTick(){ if(!tick) tick=setInterval(sendVec,120); }
function stopDrive(){
  vec={l:0,r:0}; touching=false;
  if(tick){clearInterval(tick);tick=null;}
  sendVec(); paintCmd(0,0);
  stick.classList.remove('live'); knob.style.transform='';
}
$('#halt').onclick=()=>{stopDrive();cmd('x');};
function setFromXY(nx,ny){
  const fwd=-ny,turn=nx*0.7;
  let l=fwd+turn,r=fwd-turn;
  const peak=Math.max(1,Math.abs(l),Math.abs(r));
  vec={l:Math.round(l/peak*maxSpeed),r:Math.round(r/peak*maxSpeed)};
  paintCmd(vec.l,vec.r);
}
/* แปลง -1..1 เป็นระยะพิกเซล โดยอิงขนาดสติ๊กจริง (ขนาดยืดตามจอ) */
function radius(el){const b=el.getBoundingClientRect();return b.width*0.31;}
function knobTo(el,k,nx,ny){
  const R=radius(el);
  k.style.transform=`translate(${nx*R}px,${ny*R}px)`;
}
function grabXY(el,e){
  const b=el.getBoundingClientRect(),R=radius(el);
  let dx=e.clientX-(b.left+b.width/2),dy=e.clientY-(b.top+b.height/2);
  const d=Math.hypot(dx,dy)||1;
  if(d>R){dx*=R/d;dy*=R/d;}
  let nx=dx/R,ny=dy/R;
  if(Math.hypot(nx,ny)<DEAD){nx=0;ny=0;}
  return {nx,ny,dx,dy};
}
let pid=null;
function place(e){
  const g=grabXY(stick,e);
  knob.style.transform=`translate(${g.dx}px,${g.dy}px)`;
  setFromXY(g.nx,g.ny);
}
stick.addEventListener('pointerdown',e=>{
  pid=e.pointerId;stick.setPointerCapture(pid);
  stick.classList.add('live');touching=true;startTick();place(e);
});
stick.addEventListener('pointermove',e=>{if(e.pointerId===pid)place(e)});
for(const ev of ['pointerup','pointercancel'])
  stick.addEventListener(ev,e=>{if(e.pointerId===pid){pid=null;stopDrive();}});

/* ================= สติ๊กแขนกล้อง ================= */
/* คุมเป็นความเร็ว: ค้างไว้แขนขยับต่อเนื่อง ปล่อยแล้วค้างมุมเดิม */
const ARM_RATE=70;            /* องศา/วินาที ตอนโยกสุด — ให้เท่าฝั่ง Pi */
const ARM_MS=100;
const armStick=$('#armStick'), armKnob=$('#armKnob');
let armPid=null, armVec={x:0,y:0}, armTick=null;
function armPlace(e){
  const g=grabXY(armStick,e);
  armKnob.style.transform=`translate(${g.dx}px,${g.dy}px)`;
  armVec={x:g.nx,y:g.ny};
}
function armStep(){
  if(!armVec.x&&!armVec.y) return;
  const k=ARM_RATE*ARM_MS/1000;
  fetch(`/arm?dpan=${(armVec.x*k).toFixed(2)}&dtilt=${(-armVec.y*k).toFixed(2)}`)
    .then(r=>r.json()).then(showArm).catch(()=>{});
}
armStick.addEventListener('pointerdown',e=>{
  armPid=e.pointerId;armStick.setPointerCapture(armPid);
  armStick.classList.add('live');armPlace(e);
  if(!armTick) armTick=setInterval(armStep,ARM_MS);
  armStep();
});
armStick.addEventListener('pointermove',e=>{if(e.pointerId===armPid)armPlace(e)});
for(const ev of ['pointerup','pointercancel'])
  armStick.addEventListener(ev,e=>{ if(e.pointerId!==armPid) return;
    armPid=null; armVec={x:0,y:0};
    if(armTick){clearInterval(armTick);armTick=null;}
    armStick.classList.remove('live'); armKnob.style.transform='';
  });

let armPan=90,armTilt=90;
function showArm(a){
  if(!a) return;
  armPan=a.pan; armTilt=a.tilt;
  $('#capArm').textContent=`แขน ${a.pan}° / ${a.tilt}°`;
  $('#armVal').textContent=`${a.pan}° / ${a.tilt}°`;
  $('#panVal').textContent=a.pan+'°';
  $('#tiltVal').textContent=a.tilt+'°';
  if(document.activeElement!==$('#panSl')) $('#panSl').value=a.pan;
  if(document.activeElement!==$('#tiltSl')) $('#tiltSl').value=a.tilt;
}
function armSet(pan,tilt){
  fetch(`/arm?pan=${pan}&tilt=${tilt}`).then(r=>r.json()).then(showArm).catch(()=>{});
}
$('#panSl').oninput=e=>{$('#panVal').textContent=e.target.value+'°';
  armSet(e.target.value,$('#tiltSl').value);};
$('#tiltSl').oninput=e=>{$('#tiltVal').textContent=e.target.value+'°';
  armSet($('#panSl').value,e.target.value);};
$('#armHome').onclick=()=>fetch('/arm?home=1').then(r=>r.json())
  .then(a=>{showArm(a);say('แขนกลับตำแหน่งกลาง');}).catch(()=>{});

/* ================= คีย์บอร์ด ================= */
const KEY={w:[0,-1],s:[0,1],a:[-1,0],d:[1,0]},down=new Set();
function fromKeys(){
  let x=0,y=0; for(const k of down){x+=KEY[k][0];y+=KEY[k][1];}
  if(!x&&!y){stopDrive();return;}
  touching=true; startTick(); stick.classList.add('live');
  x=clamp(x,-1,1); y=clamp(y,-1,1); setFromXY(x,y); knobTo(stick,knob,x,y);
}
addEventListener('keydown',e=>{
  if($('#viewer').classList.contains('on')){
    if(e.key==='Escape'){closeViewer();return;}
    if(e.key==='ArrowLeft'){stepPic(-1);return;}
    if(e.key==='ArrowRight'){stepPic(1);return;}
    return;
  }
  const k=e.key.toLowerCase();
  if(KEY[k]&&!e.repeat){e.preventDefault();down.add(k);fromKeys();return;}
  if(k==='escape'){stopDrive();cmd('x');sheet(null,false);}
  if(k==='k')cmd('k');
  if(k==='f')refocus();
  if(k===' '){e.preventDefault();shoot();}
  if(k==='arrowleft'||k==='arrowright'||k==='arrowup'||k==='arrowdown'){
    e.preventDefault();
    const d=ARM_RATE*0.06;
    fetch(`/arm?dpan=${k==='arrowright'?d:k==='arrowleft'?-d:0}`+
          `&dtilt=${k==='arrowup'?d:k==='arrowdown'?-d:0}`)
      .then(r=>r.json()).then(showArm).catch(()=>{});
  }
});
addEventListener('keyup',e=>{const k=e.key.toLowerCase();
  if(KEY[k]){down.delete(k);fromKeys();}});
addEventListener('blur',()=>{down.clear();stopDrive();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDrive();});

function bar(el,v){const p=clamp(v/255,-1,1)*50;
  el.style.left=(p<0?50+p:50)+'%';el.style.width=Math.abs(p)+'%';}
function paintCmd(l,r){bar($('#fl'),l);bar($('#fr'),r);
  $('#nl').textContent=l;$('#nr').textContent=r;}
function paintAct(l,r){bar($('#gl'),l);bar($('#gr'),r);}

function cmd(c){fetch('/cmd?c='+encodeURIComponent(c)).catch(()=>{});}
$('#btt').onclick=()=>cmd('k');
function showSpeed(v){maxSpeed=v;$('#sp').value=v;
  $('#spv').textContent=v;$('#tuneVal').textContent='เร็วสุด '+v;}
$('#sp').oninput=e=>{showSpeed(+e.target.value);
  fetch('/speed?v='+maxSpeed).catch(()=>{});};

/* ================= กล้อง ================= */
const dist=v=>v>0.05?(100/v).toFixed(0)+' cm':'∞';
let zoom=1;
function lens(v){v=+v;$('#ls').value=v;$('#lval').textContent=dist(v);
  fetch('/lens?v='+v).catch(()=>{});say('ระยะโฟกัส '+dist(v));}
$('#ls').oninput=e=>lens(e.target.value);
for(const b of document.querySelectorAll('[data-l]')) b.onclick=()=>lens(b.dataset.l);
function setz(v){zoom=clamp(Math.round(v*4)/4,1,8);
  $('#zLab').textContent=zoom.toFixed(2).replace(/0$/,'')+'x';
  $('#zLab2').textContent=zoom.toFixed(2).replace(/0$/,'')+'x';
  if(document.activeElement!==$('#zSl')) $('#zSl').value=zoom;
  fetch('/zoom?z='+zoom).catch(()=>{});}
for(const b of document.querySelectorAll('[data-z]')) b.onclick=()=>setz(+b.dataset.z);
const ZSTEP=0.25;
$('#zIn').onclick=()=>setz(zoom+ZSTEP);
$('#zOut').onclick=()=>setz(zoom-ZSTEP);
$('#zSl').oninput=e=>setz(+e.target.value);
$('#af').onclick=refocus; $('#af2').onclick=refocus;
$('#afscan').onclick=async()=>{
  try{
    const v=parseFloat(await(await fetch('/afscan')).text());
    if(!isNaN(v)){$('#ls').value=v;$('#lval').textContent=dist(v);}
  }catch(e){say('ติดต่อกล้องไม่ได้');}
};
async function refocus(){
  try{
    const v=parseFloat(await(await fetch('/af')).text());   // แจ้งผลใน poll()
    if(!isNaN(v)){$('#ls').value=v;$('#lval').textContent=dist(v);}
  }catch(e){say('ติดต่อกล้องไม่ได้');}
}
const shutter=$('#shutter');
shutter.onclick=shoot;
async function shoot(){
  if(shutter.classList.contains('busy'))return;
  shutter.classList.add('busy'); say('กำลังถ่าย…',true);
  try{
    await(await fetch('/photo')).json();   // แจ้งเตือน/แฟลชทำใน poll()
  }catch(e){say('ถ่ายไม่สำเร็จ — กล้องพร้อมหรือยัง');}
  finally{setTimeout(()=>shutter.classList.remove('busy'),350);}
}
let wl=null;
async function keepAwake(){try{wl=await navigator.wakeLock?.request('screen');}catch(e){}}
keepAwake();
document.addEventListener('visibilitychange',()=>{if(!document.hidden)keepAwake();});

/* ================= อัลบั้ม ================= */
let shots=[], cur=-1, picking=false;
const sel=new Set();
function dayKey(ts){const d=new Date(ts*1000);
  return `${d.getFullYear()}-${pad2(d.getMonth()+1)}-${pad2(d.getDate())}`;}
function dayLabel(key){
  const t=new Date(), y=new Date(Date.now()-864e5);
  const f=d=>`${d.getFullYear()}-${pad2(d.getMonth()+1)}-${pad2(d.getDate())}`;
  if(key===f(t))return 'วันนี้';
  if(key===f(y))return 'เมื่อวาน';
  const [Y,M,D]=key.split('-');
  return `${+D}/${+M}/${Y}`;
}
async function loadShelf(){
  const w=$('#shelfWrap');
  try{
    const g=await(await fetch('/gallery')).json();
    shots=g.items;
    $('#albumN').textContent=shots.length
      ? `${shots.length} ภาพ · ${(g.store.bytes/1048576).toFixed(1)} MB` : 'ยังไม่มีภาพ';
    if(!shots.length){
      w.innerHTML='<div class="empty"><b>อัลบั้มยังว่าง</b>'+
        '<span>กดชัตเตอร์บนภาพสด หรือปุ่ม R บนจอย</span></div>';
      exitPick(); return;
    }
    const groups=new Map();
    shots.forEach((s,i)=>{const k=dayKey(s.ts);
      if(!groups.has(k))groups.set(k,[]); groups.get(k).push(i);});
    let html='';
    for(const [k,idx] of groups){
      html+=`<div class="day"><h4>${dayLabel(k)} <em>${idx.length} ภาพ</em></h4><div class="shelf">`;
      for(const i of idx){
        const s=shots[i], d=new Date(s.ts*1000);
        html+=`<figure class="card" data-i="${i}">
          <img loading="lazy" data-full="/pic?f=${encodeURIComponent(s.name)}"
               src="/thumb?f=${encodeURIComponent(s.name)}" alt="">
          <span class="mark"></span>
          <time>${pad2(d.getHours())}:${pad2(d.getMinutes())}</time></figure>`;
      }
      html+='</div></div>';
    }
    w.innerHTML=html;
    for(const c of w.querySelectorAll('.card')){
      const i=+c.dataset.i;
      c.onclick=()=>picking?toggleSel(c,i):openViewer(i);
      let t=null;
      c.addEventListener('pointerdown',()=>{t=setTimeout(()=>{
        if(!picking){enterPick();toggleSel(c,i);}},450);});
      for(const ev of ['pointerup','pointerleave','pointercancel'])
        c.addEventListener(ev,()=>{if(t){clearTimeout(t);t=null;}});
      const im=c.querySelector('img');
      im.onerror=()=>{
        if(im.dataset.full&&im.src.indexOf('/thumb')>-1){im.src=im.dataset.full;}
        else{c.classList.add('miss');im.style.visibility='hidden';}
      };
    }
    if(picking) for(const c of w.querySelectorAll('.card'))
      if(sel.has(shots[+c.dataset.i].name)) c.classList.add('sel');
  }catch(e){
    w.innerHTML='<div class="empty"><b>โหลดอัลบั้มไม่ได้</b>'+
      '<span>ตรวจว่า server.py ยังทำงานอยู่</span></div>';
  }
}
$('#refresh').onclick=loadShelf;
function enterPick(){picking=true;sel.clear();
  $('#shelfWrap').classList.add('picking');
  $('#pick').classList.add('on');$('#pick').textContent='ยกเลิก';
  $('#wipe').hidden=false;updateWipe();}
function exitPick(){picking=false;sel.clear();
  $('#shelfWrap').classList.remove('picking');
  $('#pick').classList.remove('on');$('#pick').textContent='เลือก';
  $('#wipe').hidden=true;
  for(const c of document.querySelectorAll('.card.sel'))c.classList.remove('sel');}
function toggleSel(card,i){
  const n=shots[i].name;
  if(sel.has(n)){sel.delete(n);card.classList.remove('sel');}
  else{sel.add(n);card.classList.add('sel');}
  updateWipe();
}
function updateWipe(){$('#wipe').textContent='ลบ '+sel.size;
  $('#wipe').disabled=!sel.size;}
$('#pick').onclick=()=>picking?exitPick():enterPick();
$('#wipe').onclick=async()=>{
  if(!sel.size)return;
  if(!confirm(`ลบ ${sel.size} ภาพถาวร?`))return;
  for(const n of sel) await fetch('/delete?f='+encodeURIComponent(n)).catch(()=>{});
  exitPick(); loadShelf();
};

/* ================= ดูภาพเต็ม ================= */
function openViewer(i){
  cur=i; const s=shots[i]; if(!s)return;
  $('#vImg').src='/pic?f='+encodeURIComponent(s.name);
  $('#vMeta').textContent=`${i+1} / ${shots.length}  ·  ${(s.size/1024|0)} KB`;
  $('#vPrev').disabled=i<=0;
  $('#vNext').disabled=i>=shots.length-1;
  $('#viewer').classList.add('on');
}
function closeViewer(){$('#viewer').classList.remove('on');$('#vImg').src='';}
function stepPic(d){const n=cur+d; if(n>=0&&n<shots.length)openViewer(n);}
$('#vClose').onclick=closeViewer;
$('#vPrev').onclick=()=>stepPic(-1);
$('#vNext').onclick=()=>stepPic(1);
$('#vSave').onclick=()=>{
  const s=shots[cur]; if(!s)return;
  const a=document.createElement('a');
  a.href='/pic?f='+encodeURIComponent(s.name); a.download=s.name; a.click();
};
$('#vDel').onclick=async()=>{
  const s=shots[cur]; if(!s)return;
  if(!confirm('ลบ '+s.name+' ถาวร?'))return;
  await fetch('/delete?f='+encodeURIComponent(s.name)).catch(()=>{});
  closeViewer(); loadShelf();
};
let sx=0,sy=0;
$('#vStage').addEventListener('pointerdown',e=>{sx=e.clientX;sy=e.clientY;});
$('#vStage').addEventListener('pointerup',e=>{
  const dx=e.clientX-sx, dy=e.clientY-sy;
  if(Math.abs(dx)>56&&Math.abs(dx)>Math.abs(dy)*1.4) stepPic(dx<0?1:-1);
});

fetch('/where').then(r=>r.json()).then(a=>{
  $('#aName').textContent=a.name; $('#aIp').textContent=a.ip||'–';
}).catch(()=>{});

/* ================= telemetry ================= */
function tile(id,cls){const e=$(id);e.className='tile'+(cls?' '+cls:'');}
function pill(id,on,txt,warn){
  const e=$(id);
  e.className='pill'+(on?(warn?' warn':' up'):'');
  e.lastElementChild.textContent=txt;
}
let trimSide='', armLimits='', lastShotN=-1, focusState='';
/* จำจุดที่คมที่สุดที่เคยเจอในช่วงนี้ ไว้เป็นหลักให้เทียบตอนหมุนโฟกัสเอง */
let fomPeak=0, fomPeakT=0;
function peakPct(s){
  const now=Date.now();
  if(s.fom>=fomPeak || now-fomPeakT>6000){ fomPeak=s.fom; fomPeakT=now; }
  return s.fomMax>0 ? 100*fomPeak/s.fomMax : 0;
}
function shotFlash(name,src){
  const f=$('#flash');
  f.classList.add('on');
  requestAnimationFrame(()=>requestAnimationFrame(()=>f.classList.remove('on')));
  say('เก็บภาพแล้ว ('+src+') — '+name);
}
const CMD_TH={w:'เดินหน้า',s:'ถอยหลัง',a:'เลี้ยวซ้าย',d:'เลี้ยวขวา',
              x:'หยุด',M:'สติ๊ก (อนาล็อก)',T:'ตั้ง duty แปรง',H:'แขนกลับกลาง',
              k:'สลับแปรง','[':'แปรงช้าลง',']':'แปรงเร็วขึ้น'};
const w4=v=>String(v).padStart(4);
function teleBlock(s){
  return `[${s.online?'OK ':'---'}] cmd=${s.cmd}   ${s.moving?'กำลังหมุน':'จอดอยู่'}\n`+
    `ล้อ  สั่ง ${w4(s.cmdL)},${w4(s.cmdR)}    จริง ${w4(s.curL)},${w4(s.curR)}\n`+
    `fwd=${s.fwd}  turn=${s.turn}(${s.ratio}%)  trim=${s.trim1.toFixed(2)}/${s.trim2.toFixed(2)}\n`+
    `TT=${s.tt?s.tt_pwm:'OFF'}   jog=${s.jog?'ON':'OFF'}   สติ๊กสูงสุด=${s.speed}\n`+
    `แขน สั่ง ${s.pan}°,${s.tilt}°   จริง ${s.panFw}°,${s.tiltFw}°\n`+
    `rx=${s.rx} (${s.rxHz}/วิ)   ผู้ควบคุม=`+
    (s.src==='pad'?'จอย':s.src==='web'?'เว็บ':'ว่าง');
}

async function poll(){
  try{
    const s=await(await fetch('/status')).json();

    if(s.oldFw) pill('#pLink',true,'เฟิร์มแวร์เก่า',true);
    else pill('#pLink',s.online,s.online?'ESP32':'ไม่ตอบสนอง');
    pill('#pPad',s.pad,s.pad?'จอยพร้อม':'ไม่พบจอย');
    pill('#pBrush',!!s.tt,s.tt?('แปรง '+s.tt_pwm):'แปรงปิด',true);
    pill('#pSpeed',true,'เร็ว '+s.speed);

    if(lastShotN<0) lastShotN=s.shotN;
    else if(s.shotN!==lastShotN){ lastShotN=s.shotN;
      shotFlash(s.shotName,s.shotSrc);
      if($('#shShots').classList.contains('on')) loadShelf(); }

    if(+$('#sp').value!==s.speed) showSpeed(s.speed);
    if(armLimits!==s.panMin+','+s.panMax+','+s.tiltMin+','+s.tiltMax){
      armLimits=s.panMin+','+s.panMax+','+s.tiltMin+','+s.tiltMax;
      $('#panSl').min=s.panMin;  $('#panSl').max=s.panMax;
      $('#tiltSl').min=s.tiltMin; $('#tiltSl').max=s.tiltMax;
    }
    if(armPid===null) showArm({pan:s.pan,tilt:s.tilt});
    if(s.armMoving) $('#capArm').textContent=`แขน → ${s.panFw}° / ${s.tiltFw}°`;

    const byPad=(s.src==='pad');
    stick.classList.toggle('pad',byPad);
    $('#capDrive').textContent=byPad?'กำลังใช้จอย':'ขับเคลื่อน';
    if(byPad&&!touching){ knobTo(stick,knob,s.ax,s.ay); paintCmd(s.cmdL,s.cmdR); }
    else if(!byPad&&!touching&&!tick){ knob.style.transform=''; paintCmd(0,0); }
    paintAct(s.curL,s.curR);

    $('#trimv').textContent=s.trim1.toFixed(2)+' / '+s.trim2.toFixed(2);
    $('#btt').className=s.tt?'on':'';
    $('#feed').classList.toggle('noimg',!s.cam);

    /* ---- กล้อง: ซูม + แถบวัดความคม ---- */
    if(Math.abs(zoom-s.zoom)>0.01 && document.activeElement!==$('#zSl')) setz(s.zoom);
    $('#zSoft').textContent=s.zoomSoft
      ? 'คมถึง '+s.zoomSoft.toFixed(1)+'x เกินนั้นเป็นการขยายพิกเซล' : '–';
    if(s.fomMax>0){
      $('#fFill').style.width=Math.min(100,100*s.fom/s.fomMax).toFixed(0)+'%';
      $('#fPeak').style.left='calc('+Math.min(100,peakPct(s)).toFixed(0)+'% - 1px)';
      $('#fVal').textContent=s.fom+(s.scan?'   '+s.scan:'');
    }
    $('#afscan').classList.toggle('on',!!s.scan);
    $('#af').classList.toggle('on',s.afBusy);
    const fBusy=s.afBusy?(s.scan||'กำลังโฟกัส'):'';
    if(fBusy!==focusState){
      if(fBusy) say(fBusy+'…',true);
      else if(focusState) say(s.scan||('โฟกัสที่ '+dist(s.lens)));
      focusState=fBusy;
    }
    $('#dStill').textContent=s.stillMode||'–';

    $('#tele').textContent=teleBlock(s);
    $('#dMoving').textContent=s.moving?'กำลังหมุน':'จอดอยู่';
    $('#dCmd').textContent=(s.cmd||'–')+'   '+(CMD_TH[s.cmd]||'');
    $('#dWheel').textContent=`${s.cmdL},${s.cmdR} / ${s.curL},${s.curR}`;
    $('#dSrc').textContent=s.src==='pad'?'จอย':s.src==='web'?'เว็บ':'ว่าง';
    $('#dArm').textContent=s.armFw
      ? `${s.pan}°,${s.tilt}° / ${s.panFw}°,${s.tiltFw}°`+(s.armMoving?'  กำลังขยับ':'')
      : `${s.pan}° / ${s.tilt}°  (เฟิร์มแวร์ยังไม่รองรับ)`;
    $('#dNote').textContent=s.note||'–';
    $('#dTT').textContent=(s.tt?'เปิด':'ปิด')+' · duty '+s.tt_pwm;
    $('#dRx').textContent=s.rx+' บรรทัด  ('+s.rxHz+'/วิ)';
    $('#dSer').textContent=s.serial?'เปิดอยู่':'ยังไม่เชื่อม';
    $('#dPadName').textContent=s.padName||'–';
    $('#dUsed').textContent=(s.store.bytes/1048576).toFixed(1)+' MB';

    $('#dSetHead').textContent='สติ๊ก '+s.speed+' · ฐาน '+s.fwd;
    $('#dSpeed').textContent=s.speed;
    $('#dFwd').textContent=s.fwd;
    $('#dTurn').textContent=s.turn+'  ('+s.ratio+'% ของความเร็วฐาน)';
    $('#dTrim').textContent=s.trim1.toFixed(2)+' / '+s.trim2.toFixed(2);
    $('#dTTd').textContent=s.tt_pwm+' / 255';
    $('#dJog').textContent=s.jog?'เปิด':'ปิด';
    $('#dWiring').textContent=[s.swap?'สลับ M1/M2':'ต่อตรง',
      s.invL?'กลับทิศซ้าย':null,s.invR?'กลับทิศขวา':null].filter(Boolean).join(' · ');
    if(trimSide!==s.trimSide){ trimSide=s.trimSide;
      $('#trimDn').textContent='ล้อ'+trimSide+'ช้าลง';
      $('#trimUp').textContent='ล้อ'+trimSide+'เร็วขึ้น'; }

    $('#dTemp').textContent=s.temp??'–';
    tile('#tTemp', s.temp>=80?'bad':s.temp>=70?'warn':'good');
    $('#dLink').textContent=s.linkPct;
    tile('#tLink', s.linkPct<80?'bad':s.linkPct<95?'warn':'good');
    $('#dPad').textContent=s.pad?'พร้อม':'ไม่พบ';
    tile('#tPad', s.pad?'good':'warn');
    $('#dFps').textContent=s.fps;
    $('#dLoad').textContent=s.load;
    $('#dUp').textContent=secs(s.up);
    $('#dDrive').textContent=secs(s.driveSec);
    $('#dShots').textContent=s.store.count;
    $('#dFree').textContent=gb(s.store.free);

    const al=$('#alert'); let m='';
    if(s.oldFw) m='ESP32 เป็นเฟิร์มแวร์เก่า ('+s.fields+' ช่อง) — ล้อจะไม่หมุน ต้องแฟลช main.cpp ใหม่';
    else if(s.online&&!s.armFw) m='เฟิร์มแวร์ยังไม่รองรับแขนกล้อง — แฟลช main.cpp v3';
    else if(s.throttled&1) m='แรงดันไฟตก — ตรวจ UBEC และสาย USB-C';
    else if(s.throttled&(1<<2)) m='CPU ถูกลดความเร็วเพราะร้อน — หาที่ระบายอากาศ';
    else if(s.throttled&(1<<16)) m='เคยเจอไฟตกตั้งแต่บูต';
    else if(!s.serial) m='ยังไม่เชื่อมกับ ESP32 — ตรวจสาย USB';
    else if(!s.pad) m='ไม่พบจอย — เปิดจอยโหมด Xbox ด้วย A + Home';
    al.textContent=m; al.classList.toggle('show',!!m);
  }catch(e){
    pill('#pLink',false,'ติดต่อ Pi ไม่ได้');
  }
}
setInterval(poll,250); poll();
</script></body></html>"""


# ================= HTTP =================
class QuietServer(ThreadingHTTPServer):
    """เบราว์เซอร์ปิดแท็บ/สลับหน้า = สตรีมถูกตัดกลางคัน ไม่ใช่ error
    ไม่ต้องพ่น traceback ยาว ๆ ใส่ terminal"""
    daemon_threads = True

    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionResetError, BrokenPipeError,
                            ConnectionAbortedError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def make_handler(cam, link, mux, gal, stats, arm):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, ctype, data, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def _ok(self, ctype, data, extra=None):
            self._send(200, ctype, data, extra)

        def _json(self, obj):
            self._ok("application/json", json.dumps(obj).encode())

        def _fail(self, code=404, why=""):
            self._send(code, "text/plain; charset=utf-8", why.encode())

        def _q(self, name, default=0.0):
            try:
                return float(parse_qs(urlparse(self.path).query)[name][0])
            except Exception:
                return default

        def _qs(self, name, default=""):
            try:
                return parse_qs(urlparse(self.path).query)[name][0]
            except Exception:
                return default

        def do_GET(self):
            p = urlparse(self.path).path

            # ---------- หน้าเว็บ ----------
            if p == "/":
                self._ok("text/html; charset=utf-8", PAGE.encode())

            # ---------- ควบคุมหุ่น ----------
            elif p == "/drive":
                mux.set(self._q("l"), self._q("r"), "web")
                self._ok("text/plain", b"ok")

            elif p == "/cmd":
                c = self._qs("c")
                if c:
                    if c == "x":
                        mux.stop("web")
                    link.send(c[0])
                self._ok("text/plain", b"ok")

            elif p == "/arm":
                q = parse_qs(urlparse(self.path).query)
                if self._qs("home"):
                    pan, tilt = arm.home()
                elif "dpan" in q or "dtilt" in q:      # สติ๊กแขนในเว็บ
                    arm.nudge(self._q("dpan"), self._q("dtilt"))
                    pan, tilt = arm.snapshot()
                else:                                   # ตั้งมุมตรง ๆ
                    cur = arm.snapshot()
                    pan, tilt = arm.set(self._q("pan", cur[0]),
                                        self._q("tilt", cur[1]))
                self._json({"pan": pan, "tilt": tilt})

            elif p == "/speed":
                v = mux.set_speed(self._q("v", SPEED_DEFAULT))
                self._ok("text/plain", str(v).encode())

            elif p == "/status":
                with mux.lock:
                    cmdl, cmdr, src = mux.l, mux.r, mux.src
                    ax, ay, spd = mux.ax, mux.ay, mux.speed
                armPan, armTilt = arm.snapshot()
                body = dict(stats.snapshot())
                body.update(soc_health())
                body.update({
                    "online": link.online,
                    "serial": link.ser is not None,
                    "cmdL": cmdl, "cmdR": cmdr, "src": src,
                    "ax": round(ax, 3), "ay": round(ay, 3), "speed": spd,
                    "curL": link.st["curL"], "curR": link.st["curR"],
                    "tt": link.st["tt"], "tt_pwm": link.st["tt_pwm"],
                    "trim1": link.st["trim1"], "trim2": link.st["trim2"],
                    "jog": link.st["jog"], "fwd": link.st["fwd"],
                    "cmd": link.st["cmd"], "turn": link.st["turn"],
                    "ratio": link.st["ratio"],
                    "rx": link.rx, "rxHz": round(link.rx_hz, 1),
                    "fields": link.fields,
                    "swap": SWAP_LR, "invL": INVERT_L, "invR": INVERT_R,
                    "trimSide": "ซ้าย" if SWAP_LR else "ขวา",
                    "pan": armPan, "tilt": armTilt,
                    # มุมที่ ESP32 จ่ายออกจริง — ไล่ตามทีละน้อยเพราะ soft start
                    "panFw": link.st["pan"], "tiltFw": link.st["tilt"],
                    "armMoving": (abs(link.st["pan"] - armPan) > 1 or
                                  abs(link.st["tilt"] - armTilt) > 1),
                    "panMin": PAN_MIN, "panMax": PAN_MAX,
                    "tiltMin": TILT_MIN, "tiltMax": TILT_MAX,
                    "armFw": link.fields >= 14,   # เฟิร์มแวร์รองรับแขนไหม
                    "note": link.note,
                    "oldFw": bool(link.fields and link.fields < 12)
                             or link.st["cmd"] in "0123456789+-[]",
                    "moving": bool(link.st["curL"] or link.st["curR"]),
                    "cam": cam.ok,
                    "lens": cam.state["lens"], "zoom": cam.state["zoom"],
                    "zoomSoft": cam.state["soft"],
                    "fom": cam.state["fom"], "fomMax": cam.state["fomMax"],
                    "afBusy": cam.state["busy"], "scan": cam.state["scan"],
                    "stillMode": cam.state["still"],
                    "store": gal.usage(),
                })
                self._json(body)

            # ---------- กล้อง ----------
            elif p == "/lens":
                if cam.ok:
                    cam.set_lens(self._q("v", 3.33))
                self._ok("text/plain", f"{cam.state['lens']:.3f}".encode())

            elif p == "/afscan":
                if cam.ok:
                    cam.focus_scan()
                    for _ in range(300):          # รอผลไม่เกิน 30 วินาที
                        time.sleep(0.1)
                        if not cam.state["busy"]:
                            break
                    self._ok("text/plain", f"{cam.state['lens']:.2f}".encode())
                else:
                    self._fail(503, "camera off")

            elif p == "/af":
                if cam.ok:
                    cam.af_once()
                    for _ in range(60):
                        if not cam.state["busy"]:
                            break
                        time.sleep(0.1)
                self._ok("text/plain", f"{cam.state['lens']:.3f}".encode())

            elif p == "/zoom":
                if cam.ok:
                    cam.apply_zoom(self._q("z", 1.0))
                self._ok("text/plain", b"ok")

            # ---------- ถ่ายภาพ -> เก็บลง Pi ----------
            elif p == "/photo":
                shot = take_photo(cam, gal, stats)
                if shot is None:
                    self._fail(503, "camera off")
                    return
                self._json({"name": shot[0], "size": shot[1]})

            # ---------- คลังภาพ ----------
            elif p == "/gallery":
                self._json({"items": gal.list(), "store": gal.usage()})

            elif p == "/pic":
                data = gal.read(self._qs("f"), thumb=False)
                if data is None:
                    self._fail(404, "not found")
                    return
                self._ok("image/jpeg", data,
                         {"Content-Disposition":
                          f'attachment; filename="{gal.safe(self._qs("f"))}"'})

            elif p == "/thumb":
                data = gal.read(self._qs("f"), thumb=True)
                if data is None:
                    self._fail(404, "not found")
                    return
                self._ok("image/jpeg", data)

            elif p == "/delete":
                ok = gal.delete(self._qs("f"))
                self._json({"ok": ok})

            # ---------- สตรีม ----------
            elif p == "/stream":
                if not cam.ok:
                    self._fail(503, "camera off")
                    return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=FRAME")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seen = 0
                try:
                    while True:
                        with cam.cond:
                            cam.cond.wait_for(lambda: cam.n != seen, timeout=5)
                            f, seen = cam.frame, cam.n
                        if f is None:
                            continue
                        stats.tick_fps(seen)
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(f)}\r\n\r\n".encode())
                        self.wfile.write(f)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            else:
                self._fail(404, "not found")

    return Handler


# ================= main =================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--serial", default=SERIAL_DEV)
    ap.add_argument("-p", "--port", type=int, default=HTTP_PORT)
    ap.add_argument("--photos", default=PHOTO_DIR, help="โฟลเดอร์เก็บภาพ")
    ap.add_argument("--pad-device", help="/dev/input/eventN")
    ap.add_argument("--no-cam", action="store_true")
    ap.add_argument("--no-pad", action="store_true")
    ap.add_argument("--pad-list", action="store_true")
    ap.add_argument("--pad-raw", action="store_true")
    ap.add_argument("--swap", action=argparse.BooleanOptionalAction,
                    default=None, help="สลับล้อซ้าย/ขวา (--swap หรือ --no-swap)")
    ap.add_argument("--invert-l", action="store_true", help="กลับทิศล้อซ้าย")
    ap.add_argument("--invert-r", action="store_true", help="กลับทิศล้อขวา")
    args = ap.parse_args()

    global SWAP_LR, INVERT_L, INVERT_R
    if args.swap is not None:
        SWAP_LR = args.swap
    if args.invert_l:
        INVERT_L = True
    if args.invert_r:
        INVERT_R = True

    if args.pad_list or args.pad_raw:
        from evdev import InputDevice, list_devices, ecodes
        if args.pad_list:
            for path in list_devices():
                try:
                    d = InputDevice(path)
                except OSError:
                    continue
                caps = d.capabilities()
                axes = len(caps.get(ecodes.EV_ABS, []))
                keys = len(caps.get(ecodes.EV_KEY, []))
                print(f"{d.path:22s} {d.name:34s} axes={axes:2d} keys={keys}")
            return
        dev = (InputDevice(args.pad_device) if args.pad_device
               else pad_find(ecodes, InputDevice, list_devices))
        if dev is None:
            print("ไม่เจอจอย")
            return
        print(f"อ่านจาก {dev.name}  (Ctrl+C เพื่อออก)\n")
        for ev in dev.read_loop():
            if ev.type == ecodes.EV_ABS:
                print(f"ABS  {ecodes.ABS.get(ev.code, ev.code):12} = {ev.value}")
            elif ev.type == ecodes.EV_KEY and ev.value == 1:
                name = ecodes.BTN.get(ev.code) or ecodes.KEY.get(ev.code, ev.code)
                print(f"BTN  {name}")
        return

    os.makedirs(args.photos, exist_ok=True)
    gal = Gallery(args.photos)
    stats = Stats()

    cam = Camera()
    if not args.no_cam:
        try:
            cam.start()
        except Exception as e:
            print(f"camera: เปิดไม่ได้ ({e}) — รันต่อโดยไม่มีภาพ")

    link = RobotLink(args.serial, BAUD)
    mux = DriveMux(link, stats)
    arm = Arm(link)
    mux.arm = arm              # drive loop เป็นคนส่ง #S ให้แขนด้วย

    threading.Thread(target=mux.loop, daemon=True).start()
    if not args.no_pad:
        threading.Thread(target=pad_thread,
                         args=(mux, link, stats, arm, cam, gal,
                               args.pad_device),
                         daemon=True).start()

    srv = QuietServer(("0.0.0.0", args.port),
                      make_handler(cam, link, mux, gal, stats, arm))
    srv.daemon_threads = True
    print(f"photos: {args.photos}")
    print(f"web:    http://<pi-ip>:{args.port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down")
        mux.stop()
        link.drive(0, 0)
        link.send("x")
        cam.stop()


if __name__ == "__main__":
    main()