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
import base64
import glob
import hmac
import io
import json
import os
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial

# ---------------- ตั้งค่า ----------------
# พอร์ตของ ESP32-S3 — ใส่ "auto" แทนได้ ถ้าอยากให้หาเองจาก by-id / ttyACM* / ttyUSB*
# (เลขท้าย ttyACM เลื่อนได้เวลาเสียบใหม่หรือมีอุปกรณ์ USB อื่นมาแย่งเลข 0)
SERIAL_DEV = os.environ.get("ROBOT_SERIAL", "/dev/ttyACM0")
BAUD = 115200
HTTP_PORT = 9000

# ---------------- รหัสผ่านเข้าหน้าเว็บ ----------------
# ใช้ HTTP Basic Auth เบราว์เซอร์จะเด้งกล่องให้กรอกเอง และส่งรหัสซ้ำให้ทุก
# request รวมถึง <img src="/stream"> ด้วย — ภาพจากกล้องจึงถูกป้องกันไปด้วย
# ตั้ง ROBOT_PASS="" เพื่อปิดระบบรหัสผ่าน
AUTH_USER = os.environ.get("ROBOT_USER", "admin")
AUTH_PASS = os.environ.get("ROBOT_PASS", "12345678")
AUTH_REALM = "Robot"
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
SWAP_LR = False     # ช่อง M1 ต่อล้อซ้าย / M2 ต่อล้อขวา — ต่อตรงอยู่แล้ว
INVERT_L = True     # กลับทิศล้อซ้าย  — มอเตอร์ต่อกลับขั้ว
INVERT_R = True     # กลับทิศล้อขวา   — มอเตอร์ต่อกลับขั้ว
# ★ การแก้ทิศล้อทั้งหมดอยู่ที่นี่ที่เดียว ใน main.cpp ต้องเป็น false ทั้งสามตัว
#   ไม่งั้นจะกลับสองรอบแล้วเท่ากับไม่ได้แก้
#
# อาการจริงคือมอเตอร์ทั้งสองตัวต่อกลับขั้ว ส่วนช่อง M1/M2 ไม่ได้สลับ
# ตอนแรกเข้าใจผิดว่าสลับช่อง เพราะ "มอเตอร์กลับขั้วทั้งคู่" ทำให้ทิศเลี้ยวผิด
# ซึ่งดูเหมือนอาการสลับช่องเป๊ะ ๆ  แต่สองอย่างนี้ต่างกันตรงเดินหน้า/ถอยหลัง:
#   สลับช่อง       -> เลี้ยวผิด  เดินหน้าถูก
#   กลับขั้วทั้งคู่ -> เลี้ยวผิด  เดินหน้าผิดด้วย   <- เคสนี้
# เวลาไล่ปัญหาทิศล้อ ให้ทดสอบ "เดินหน้า" ก่อนเสมอ แล้วค่อยทดสอบ "เลี้ยว"


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
    "estop": "BTN_EAST",       # B -> เข้า/ออกโหมด cleaner (เดิมคือหยุดล้อ)
    "focus": "BTN_TL",         # L -> โฟกัสกล้อง
    "shot": "BTN_TR",          # R -> ถ่ายรูป
    # ★ ชื่อปุ่มในเคอร์เนลไม่ตรงกับตำแหน่งบนจอย Xbox:
    #   ปุ่ม X ที่อยู่ "ซ้าย" ถูกรายงานเป็น BTN_NORTH (0x133)
    #   ปุ่ม Y ที่อยู่ "บน"  ถูกรายงานเป็น BTN_WEST  (0x134)
    #   ถ้าจอยของคุณสลับกัน ให้รัน --pad-raw ดูชื่อจริงแล้วสลับสองบรรทัดนี้
    "focus_near": "BTN_NORTH",  # X -> เพิ่มโฟกัส (เลื่อนเข้าใกล้)
    "focus_far": "BTN_WEST",    # Y -> ลดโฟกัส (เลื่อนออกไกล)
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
LENS_STEP = 0.25       # X/Y ปรับโฟกัสทีละกี่ไดออปเตอร์ (1/เมตร)
LENS_REPEAT = 0.12     # กดค้างแล้วซ้ำถี่เท่าซูม

# ---------------- กล้อง ----------------
# ---------------- แบนด์วิดท์ของสตรีม ----------------
# MJPEG ไม่มีการบีบอัดข้ามเฟรม ทุกเฟรมคือ JPEG เต็มใบ
# 1280x720 ที่ 15fps กินราว 10-18 Mbit/s ซึ่งชน WiFi กลางแจ้งพอดี
# ★ ถ้าภาพค้างบ่อย ลดสองค่านี้ช่วยได้มากที่สุด:
#     960x540  -> พิกเซลลดลง 44%  แบนด์วิดท์เหลือราวครึ่ง
#     640x480  -> เหลือราว 1/4  แต่อ่านตัวเลขมิเตอร์จากภาพสตรีมยากขึ้น
#   (ภาพที่กดชัตเตอร์ถ่ายยังเต็มเซนเซอร์เหมือนเดิม ไม่เกี่ยวกับค่านี้)
STREAM_SIZE = (1280, 720)
STREAM_QUALITY = 75         # คุณภาพ JPEG 1-100 — ลดเป็น 60 ประหยัดอีกราว 30%
# เพดาน fps ของสตรีม — ปรับสดได้จากหน้าเว็บ ไม่ต้องรีสตาร์ต
# แบนด์วิดท์ = fps x ขนาดเฟรม  ลด fps ครึ่งหนึ่ง = แบนด์วิดท์ครึ่งหนึ่ง
# และยังได้รายละเอียดเต็มทุกเฟรมที่ส่ง (ต่างจากลดความละเอียดที่เสียรายละเอียด)
STREAM_FPS = 12.0
# จำนวนบัฟเฟอร์ของกล้อง — ยิ่งน้อยยิ่งหน่วงน้อย แต่ถ้าน้อยเกินเฟรมจะตก
# 2 คือต่ำสุดที่ libcamera ยังทำงานได้ (ตัวหนึ่งกำลังถ่าย ตัวหนึ่งกำลังอ่าน)
# ถ้าตั้ง 2 แล้วเฟรมตกเยอะ (fps ส่งออกต่ำกว่าเพดาน) ให้ขึ้นเป็น 3
STREAM_BUFFERS = int(os.environ.get("ROBOT_BUFFERS", "2"))
# เพดาน buffer ส่งของ socket ต่อหนึ่งการเชื่อมต่อ (ไบต์)
# ★ นี่คือเพดาน "ความเก่าของภาพ" ไม่ใช่แค่เรื่องหน่วยความจำ
#   Linux ขยาย buffer นี้เองได้ถึงหลายเมกะไบต์ = กองเฟรมได้หลายสิบใบ
#   ทุกใบที่กองอยู่คือความเก่าเพิ่ม 1/fps และมันโตขึ้นเรื่อย ๆ ไม่มีเพดาน
#   วัดแล้ว: buffer 4MB -> ภาพเก่าขึ้นจาก 3 วิ เป็น 21 วิ ใน 30 วินาที
#            buffer 32KB -> ค้างที่ 0.8 วิ ไม่โตขึ้น
#   0 = ปรับตามขนาดเฟรมจริง (ประมาณหนึ่งเฟรม อย่างน้อย 32KB)
#       kernel มักให้จริงเป็นสองเท่าของที่ขอ -> ได้ ~2 เฟรมพอดี
#   ใส่เลขเองได้ทาง ROBOT_SNDBUF ถ้าอยากทดลอง
STREAM_SNDBUF = int(os.environ.get("ROBOT_SNDBUF", "0"))
SNDBUF_FLOOR = 32 * 1024
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

# ---------------- โหมด cleaner ----------------
# กด B บนจอย (หรือปุ่มในเว็บ) = เข้า/ออกโหมดนี้
#   เข้าโหมด -> แขนวิ่งไปตำแหน่งกวาด แล้ว "ล็อกแกนหันซ้าย/ขวา" ไว้
#   เหลือแค่ก้ม/เงย  ออกได้ทางเดียวคือกด B ซ้ำ
CLEAN_PAN, CLEAN_TILT = 87, 33

# ---------------- HUD บนภาพกล้อง ----------------
# กล้องอยู่บนแขนที่หันได้ แนวกลางหุ่น (จุดที่แปรงปัดขยะเข้า) จึงเลื่อนไปมา
# ในภาพตามมุม pan  ต้องรู้มุมมองของกล้องถึงจะคำนวณตำแหน่งได้
# ★ วัดเองได้: วางไม้บรรทัดห่างกล้อง 1 เมตร ดูว่าภาพกว้างกี่เซนติเมตร
#   HFOV = 2 * atan((กว้าง/2) / 100) องศา
#   Pi Camera v2 ~62°, v3 ~66°, เลนส์มุมกว้าง ~102°
CAM_HFOV = 66.0

# PAN_FORWARD = มุม pan ที่กล้องมองตรงไปตามแนวกลางหุ่นพอดี
# ★ ไม่ต้องเดาเอง — กดปุ่ม "ตั้งเส้นกลาง" ในเว็บ แล้วแตะบนภาพตรงกลาง
#   ระหว่างแปรงสองตัว ระบบจะคำนวณค่านี้ให้แล้วจำไว้ในไฟล์
CALIB_FILE = os.path.expanduser(os.environ.get("ROBOT_CALIB", "~/robot_calib.json"))
PAN_FORWARD = 90.0


def calib_load():
    """อ่านค่าที่ตั้งไว้ครั้งก่อน — ไม่มีไฟล์ก็ใช้ค่าเริ่มต้น ไม่ต้องตกใจ"""
    global PAN_FORWARD, STREAM_FPS
    try:
        with open(CALIB_FILE, encoding="utf-8") as f:
            d = json.load(f)
        PAN_FORWARD = min(180.0, max(0.0, float(d.get("panFwd", PAN_FORWARD))))
        STREAM_FPS = min(30.0, max(1.0, float(d.get("streamFps", STREAM_FPS))))
        print(f"calib: เส้นกลาง {PAN_FORWARD:.1f}°  สตรีม {STREAM_FPS:.0f} fps "
              f"({CALIB_FILE})")
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        pass
    return PAN_FORWARD


def calib_write():
    try:
        with open(CALIB_FILE, "w", encoding="utf-8") as f:
            json.dump({"panFwd": round(PAN_FORWARD, 2),
                       "streamFps": round(STREAM_FPS, 1)}, f)
    except OSError as e:
        print(f"calib: เซฟไม่ได้ ({e}) — ค่าจะหายเมื่อรีสตาร์ต")


def calib_save(v):
    global PAN_FORWARD
    PAN_FORWARD = min(180.0, max(0.0, float(v)))
    calib_write()
    return PAN_FORWARD


def frame_limits(fps=None):
    """ช่วงเวลาต่อเฟรมที่กล้องต้องยึด (ไมโครวินาที) — ตรึงหัวท้ายให้เท่ากัน
    เพื่อบังคับเฟรมเรตจริง ไม่ปล่อยให้ ISP เร่งขึ้นเองตอนแสงดี

    ผลพลอยได้: เพดานเวลาเปิดหน้ากล้องเท่ากับช่วงนี้ ที่ 12fps = 83ms
    ซึ่งนานพอสำหรับงานกลางแจ้งอยู่แล้ว ไม่ต้องห่วงเรื่องภาพมืด"""
    f = max(1.0, float(STREAM_FPS if fps is None else fps))
    us = int(round(1000000.0 / f))
    return (us, us)


def stream_fps_save(v, camera=None):
    """camera ต้องส่งเข้ามา — ตัว cam เป็นตัวแปรใน main() ไม่ใช่ global
    ฟังก์ชันนี้มองไม่เห็น ถ้าลืมส่งก็แค่ไม่ลดงานกล้อง ไม่พังอย่างอื่น"""
    global STREAM_FPS
    STREAM_FPS = min(30.0, max(1.0, float(v)))
    calib_write()
    # ลดเพดานส่งออกแล้วต้องลดงานเข้ารหัสด้วย ไม่งั้นซีพียูยังทำงานเท่าเดิม
    try:
        if camera is not None and camera.ok:
            camera.set_fps(STREAM_FPS)
    except Exception as e:
        print("stream fps -> camera:", e)
    return STREAM_FPS


# ================= ลิงก์ไป ESP32 =================
class RobotLink:
    """เปิด serial ตัวเดียว ใช้ร่วมกันทุก thread"""

    def __init__(self, port, baud):
        self.want = port          # "auto" = ให้ _find_port() หาเอง
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.buf = b""
        # สถานะที่ ESP32 รายงานกลับ
        self.st = {"cmd": "x", "tt": 0, "tt_pwm": 0, "tt_out": None,
                   "fwd": 0, "turn": 0,
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

    def _find_port(self):
        """หาพอร์ต ESP32 เอง — เลขท้าย ttyACM เลื่อนได้ทุกครั้งที่เสียบใหม่

        ลำดับที่ลอง:
          1. /dev/serial/by-id/*Espressif*  (ผูกกับ serial number ของชิป ไม่มีวันเลื่อน)
          2. /dev/ttyACM*  ตัวแรกที่เจอ
          3. /dev/ttyUSB*  (เผื่อเสียบช่อง UART ที่ผ่านชิป USB-to-serial)
        """
        if self.want != "auto":
            return self.want
        for pat in ("/dev/serial/by-id/*Espressif*",
                    "/dev/serial/by-id/*303a*",
                    "/dev/ttyACM*", "/dev/ttyUSB*"):
            hits = sorted(glob.glob(pat))
            if hits:
                return hits[0]
        return None

    def _open(self):
        self.port = self._find_port()
        if self.port is None:
            self.ser = None
            self._next_try = time.time() + 2.0
            msg = "ไม่เจอพอร์ต ESP32 (ลองแล้วทั้ง by-id, ttyACM*, ttyUSB*)"
            if msg != self._err:
                self._err = msg
                print(f"serial: {msg} — จะลองใหม่ทุก 2 วินาที")
            return
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
            if len(f) >= 15:             # v7 ขึ้นไป — duty ที่จ่ายออกจริง
                self.st["tt_out"] = int(f[14])
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
        self.cleaner = False     # โหมดกวาด: ล็อกแกนหันซ้าย/ขวา
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
            if self.cleaner:
                dpan = 0.0        # โหมด cleaner — แกนหันถูกล็อก
            self.pan = min(PAN_MAX, max(PAN_MIN, self.pan + dpan))
            self.tilt = min(TILT_MAX, max(TILT_MIN, self.tilt + dtilt))
            self.dirty = True

    def set(self, pan, tilt):
        """ตั้งมุมตรง ๆ (สไลเดอร์ในเว็บ) — ในโหมด cleaner จะขยับได้แต่ก้ม/เงย"""
        with self.lock:
            if not self.cleaner:
                self.pan = min(PAN_MAX, max(PAN_MIN, float(pan)))
            self.tilt = min(TILT_MAX, max(TILT_MIN, float(tilt)))
            self.dirty = True
            return int(round(self.pan)), int(round(self.tilt))

    def home(self):
        return self.set(PAN_HOME, TILT_HOME)

    # ---- โหมด cleaner ----
    def _cleaner_locked(self, on):
        """ต้องถือ self.lock อยู่แล้วก่อนเรียก"""
        self.cleaner = bool(on)
        if self.cleaner:
            self.pan = float(min(PAN_MAX, max(PAN_MIN, CLEAN_PAN)))
            self.tilt = float(min(TILT_MAX, max(TILT_MIN, CLEAN_TILT)))
            self.dirty = True
        return self.cleaner

    def set_cleaner(self, on):
        with self.lock:
            return self._cleaner_locked(on)

    def toggle_cleaner(self):
        with self.lock:
            return self._cleaner_locked(not self.cleaner)

    def snapshot(self):
        with self.lock:
            return int(round(self.pan)), int(round(self.tilt))

    def state(self):
        with self.lock:
            return int(round(self.pan)), int(round(self.tilt)), self.cleaner

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


# ================= ตัวนับเฟรมต่อการเชื่อมต่อสตรีม =================
# ★ ทำไมต้องมี: <img src="/stream"> ไม่มีทางรู้ว่าไบต์ยังไหลเข้ามาไหม
#   ถ้า WiFi หลุดครึ่งใบ เบราว์เซอร์ค้างเฟรมสุดท้ายไว้เฉย ๆ ไม่ยิง error
#   วิธีเดิมคือย่อภาพเป็น 32x18 แล้วดูว่าพิกเซลเปลี่ยนไหม — ใช้ไม่ได้
#   เพราะย่อขนาดนั้นมันเฉลี่ยพิกเซลทีละบล็อก 40x40 noise หายเกลี้ยง
#   ฉากนิ่ง ๆ (หุ่นจอด กล้องส่องกำแพง) ให้ลายนิ้วมือเดิมเป๊ะ -> เตือนผิด
#   วิธีนี้แทน: เว็บสร้างรหัสประจำการเชื่อมต่อ ส่งมาเป็น /stream?id=xxx
#   ฝั่งนี้นับทุกเฟรมที่ "เขียนออกไปสำเร็จ" ของ id นั้น แล้วบอกกลับทาง /status
#   ตัวเลขเดิน = สายยังดี / ตัวเลขหยุด = สายตายจริง  ไม่เกี่ยวกับภาพนิ่งหรือไม่
class StreamTally:
    TTL = 60.0          # ทิ้ง id ที่เงียบเกินหนึ่งนาที กันดิกชันนารีบวม

    def __init__(self):
        self.lock = threading.Lock()
        self.n = {}      # id -> [จำนวนเฟรมที่ส่งออกสำเร็จ, เวลาล่าสุด]

    def tick(self, sid):
        if not sid:
            return
        with self.lock:
            e = self.n.get(sid)
            if e is None:
                self.n[sid] = [1, time.time()]
            else:
                e[0] += 1
                e[1] = time.time()

    def get(self, sid):
        """คืนจำนวนเฟรมของ id นั้น — None = ยังไม่เคยส่งอะไรให้ id นี้เลย"""
        if not sid:
            return None
        now = time.time()
        with self.lock:
            for k in [k for k, v in self.n.items() if now - v[1] > self.TTL]:
                del self.n[k]
            e = self.n.get(sid)
            return e[0] if e else None


tally = StreamTally()


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
        self.tx_bytes = 0          # ไบต์ที่ส่งออกไปทางสตรีมทั้งหมด
        self._tx0 = 0
        self.kbps = 0.0            # อัตราจริงที่วัดได้ (กิโลบิต/วินาที)
        self.drops = 0             # เฟรมที่ทิ้งเพราะปลายทางรับไม่ทัน
        self.drop_hz = 0.0
        self._dp0 = 0
        self._dp_t = time.time()
        self.lag_ms = 0.0          # เฟรมรอที่ฝั่ง Pi นานเท่าไหร่ก่อนถูกส่ง
        self.lag_max = 0.0         # ค่าแย่สุดในหน้าต่างล่าสุด
        self._lag_n = 0
        self._lag_sum = 0.0
        self._lag_pk = 0.0
        self._lag_t = time.time()
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

    def tick_drop(self):
        """นับเฟรมที่ถูกทิ้งเพราะปลายทางรับไม่ทัน
        ตัวเลขนี้คือสัญญาณเตือนล่วงหน้าว่าภาพกำลังจะเริ่มแล็ค
        ถ้ามันเดินเรื่อย ๆ แปลว่าเบราว์เซอร์หรือเน็ตตามไม่ทันที่เราส่ง"""
        self.drops += 1
        now = time.time()
        if now - self._dp_t >= 1.0:
            self.drop_hz = (self.drops - self._dp0) / (now - self._dp_t)
            self._dp0, self._dp_t = self.drops, now

    def tick_lag(self, ms):
        """อายุของเฟรมตอนถูกส่งออก = ความหน่วงที่เกิดขึ้นบน Pi เท่านั้น
        ไม่รวมเน็ตและเบราว์เซอร์ ถ้าตัวเลขนี้ต่ำแต่ภาพยังช้า
        แปลว่าหน่วงอยู่หลัง Pi ไม่ใช่ที่โค้ดนี้"""
        self._lag_n += 1
        self._lag_sum += ms
        if ms > self._lag_pk:
            self._lag_pk = ms
        now = time.time()
        if now - self._lag_t >= 1.0:
            self.lag_ms = self._lag_sum / max(1, self._lag_n)
            self.lag_max = self._lag_pk
            self._lag_n, self._lag_sum, self._lag_pk = 0, 0.0, 0.0
            self._lag_t = now

    def tick_fps(self, frames, tx=0):
        """เรียกทุกเฟรมที่ส่งออกไปจริง — วัดทั้ง fps และแบนด์วิดท์"""
        self.tx_bytes += tx
        now = time.time()
        dt = now - self._ft
        if dt >= 1.0:
            self.fps = (frames - self._frames0) / dt
            self.kbps = (self.tx_bytes - self._tx0) * 8 / 1000.0 / dt
            self._frames0, self._ft = frames, now
            self._tx0 = self.tx_bytes

    def snapshot(self):
        with self.lock:
            n, ok = self.link_n, self.link_ok
            return {
                "up": int(time.time() - self.t0),
                "driveSec": int(self.drive_ms),
                "photos": self.photos,
                "reconnects": self.reconnects,
                "fps": round(self.fps, 1),
                "kbps": round(self.kbps),
                "lagMs": round(self.lag_ms),
                "lagMax": round(self.lag_max),
                "drops": self.drops,
                "dropHz": round(self.drop_hz, 1),
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
        self.frame_t = 0.0        # เวลาที่เฟรมนี้ออกจากตัวเข้ารหัส
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
                    outer.frame_t = time.time()   # ใช้วัดความหน่วงฝั่ง Pi
                    outer.n += 1
                    outer.cond.notify_all()

        self.picam2 = Picamera2()
        # ★★ สำคัญที่สุดสำหรับ Pi 5: จำกัดเฟรมเรตของ "ตัวกล้อง" ด้วย ★★
        #   Pi 5 ไม่มีตัวเข้ารหัสวิดีโอฮาร์ดแวร์ (Raspberry Pi ยืนยันเอง)
        #   JpegEncoder จึงบีบอัดด้วยซีพียูทุกเฟรมที่เซนเซอร์ส่งมา
        #   ถ้าไม่ตั้งอะไร กล้องวิ่ง 30fps -> เข้ารหัส 30 ใบ/วินาที
        #   แต่ /stream ส่งออกแค่ STREAM_FPS ใบ ที่เหลือโดนทิ้ง
        #   = จ่ายค่าซีพียูเต็ม แต่ได้ภาพแค่เสี้ยวเดียว
        #   ตั้ง FrameDurationLimits ให้ตรงกับที่ส่งออก ก็หยุดเข้ารหัสทิ้ง
        cfg = self.picam2.create_video_configuration(
            main={"size": STREAM_SIZE},
            transform=Transform(hflip=1, vflip=1),
            # ★ ความหน่วง: คิวของกล้องนับเป็น "จำนวนเฟรม" ไม่ใช่เวลา
            #   ที่ 12fps หนึ่งเฟรมในคิว = หน่วง 83ms
            #   ค่าเริ่มต้นของโหมดวิดีโอตั้งไว้สูงเพื่อกันเฟรมตก
            #   แต่หุ่นที่ต้องขับตามภาพต้องการภาพสด ไม่ใช่ภาพครบ
            #   ยอมให้ตกบ้างดีกว่าเห็นอดีต -> ลดคิวให้เหลือน้อยที่สุดที่ยังเดินได้
            buffer_count=STREAM_BUFFERS,
            controls={"FrameDurationLimits": frame_limits()})
        self.picam2.configure(cfg)
        try:
            print(f"camera: คิวเฟรม {cfg.get('buffer_count')} เฟรม "
                  f"(~{1000.0 * (cfg.get('buffer_count') or 0) / max(1.0, STREAM_FPS):.0f}ms "
                  f"ที่ {STREAM_FPS:.0f}fps)")
        except Exception:
            pass
        self.FULL = self.picam2.camera_properties["ScalerCropMaximum"]
        self.crop = tuple(self.FULL)
        # เกินเท่านี้คือขยายพิกเซลล้วน ๆ ภาพจะเริ่มเบลอ
        self.state["soft"] = round(self.FULL[2] / float(STREAM_SIZE[0]), 1)
        # ขนาดภาพนิ่ง — ใช้เต็มเซนเซอร์ถ้าไม่ได้กำหนด
        self.still_size = STILL_SIZE or tuple(
            self.picam2.camera_properties.get("PixelArraySize", self.FULL[2:4]))

        self.picam2.start_recording(JpegEncoder(q=STREAM_QUALITY),
                                    FileOutput(StreamOut()))
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
    def set_fps(self, fps):
        """เปลี่ยนเฟรมเรตของตัวกล้องแบบสด ๆ ไม่ต้องตั้งค่าใหม่ทั้งชุด
        เรียกจาก /streamfps เพื่อให้งานเข้ารหัสลดตามเพดานที่ส่งออกจริง"""
        if not self.ok:
            return
        try:
            self.picam2.set_controls({"FrameDurationLimits": frame_limits(fps)})
            print(f"camera: เฟรมเรตกล้อง -> {float(fps):.1f} fps "
                  f"(เข้ารหัสเท่าที่ส่งออก)")
        except Exception as e:
            print("set_fps:", e)

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
        from picamera2.encoders import JpegEncoder
        from picamera2.outputs import FileOutput

        # ★ ระหว่างนี้กล้องหยุดส่งเฟรมจริง ๆ ภาพในเว็บจะค้าง 1-3 วินาที
        #   ต้องบอกหน้าเว็บด้วย ไม่งั้นยามเฝ้าสตรีมจะเตือนว่า "กล้องหยุดส่งภาพ"
        #   ทั้งที่เป็นเรื่องปกติของการกดชัตเตอร์
        self.state["busy"] = True

        # 1. หยุดบันทึกสตรีมวิดีโอชั่วคราว ป้องกัน Thread ชนกันตอนสลับโหมด
        try:
            self.picam2.stop_recording()
        except Exception:
            pass

        cfg = self.picam2.create_still_configuration(
            main={"size": self.still_size},
            transform=self.Transform(hflip=1, vflip=1),
            controls={"ScalerCrop": self.crop,
                      "AfMode": self.controls.AfModeEnum.Manual,
                      "LensPosition": self.state["lens"]})
        buf = io.BytesIO()
        
        try:
            self.picam2.switch_mode_and_capture_file(cfg, buf, format="jpeg")
        finally:
            # 2. เริ่มสตรีมวิดีโอใหม่อีกครั้ง
            outer = self
            class StreamOut(io.BufferedIOBase):
                def write(self, b):
                    with outer.cond:
                        outer.frame = b
                        outer.n += 1
                        outer.cond.notify_all()
            
            self.picam2.start_recording(JpegEncoder(q=STREAM_QUALITY),
                                        FileOutput(StreamOut()))
            # ตอกเฟรมเรตกลับหลังสลับโหมด — โหมดภาพนิ่งมี FrameDurationLimits
            # ของตัวเอง ถ้าไม่ตอกกลับ กล้องอาจกลับไปวิ่ง 30fps เงียบ ๆ
            # แล้วซีพียูก็กลับไปเข้ารหัสทิ้งเหมือนเดิมโดยไม่มีใครรู้
            try:
                self.picam2.set_controls({"FrameDurationLimits": frame_limits()})
            except Exception:
                pass
            self.state["busy"] = False

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
    # X / Y ปรับโฟกัสด้วยมือ — เก็บสถานะกดค้างไว้เพื่อให้ซ้ำได้
    lens_btn = {code["focus_near"]: 0, code["focus_far"]: 0}
    lens_next = {code["focus_near"]: 0.0, code["focus_far"]: 0.0}
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

    def bump_lens(c):
        """X = เพิ่มค่า LensPosition (โฟกัสใกล้ขึ้น) / Y = ลดค่า (ไกลขึ้น)
        set_lens() จะสลับกล้องเป็นโหมดแมนนวลให้เอง และคุมช่วง 0-10 ไว้แล้ว"""
        if not cam.ok:
            return
        step = LENS_STEP if c == code["focus_near"] else -LENS_STEP
        cam.set_lens(cam.state["lens"] + step)

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

            elif ev.type == ecodes.EV_KEY and ev.code in lens_btn:
                # X / Y: ทำทันทีตอนกด แล้วค่อยซ้ำถ้ายังกดค้าง
                # (เหมือน D-pad — ถ้ารอให้ถึงรอบ tick การแตะสั้น ๆ จะหลุด)
                lens_btn[ev.code] = ev.value
                if ev.value == 1:
                    bump_lens(ev.code)
                    lens_next[ev.code] = time.time() + REPEAT_DELAY
                elif ev.value == 0:
                    lens_next[ev.code] = 0.0

            elif ev.type == ecodes.EV_KEY and ev.value == 1:
                if ev.code == code["tt_toggle"]:        # A
                    link.send("k")
                elif ev.code == code["estop"]:          # B -> เข้า/ออกโหมด cleaner
                    on = arm.toggle_cleaner()
                    print("pad: โหมด cleaner", "เปิด" if on else "ปิด")
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

        # ---- X / Y กดค้าง -> ปรับโฟกัสต่อเนื่อง ----
        for c in lens_btn:
            if lens_btn[c]:
                if now >= lens_next[c]:
                    bump_lens(c)
                    lens_next[c] = now + LENS_REPEAT
            else:
                lens_next[c] = 0.0

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
# ================= หน้าวัดความหน่วงกระจกถึงกระจก =================
# ทำไมต้องวัดแบบนี้: ตัวเลขฝั่งเซิร์ฟเวอร์บอกได้แค่ว่าโค้ดเราหน่วงเท่าไหร่
# แต่ความหน่วงที่ตาคนเห็นรวมคิวของกล้อง, ISP, เน็ต, และการถอดภาพในเบราว์เซอร์
# ซึ่งวัดจากข้างในไม่ได้เลย ต้องให้กล้องส่องนาฬิกาแล้วเทียบตัวเลขสองตัว
LAGPAGE = """<!DOCTYPE html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>วัดความหน่วง</title><style>
*{box-sizing:border-box}
body{margin:0;background:#000;color:#fff;font-family:ui-monospace,monospace;
     display:flex;flex-direction:column;height:100vh;overflow:hidden}
.top{display:flex;align-items:center;gap:16px;padding:10px 14px;
     background:#111;border-bottom:1px solid #333;flex-wrap:wrap}
#clock{font-size:clamp(40px,9vw,110px);font-weight:700;letter-spacing:.04em;
       color:#0f0;line-height:1;font-variant-numeric:tabular-nums}
.hint{font-size:13px;color:#aaa;max-width:46ch;line-height:1.5;
      font-family:system-ui,sans-serif}
.wrap{flex:1;position:relative;min-height:0;background:#000}
img{width:100%;height:100%;object-fit:contain;display:block}
.srv{position:absolute;left:10px;bottom:10px;background:#000b;padding:6px 10px;
     border-radius:8px;font-size:13px;color:#8f8}
</style></head><body>
<div class="top">
  <div id="clock">0.000</div>
  <div class="hint">เอากล้องหุ่นส่องมาที่จอนี้ให้เห็นเลขเขียว แล้ว
    <b>ถ่ายรูปจอทั้งหน้า</b> ด้วยมือถือ<br>
    เลขในภาพสดข้างล่าง = อดีต &nbsp;|&nbsp; เลขเขียว = ปัจจุบัน<br>
    <b>ผลต่าง = ความหน่วงจริง</b> (หน่วยวินาที)</div>
</div>
<div class="wrap">
  <img id="v" alt="ภาพสด">
  <div class="srv" id="srv">—</div>
</div>
<script>
/* นาฬิกาเดินด้วย rAF ให้ลื่นที่สุดเท่าที่จอทำได้ ไม่ใช้ setInterval
   เพราะ setInterval จะกระตุกและทำให้อ่านค่าคลาดได้ */
const t0=performance.now(), cl=document.getElementById('clock');
(function tick(){
  cl.textContent=((performance.now()-t0)/1000).toFixed(3);
  requestAnimationFrame(tick);
})();
document.getElementById('v').src='/stream?id=lag'+Date.now();
setInterval(async()=>{
  try{
    const s=await(await fetch('/status?id=lagprobe')).json();
    document.getElementById('srv').textContent=
      `ฝั่งหุ่น: หน่วง ${s.lagMs}ms (แย่สุด ${s.lagMax}ms) | `+
      `ส่งออก ${s.fps}fps | กล้องคิว ${s.buffers||'?'} เฟรม`;
  }catch(e){}
},1000);
</script></body></html>"""


PAGE = """<!DOCTYPE html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#0A0908">
<title>Robot cockpit</title>
<!-- Inter Variable: โหลดแบบไม่บล็อกการแสดงผล ถ้าหุ่นออฟไลน์อยู่หน้าเว็บก็ขึ้นทันที
     แล้วตกไปใช้ฟอนต์ระบบแทน (ข้อความไทยใช้ Noto Sans Thai อยู่แล้ว) -->
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" media="print" onload="this.media='all'"
      href="https://fonts.googleapis.com/css2?family=Inter:ital,opsz,wght@0,14..32,400;0,14..32,500;0,14..32,700&display=swap">
<style>
/* ===================================================================
   ระบบสี/ตัวอักษรตาม DESIGN.md  (Framer dark canvas)
   แก้สีทั้งหน้าได้จากบล็อกนี้บล็อกเดียว
   =================================================================== */
:root{
  /* ---- พื้นผิว: canvas -> surface-1 -> surface-2 คือการไล่ระดับความสำคัญ
         บนพื้นมืด ใช้ "ยกพื้นผิว" บอกลำดับชั้น ไม่ใช่ลดความทึบของตัวอักษร ---- */
  --canvas:#0A0908;        /* ดำอมอุ่น — พื้นหลังหลัก */
  --surface-1:#171615;     /* การ์ด ปุ่มรอง แผงย่อย */
  --surface-2:#232221;     /* การ์ดเด่น แท็บที่เลือกอยู่ */
  --hairline:rgba(255,255,255,.12);
  --hairline-soft:rgba(255,255,255,.07);

  /* ---- ตัวอักษร: มีแค่สองระดับ ink กับ ink-muted ไม่มีเทากลางอื่น ---- */
  --ink:#FFFFFF;
  --ink-muted:#999999;
  --on-primary:#0A0908;    /* ตัวอักษรบนปุ่มขาว */

  /* ---- สีเน้นเดียวของระบบ: ใช้กับลิงก์ โฟกัส และสถานะที่ถูกเลือกเท่านั้น
         ห้ามเอาไปถมเป็นพื้นปุ่มหรือพื้นหลัง ---- */
  --accent:#0099FF;
  --accent-ring:rgba(0,153,255,.15);

  /* ---- สีบอกสถานะ ----
     DESIGN.md เป็นสเปกหน้าเว็บการตลาด จึงมีแค่เขียว success
     แต่หน้านี้คือแผงควบคุมหุ่น สถานะอันตรายต้องอ่านออกทันทีแม้ชำเลืองมอง
     จึงเพิ่มแดงกับเหลืองเข้ามาเป็น "สีสัญญาณ" ไม่ใช่สีแบรนด์
     ใช้เฉพาะกับไฟสถานะ ปุ่มหยุดฉุกเฉิน และคำเตือน */
  --ok:#30C85E;
  --danger:#FF4A3D;
  --caution:#F0A500;

  /* ---- การ์ดเรืองแสง (ลายเซ็นของระบบ) ใช้ทั้งหน้าแค่ใบเดียว ---- */
  --grad-violet:linear-gradient(145deg,#6F3BFF 0%,#A932E8 52%,#D4349B 100%);

  /* ---- มุมโค้ง ---- */
  --r-xs:4px; --r-sm:6px; --r-md:10px; --r-lg:15px;
  --r-xl:20px; --r-xxl:30px; --r-pill:100px; --r-full:9999px;

  /* ---- ระยะห่าง ฐาน 5px ---- */
  --s-xxs:4px; --s-xs:8px; --s-sm:12px; --s-md:15px; --s-lg:20px; --s-xl:30px;

  /* ---- กระจกฝ้าสำหรับชั้นลอยบนภาพกล้อง ---- */
  --glass:rgba(10,9,8,.66);
  --blur:blur(14px) saturate(1.1);

  --pad:calc(10px + env(safe-area-inset-left,0px));
  box-sizing:border-box;
}
*,*::before,*::after{box-sizing:inherit}
html,body{height:100%;overflow:hidden}
body{
  margin:0;background:var(--canvas);color:var(--ink);
  /* Inter พร้อม character variant ตามสเปก — เป็นเสียงของแบรนด์
     Noto Sans Thai ต้องอยู่ในสแต็กด้วย เพราะ Inter ไม่มีสระวรรณยุกต์ไทย */
  font-family:"Inter var",Inter,ui-sans-serif,system-ui,"Segoe UI",Roboto,
              "Noto Sans Thai",sans-serif;
  font-size:15px; font-weight:400; line-height:1.3; letter-spacing:-.15px;
  font-feature-settings:"cv01","cv05","cv09","cv11","ss03","ss07","dlig","tnum";
  font-variant-numeric:tabular-nums;
  overscroll-behavior:none;-webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent;
}
button{font:inherit;letter-spacing:-.14px;color:inherit;background:none;border:0;
  cursor:pointer;touch-action:manipulation}
/* วงแหวนน้ำเงินจาง ๆ คือสัญญาณโฟกัสเดียวของทั้งระบบ */
button:focus-visible,input:focus-visible{outline:none;
  box-shadow:0 0 0 1px var(--accent),0 0 0 4px var(--accent-ring)}
a{color:var(--accent);text-underline-offset:2px}
::selection{background:var(--accent-ring);color:var(--ink)}

/* ================= ภาพสดเต็มจอ ================= */
.feed{position:fixed;inset:0;background:#000;z-index:0}
.feed img{width:100%;height:100%;object-fit:contain;display:block}
@media (orientation:landscape){.feed img{object-fit:cover}}
.feed.fill img{object-fit:cover}
.feed.fit img{object-fit:contain}
.feed.noimg img{opacity:.1}
.flash{position:fixed;inset:0;z-index:9;background:#FFFFFF;opacity:0;
  pointer-events:none;transition:opacity .28s ease-out}
.flash.on{opacity:.8;transition:none}
.feed.noimg::after{content:"ยังไม่มีภาพจากกล้อง";position:absolute;inset:0;
  display:grid;place-items:center;color:var(--ink-muted);
  font-size:15px;letter-spacing:-.15px;pointer-events:none}

/* ================= HUD: แนวกลางหุ่นบนภาพ =================
   กล้องอยู่บนแขนที่หันได้ แนวกลางหุ่นจึงไม่ได้อยู่กลางภาพเสมอ
   เส้นนี้บอกว่า "แปรงกำลังปัดขยะมารวมกันตรงไหน" เทียบกับสิ่งที่เห็น */
.hud{inset:0}
.hud.off{display:none}
.hud .axis{position:absolute;top:8%;bottom:8%;width:0;
  border-left:2px dashed rgba(255,255,255,.6);
  transition:left .1s linear,border-color .15s}
.hud .axis .tip{position:absolute;left:-8px;bottom:38%;width:16px;height:16px;
  border-radius:var(--r-full);border:2px solid var(--ink);
  background:rgba(0,0,0,.35);box-sizing:border-box}
.hud .axis .tip::after{content:"";position:absolute;inset:4px;
  border-radius:var(--r-full);background:var(--ink)}
.hud .axis .tag{position:absolute;left:12px;bottom:calc(38% - 8px);
  font-size:12px;font-weight:500;letter-spacing:-.12px;color:var(--ink);
  white-space:nowrap;text-shadow:0 1px 4px rgba(0,0,0,.95)}
/* เส้นอยู่ค่อนไปทางขวา -> พลิกป้ายไปฝั่งซ้าย ไม่งั้นข้อความล้นขอบจอ */
.hud.flip .axis .tag{left:auto;right:12px}
/* โหมดตั้งเส้นกลาง — คลุมทั้งจอเพื่อรับการแตะ
   ★ ต้องเขียน .layer.calib ให้จำเพาะกว่า .layer{pointer-events:none}
     และ z-index ต้องสูงกว่าแผงเลื่อน (20) ไม่งั้นแผงที่เปิดอยู่จะบังการแตะ */
.layer.calib{inset:0;pointer-events:auto;z-index:30;background:rgba(0,0,0,.35);
  display:grid;place-items:start center;
  padding-top:calc(72px + env(safe-area-inset-top,0px))}
.calib.off{display:none}
.calib .hint{padding:12px 18px;border-radius:var(--r-pill);
  background:var(--surface-2);color:var(--ink);
  font-size:15px;font-weight:500;letter-spacing:-.15px;text-align:center;
  box-shadow:inset 0 .5px 0 rgba(255,255,255,.1),0 10px 30px rgba(0,0,0,.4)}
/* แนวกลางหลุดออกนอกเฟรม — ย้อมเหลืองแล้วชี้ว่าอยู่ทางไหน */
.hud.out .axis{border-left-color:var(--caution)}
.hud.out .axis .tip{border-color:var(--caution)}
.hud.out .axis .tip::after{background:var(--caution)}
.hud.out .axis .tag{color:var(--caution)}

/* ================= หน้าปัดบอกทิศแขนเทียบตัวหุ่น ================= */
.nav{left:var(--pad);top:50%;transform:translateY(-50%);
  display:flex;flex-direction:column;align-items:center;gap:3px;
  padding:7px 8px 8px;border-radius:var(--r-xl);
  background:var(--glass);border:1px solid var(--hairline-soft);
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur)}
.nav svg{width:84px;height:84px;display:block}
.nav .rim{fill:none;stroke:var(--hairline);stroke-width:1}
.nav .cone{fill:rgba(0,153,255,.22);stroke:var(--accent);stroke-width:1}
.nav .body{fill:var(--surface-2);stroke:var(--hairline);stroke-width:1}
.nav .fwd{fill:var(--ink)}
.nav .mid{stroke:var(--ink);stroke-width:1.5;stroke-dasharray:3 3}
.nav .spin{fill:none;stroke:var(--caution);stroke-width:3;stroke-linecap:round}
.nav .hub{fill:var(--ink)}
.nav .lab{font-size:12px;font-weight:500;letter-spacing:-.12px;color:var(--ink);
  white-space:nowrap}
.nav .lab2{font-size:12px;letter-spacing:-.12px;color:var(--ink-muted);
  white-space:nowrap}
/* โหมด cleaner — ขอบฟ้าบอกว่าแกนหันถูกล็อกอยู่ */
.nav.clean{box-shadow:0 0 0 1px var(--accent),0 0 0 4px var(--accent-ring)}

/* ป้ายโหมดกดได้ */
.pill.btn{cursor:pointer}
.pill.mode{color:var(--ink)}
.pill.mode.on{background:var(--accent);border-color:transparent;color:#FFF}
.pill.mode.on i{background:#FFF}

/* สติ๊กแขนตอนถูกล็อกแกนหัน — บีบให้เห็นว่าเหลือแต่แกนตั้ง */
.stick.arm.lockx::before{inset:30% 44%;border-style:solid;
  border-color:var(--accent)}

/* ================= ชั้นลอย ================= */
.layer{position:fixed;z-index:5;pointer-events:none}
.layer > *{pointer-events:auto}
/* ปุ่มมุมขวาบนกว้างขึ้นหลังเปลี่ยนเป็นทรงยา ป้ายซ้ายจึงต้องหดตาม ไม่งั้นทับกัน */
.tl{top:calc(9px + env(safe-area-inset-top,0px));left:var(--pad);
    display:flex;flex-wrap:wrap;gap:var(--s-xxs);
    max-width:min(62vw,calc(100vw - 232px))}
.tr{top:calc(9px + env(safe-area-inset-top,0px));
    right:calc(10px + env(safe-area-inset-right,0px));display:flex;gap:var(--s-xxs)}

/* ---- ป้ายสถานะ: ทรงยา (pill) ตามคำศัพท์ของระบบ ---- */
.pill{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;
  border-radius:var(--r-pill);
  font-size:13px;font-weight:500;line-height:1.2;letter-spacing:-.13px;
  color:var(--ink-muted);background:var(--glass);
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur);
  border:1px solid var(--hairline-soft)}
.pill i{width:7px;height:7px;border-radius:var(--r-full);background:var(--danger);flex:none}
/* สถานะปกติ = ยกพื้นผิวขึ้นหนึ่งขั้นแล้วเปลี่ยนเป็น ink ไม่ใช่เปลี่ยนเป็นสีเขียว */
.pill.up{color:var(--ink);background:rgba(35,34,33,.8);border-color:var(--hairline)}
.pill.up i{background:var(--ok)}
.pill.warn{color:var(--ink);border-color:rgba(240,165,0,.45)}
.pill.warn i{background:var(--caution)}

/* ---- ปุ่มรองแบบโปร่ง (button-translucent) ---- */
.gb{padding:9px 13px;border:1px solid var(--hairline);border-radius:var(--r-pill);
  font-size:13px;font-weight:500;line-height:1;min-height:40px;
  background:var(--glass);backdrop-filter:var(--blur);
  -webkit-backdrop-filter:var(--blur);white-space:nowrap}
.gb:active{background:rgba(35,34,33,.9)}
/* เปิดอยู่ = ยกพื้นผิว + วงแหวนน้ำเงิน (สถานะที่ถูกเลือก) */
.gb.on{background:var(--surface-2);border-color:transparent;color:var(--ink);
  box-shadow:0 0 0 1px var(--accent-ring)}

/* ================= แถบกล้อง ขอบขวา ================= */
.cam{top:50%;right:calc(10px + env(safe-area-inset-right,0px));
  transform:translateY(-50%);display:flex;flex-direction:column;align-items:center;
  gap:var(--s-xs)}
/* ชัตเตอร์ = ปุ่มหลักของหน้านี้ จึงเป็นวงกลมขาวล้วนตามกติกาปุ่มหลัก */
.shutter{width:58px;height:58px;border-radius:var(--r-full);
  border:3px solid rgba(255,255,255,.28);
  background:var(--glass);padding:0;position:relative;
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur);
  transition:transform .12s}
.shutter::after{content:"";position:absolute;inset:6px;border-radius:var(--r-full);
  background:var(--ink);transition:background .12s}
/* กดแล้วย่อ ไม่ใช่เปลี่ยนสีให้เข้มลง — ตามสเปกปุ่มหลักตอนถูกกด */
.shutter:active{transform:scale(.93)}
.shutter.busy::after{background:var(--caution)}
.zoomer{display:flex;flex-direction:column;align-items:center;gap:2px;padding:5px 7px;
  border:1px solid var(--hairline);border-radius:var(--r-xl);background:var(--glass);
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur)}
.zoomer button{width:34px;height:32px;font-size:17px;line-height:1;
  border-radius:var(--r-full);color:var(--ink)}
.zoomer button:active{background:var(--surface-2)}
.zoomer span{font-size:12px;color:var(--ink-muted);letter-spacing:-.12px}

/* ================= สติ๊ก ================= */
/* จอแคบ/เตี้ย — หน้าปัดย่อลงไม่ให้ชนสติ๊ก */
@media (max-height:520px){.nav svg{width:64px;height:64px}}
@media (max-width:380px){.nav svg{width:70px;height:70px}}

.sticks{left:0;right:0;bottom:calc(10px + env(safe-area-inset-bottom,0px));
  display:flex;align-items:flex-end;gap:var(--s-xs);padding:0 var(--pad)}
.sticks .mid{flex:1 1 auto;min-width:0;display:flex;flex-direction:column;
  align-items:center;gap:var(--s-xs);pointer-events:none}
.sticks .mid > *{pointer-events:auto}

.stick{position:relative;flex:0 0 auto;
  width:clamp(104px,27vw,152px);height:clamp(104px,27vw,152px);
  border-radius:var(--r-full);
  background:radial-gradient(circle at 50% 42%,rgba(35,34,33,.85) 0%,rgba(10,9,8,.72) 74%);
  border:1px solid var(--hairline);touch-action:none;
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur)}
.stick::before{content:"";position:absolute;inset:30%;border-radius:var(--r-full);
  border:1px dashed var(--hairline-soft)}
.knob{position:absolute;left:50%;top:50%;width:38%;height:38%;
  transform:translate(0,0);margin:-19% 0 0 -19%;
  border-radius:var(--r-full);background:var(--surface-2);
  border:1px solid var(--hairline);
  /* ชั้นความลึกที่ 2: ขอบสว่างบาง ๆ ด้านบน + เงาทิ้งตัว */
  box-shadow:inset 0 .5px 0 rgba(255,255,255,.10),0 10px 30px rgba(0,0,0,.25);
  transition:background .12s,border-color .12s,box-shadow .12s}
/* กำลังสั่งอยู่ = วงแหวนน้ำเงิน (สถานะถูกเลือก) ไม่ใช่เปลี่ยนสีลูกบิด */
.stick.live .knob{background:#2E2C2A;
  box-shadow:0 0 0 1px var(--accent),0 0 0 5px var(--accent-ring),0 10px 30px rgba(0,0,0,.3)}
.stick.pad .knob{background:var(--ink)}
.stick.arm.live .knob{background:#2E2C2A}
.cap{position:absolute;left:0;right:0;top:-16px;text-align:center;
  font-size:12px;letter-spacing:-.12px;color:var(--ink-muted);
  pointer-events:none;white-space:nowrap}
.stick.pad .cap{color:var(--ink)}
.stick.arm .cap{color:var(--ink-muted)}

/* มาตรวัดล้อ */
.meters{width:100%;max-width:280px;display:flex;flex-direction:column;gap:5px;
  padding:var(--s-sm);border:1px solid var(--hairline-soft);border-radius:var(--r-lg);
  background:var(--glass);
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur)}
.mrow{display:grid;grid-template-columns:28px 1fr 38px;gap:var(--s-xs);align-items:center;
  font-size:12px;letter-spacing:-.12px;color:var(--ink-muted)}
.track{position:relative;height:12px;border-radius:var(--r-xs);background:rgba(0,0,0,.5);
  border:1px solid var(--hairline-soft)}
.track::before{content:"";position:absolute;left:50%;top:0;bottom:0;width:1px;
  background:var(--hairline)}
.fill{position:absolute;top:2px;bottom:2px;left:50%;width:0;background:var(--ink);
  border-radius:var(--r-xs)}
.ghost{position:absolute;top:0;bottom:0;left:50%;width:0;
  border:1px solid rgba(255,255,255,.35);border-radius:var(--r-xs)}
.mrow b{font-weight:500;color:var(--ink);text-align:right}
/* หยุดฉุกเฉิน — สีสัญญาณ ไม่ใช่สีแบรนด์ แต่ยังเป็นทรงยาตามคำศัพท์ของระบบ */
.halt{padding:11px 20px;border:1px solid var(--danger);border-radius:var(--r-pill);
  color:var(--danger);font-size:14px;font-weight:500;line-height:1;min-height:44px;
  background:rgba(255,74,61,.1);
  backdrop-filter:var(--blur);-webkit-backdrop-filter:var(--blur)}
.halt:active{background:var(--danger);color:var(--ink)}

/* ================= ข้อความชั่วคราว ================= */
.toast{left:50%;top:calc(54px + env(safe-area-inset-top,0px));transform:translateX(-50%);
  max-width:80vw}
.toast span{display:block;padding:8px 15px;border-radius:var(--r-pill);
  font-size:13px;font-weight:500;letter-spacing:-.13px;
  color:var(--ink);background:var(--surface-2);border:1px solid var(--hairline-soft);
  box-shadow:inset 0 .5px 0 rgba(255,255,255,.10),0 10px 30px rgba(0,0,0,.25);
  text-align:center}
.toast span:empty{display:none}
.alert{left:50%;bottom:calc(178px + env(safe-area-inset-bottom,0px));
  transform:translateX(-50%);max-width:min(560px,90vw)}
.alert span{display:none;padding:11px 15px;border-radius:var(--r-md);
  font-size:14px;letter-spacing:-.14px;
  color:var(--ink);background:var(--surface-1);border:1px solid var(--caution)}
.alert span.show{display:block}

/* ================= แผงเลื่อนจากขวา ================= */
.sheet{position:fixed;top:0;bottom:0;right:0;width:min(430px,94vw);z-index:20;
  background:rgba(10,9,8,.95);backdrop-filter:blur(20px);
  -webkit-backdrop-filter:blur(20px);border-left:1px solid var(--hairline-soft);
  transform:translateX(101%);transition:transform .22s ease;
  display:flex;flex-direction:column}
.sheet.on{transform:none}
.shead{display:flex;align-items:center;gap:var(--s-xs);padding:var(--s-sm) 14px;
  padding-top:calc(var(--s-sm) + env(safe-area-inset-top,0px));
  border-bottom:1px solid var(--hairline-soft)}
.shead .stabs{display:flex;gap:var(--s-xxs)}
/* แท็บ: ที่ไม่ได้เลือก = พื้น canvas + ink-muted / ที่เลือก = ยกเป็น surface-2 + ink
   "ถูกเลือก" สื่อด้วยการยกพื้นผิว ไม่ใช่การถมสี */
.shead .stabs button{padding:9px 15px;border-radius:var(--r-pill);
  font-size:14px;font-weight:500;line-height:1;min-height:40px;
  color:var(--ink-muted);background:var(--canvas);border:1px solid transparent}
.shead .stabs button.on{color:var(--ink);background:var(--surface-2);border-color:transparent}
.shead .sp{margin-left:auto}
.sbody{flex:1 1 auto;overflow-y:auto;padding:var(--s-sm);display:flex;
  flex-direction:column;gap:var(--s-sm);
  padding-bottom:calc(var(--s-sm) + env(safe-area-inset-bottom,0px))}
.sbody > *{flex:none}
/* ปิดแผง = ปุ่มรองสีถ่าน */
.sheet .x{padding:9px 15px;border:0;border-radius:var(--r-pill);
  font-size:14px;font-weight:500;line-height:1;min-height:40px;
  background:var(--surface-1);color:var(--ink)}
.sheet .x:active{background:var(--surface-2)}

/* ================= ชิ้นส่วนในแผง ================= */
.panel{border:0;border-radius:var(--r-xl);background:var(--surface-1);overflow:hidden}
.head{display:flex;align-items:center;gap:var(--s-xs);padding:16px;width:100%;
  text-align:left;min-height:44px}
.head h3{margin:0;font-size:15px;font-weight:500;letter-spacing:-.15px}
.head .val{margin-left:auto;font-size:14px;font-weight:500;letter-spacing:-.14px;
  color:var(--ink-muted)}
.head .caret{color:var(--ink-muted);transition:transform .18s}
.panel.open .caret{transform:rotate(90deg)}
.body{display:none;padding:0 16px 16px;flex-direction:column;gap:var(--s-sm)}
.panel.open .body{display:flex}
/* ปุ่มในแผง = ปุ่มรองทรงยา ไม่มีเส้นขอบ ใช้พื้นผิวบอกว่าเป็นปุ่ม */
.seg{display:flex;gap:var(--s-xs);flex-wrap:wrap}
.seg button{flex:1 1 auto;min-width:62px;padding:11px 15px;border:0;
  border-radius:var(--r-pill);font-size:14px;font-weight:500;line-height:1;
  min-height:44px;background:var(--surface-2);color:var(--ink)}
.seg button:active{background:#2E2C2A}
/* ปุ่มสั่งงานหลักของแผง = ปุ่มขาว ตามกติกา primary CTA */
.seg button.go{background:var(--ink);color:var(--on-primary)}
.seg button.go:active{background:#E6E6E6}
/* กำลังทำงาน/เปิดอยู่ = ยกพื้นผิวขึ้นหนึ่งขั้น + วงแหวนน้ำเงิน
   "ถูกเลือก" สื่อด้วยความลึก ไม่ใช่การถมสี — และวางไว้หลัง .go เพื่อให้ทับกันได้
   ตอนปุ่มเดียวเป็นทั้งปุ่มสั่งงานและกำลังทำงานอยู่ (เช่น ปุ่มสแกนโฟกัส) */
.seg button.on{background:var(--surface-2);color:var(--ink);
  box-shadow:0 0 0 1px var(--accent),0 0 0 4px var(--accent-ring)}
input[type=range]{width:100%;accent-color:var(--accent);background:transparent;margin:0}
.slab{display:flex;justify-content:space-between;gap:var(--s-xs);
  font-size:14px;font-weight:500;letter-spacing:-.14px;color:var(--ink-muted)}
.slab span:last-child{color:var(--ink);text-align:right}

.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(104px,1fr));gap:var(--s-xs)}
.tile{border:0;border-radius:var(--r-lg);background:var(--surface-1);padding:14px 15px}
.tile span{display:block;font-size:13px;font-weight:500;letter-spacing:-.13px;
  color:var(--ink-muted);margin-bottom:4px}
/* ตัวเลขหลักใช้ display-md: 32px tracking -1px ตามสเปก */
.tile strong{display:block;font-weight:500;font-size:32px;line-height:1.13;
  letter-spacing:-1px;color:var(--ink)}
.tile small{font-size:14px;font-weight:500;letter-spacing:-.14px;
  color:var(--ink-muted);margin-left:4px}
/* ค่าผิดปกติ: ยกเป็น surface-2 แล้วย้อมเฉพาะตัวเลข ไม่ตีกรอบสี */
.tile.warn{background:var(--surface-2)} .tile.warn strong{color:var(--caution)}
.tile.bad{background:var(--surface-2)}  .tile.bad strong{color:var(--danger)}
.tile.good strong{color:var(--ok)}
.tele{margin:0;padding:15px;border:0;border-radius:var(--r-lg);
  background:var(--surface-1);color:var(--ink-muted);
  white-space:pre-wrap;word-break:break-word;
  font:400 13px/1.6 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  letter-spacing:0;font-feature-settings:"tnum"}
.fbar{position:relative;height:16px;border-radius:var(--r-sm);background:var(--surface-2);
  overflow:hidden}
.fbar i{position:absolute;left:0;top:0;bottom:0;width:0;background:var(--ink);
  transition:width .12s linear}
.fbar b{position:absolute;top:0;bottom:0;width:2px;background:var(--accent);
  left:0;transition:left .3s}
.keymap{display:flex;flex-direction:column;gap:var(--s-xs)}
.keymap div{display:flex;align-items:baseline;gap:var(--s-sm);
  font-size:14px;letter-spacing:-.14px;color:var(--ink-muted)}
.keymap kbd{flex:none;min-width:68px;padding:5px 9px;border:0;border-radius:var(--r-sm);
  background:var(--surface-2);color:var(--ink);font:inherit;font-size:13px;font-weight:500;
  text-align:center}

/* ================= อัลบั้ม ================= */
figure{margin:0}
.albumbar{display:flex;align-items:center;gap:var(--s-xs);flex-wrap:wrap}
.albumbar .n{font-size:14px;font-weight:500;letter-spacing:-.14px;color:var(--ink-muted)}
.albumbar .sp{margin-left:auto}
.albumbar button{padding:9px 15px;border:0;border-radius:var(--r-pill);
  font-size:14px;font-weight:500;line-height:1;min-height:40px;
  background:var(--surface-1);color:var(--ink)}
.albumbar button:active{background:var(--surface-2)}
.albumbar button.on{background:var(--ink);color:var(--on-primary)}
.albumbar button.danger{background:var(--surface-1);color:var(--danger)}
.albumbar button.danger:active{background:var(--danger);color:var(--ink)}
.day h4{margin:2px 0 var(--s-xs);font-size:13px;font-weight:500;letter-spacing:-.13px;
  color:var(--ink-muted);display:flex;align-items:baseline;gap:var(--s-xs)}
.day h4 em{font-style:normal;font-size:13px;color:var(--ink-muted);opacity:.7}
.shelf{display:grid;grid-template-columns:repeat(auto-fill,minmax(96px,1fr));gap:6px}
/* การ์ดรูปใช้ rounded.lg 15px ตามสเปกการ์ดธัมบ์เนล */
.card{position:relative;aspect-ratio:1;border-radius:var(--r-lg);overflow:hidden;
  background:var(--surface-1);border:0;cursor:pointer}
.card img{width:100%;height:100%;object-fit:cover;display:block;background:var(--surface-1)}
.card.miss::after{content:"ภาพเสีย";position:absolute;inset:0;display:grid;
  place-items:center;color:var(--ink-muted);font-size:13px}
.card time{position:absolute;left:0;right:0;bottom:0;padding:16px 8px 6px;
  font-size:12px;font-weight:500;letter-spacing:-.12px;
  color:var(--ink);background:linear-gradient(transparent,rgba(0,0,0,.8))}
.card .mark{position:absolute;top:7px;right:7px;width:22px;height:22px;
  border-radius:var(--r-full);border:1.5px solid rgba(255,255,255,.75);
  background:rgba(0,0,0,.35);display:none}
.picking .card .mark{display:block}
.card.sel{box-shadow:inset 0 0 0 2px var(--accent)}
.card.sel img{opacity:.5}
.card.sel .mark{background:var(--accent);border-color:var(--accent)}
.card.sel .mark::after{content:"";position:absolute;left:7px;top:3px;width:5px;height:10px;
  border:solid var(--ink);border-width:0 2px 2px 0;transform:rotate(45deg)}

/* ---- การ์ดเรืองแสง: ของตกแต่งชิ้นเดียวของทั้งหน้า ----
   สเปกบอกว่าหนึ่งถึงสองใบต่อหน้ายาว ๆ สามใบขึ้นไปกลายเป็นมู้ดบอร์ด
   หน้านี้ใช้ใบเดียว วางไว้ที่อัลบั้มตอนยังไม่มีรูป */
.empty{padding:var(--s-xl);text-align:center;border:0;border-radius:var(--r-xxl);
  background:var(--grad-violet);color:var(--ink)}
.empty b{display:block;font-weight:500;font-size:24px;line-height:1.3;
  letter-spacing:-.01px;margin-bottom:var(--s-xs)}
.empty span{font-size:15px;letter-spacing:-.15px;opacity:.82}

/* ================= ดูภาพเต็ม ================= */
.viewer{position:fixed;inset:0;background:var(--canvas);display:none;
  flex-direction:column;z-index:40;padding-top:env(safe-area-inset-top,0)}
.viewer.on{display:flex}
.vtop{display:flex;align-items:center;gap:var(--s-xs);padding:var(--s-sm) 14px;
  font-size:14px;font-weight:500;letter-spacing:-.14px;color:var(--ink-muted)}
.vtop .sp{margin-left:auto}
.vtop button{padding:9px 15px;border:0;border-radius:var(--r-pill);
  font-size:14px;font-weight:500;line-height:1;min-height:40px;
  background:var(--surface-1);color:var(--ink)}
.vstage{flex:1 1 auto;min-height:0;position:relative;display:flex;align-items:center;
  justify-content:center;touch-action:pan-y}
.vstage img{max-width:100%;max-height:100%;object-fit:contain}
/* ปุ่มลูกศร = ปุ่มไอคอนทรงกลม 44px บนจอสัมผัส */
.arrow{position:absolute;top:50%;transform:translateY(-50%);width:44px;height:44px;
  border-radius:var(--r-full);background:var(--surface-1);border:0;
  font-size:19px;color:var(--ink);
  box-shadow:inset 0 .5px 0 rgba(255,255,255,.10),0 10px 30px rgba(0,0,0,.25)}
.arrow.l{left:12px} .arrow.r{right:12px}
.arrow[disabled]{opacity:.25}
.vbar{display:flex;gap:var(--s-xs);padding:var(--s-sm);
  padding-bottom:calc(var(--s-sm) + env(safe-area-inset-bottom,0))}
.vbar button{flex:1;padding:13px 15px;border:0;border-radius:var(--r-pill);
  font-size:14px;font-weight:500;line-height:1;min-height:44px;
  background:var(--surface-1);color:var(--ink)}
.vbar button:active{background:var(--surface-2)}
.vbar .del{background:var(--surface-1);color:var(--danger)}
.vbar .del:active{background:var(--danger);color:var(--ink)}

/* แนวนอน — ย้ายแถบกล้องไปขอบบนกลาง ไม่งั้นชนสติ๊กแขนที่มุมขวาล่าง */
@media (orientation:landscape){
  /* แนวนอนจอเตี้ย — สติ๊กขับกินพื้นที่มุมซ้ายล่างขึ้นมาเกือบกลางจอ
     ย้ายหน้าปัดไปเกาะขอบล่าง เหนือสติ๊กพอดี แทนที่จะลอยกลางแล้วทับกัน */
  .nav{top:auto;transform:none;
    bottom:calc(20px + clamp(104px,27vw,152px) + env(safe-area-inset-bottom,0px))}
  .cam{top:calc(8px + env(safe-area-inset-top,0px));left:50%;right:auto;
    transform:translateX(-50%);flex-direction:row;align-items:center;gap:var(--s-xxs)}
  .zoomer{flex-direction:row;align-items:center;padding:3px 6px;gap:var(--s-xxs)}
  .zoomer button{width:30px;height:28px}
  .shutter{width:44px;height:44px;border-width:2px}
  .shutter::after{inset:5px}
  .tl{max-width:34vw}
}
@media (max-height:430px){
  .meters{display:none}
}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>

<!-- ไม่ใส่ src ตรงนี้ — openStream() ใน JS เปิดให้ พร้อมรหัสประจำสาย
     ถ้าใส่ไว้ด้วยจะเปิดสองสายซ้อน สายแรกไม่มี id เลยนับไม่ได้ -->
<div class="feed" id="feed"><img id="v" alt="ภาพสดจากกล้องหุ่น"></div>
<div class="flash" id="flash"></div>

<!-- ---------- HUD: แนวกลางหุ่น = จุดที่แปรงปัดขยะเข้า ---------- -->
<div class="layer hud off" id="hud">
  <div class="axis" id="hudAxis">
    <span class="tip"></span>
    <span class="tag" id="hudTag">กลางหุ่น</span>
  </div>
</div>

<!-- ---------- โหมดตั้งเส้นกลาง: แตะบนภาพตรงจุดที่ถูกต้อง ---------- -->
<div class="layer calib off" id="calib">
  <div class="hint">แตะบนภาพ ตรงกลางระหว่างแปรงสองตัว<br>(แตะนอกภาพ = ยกเลิก)</div>
</div>

<!-- ---------- หน้าปัด: แขนหันไปทางไหนเทียบตัวหุ่น + หุ่นกำลังหมุนแค่ไหน ---------- -->
<div class="layer nav" id="nav">
  <svg viewBox="0 0 100 100" aria-label="ทิศแขนเทียบตัวหุ่น">
    <circle class="rim" cx="50" cy="50" r="46"/>
    <path class="cone" id="navCone" d=""/>
    <rect class="body" x="36" y="33" width="28" height="37" rx="7"/>
    <line class="mid" x1="50" y1="27" x2="50" y2="74"/>
    <path class="fwd" d="M50 18 L56 30 L44 30 Z"/>
    <path class="spin" id="navSpin" d=""/>
    <circle class="hub" cx="50" cy="50" r="3"/>
  </svg>
  <span class="lab" id="navArm">แขนตรง</span>
  <span class="lab2" id="navTurn">จอดอยู่</span>
</div>

<!-- ---------- ขอบบนซ้าย: สถานะ ---------- -->
<div class="layer tl">
  <button class="pill btn mode" id="pMode"><i></i><span>โหมดขับ</span></button>
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
        <div class="seg">
          <button id="armHome">กลับตำแหน่งกลาง</button>
          <button id="bClean">โหมด cleaner</button>
        </div>
        <div class="slab"><span>โหมด cleaner (ปุ่ม B บนจอย)</span>
          <span id="cleanVal">ปิด</span></div>
        <div class="seg">
          <button id="bCalib">ตั้งเส้นกลาง</button>
          <button id="calibL">◀ ซ้าย</button>
          <button id="calibR">ขวา ▶</button>
        </div>
        <div class="slab"><span>มุมที่กล้องมองตรงแนวกลางหุ่น</span>
          <span id="calibVal">90.0°</span></div>
      </div>
    </div>

    <div class="panel">
      <button class="head" data-toggle><span class="caret">›</span>
        <h3>คุณภาพสตรีม</h3><span class="val" id="bwVal">–</span></button>
      <div class="body">
        <div class="slab"><span>ภาพค้างบ่อย = ท่อเน็ตแคบ ลด fps ช่วยได้ทันที</span>
          <span id="sfVal">12 fps</span></div>
        <div class="seg">
          <button data-sf="4">4</button>
          <button data-sf="6">6</button>
          <button data-sf="8">8</button>
          <button data-sf="12">12</button>
          <button data-sf="20">20</button>
        </div>
        <div class="slab"><span>ขนาดภาพสตรีม (แก้ใน server.py)</span>
          <span id="ssVal">–</span></div>
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
          <button onclick="cmd('r')">สลับทิศแปรง</button>
          <button onclick="cmd(',')">หรี่แปรง A</button>
          <button onclick="cmd('.')">หรี่แปรง B</button>
          <button onclick="cmd('p')">ทดสอบขาล้อ</button>
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
      <div class="tile" id="tBw"><span>แบนด์วิดท์ภาพ</span>
        <strong id="dBw">–</strong><small>Mbit/s</small></div>
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
        <div><kbd>B</kbd><span>เข้า / ออกโหมด cleaner (ล็อกแกนหันของแขน)</span></div>
        <div><kbd>L</kbd><span>แตะ = โฟกัสอัตโนมัติ · กดค้าง = สแกนหาจุดคมสุด</span></div>
        <div><kbd>R</kbd><span>ถ่ายรูป</span></div>
        <div><kbd>X / Y</kbd><span>โฟกัสด้วยมือ — เข้าใกล้ / ออกไกล (กดค้างได้)</span></div>
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
  const dp = cleanerOn ? 0 : armVec.x*k;   // โหมด cleaner ล็อกแกนหัน
  fetch(`/arm?dpan=${dp.toFixed(2)}&dtilt=${(-armVec.y*k).toFixed(2)}`)
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
/* ================= โหมด cleaner ================= */
let cleanerOn=false;
function showCleaner(on){
  on=!!on;
  cleanerOn=on;
  $('#pMode').classList.toggle('on',on);
  $('#pMode').querySelector('span').textContent = on?'CLEANER':'โหมดขับ';
  $('#armStick').classList.toggle('lockx',on);
  $('#nav').classList.toggle('clean',on);
  $('#panSl').disabled=on;
  $('#cleanVal').textContent=on?'เปิด — แกนหันถูกล็อก':'ปิด';
  $('#bClean').classList.toggle('on',on);
}
function toggleCleaner(){
  fetch('/cleaner').then(r=>r.json()).then(j=>{
    showCleaner(j.cleaner);
    showArm(j);
    say(j.cleaner?`โหมด cleaner — แขนไป ${j.pan}°/${j.tilt}° ล็อกแกนหัน`
                 :'ออกจากโหมด cleaner');
  }).catch(()=>{});
}
$('#pMode').onclick=toggleCleaner;
$('#bClean').onclick=toggleCleaner;

/* ================= HUD + หน้าปัด =================
   ไม่มี IMU บนหุ่น จึงไม่รู้ทิศสัมบูรณ์ว่าหันไปทางทิศเหนือใต้
   สิ่งที่รู้จริงมีสองอย่าง แล้วแสดงแค่สองอย่างนี้:
     1. แขนเบนจากแนวกลางหุ่นกี่องศา  (จากมุม pan — แม่นยำ)
     2. ตอนนี้สั่งให้หมุนแรงแค่ไหน    (จากผลต่างความเร็วล้อซ้าย-ขวา)
   ไม่คำนวณ "หันไปแล้วกี่องศา" เพราะต้องสะสมค่าแล้วมันเพี้ยนสะสมไปเรื่อย ๆ */
const polar=(cx,cy,r,deg)=>{const t=(deg-90)*Math.PI/180;
  return [cx+r*Math.cos(t), cy+r*Math.sin(t)];};
function wedgePath(cx,cy,r,a0,a1){
  const [x0,y0]=polar(cx,cy,r,a0), [x1,y1]=polar(cx,cy,r,a1);
  const big=(a1-a0)>180?1:0;
  return `M${cx} ${cy} L${x0.toFixed(1)} ${y0.toFixed(1)} `+
         `A${r} ${r} 0 ${big} 1 ${x1.toFixed(1)} ${y1.toFixed(1)} Z`;
}
function arcPath(cx,cy,r,a0,a1){
  const [x0,y0]=polar(cx,cy,r,a0), [x1,y1]=polar(cx,cy,r,a1);
  const big=Math.abs(a1-a0)>180?1:0, sweep=a1>a0?1:0;
  return `M${x0.toFixed(1)} ${y0.toFixed(1)} `+
         `A${r} ${r} 0 ${big} ${sweep} ${x1.toFixed(1)} ${y1.toFixed(1)}`;
}
/* pan น้อยลง = กล้องหันขวา  ->  แขนเบนขวาเป็นบวก */
const armBearing=s=>(s.panFwd||90)-(s.pan||90);
const hfovNow=s=>(s.hfov||66)/Math.max(1,s.zoom||1);

function updateNav(s){
  const bear=armBearing(s), half=hfovNow(s)/2;
  $('#navCone').setAttribute('d', wedgePath(50,50,44,bear-half,bear+half));
  $('#navArm').textContent = Math.abs(bear)<1 ? 'แขนตรงหน้า'
    : `แขนเบน${bear>0?'ขวา':'ซ้าย'} ${Math.abs(bear)}°`;

  /* หมุนขวา = ล้อซ้ายเดินหน้า ล้อขวาถอย -> curL-curR เป็นบวก */
  const spin=((s.curL||0)-(s.curR||0))/2;
  const pct=Math.round(Math.min(100,Math.abs(spin)/255*100));
  const deg=Math.min(120,Math.abs(spin)/255*240)*(spin<0?-1:1);
  $('#navSpin').setAttribute('d', pct<2?'':arcPath(50,50,40,0,deg));
  const fwd=((s.curL||0)+(s.curR||0))/2;
  $('#navTurn').textContent =
    pct>=2 ? `หมุน${spin>0?'ขวา':'ซ้าย'} ${pct}%`
    : Math.abs(fwd)>=3 ? (fwd>0?'เดินหน้าตรง':'ถอยหลังตรง')
    : 'จอดอยู่';
}

/* ================= ยามเฝ้าสตรีม =================
   MJPEG ผ่าน <img> มีจุดอ่อนที่เจ็บมาก: ถ้าการเชื่อมต่อตายกลางคัน
   เบราว์เซอร์จะ "ค้างเฟรมสุดท้ายไว้เฉย ๆ" ไม่ยิง error ไม่ยิง load
   หน้าเว็บเลยไม่มีทางรู้ด้วยตัวเองว่าภาพหยุดไปแล้ว

   ★ วิธีเดิมที่ใช้ไม่ได้: ย่อภาพเป็น 32x18 แล้วดูว่าพิกเซลเปลี่ยนไหม
     ย่อขนาดนั้นมันเฉลี่ยพิกเซลทีละบล็อก 40x40 -> noise เซนเซอร์หายเกลี้ยง
     หุ่นจอดนิ่ง กล้องส่องฉากนิ่ง = ลายนิ้วมือเดิมเป๊ะ = เตือนผิดทั้งที่ภาพไหลปกติ

   วิธีที่ใช้จริง: ถามฝั่งหุ่นตรง ๆ ว่า "ส่งเฟรมให้การเชื่อมต่อของฉันไปกี่เฟรมแล้ว"
     แต่ละครั้งที่ต่อ เว็บสร้างรหัส id ใหม่ แปะไปกับ /stream?id=xxx
     ฝั่งหุ่นนับทุกเฟรมที่ "เขียนออกสำเร็จ" แล้วตอบกลับมาเป็น txN ใน /status
       txN เดิน            -> ไบต์ออกไปถึงจริง สายดี  (ภาพนิ่งก็ไม่เตือน)
       txN หยุด + frameN เดิน -> กล้องถ่ายอยู่แต่ส่งไม่ออก = สายตาย -> ต่อใหม่
       txN หยุด + frameN หยุด -> กล้องเองมีปัญหา ต่อใหม่ไม่ช่วย -> แค่บอก */
const STREAM_STALL_MS=5000;
let wId='', wTx=-1, wTxAt=Date.now(), wFrameN=-1, wFrameNAt=Date.now(),
    wReconnectAt=0, wCamWarn=0;
const newStreamId=()=>'c'+Date.now().toString(36)+Math.random().toString(36).slice(2,7);
function openStream(){
  wId=newStreamId();
  wTx=-1; wTxAt=Date.now();            /* ให้เวลาตั้งตัวก่อนเริ่มนับถอยหลัง */
  $('#v').src='/stream?id='+wId;
}
function restartStream(why){
  const now=Date.now();
  if(now-wReconnectAt<4000) return;      /* กันต่อใหม่รัว ๆ */
  wReconnectAt=now;
  openStream();                          /* id ใหม่ = สายใหม่แน่นอน ไม่ติด cache */
  say('ภาพค้าง — กำลังต่อใหม่ ('+why+')');
}
function streamWatch(s){
  const now=Date.now();
  /* กล้องยังถ่ายอยู่ไหม — ตัวนับของกล้องเอง ไม่ขึ้นกับว่ามีใครดูสตรีมอยู่ */
  if(s.frameN!=null && s.frameN!==wFrameN){ wFrameN=s.frameN; wFrameNAt=now; }
  if(!s.cam) return;
  /* กล้องกำลังถ่ายภาพนิ่ง/สแกนโฟกัส -> หยุดส่งเฟรมชั่วคราวเป็นเรื่องปกติ */
  if(s.afBusy){ wTxAt=now; wFrameNAt=now; return; }
  /* txN เดินไปแม้แต่เฟรมเดียว = การเชื่อมต่อของเรายังมีชีวิต */
  if(s.txN!=null && s.txN!==wTx){ wTx=s.txN; wTxAt=now; return; }
  if(now-wTxAt<=STREAM_STALL_MS) return;
  if(now-wFrameNAt<3000) restartStream('สายหลุด');
  else if(now-wCamWarn>15000){
    wCamWarn=now; say('กล้องหยุดส่งภาพ — ปัญหาอยู่ที่ฝั่งหุ่น ไม่ใช่เน็ต');
  }
}
/* สลับแอปแล้วกลับมา — มือถือมักพักการเชื่อมต่อทิ้งไว้ ต่อใหม่เลยดีกว่ารอ */
document.addEventListener('visibilitychange',()=>{
  if(!document.hidden) restartStream('กลับมาที่หน้านี้');
});
$('#v').addEventListener('error',()=>restartStream('โหลดภาพไม่ได้'));
openStream();          /* เปิดสายแรกด้วย id ของเราเอง ไม่ใช่ src ใน HTML */

/* ---- ภาพกับหน้าจอไม่ใช่ระบบพิกัดเดียวกัน ----
   object-fit:contain ทำให้มีขอบดำ / cover ทำให้ภาพถูกครอปออกนอกจอ
   ถ้าวางเส้นเป็น % ของหน้าจอตรง ๆ มันจะเพี้ยนทั้งสองกรณี
   สองฟังก์ชันนี้แปลงไปกลับระหว่าง "สัดส่วนในเฟรม" กับ "พิกเซลบนจอ" */
function frameBox(){
  const img=$('#v'), r=img.getBoundingClientRect();
  const nw=img.naturalWidth||1280, nh=img.naturalHeight||720;
  const fit=getComputedStyle(img).objectFit;
  const sc = fit==='cover' ? Math.max(r.width/nw, r.height/nh)
                           : Math.min(r.width/nw, r.height/nh);
  const dw=nw*sc;
  return {x0:r.left+(r.width-dw)/2, w:dw};
}
const fracToPx=f=>{const b=frameBox(); return b.x0+f*b.w;};
const pxToFrac=x=>{const b=frameBox(); return b.w?(x-b.x0)/b.w:0.5;};

function updateHud(s){
  const hud=$('#hud');
  if(!s.cam){ hud.classList.add('off'); return; }
  hud.classList.remove('off');
  /* กล้องหันขวา b องศา -> ของที่อยู่ตรงหน้าหุ่นจะไปอยู่ทางซ้ายของภาพ b องศา */
  const frac=0.5 - armBearing(s)/hfovNow(s);
  const out=frac<0.01||frac>0.99;
  hud.classList.toggle('out',out);
  const clamped=Math.min(0.99,Math.max(0.01,frac));
  /* วางเป็นพิกเซล เทียบกับ "ภาพ" ไม่ใช่ขอบจอ */
  const px=Math.min(window.innerWidth-2,Math.max(2,fracToPx(clamped)));
  $('#hudAxis').style.left=px.toFixed(1)+'px';
  hud.classList.toggle('flip',px>window.innerWidth*0.58);
  $('#hudTag').textContent = out
    ? (frac<0.5?'กลางหุ่น ← นอกจอ':'กลางหุ่น → นอกจอ')
    : 'กลางหุ่น · แปรงปัดเข้าตรงนี้';
}

/* ================= ตั้งเส้นกลาง =================
   แตะบนภาพตรงจุดที่แปรงปัดขยะมารวมกันจริง ๆ แล้วถอดกลับเป็นมุม:
     สัดส่วนในเฟรม -> กล้องเบนจากจุดนั้นกี่องศา -> มุม pan ที่มองตรงแนวกลาง
   ค่าที่ได้ถูกเซฟลงไฟล์บน Pi เปิดเครื่องใหม่ก็ยังอยู่ */
let lastStatus={};
function showCalib(v){ $('#calibVal').textContent=Number(v).toFixed(1)+'°'; }
function calibFrom(clientX){
  const s=lastStatus, frac=pxToFrac(clientX);
  if(frac<-0.05||frac>1.05){ say('แตะนอกภาพ — ยกเลิก'); return false; }
  const bearing=(0.5-frac)*hfovNow(s);          // กล้องต้องเบนขวาเท่านี้
  const fwd=(s.pan||90)+bearing;
  fetch('/calib?fwd='+fwd.toFixed(2)).then(r=>r.json()).then(j=>{
    showCalib(j.panFwd); say(`ตั้งเส้นกลางแล้ว (${j.panFwd.toFixed(1)}°)`);
  }).catch(()=>{});
  return true;
}
$('#calib').addEventListener('pointerdown',e=>{
  e.preventDefault(); calibFrom(e.clientX);
  $('#calib').classList.add('off');
});
$('#bCalib').onclick=()=>{
  sheet(null,false);                 // ปิดแผงก่อน จะได้เห็นภาพเต็มจอ
  $('#calib').classList.remove('off');
  say('แตะบนภาพ ตรงกลางระหว่างแปรงสองตัว',true);
};
const calibNudge=d=>fetch('/calib?d='+d).then(r=>r.json())
  .then(j=>{showCalib(j.panFwd);say(`เส้นกลาง ${j.panFwd.toFixed(1)}°`);}).catch(()=>{});
$('#calibL').onclick=()=>calibNudge(0.5);    // เส้นขยับไปทางซ้าย
$('#calibR').onclick=()=>calibNudge(-0.5);   // เส้นขยับไปทางขวา

/* ================= คุณภาพสตรีม =================
   แบนด์วิดท์ = fps x ขนาดเฟรม  ลด fps ครึ่งหนึ่ง = แบนด์วิดท์ครึ่งหนึ่ง
   และยังได้รายละเอียดเต็มในทุกเฟรมที่ส่ง ต่างจากการลดความละเอียด */
function showStreamFps(v){
  $('#sfVal').textContent=Math.round(v)+' fps';
  for(const b of document.querySelectorAll('[data-sf]'))
    b.classList.toggle('on', Math.round(+b.dataset.sf)===Math.round(v));
}
for(const b of document.querySelectorAll('[data-sf]'))
  b.onclick=()=>fetch('/streamfps?v='+b.dataset.sf).then(r=>r.json())
    .then(j=>{showStreamFps(j.fps);say('สตรีม '+Math.round(j.fps)+' fps');})
    .catch(()=>{});

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
  if(k==='b')toggleCleaner();
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
function lens(v){v=+v;lastLens=v;lensHold=Date.now()+700;
  $('#ls').value=v;$('#lval').textContent=dist(v);
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
let trimSide='', armLimits='', lastShotN=-1, focusState='', lastLens=-1, lensHold=0;
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
              k:'สลับแปรง','[':'แปรงช้าลง',']':'แปรงเร็วขึ้น',r:'สลับทิศแปรง',',':'หรี่แปรง A','.':'หรี่แปรง B',p:'ทดสอบขาล้อ'};
const w4=v=>String(v).padStart(4);
function teleBlock(s){
  return `[${s.online?'OK ':'---'}] cmd=${s.cmd}   ${s.moving?'กำลังหมุน':'จอดอยู่'}\n`+
    `ล้อ  สั่ง ${w4(s.cmdL)},${w4(s.cmdR)}    จริง ${w4(s.curL)},${w4(s.curR)}\n`+
    `fwd=${s.fwd}  turn=${s.turn}(${s.ratio}%)  trim=${s.trim1.toFixed(2)}/${s.trim2.toFixed(2)}\n`+
    `TT=${s.tt?((s.tt_out!=null&&s.tt_out!==s.tt_pwm)?s.tt_out+'/'+s.tt_pwm:s.tt_pwm):'OFF'}   jog=${s.jog?'ON':'OFF'}   สติ๊กสูงสุด=${s.speed}\n`+
    `แขน สั่ง ${s.pan}°,${s.tilt}°   จริง ${s.panFw}°,${s.tiltFw}°\n`+
    `rx=${s.rx} (${s.rxHz}/วิ)   ผู้ควบคุม=`+
    (s.src==='pad'?'จอย':s.src==='web'?'เว็บ':'ว่าง');
}

async function poll(){
  try{
    const s=await(await fetch('/status?id='+wId)).json();

    if(s.oldFw) pill('#pLink',true,'เฟิร์มแวร์เก่า',true);
    else pill('#pLink',s.online,s.online?'ESP32':'ไม่ตอบสนอง');
    pill('#pPad',s.pad,s.pad?'จอยพร้อม':'ไม่พบจอย');
    pill('#pBrush',!!s.tt,s.tt?('แปรง '+(s.tt_out!=null?s.tt_out:s.tt_pwm)):'แปรงปิด',true);
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
    // โฟกัสถูกปรับจากจอย (ปุ่ม X/Y) -> ขยับสไลเดอร์ในเว็บตามให้เห็นตรงกัน
    // ถ้าเว็บเป็นคนสั่งเอง lastLens ถูกจดไว้แล้ว จะไม่แจ้งซ้ำ
    // lensHold: หลังเราสั่งเอง /status ยังตอบค่าเก่าอยู่อีกพักหนึ่ง
    // ถ้าไม่กันไว้ สไลเดอร์จะเด้งกลับค่าเดิมแวบหนึ่งก่อนจะตามมา
    if(Date.now()>lensHold && Math.abs(s.lens-lastLens)>0.004){
      $('#ls').value=s.lens; $('#lval').textContent=dist(s.lens);
      if(lastLens>=0 && !s.afBusy) say('โฟกัส '+dist(s.lens));
      lastLens=s.lens;
    }
    lastStatus=s;
    streamWatch(s);
    if(s.panFwd!=null && document.activeElement!==$('#bCalib')) showCalib(s.panFwd);
    if(s.cleaner!==cleanerOn) showCleaner(s.cleaner);
    updateNav(s);
    updateHud(s);
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
    $('#dTT').textContent=(s.tt?'เปิด':'ปิด')+' · duty '+(s.tt_out!=null&&s.tt?s.tt_out:s.tt_pwm);
    $('#dRx').textContent=s.rx+' บรรทัด  ('+s.rxHz+'/วิ)';
    $('#dSer').textContent=s.serial?'เปิดอยู่':'ยังไม่เชื่อม';
    $('#dPadName').textContent=s.padName||'–';
    $('#dUsed').textContent=(s.store.bytes/1048576).toFixed(1)+' MB';

    $('#dSetHead').textContent='สติ๊ก '+s.speed+' · ฐาน '+s.fwd;
    $('#dSpeed').textContent=s.speed;
    $('#dFwd').textContent=s.fwd;
    $('#dTurn').textContent=s.turn+'  ('+s.ratio+'% ของความเร็วฐาน)';
    $('#dTrim').textContent=s.trim1.toFixed(2)+' / '+s.trim2.toFixed(2);
    // เพดานจริงอยู่ที่ TT_MAX ในเฟิร์มแวร์ ซึ่งคำนวณจากแรงดันราง
    // ฝั่งนี้ไม่รู้ค่านั้น เลยแสดงแค่ duty ปัจจุบัน ไม่ใส่ตัวหารหลอก ๆ
    // ถ้า LOAD_SHARE หรี่อยู่ ค่าที่จ่ายจริงจะไม่เท่ากับค่าที่ตั้งไว้
    $('#dTTd').textContent = !s.tt ? ('ปิดอยู่ · ตั้งไว้ '+s.tt_pwm)
      : (s.tt_out!=null && s.tt_out!==s.tt_pwm)
        ? ('ตั้งไว้ '+s.tt_pwm+' · จ่ายจริง '+s.tt_out+'  ← กำลังหรี่')
        : String(s.tt_pwm);
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
    /* แบนด์วิดท์จริงที่วัดได้ — ตัวเลขนี้บอกตรง ๆ ว่าท่อต้องรับไหวแค่ไหน
       WiFi 2.4GHz กลางแจ้งมักไหวจริงราว 5-15 Mbit/s */
    const mb=(s.kbps||0)/1000;
    $('#dBw').textContent=mb?mb.toFixed(1):'–';
    $('#tBw').className='tile'+(mb>=8?' bad':mb>=5?' warn':'');
    $('#bwVal').textContent=(mb?mb.toFixed(1)+' Mbit/s':'–');
    if(s.streamFps!=null) showStreamFps(s.streamFps);
    if(s.streamSize) $('#ssVal').textContent=s.streamSize;
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

        # ---------- รหัสผ่าน ----------
        _WANT = base64.b64encode(
            f"{AUTH_USER}:{AUTH_PASS}".encode()).decode()

        def _auth_ok(self):
            if not AUTH_PASS:          # ปิดระบบรหัสผ่าน
                return True
            got = self.headers.get("Authorization", "")
            if not got.startswith("Basic "):
                return False
            # compare_digest กันการเดารหัสจากเวลาที่ใช้เปรียบเทียบ
            return hmac.compare_digest(got[6:].strip(), self._WANT)

        def _need_auth(self):
            self._send(401, "text/html; charset=utf-8",
                       "<h3>ต้องใส่รหัสผ่านก่อน</h3>".encode(),
                       {"WWW-Authenticate":
                        f'Basic realm="{AUTH_REALM}", charset="UTF-8"'})

        def do_GET(self):
            if not self._auth_ok():
                return self._need_auth()
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

            elif p == "/cleaner":
                v = self._qs("v")
                on = (arm.toggle_cleaner() if v == ""
                      else arm.set_cleaner(v not in ("0", "off", "false")))
                pan, tilt = arm.snapshot()
                self._json({"cleaner": on, "pan": pan, "tilt": tilt})

            elif p == "/calib":
                # fwd = ตั้งค่าตรง ๆ   d = ขยับทีละน้อย
                if self._qs("fwd") != "":
                    calib_save(self._q("fwd", PAN_FORWARD))
                elif self._qs("d") != "":
                    calib_save(PAN_FORWARD + self._q("d", 0.0))
                self._json({"panFwd": round(PAN_FORWARD, 2)})

            elif p == "/lagtest":
                # วัดความหน่วงจริงแบบ "กระจกถึงกระจก" (glass-to-glass)
                # วิธีเดียวที่วัดได้ทั้งเส้น: กล้อง -> ISP -> เข้ารหัส -> เน็ต
                #                            -> ถอดภาพ -> วาดลงจอ
                # หน้านี้โชว์นาฬิกาตัวโต ๆ กับภาพสดข้าง ๆ กัน
                # เอากล้องหุ่นส่องจอ แล้วถ่ายรูปจอทั้งหน้า
                # เลขในภาพสด = อดีต  เลขนาฬิกา = ปัจจุบัน  ผลต่าง = ความหน่วง
                self._ok("text/html; charset=utf-8", LAGPAGE.encode())

            elif p == "/streamfps":
                if self._qs("v") != "":
                    stream_fps_save(self._q("v", STREAM_FPS), cam)
                self._json({"fps": round(STREAM_FPS, 1)})

            elif p == "/speed":
                v = mux.set_speed(self._q("v", SPEED_DEFAULT))
                self._ok("text/plain", str(v).encode())

            elif p == "/status":
                with mux.lock:
                    cmdl, cmdr, src = mux.l, mux.r, mux.src
                    ax, ay, spd = mux.ax, mux.ay, mux.speed
                armPan, armTilt, armClean = arm.state()
                body = dict(stats.snapshot())
                body.update(soc_health())
                body.update({
                    "online": link.online,
                    "serial": link.ser is not None,
                    "cmdL": cmdl, "cmdR": cmdr, "src": src,
                    "ax": round(ax, 3), "ay": round(ay, 3), "speed": spd,
                    "curL": link.st["curL"], "curR": link.st["curR"],
                    "tt": link.st["tt"], "tt_pwm": link.st["tt_pwm"],
                    "tt_out": link.st["tt_out"],
                    "trim1": link.st["trim1"], "trim2": link.st["trim2"],
                    "jog": link.st["jog"], "fwd": link.st["fwd"],
                    "cmd": link.st["cmd"], "turn": link.st["turn"],
                    "ratio": link.st["ratio"],
                    "rx": link.rx, "rxHz": round(link.rx_hz, 1),
                    "fields": link.fields,
                    "swap": SWAP_LR, "invL": INVERT_L, "invR": INVERT_R,
                    "trimSide": "ซ้าย" if SWAP_LR else "ขวา",
                    "pan": armPan, "tilt": armTilt,
                    # โหมด cleaner + ค่าที่หน้าเว็บใช้วาด HUD และหน้าปัด
                    "cleaner": armClean,
                    "cleanPan": CLEAN_PAN, "cleanTilt": CLEAN_TILT,
                    "hfov": CAM_HFOV, "panFwd": round(PAN_FORWARD, 2),
                    # ตัวนับเฟรมของกล้องเอง ไม่ขึ้นกับว่ามีใครดูสตรีมอยู่
                    # หน้าเว็บใช้เทียบว่า "กล้องยังถ่ายอยู่แต่ภาพไม่ขยับ" = สายตาย
                    "frameN": cam.n,
                    # เฟรมที่ส่งออกสำเร็จให้ "การเชื่อมต่อของผู้ถามคนนี้"
                    # null = ยังไม่เคยส่งให้ id นี้ (เพิ่งต่อ / ต่อไม่ติด)
                    "txN": tally.get(self._qs("id")),
                    "streamFps": round(STREAM_FPS, 1),
                    "buffers": STREAM_BUFFERS,
                    "streamSize": f"{STREAM_SIZE[0]}x{STREAM_SIZE[1]}",
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
                # ★ ถ้า WiFi หลุดแบบ "ครึ่งใบ" (ฝั่งเราไม่รู้ว่าอีกฝั่งหายไปแล้ว)
                #   write() จะค้างรอตลอดกาล เธรดนี้ไม่มีวันจบ และเบราว์เซอร์
                #   ก็ค้างภาพสุดท้ายไว้โดยไม่มี error ให้จับ
                #   ตั้ง timeout ไว้ พอเขียนไม่ออกภายใน 10 วิ ก็ตัดทิ้งไปเลย
                try:
                    self.connection.settimeout(10)
                except OSError:
                    pass
                # ★ ปิด Nagle: อัลกอริทึมนี้จะกลั้นแพ็กเก็ตเล็ก ๆ ไว้รอรวมกับ
                #   ก้อนถัดไป ดีสำหรับโหลดไฟล์ แต่สำหรับภาพสดคือการเพิ่มหน่วง
                #   ฟรี ๆ ท้ายเฟรมเป็นก้อนเล็กเสมอ มันจึงโดนกลั้นทุกเฟรม
                try:
                    self.connection.setsockopt(socket.IPPROTO_TCP,
                                               socket.TCP_NODELAY, 1)
                except (OSError, AttributeError):
                    pass
                # ★★★ ตัวที่ทำให้ "ลื่นตอนแรก แล้วแล็คขึ้นเรื่อย ๆ แล้วหลุด" ★★★
                #   Linux ขยาย buffer ส่งของ socket เองได้ถึงหลายเมกะไบต์
                #   = กองเฟรมได้หลายสิบใบ ทุกใบที่กองอยู่คือความเก่าเพิ่ม 1/fps
                #   พอเบราว์เซอร์ถอดภาพช้ากว่าที่เราส่งแค่นิดเดียว เฟรมจะกอง
                #   แล้วภาพที่ปลายทางเห็นก็เก่าขึ้นทุกวินาที จนสุดท้าย write()
                #   บล็อกนานเกิน 10 วิ -> timeout -> สายหลุดจริง
                #   จำกัด buffer ไว้แค่ไม่กี่เฟรม ความเก่าจะถูกล็อกเพดานไว้
                #   และเพราะ cam.frame เก็บแต่เฟรมล่าสุด การรอให้ buffer ว่าง
                #   จึงกลายเป็นการ "ข้ามเฟรมเก่าทิ้งเอง" โดยอัตโนมัติ
                try:
                    want = STREAM_SNDBUF
                    if want <= 0:        # ปรับตามขนาดเฟรมจริงของความละเอียดนี้
                        cur = cam.frame
                        want = max(SNDBUF_FLOOR, len(cur) if cur else 0)
                    self.connection.setsockopt(
                        socket.SOL_SOCKET, socket.SO_SNDBUF, want)
                except OSError:
                    pass
                sid = self._qs("id")      # รหัสประจำการเชื่อมต่อจากฝั่งเว็บ
                sock = self.connection
                gapw = 1.0 / max(1.0, STREAM_FPS)
                seen = 0
                next_tx = 0.0
                try:
                    while True:
                        with cam.cond:
                            fresh = cam.cond.wait_for(lambda: cam.n != seen,
                                                      timeout=5)
                            f, seen, ft = cam.frame, cam.n, cam.frame_t
                        # ครบ 5 วิแล้วยังไม่มีเฟรมใหม่ -> อย่าส่งเฟรมเดิมซ้ำ
                        # ปล่อยให้วนรอต่อ ฝั่งเบราว์เซอร์จะได้รู้ว่าภาพนิ่งจริง
                        if f is None or not fresh:
                            continue
                        # ★ เพดาน fps — กล้องถ่าย 30fps แต่เราไม่จำเป็นต้องส่งหมด
                        #   ข้ามเฟรมที่เร็วเกินเพดาน ช่วยลดแบนด์วิดท์ตรง ๆ
                        #   ใช้ "เวลานัดครั้งถัดไป" แบบสะสม ไม่ใช่วัดช่วงห่างเฉย ๆ
                        #   ถ้าวัดช่วงห่าง เฟรมที่มาถึงก่อนเส้นตายนิดเดียวจะโดนทิ้ง
                        #   ทำให้ได้ fps จริงต่ำกว่าที่ตั้งไว้ (ขอ 20 ได้ 15)
                        now = time.time()
                        if now < next_tx:
                            continue
                        gap = 1.0 / max(1.0, STREAM_FPS)
                        next_tx = max(now, next_tx + gap)
                        # ★ ถามก่อนเขียน: socket ว่างพอจะรับไหม
                        #   รอได้แค่หนึ่งช่วงเฟรม ไม่ว่างก็ "ทิ้งเฟรมนี้" ไปเลย
                        #   สำหรับภาพสด เฟรมใหม่มีค่ากว่าเฟรมครบ
                        #   ทิ้งใบนี้แล้ววนไปเอาใบล่าสุด ดีกว่านั่งรอส่งใบเก่า
                        try:
                            ready = select.select([], [sock], [], gapw)[1]
                        except (OSError, ValueError):
                            break
                        if not ready:
                            stats.tick_drop()
                            next_tx = 0.0   # อย่าหน่วงซ้ำ ใบถัดไปส่งได้ทันที
                            continue
                        head = (b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                                + f"Content-Length: {len(f)}\r\n\r\n".encode())
                        stats.tick_fps(seen, len(head) + len(f) + 2)
                        # เฟรมนี้นอนรออยู่ฝั่ง Pi กี่มิลลิวินาทีก่อนได้ออกเดินทาง
                        # แยกความหน่วงของหุ่น ออกจากความหน่วงของเน็ต+เบราว์เซอร์
                        stats.tick_lag((now - ft) * 1000.0 if ft else 0.0)
                        # ★ เขียนครั้งเดียว ไม่แยกสามครั้ง
                        #   แยกเขียนทำให้ท้ายเฟรมกลายเป็นแพ็กเก็ตเล็กหางแถว
                        #   ซึ่งเป็นตัวที่โดน Nagle กลั้นและทำให้หน่วงเพิ่ม
                        self.wfile.write(head + f + b"\r\n")
                        # นับหลัง write เท่านั้น — write ที่ค้างหรือพังจะไม่นับ
                        # ตัวเลขนี้จึงเป็นหลักฐานว่า "ไบต์ออกไปถึงปลายทางแล้ว"
                        tally.tick(sid)
                except (BrokenPipeError, ConnectionResetError,
                        ConnectionAbortedError, TimeoutError, OSError):
                    pass   # ลูกค้าหายไป — ปล่อยเธรดนี้จบ

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

    calib_load()
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
    print(f"login:  {AUTH_USER} / {'*' * len(AUTH_PASS)}"
          if AUTH_PASS else "login:  ปิดระบบรหัสผ่านอยู่ (ใครก็เข้าได้)")
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
