# -*-coding:utf-8-*-
from robomaster import robot
import time
from collections import deque
import statistics
import math

# ============ Parameters ============
# กำหนดค่าพารามิเตอร์ต่าง ๆ
WINDOW_SIZE = 5            # ขนาดหน้าต่างสำหรับฟิลเตอร์ Moving Average และ Median
DIST_THRESHOLD = 40        # ระยะทางที่หุ่นยนต์จะหยุด (หน่วย: เซนติเมตร)
SAMPLE_RATE = 5            # ความถี่ในการอ่านค่าจากเซ็นเซอร์ TOF (หน่วย: Hz)

# ============ Filters ============
# คลาสสำหรับฟิลเตอร์ Moving Average
class MovingAverageFilter:
    def __init__(self, window_size):
        self.values = deque(maxlen=window_size)  # ใช้ deque เก็บค่าล่าสุดตามขนาดหน้าต่าง

    def filter(self, new_value):
        self.values.append(new_value)           # เพิ่มค่าล่าสุดเข้าไปใน deque
        return sum(self.values) / len(self.values)  # คำนวณค่าเฉลี่ยของค่าที่เก็บไว้

# คลาสสำหรับฟิลเตอร์ Median
class MedianFilter:
    def __init__(self, window_size):
        self.values = deque(maxlen=window_size)  # ใช้ deque เก็บค่าล่าสุดตามขนาดหน้าต่าง

    def filter(self, new_value):
        self.values.append(new_value)           # เพิ่มค่าล่าสุดเข้าไปใน deque
        return statistics.median(self.values)   # คำนวณค่ามัธยฐานของค่าที่เก็บไว้

# คลาสสำหรับฟิลเตอร์ Low-pass
class LowPassFilter:
    def __init__(self, cutoff_freq, sample_rate):
        self.dt = 1.0 / sample_rate             # คำนวณค่า dt จาก sample rate
        self.alpha = (2 * math.pi * cutoff_freq * self.dt) / (2 * math.pi * cutoff_freq * self.dt + 1)  # คำนวณค่า alpha
        self.last_output = None                # เก็บค่าผลลัพธ์ล่าสุด

    def filter(self, new_value):
        if self.last_output is None:           # ถ้ายังไม่มีค่าผลลัพธ์ล่าสุด ให้ใช้ค่าปัจจุบัน
            self.last_output = new_value
        else:
            # คำนวณค่าผลลัพธ์ใหม่โดยใช้สูตร Low-pass Filter
            self.last_output = self.alpha * new_value + (1 - self.alpha) * self.last_output
        return self.last_output

# ============ Main Program ============

if __name__ == '__main__':
    # เริ่มต้นการเชื่อมต่อกับหุ่นยนต์
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")  # ใช้การเชื่อมต่อแบบ Wi-Fi (sta)

    ep_chassis = ep_robot.chassis  # เข้าถึงโมดูล chassis (การเคลื่อนที่)
    ep_sensor = ep_robot.sensor    # เข้าถึงโมดูล sensor (เซ็นเซอร์)

    # สมัครสมาชิกเพื่อรับข้อมูลจากเซ็นเซอร์ TOF
    ep_sensor.sub_distance(freq=SAMPLE_RATE)
    # เรียกใช้ ฟิลเตอร์ต่าง ๆ
    try:
        # เริ่มให้หุ่นยนต์เคลื่อนที่ไปข้างหน้าด้วยความเร็ว 0.25 m/s
        ep_chassis.drive_speed(x=0.25, y=0, z=0)

        while True:
            # อ่านค่าระยะทางดิบจากเซ็นเซอร์ TOF
            tof_data = ep_sensor.get_distance_info()
            raw_distance = tof_data['distance']  # ดึงค่าระยะทางดิบออกมา

            # แสดงค่าระยะทางดิบ
            print(f"Raw Distance: {raw_distance} cm")

            # ตรวจสอบว่าระยะทางดิบต่ำกว่าหรือเท่ากับค่าที่กำหนดหรือไม่
            if raw_distance <= DIST_THRESHOLD:
                # หยุดการเคลื่อนที่ของหุ่นยนต์
                ep_chassis.drive_speed(x=0, y=0, z=0)
                print("Object detected within threshold. Stopping.")  # แจ้งเตือนว่าหยุดการเคลื่อนที่
                break

            # รอเวลาตาม sample rate ก่อนอ่านค่าครั้งถัดไป
            time.sleep(1.0 / SAMPLE_RATE)

    except KeyboardInterrupt:
        # กรณีที่ผู้ใช้หยุดโปรแกรมด้วย Ctrl+C
        print("Program interrupted by user.")

    finally:
        # ยกเลิกการสมัครสมาชิกเซ็นเซอร์และปิดการเชื่อมต่อกับหุ่นยนต์
        ep_sensor.unsub_distance()
        ep_robot.close()