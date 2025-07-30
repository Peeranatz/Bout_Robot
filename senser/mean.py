# -*-coding:utf-8-*-
from robomaster import robot
import time
from collections import deque
import statistics
import math
import csv
import threading
import os

# ============ Parameters ============
WINDOW_SIZE = 5            
TARGET_DISTANCE = 40.0     # ระยะที่ต้องการให้หยุด (cm)
SAMPLE_RATE = 50           # กลับเป็น 50 Hz เหมือนเดิม
SENSOR_OFFSET = 8          

# ============ Global Variables ============
stop_robot = False
csv_writer = None
robot_moving = True
lock = threading.Lock()
t_detect = None            # เวลาที่ตรวจพบวัตถุ
t_stop_command = None      # เวลาที่สั่งหยุด
response_time = None       # เวลาตอบสนอง
start_time = None          # เพิ่มตัวแปรเก็บเวลาเริ่มต้น
median_filter = None       # เพิ่มตัวแปรสำหรับ Median Filter

# ============ Filters ============
class MovingAverageFilter:
    def __init__(self, window_size):
        self.values = deque(maxlen=window_size)

    def filter(self, new_value):
        self.values.append(new_value)
        return sum(self.values) / len(self.values)

class MedianFilter:
    def __init__(self, window_size):
        self.values = deque(maxlen=window_size)

    def filter(self, new_value):
        self.values.append(new_value)
        return statistics.median(self.values)

class LowPassFilter:
    def __init__(self, cutoff_freq, sample_rate):
        self.dt = 1.0 / sample_rate
        self.alpha = (2 * math.pi * cutoff_freq * self.dt) / (2 * math.pi * cutoff_freq * self.dt + 1)
        self.last_output = None

    def filter(self, new_value):
        if self.last_output is None:
            self.last_output = new_value
        else:
            self.last_output = self.alpha * new_value + (1 - self.alpha) * self.last_output
        return self.last_output

# ============ Enhanced Filters ============
class AdaptiveFilter:
    def __init__(self, window_size, cutoff_freq, sample_rate):
        self.ma_filter = MovingAverageFilter(window_size)
        self.lp_filter = LowPassFilter(cutoff_freq, sample_rate)
        self.last_values = deque(maxlen=3)
        
    def filter(self, new_value):
        self.last_values.append(new_value)
        
        # คำนวณความแปรปรวนเพื่อตัดสินใจว่าจะใช้ฟิลเตอร์แบบไหน
        if len(self.last_values) >= 3:
            variance = statistics.variance(self.last_values)
            if variance > 100:  # ค่าเปลี่ยนแปลงมากใช้ฟิลเตอร์อ่อน
                return self.lp_filter.filter(new_value)
            else:  # ค่าเสถียรใช้ฟิลเตอร์แรง
                return self.ma_filter.filter(new_value)
        else:
            return new_value

# ============ Sensor Callback Function ============
def distance_callback(sub_info):
    global stop_robot, csv_writer, robot_moving, ep_chassis
    global t_detect, t_stop_command, response_time, start_time, median_filter
    
    try:
        # ข้อมูลจาก TOF sensor มาเป็นค่าเดียว (หน่วย mm)
        distance = sub_info[0] if isinstance(sub_info, (list, tuple)) else sub_info
        tof_distance = distance / 10.0  # แปลงจาก mm เป็น cm
        
        # กรองข้อมูลด้วย Median Filter
        filtered_distance = median_filter.filter(tof_distance)
        
        # คำนวณเวลาที่ผ่านไปตั้งแต่เริ่มต้น
        elapsed_time = time.time() - start_time if start_time else 0
        
        # บันทึกข้อมูลลง CSV (ทั้งข้อมูลดิบและที่กรองแล้ว)
        with lock:
            if csv_writer:
                csv_writer.writerow([elapsed_time, tof_distance, filtered_distance])
        
        # แสดงค่าระยะทาง
        print(f"Time: {elapsed_time:.3f}s, Raw: {tof_distance:.1f} cm, Filtered: {filtered_distance:.1f} cm")
        
        # ตรวจสอบเงื่อนไขหยุดด้วยค่าที่กรองแล้ว
        if filtered_distance < TARGET_DISTANCE and robot_moving:
            t_detect = time.time()
            
            with lock:
                stop_robot = True
                robot_moving = False
            
            ep_chassis.drive_speed(x=0, y=0, z=0)
            t_stop_command = time.time()
            
            response_time = t_stop_command - t_detect
            
            print(f"Object detected! Raw: {tof_distance:.1f} cm, Filtered: {filtered_distance:.1f} cm")
            print(f"Response time: {response_time*1000:.2f} ms")

    except Exception as e:
        print(f"Error in callback: {e}")

# ============ Main Program ============
if __name__ == '__main__':
    # สร้างพาธไฟล์ CSV ในโฟลเดอร์เดียวกับโค้ด
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, "mean.csv")
    
    # สร้าง Median Filter
    median_filter = MedianFilter(WINDOW_SIZE)
    
    # เปิดไฟล์ CSV สำหรับบันทึกข้อมูล
    with open(csv_path, mode="w", newline="", encoding="utf-8") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Time", "Raw_Distance", "Filtered_Distance"])  # เพิ่มคอลัมน์ filtered

        # เริ่มต้นการเชื่อมต่อกับหุ่นยนต์
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")

        ep_chassis = ep_robot.chassis
        ep_sensor = ep_robot.sensor

        try:
            # บันทึกเวลาเริ่มต้น
            start_time = time.time()
            
            # สมัครสมาชิกเซ็นเซอร์ TOF ด้วย callback
            ep_sensor.sub_distance(freq=SAMPLE_RATE, callback=distance_callback)
            
            print(f"Robot started moving forward...")
            print(f"TOF Sensor sampling at {SAMPLE_RATE} Hz")
            print(f"Median Filter window size: {WINDOW_SIZE}")
            print(f"Will stop when filtered distance < {TARGET_DISTANCE} cm")
            print(f"Data saving to: {csv_path}")
            
            # เริ่มเคลื่อนที่ทันที
            ep_chassis.drive_speed(x=0.25, y=0, z=0)

            # รอจนกว่าจะได้รับสัญญาณหยุด
            while not stop_robot:
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nProgram interrupted by user.")
            
        finally:
            # หยุดหุ่นยนต์และยกเลิกการสมัครสมาชิก
            with lock:
                stop_robot = True
                robot_moving = False
            ep_chassis.drive_speed(x=0, y=0, z=0)
            ep_sensor.unsub_distance()
            ep_robot.close()
            print("Robot stopped and disconnected.")
            print(f"Data saved to: {csv_path}")