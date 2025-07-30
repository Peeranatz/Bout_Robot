import math
import time
import csv
import os
from robomaster import robot

position = [0, 0, 0]
position_log = []
pd_controller_log = []  # เปลี่ยนจาก p_controller_log เป็น pd_controller_log
start_time = None
start_x = None
TARGET_DISTANCE = 2.0
TOLERANCE = 0.02
movement_completed = False
kp = 1.0  # ลดค่า kp ให้เหมาะกับการควบคุมความเร็ว
kd = 1.2  # เพิ่มค่า kd เพื่อลด overshoot
e = 2.0

# ตัวแปรสำหรับ PD Controller
previous_error = None
previous_time = None
pd_control_active = False


def sub_position_handler(position_info):
    global position, position_log, start_time, start_x, movement_completed
    global previous_error, previous_time, pd_control_active, pd_controller_log

    position = position_info
    x = position[0]

    # PD Controller - ควบคุมความเร็วแบบต่อเนื่อง (ทำงานตั้งแต่เริ่มต้น)
    if start_time is not None and start_x is not None and pd_control_active:
        current_time = time.time()
        current_distance = abs(x - start_x)
        current_error = TARGET_DISTANCE - current_distance
        elapsed = current_time - start_time

        # บันทึกข้อมูลการเคลื่อนที่
        position_log.append((elapsed, current_distance, current_error))
        print(f"Time: {elapsed:.2f}s, Distance: {current_distance:.3f}m, Error: {current_error:.3f}m")

        if abs(current_error) > TOLERANCE:
            # คำนวณ P
            p_value = kp * current_error

            # คำนวณ D
            d_value = 0
            if previous_error is not None and previous_time is not None:
                dt = current_time - previous_time
                if dt > 0:
                    derror = current_error - previous_error
                    d_value = kd * (derror / dt)

            # คำนวณ PD output เป็นความเร็ว
            pd_speed = p_value + d_value

            # จำกัดความเร็วสูงสุด
            max_speed = 10
            if pd_speed > max_speed:
                pd_speed = max_speed
            elif pd_speed < -max_speed:
                pd_speed = -max_speed

            # บันทึกข้อมูล PD Controller
            pd_controller_log.append((elapsed, current_distance, current_error, p_value, d_value, pd_speed))

            print(f"  P={p_value:.3f}, D={d_value:.3f}, PD_Speed={pd_speed:.3f}")

            # ควบคุมความเร็วแบบต่อเนื่อง
            if current_error > 0:
                ep_chassis.drive_speed(x=pd_speed, y=0, z=0)
                print(f"  → เคลื่อนที่ไปข้างหน้าด้วยความเร็ว {pd_speed:.3f} m/s")
            else:
                ep_chassis.drive_speed(x=0, y=0, z=0)
                print(f"  → หยุดการเคลื่อนที่ (เลยเป้าหมายแล้ว)")

            # อัพเดทค่าก่อนหน้า
            previous_error = current_error
            previous_time = current_time
        else:
            # ถึงเป้าหมายแล้ว - หยุดการเคลื่อนที่
            ep_chassis.drive_speed(x=0, y=0, z=0)
            print(f"PD Controller เสร็จสิ้น! Error: {current_error:.3f}m <= {TOLERANCE}m")
            pd_control_active = False


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    print("เริ่มการเชื่อมต่อกับ RoboMaster...")

    # Subscribe ตำแหน่ง
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    print("รอการเชื่อมต่อเซ็นเซอร์...")
    time.sleep(2)  # รอให้เซ็นเซอร์เชื่อมต่อ

    # บันทึกตำแหน่งเริ่มต้นและเริ่มจับเวลา
    start_x, start_y, _ = position
    start_time = time.time()
    print(f"เริ่มตำแหน่ง: X={start_x:.3f}, Y={start_y:.3f}")

    # ใช้ PD Controller แทนการเคลื่อนที่แบบ open-loop
    print("เริ่มเคลื่อนที่ด้วย PD Controller...")
    
    # เปิดใช้งาน PD Controller ทันที
    pd_control_active = True
    previous_error = TARGET_DISTANCE  # error เริ่มต้น = 2.0
    previous_time = time.time()
    
    # รอให้ PD Controller ทำงานจนถึงเป้าหมาย
    timeout = 30
    start_pd_time = time.time()
    
    while pd_control_active and (time.time() - start_pd_time) < timeout:
        time.sleep(0.05)  # sampling rate 20 Hz
    
    # หยุดการเคลื่อนที่
    ep_chassis.drive_speed(x=0, y=0, z=0)
    
    if not pd_control_active:
        print("PD Controller เสร็จสิ้น - ถึงเป้าหมายแล้ว")
    else:
        print("PD Controller หมดเวลา (30 วินาที)")
        pd_control_active = False

    print("การเคลื่อนที่เสร็จสิ้น")
    time.sleep(1)  # รอให้ข้อมูลสุดท้ายมา

    # บันทึกตำแหน่งสุดท้าย
    end_x, end_y, _ = position
    final_distance = abs(end_x - start_x)
    final_error = TARGET_DISTANCE - final_distance

    print(f"ระยะทางที่เคลื่อนที่จริง: {final_distance:.3f} เมตร")
    print(f"Error สุดท้าย: {final_error:.3f} เมตร")

    # บันทึกลง CSV
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, "pd_xposition12.csv")
    pd_csv_path = os.path.join(current_dir, "pd_controller.csv")

    # บันทึกข้อมูลการเคลื่อนที่
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "distance"])
        for row in position_log:
            writer.writerow(row)

    # บันทึกข้อมูล PD Controller
    with open(pd_csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_measured", "error", "p_value", "d_value", "pd_speed"])
        for row in pd_controller_log:
            writer.writerow(row)

    print(f"บันทึกข้อมูลเสร็จสิ้น:")
    print(f"  - การเคลื่อนที่: {len(position_log)} บรรทัด ({csv_path})")
    print(f"  - PD Controller: {len(pd_controller_log)} บรรทัด ({pd_csv_path})")

    # ยกเลิก subscription ก่อนปิด
    print("กำลังปิดการเชื่อมต่อ...")
    ep_chassis.unsub_position()
    ep_robot.close()
    print("เสร็จสิ้น!")
    # บันทึกข้อมูลการเคลื่อนที่หลัก (ตอนนี้คือ PD Controller)
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_traveled"])
        for row in position_log:
            writer.writerow(row)

    # บันทึกข้อมูล PD Controller
    with open(pd_csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_measured", "error", "p_value", "d_value", "pd_speed"])
        for row in pd_controller_log:
            writer.writerow(row)

    print(f"บันทึกข้อมูลเสร็จสิ้น:")
    print(f"  - การเคลื่อนที่: {len(position_log)} บรรทัด ({csv_path})")
    print(f"  - PD Controller: {len(pd_controller_log)} บรรทัด ({pd_csv_path})")

    # ยกเลิก subscription ก่อนปิด
    print("กำลังปิดการเชื่อมต่อ...")
    ep_chassis.unsub_position()
    ep_robot.close()
    print("เสร็จสิ้น!")
