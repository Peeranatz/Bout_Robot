import math
import time
import csv
import os
from robomaster import robot

position = [0, 0, 0]
position_log = []
start_time = None
start_x = None
TARGET_DISTANCE = 2.0
sensor_connected = False
data_received_count = 0


def sub_position_handler(position_info):
    global position, position_log, start_time, start_x, sensor_connected, data_received_count
    position = position_info
    sensor_connected = True
    data_received_count += 1
    x = position[0]
    
    # Debug: แสดงข้อมูลที่ได้รับ
    if data_received_count <= 5:  # แสดงข้อมูล 5 ครั้งแรก
        print(f"Received data #{data_received_count}: x={x:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")

    # บันทึกข้อมูลตลอดเวลาหลังจากเริ่มการเคลื่อนที่
    if start_time is not None and start_x is not None:
        elapsed = time.time() - start_time
        distance_traveled = x - start_x  # ระยะที่เดินได้
        error = TARGET_DISTANCE - distance_traveled  # error = 2 - ระยะที่เดินได้
        position_log.append((elapsed, distance_traveled, error))
        print(f"Time: {elapsed:.2f}s, Distance: {distance_traveled:.3f}m, Error: {error:.3f}m")


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    print("เริ่มการเชื่อมต่อกับ RoboMaster...")
    
    # Subscribe ตำแหน่ง
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    print("รอการเชื่อมต่อเซ็นเซอร์...")
    
    # รอให้เซ็นเซอร์เชื่อมต่อและได้รับข้อมูลเสถียร
    wait_time = 0
    max_wait_time = 15  # เพิ่มเวลารอ
    stable_data_count = 3  # ต้องได้รับข้อมูลอย่างน้อย 3 ครั้ง
    
    while (not sensor_connected or data_received_count < stable_data_count) and wait_time < max_wait_time:
        time.sleep(0.5)
        wait_time += 0.5
        if sensor_connected:
            print(f"รอข้อมูลเสถียร... ({wait_time:.1f}s) - ได้รับข้อมูล {data_received_count} ครั้ง")
        else:
            print(f"รอเซ็นเซอร์... ({wait_time:.1f}s)")

    # ตรวจสอบว่าได้รับข้อมูลแล้วหรือไม่
    if not sensor_connected or data_received_count < stable_data_count:
        print(f"ไม่ได้รับข้อมูลจากเซ็นเซอร์เพียงพอ! (ได้รับ {data_received_count} ครั้ง)")
        print("กำลังปิดการเชื่อมต่อ...")
        ep_chassis.unsub_position()
        ep_robot.close()
        exit()

    print(f"เซ็นเซอร์เชื่อมต่อเรียบร้อย! ได้รับข้อมูล {data_received_count} ครั้ง")

    # รอให้ข้อมูลเสถียรอีกสักครู่
    time.sleep(1)

    # บันทึกตำแหน่งเริ่มต้นและเริ่มจับเวลา
    start_x, start_y, _ = position
    start_time = time.time()
    print(f"เริ่มตำแหน่ง: X={start_x:.3f}, Y={start_y:.3f}")

    # เคลื่อนที่ 2 เมตร
    print("เริ่มเคลื่อนที่...")
    ep_chassis.move(x=2.0, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(2)  # รอให้ข้อมูลสุดท้ายมา

    # บันทึกตำแหน่งสุดท้าย
    end_x, end_y, _ = position
    final_distance = end_x - start_x
    final_error = TARGET_DISTANCE - final_distance

    print(f"ระยะทางที่เคลื่อนที่จริง: {final_distance:.3f} เมตร")
    print(f"Error สุดท้าย: {final_error:.3f} เมตร")

    # บันทึกลง CSV
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, "Po_sky.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_traveled", "error"])
        for row in position_log:
            writer.writerow(row)

    print(f"บันทึกข้อมูลเสร็จสิ้น จำนวน {len(position_log)} บรรทัด")
    print(f"ไฟล์: {csv_path}")

    # ยกเลิก subscription ก่อนปิด
    print("กำลังปิดการเชื่อมต่อ...")
    ep_chassis.unsub_position()
    ep_robot.close()
    print("เสร็จสิ้น!")