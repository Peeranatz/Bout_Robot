import math
import time
import csv
from robomaster import robot

KP = 0.1  # ค่าคงที่สัดส่วน

position = [0, 0, 0]

# เก็บข้อมูล x และ timestamp
position_log = []
start_time = None
start_x = None
recording = False


def sub_position_handler(position_info):
    global position, position_log, start_time, start_x, recording
    position = position_info
    x = position[0]
    now = time.time()
    if start_x is not None:
        if not recording and x > start_x:
            # เริ่มจับเวลาเมื่อ x เพิ่มขึ้น
            start_time = now
            recording = True
        if recording:
            elapsed = now - start_time
            x_offset = x - start_x  # ปรับค่า x ให้เริ่มที่ 0
            position_log.append((elapsed, x_offset))


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    # Subscribe ตำแหน่ง
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    time.sleep(0.5)  # รอให้ตำแหน่งอัปเดต

    # บันทึกตำแหน่งเริ่มต้น
    start_x, start_y, _ = position

    # เคลื่อนที่ 2 เมตร
    ep_chassis.move(x=2.0, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(0.5)  # รอให้ตำแหน่งอัปเดต

    # บันทึกตำแหน่งสุดท้าย
    end_x, end_y, _ = position

    # คำนวณระยะทางที่เคลื่อนที่จริง
    distance = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)
    print(f"ระยะทางที่เคลื่อนที่จริง: {distance:.2f} เมตร")

    # คำนวณ Error
    ERROR = 2.0 - distance
    print(f"ERROR: {ERROR:.2f}")

    # คำนวณค่าชดเชยแบบสัดส่วน
    P = ERROR * KP
    print(f"P (ชดเชย): {P:.2f}")

    # ถ้าเดินเกินหรือขาด ให้ชดเชยทันที
    if abs(P) > 0.01:
        print(f"ชดเชยระยะทาง: {P:.2f} เมตร")
        ep_chassis.move(x=P, y=0, z=0, xy_speed=0.7).wait_for_completed()

    # บันทึกข้อมูลลงไฟล์ CSV
    with open("Po_sky.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "x_position"])
        for row in position_log:
            writer.writerow(row)

    ep_robot.close()