import math
import time
import csv
import os
from robomaster import robot

position = [0, 0, 0]
position_log = []
p_controller_log = []  # เพิ่มสำหรับบันทึกข้อมูล P Controller
start_time = None
start_x = None
TARGET_DISTANCE = 2.0
TOLERANCE = 0.02  # เปลี่ยนจาก 0.01 เป็น 0.02 เพื่อให้แม่นยำมากขึ้น
movement_completed = False
kp = 1.5
e = 2.0  # ระยะทางเป้าหมายที่ต้องการเคลื่อนที่


def sub_position_handler(position_info):
    global position, position_log, start_time, start_x, movement_completed
    position = position_info
    x = position[0]

    # บันทึกข้อมูลตลอดเวลาหลังจากเริ่มการเคลื่อนที่ และยังไม่เดินเสร็จ
    if start_time is not None and start_x is not None and not movement_completed:
        elapsed = time.time() - start_time
        distance_traveled = abs(x - start_x)  # ระยะที่เดินได้ (ค่าสัมบูรณ์)
        error = TARGET_DISTANCE - distance_traveled  # error = 2 - ระยะที่เดินได้
        position_log.append((elapsed, distance_traveled, error))
        print(
            f"Time: {elapsed:.2f}s, Distance: {distance_traveled:.3f}m, Error: {error:.3f}m"
        )


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

    # เคลื่อนที่ 2 เมตร
    print("เริ่มเคลื่อนที่...")
    ep_chassis.move(x=e, y=0, z=0, xy_speed=kp).wait_for_completed()

    # หยุดการบันทึกข้อมูลเมื่อเดินเสร็จ
    movement_completed = True
    print("การเคลื่อนที่เสร็จสิ้น - หยุดการบันทึกข้อมูล")

    time.sleep(1)  # รอให้ข้อมูลสุดท้ายมา

    # บันทึกตำแหน่งสุดท้าย
    end_x, end_y, _ = position
    final_distance = abs(end_x - start_x)  # ค่าสัมบูรณ์
    final_error = TARGET_DISTANCE - final_distance

    print(f"ระยะทางที่เคลื่อนที่จริง: {final_distance:.3f} เมตร")
    print(f"Error สุดท้าย: {final_error:.3f} เมตร")

    # P Controller Loop
    print("\n=== เริ่ม P Controller ===")
    print(
        f"ตรวจสอบ: abs({final_error:.3f}) > {TOLERANCE} = {abs(final_error) > TOLERANCE}"
    )

    if abs(final_error) <= TOLERANCE:
        print(f"หุ่นยนต์อยู่ในระยะที่ยอมรับได้แล้ว (±{TOLERANCE}m)")
    else:
        print(f"เริ่มปรับแต่งด้วย P Controller...")

    p_start_time = time.time()
    iteration = 0

    while abs(final_error) > TOLERANCE:
        iteration += 1
        current_x, _, _ = position
        current_distance = abs(current_x - start_x)
        error = TARGET_DISTANCE - current_distance
        p_value = kp * error

        # บันทึกข้อมูล P Controller
        elapsed = time.time() - p_start_time
        p_controller_log.append((elapsed, current_distance, error, p_value))

        print(
            f"P-Loop {iteration}: Distance={current_distance:.3f}m, Error={error:.3f}m, P={p_value:.3f}"
        )

        # ตัดสินใจการเคลื่อนที่
        if p_value > 0:
            print(f"  → เคลื่อนที่ไปข้างหน้า {abs(p_value):.3f}m")
            ep_chassis.move(x=abs(p_value), y=0, z=0, xy_speed=0.3).wait_for_completed()
        elif p_value < 0:
            print(f"  → เคลื่อนที่ถอยหลัง {abs(p_value):.3f}m")
            ep_chassis.move(
                x=-abs(p_value), y=0, z=0, xy_speed=0.3
            ).wait_for_completed()

        time.sleep(0.5)  # รอให้ระบบเสถียร

        # อัพเดทค่าสำหรับลูปถัดไป
        current_x, _, _ = position
        final_distance = abs(current_x - start_x)
        final_error = TARGET_DISTANCE - final_distance

        print(f"  → ระยะใหม่: {final_distance:.3f}m, Error ใหม่: {final_error:.3f}m")

        # ป้องกันลูปไม่สิ้นสุด
        if iteration > 20:
            print("หยุดลูป P Controller (เกิน 20 รอบ)")
            break

    print(
        f"P Controller เสร็จสิ้น! ระยะสุดท้าย: {final_distance:.3f}m, Error: {final_error:.3f}m"
    )

    # บันทึกลง CSV
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, "status.csv")
    p_csv_path = os.path.join(current_dir, "p_controller.csv")

    # บันทึกข้อมูลการเคลื่อนที่หลัก
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_traveled", "error"])
        for row in position_log:
            writer.writerow(row)

    # บันทึกข้อมูล P Controller
    with open(p_csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_measured", "error", "p_value"])
        for row in p_controller_log:
            writer.writerow(row)

    print(f"บันทึกข้อมูลเสร็จสิ้น:")
    print(f"  - การเคลื่อนที่หลัก: {len(position_log)} บรรทัด ({csv_path})")
    print(f"  - P Controller: {len(p_controller_log)} บรรทัด ({p_csv_path})")

    # ยกเลิก subscription ก่อนปิด
    print("กำลังปิดการเชื่อมต่อ...")
    ep_chassis.unsub_position()
    ep_robot.close()
    print("เสร็จสิ้น!")
