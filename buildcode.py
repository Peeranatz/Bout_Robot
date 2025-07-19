import time
import csv
from robomaster import robot

# ระยะทางเป้าหมาย (เมตร)
TARGET_DISTANCE = 2.0

# ตัวแปรเก็บตำแหน่งและข้อมูล
position = [0, 0, 0]
position_log = []
start_time = None
start_x = None

def sub_position_handler(position_info):
    """
    Callback สำหรับอัปเดตตำแหน่งหุ่นยนต์
    """
    global position, position_log, start_time, start_x
    position = position_info
    x = position[0]
    now = time.time()
    if start_time is not None and start_x is not None:
        elapsed = now - start_time
        distance_traveled = x - start_x  # ระยะที่เดินได้
        error = TARGET_DISTANCE - distance_traveled  # Error = ระยะทางเป้าหมาย - ระยะที่เดินได้
        position_log.append((elapsed, distance_traveled, error))
        print(f"Time: {elapsed:.2f}s, Distance: {distance_traveled:.3f}m, Error: {error:.3f}m")

if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    # Subscribe ตำแหน่ง
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    time.sleep(0.5)  # รอให้ตำแหน่งอัปเดต

    # บันทึกตำแหน่งเริ่มต้นและเริ่มจับเวลา
    start_x, start_y, _ = position
    start_time = time.time()
    print(f"Starting position: X={start_x:.3f}, Y={start_y:.3f}")

    # เคลื่อนที่ 2 เมตร
    print("Starting movement...")
    ep_chassis.move(x=TARGET_DISTANCE, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(0.5)  # รอให้ข้อมูลสุดท้ายมา

    # บันทึกตำแหน่งสุดท้าย
    end_x, end_y, _ = position
    final_distance = end_x - start_x
    final_error = TARGET_DISTANCE - final_distance

    print(f"Final distance traveled: {final_distance:.3f} meters")
    print(f"Final error: {final_error:.3f} meters")

    # บันทึกลง CSV
    with open("distance_log.csv", "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_elapsed_sec", "distance_traveled", "error"])
        for row in position_log:
            writer.writerow(row)

    print(f"Data saved to CSV file with {len(position_log)} rows.")

    # ยกเลิก subscription และปิดการเชื่อมต่อ
    ep_chassis.unsub_position()
    ep_robot.close()
    print("Finished!")

