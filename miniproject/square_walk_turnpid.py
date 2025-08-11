import math
import time
import csv
import os
from robomaster import robot
import matplotlib.pyplot as plt

# -------------------- ค่าพารามิเตอร์เดินตรง --------------------
TARGET_DISTANCE = 0.6
TOLERANCE = 0.02
kp = 1.2
ki = 1.0
kd = 0.2
max_speed = 10
max_integral = 5.0

# -------------------- ค่าพารามิเตอร์หมุน (ปรับใหม่) --------------------
kp_turn = 0.5
ki_turn = 0.2
kd_turn = 0.1
max_turn_speed = 30
TOLERANCE_ANGLE = 1.0

# -------------------- ตัวแปรสถานะ --------------------
position = [0, 0, 0]
yaw_angle = 0
start_time = None
start_x = None
start_y = None
previous_error = None
previous_time = None
integral_error = 0.0
pid_control_active = False

target_angle = None
previous_angle_error = None
integral_angle_error = 0.0
previous_angle_time = None
pid_turn_active = False

xy_paths = [[] for _ in range(4)]
error_log_per_round = [[] for _ in range(4)]
current_round = 0

log_folder = "robot_logs_PID_full"
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# -------------------- Callback ตำแหน่ง --------------------
def sub_position_handler(position_info):
    global position, start_time, start_x, start_y
    global previous_error, previous_time, integral_error
    global xy_paths, error_log_per_round, current_round, pid_control_active

    position = position_info
    x = position[0]
    y = position[1]

    if pid_control_active:
        xy_paths[current_round].append((x, y))

    if start_time and start_x is not None and start_y is not None and pid_control_active:
        current_time = time.time()
        elapsed = current_time - start_time
        current_distance = math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)
        current_error = TARGET_DISTANCE - current_distance

        # PID เดินตรง
        p_value = kp * current_error
        dt = current_time - previous_time if previous_time else 0
        if dt > 0:
            integral_error += current_error * dt
            integral_error = max(min(integral_error, max_integral), -max_integral)
        i_value = ki * integral_error
        d_value = 0
        if previous_error is not None and dt > 0:
            derror = current_error - previous_error
            d_value = kd * (derror / dt)

        pid_speed = p_value + i_value + d_value
        pid_speed = max(min(pid_speed, max_speed), -max_speed)

        error_log_per_round[current_round].append([elapsed, x, y, current_error, pid_speed])

        print(f"Time: {elapsed:.2f}s | Dist: {current_distance:.3f} | Err: {current_error:.3f} | "
            f"P: {p_value:.2f}, I: {i_value:.2f}, D: {d_value:.2f} → Speed: {pid_speed:.2f}")

        if abs(current_error) > TOLERANCE:
            ep_chassis.drive_speed(x=pid_speed, y=0, z=0)
        else:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            pid_control_active = False
            print("🎯 ถึงเป้าหมายแล้ว")

        previous_error = current_error
        previous_time = current_time

# -------------------- Callback มุม yaw --------------------
def sub_attitude_handler(attitude_info):
    global yaw_angle
    yaw_angle = attitude_info[0]
    if pid_turn_active:
        pid_turn_control()

# -------------------- PID หมุน (แก้ไขเงื่อนไข ลด overshoot) --------------------
def pid_turn_control():
    global previous_angle_error, integral_angle_error, previous_angle_time, pid_turn_active

    current_time = time.time()
    dt = current_time - previous_angle_time if previous_angle_time else 0.001  # ป้องกันหารด้วย 0

    # คำนวณ error (จัดการ wrap-around)
    angle_error = target_angle - yaw_angle
    while angle_error > 180:
        angle_error -= 360
    while angle_error < -180:
        angle_error += 360

    p_val = kp_turn * angle_error
    if dt > 0:
        integral_angle_error += angle_error * dt
    i_val = ki_turn * integral_angle_error
    d_val = 0
    if previous_angle_error is not None and dt > 0:
        d_val = kd_turn * ((angle_error - previous_angle_error) / dt)

    pid_speed = p_val + i_val + d_val

    # ลด speed ลงครึ่งเพื่อนุ่มขึ้น ลด overshoot
    pid_speed = pid_speed * 0.5

    # จำกัดความเร็วหมุน
    pid_speed = max(min(pid_speed, max_turn_speed), -max_turn_speed)

    if abs(angle_error) <= TOLERANCE_ANGLE:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        pid_turn_active = False
        print(f"🎯 หมุนครบ {target_angle:.1f}° แล้ว")
    else:
        ep_chassis.drive_speed(x=0, y=0, z=pid_speed)

    previous_angle_error = angle_error
    previous_angle_time = current_time

def rotate_pid(angle):
    global target_angle, pid_turn_active, previous_angle_error
    global integral_angle_error, previous_angle_time

    target_angle = yaw_angle + angle
    while target_angle > 180:
        target_angle -= 360
    while target_angle < -180:
        target_angle += 360

    previous_angle_error = None
    integral_angle_error = 0.0
    previous_angle_time = time.time()

    pid_turn_active = True
    timeout = 10
    start_time_turn = time.time()
    while pid_turn_active and (time.time() - start_time_turn < timeout):
        time.sleep(0.005)  # ลด sleep ให้ PID ตอบสนองถี่ขึ้น

# -------------------- บันทึก log --------------------
def save_path_to_csv_per_round(round_num):
    filename = os.path.join(log_folder, f"robot_path_log_round_{round_num + 1}.csv")
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time (s)", "X (m)", "Y (m)", "Error (m)", "Speed"])
        writer.writerows(error_log_per_round[round_num])
    print(f"💾 บันทึก log รอบที่ {round_num + 1} → {filename}")

# -------------------- Main --------------------
if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=50, callback=sub_attitude_handler)
    time.sleep(2)

    for round_num in range(4):
        current_round = round_num
        print(f"\n🔁 เริ่มรอบที่ {current_round + 1}/4")

        for i in range(4):
            print(f"\n===== ด้านที่ {i + 1}/4 =====")
            while position == [0, 0, 0]:
                time.sleep(0.01)

            # PID เดินตรง
            start_x, start_y, _ = position
            start_time = time.time()
            previous_error = TARGET_DISTANCE
            previous_time = time.time()
            integral_error = 0.0
            pid_control_active = True

            timeout = 30
            start_pid_time = time.time()
            while pid_control_active and (time.time() - start_pid_time < timeout):
                time.sleep(0.05)

            ep_chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(1)

            if i < 4:
                print("↪️ หมุน 90° ด้วย PID...")
                rotate_pid(-87.5)
                time.sleep(1)

        save_path_to_csv_per_round(current_round)

    # วาดเส้นทาง
    colors = ['blue', 'green', 'red', 'orange']
    plt.figure(figsize=(6, 6))
    for i, path in enumerate(xy_paths):
        if path:
            xs, ys = zip(*path)
            plt.plot(xs, ys, marker='o', label=f'round {i + 1}', color=colors[i])
    plt.title("Robot Path (4 Rounds)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_robot.close()
    print("✅ เสร็จสิ้น!")
