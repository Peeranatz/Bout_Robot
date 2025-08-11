import math
import time
import csv
import os
from robomaster import robot

# ==================== Global State ====================
position = [0, 0, 0]
yaw_angle = 0

# สำหรับเดินตรง
previous_error = None
previous_time = None
integral_error = 0.0
pid_control_active = False
target_distance = 0.0

# สำหรับหมุน
target_angle = None
previous_angle_error = None
integral_angle_error = 0.0
previous_angle_time = None
pid_turn_active = False

# ==================== Callback ====================
def sub_position_handler(position_info):
    global position
    position = position_info
    if pid_control_active:
        pid_move_control()

def sub_attitude_handler(attitude_info):
    global yaw_angle
    yaw_angle = attitude_info[0]
    if pid_turn_active:
        pid_turn_control()

# ==================== PID เดินตรง ====================
def pid_move_control():
    global previous_error, previous_time, integral_error, pid_control_active

    x, y, _ = position
    current_time = time.time()
    dt = current_time - previous_time if previous_time else 0.001

    # คำนวณระยะจากจุดเริ่ม
    current_distance = math.sqrt((x - start_x)**2 + (y - start_y)**2)
    error = target_distance - current_distance

    # PID
    p_val = move_kp * error
    if dt > 0:
        integral_error += error * dt
        integral_error = max(min(integral_error, move_max_integral), -move_max_integral)
    i_val = move_ki * integral_error
    d_val = 0
    if previous_error is not None and dt > 0:
        d_val = move_kd * ((error - previous_error) / dt)

    pid_speed = p_val + i_val + d_val
    pid_speed = max(min(pid_speed, move_max_speed), -move_max_speed)

    # คำสั่งเคลื่อนที่
    if abs(error) > move_tolerance:
        ep_chassis.drive_speed(x=pid_speed, y=0, z=0)
    else:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        pid_control_active = False
        print(f"🎯 เดินครบ {target_distance} m แล้ว")

    previous_error = error
    previous_time = current_time

def move_forward_pid(distance, kp=1.2, ki=1.0, kd=0.2, max_speed=10, tolerance=0.02, max_integral=5.0):
    global start_x, start_y, previous_error, previous_time, integral_error, pid_control_active
    global target_distance, move_kp, move_ki, move_kd, move_max_speed, move_tolerance, move_max_integral

    # ตั้งค่าพารามิเตอร์
    move_kp, move_ki, move_kd = kp, ki, kd
    move_max_speed = max_speed
    move_tolerance = tolerance
    move_max_integral = max_integral
    target_distance = distance

    # เริ่มต้น PID
    start_x, start_y, _ = position
    previous_error = distance
    previous_time = time.time()
    integral_error = 0.0
    pid_control_active = True

    timeout = 30
    start_time_move = time.time()
    while pid_control_active and (time.time() - start_time_move < timeout):
        time.sleep(0.02)

# ==================== PID หมุน ====================
def pid_turn_control():
    global previous_angle_error, integral_angle_error, previous_angle_time, pid_turn_active

    current_time = time.time()
    dt = current_time - previous_angle_time if previous_angle_time else 0.001

    # คำนวณ error แบบ wrap-around
    angle_error = target_angle - yaw_angle
    while angle_error > 180:
        angle_error -= 360
    while angle_error < -180:
        angle_error += 360

    p_val = turn_kp * angle_error
    if dt > 0:
        integral_angle_error += angle_error * dt
    i_val = turn_ki * integral_angle_error
    d_val = 0
    if previous_angle_error is not None and dt > 0:
        d_val = turn_kd * ((angle_error - previous_angle_error) / dt)

    pid_speed = (p_val + i_val + d_val) * 0.5
    pid_speed = max(min(pid_speed, turn_max_speed), -turn_max_speed)

    if abs(angle_error) <= turn_tolerance_angle:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        pid_turn_active = False
        print(f"🎯 หมุนครบ {target_angle:.1f}° แล้ว")
    else:
        ep_chassis.drive_speed(x=0, y=0, z=pid_speed)

    previous_angle_error = angle_error
    previous_angle_time = current_time

def turn_pid(angle, kp=0.5, ki=0.2, kd=0.1, max_speed=30, tolerance_angle=1.0):
    global target_angle, pid_turn_active, previous_angle_error
    global integral_angle_error, previous_angle_time
    global turn_kp, turn_ki, turn_kd, turn_max_speed, turn_tolerance_angle

    turn_kp, turn_ki, turn_kd = kp, ki, kd
    turn_max_speed = max_speed
    turn_tolerance_angle = tolerance_angle

    # คำนวณมุมเป้าหมาย
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
        time.sleep(0.005)

# ==================== ตัวอย่างการใช้งาน ====================
if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=50, callback=sub_attitude_handler)
    time.sleep(2)

    # ตัวอย่างการเดินสำรวจ: เดิน 0.6 m แล้วหมุน 90° ทำ 4 รอบ
    for _ in range(4):
        move_forward_pid(0.6)
        time.sleep(0.5)
        turn_pid(90)
        time.sleep(0.5)

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_robot.close()
    print("✅ เสร็จสิ้น!")
