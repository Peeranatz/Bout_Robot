# -*-coding:utf-8-*-
import csv
import robomaster
from robomaster import robot

yaw_data = []  # ลิสต์เก็บข้อมูล [round_number, yaw]

def record_yaw(ep_gimbal, round_num):
    angle_info = ep_gimbal.get_angle()
    _, yaw_angle, _, _ = angle_info
    print(f"Round {round_num}: Yaw angle = {yaw_angle:.2f}")
    yaw_data.append([round_num, yaw_angle])

if _name_ == '_main_':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal

    pitch_val = 0
    yaw_val = 15
    yaw_speed_val = 50
    repeat = 6
    round_counter = 1

    # หมุนซ้าย repeat ครั้ง
    for _ in range(repeat):
        ep_gimbal.move(pitch=0, yaw=yaw_val, yaw_speed=yaw_speed_val).wait_for_completed()
        record_yaw(ep_gimbal, round_counter)
        round_counter += 1

    # หมุนขวา repeat ครั้ง
    for _ in range(repeat):
        ep_gimbal.move(pitch=0, yaw=-yaw_val, yaw_speed=yaw_speed_val).wait_for_completed()
        record_yaw(ep_gimbal, round_counter)
        round_counter += 1

    # หมุนขวาซ้ำ repeat ครั้ง
    for _ in range(repeat):
        ep_gimbal.move(pitch=0, yaw=-yaw_val, yaw_speed=yaw_speed_val).wait_for_completed()
        record_yaw(ep_gimbal, round_counter)
        round_counter += 1

    # หมุนซ้ายซ้ำ repeat ครั้ง
    for _ in range(repeat):
        ep_gimbal.move(pitch=0, yaw=yaw_val, yaw_speed=yaw_speed_val).wait_for_completed()
        record_yaw(ep_gimbal, round_counter)
        round_counter += 1

    # บันทึกค่า Yaw และรอบ ลงไฟล์ CSV
    with open("yaw_round_log.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Round", "Yaw Angle"])
        for row in yaw_data:
            writer.writerow(row)
    print("บันทึกค่า Yaw และรอบ ลงไฟล์ yaw_round_log.csv แล้ว")

    ep_robot.close()