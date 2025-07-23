# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import robomaster
from robomaster import robot
import time
import csv


def sub_attitude_info_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))


def sub_tof_handler(tof_info):
    distance = tof_info[0]  # ระยะทางจาก ToF Sensor
    print("ToF distance: {0} cm".format(distance))
    return distance


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_tof = ep_robot.sensor

    # บันทึกข้อมูลระยะจาก ToF Sensor
    sampling_rate = 20  # Hz
    duration = 30  # seconds
    interval = 1 / sampling_rate
    data = []

    print("Starting ToF data collection for {0} seconds...".format(duration))
    start_time = time.time()

    # สมัครรับข้อมูลระยะทางจาก ToF Sensor
    def tof_callback(tof_info):
        distance_mm = tof_info[0]  # ระยะทางจาก ToF Sensor (หน่วย: มม.)
        distance_cm = (distance_mm - 80) / 10.0  # ลบ 79 และแปลงเป็นเซนติเมตร
        timestamp = time.time() - start_time
        data.append((timestamp, distance_cm))
        print("Time: {0:.2f}s, Distance: {1:.2f} cm".format(timestamp, distance_cm))

    ep_tof.sub_distance(freq=sampling_rate, callback=tof_callback)

    # รอให้การเก็บข้อมูลเสร็จสิ้น
    time.sleep(duration)

    # ยกเลิกการสมัครรับข้อมูล
    ep_tof.unsub_distance()

    # บันทึกข้อมูลลงไฟล์ CSV (เฉพาะเวลาและระยะทาง)
    with open("tof_paper.csv", "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Time (s)", "Distance (cm)"])
        csv_writer.writerows(data)

    print("Data collection complete. Saved to 'tof_data.csv'.")

    ep_robot.close()
