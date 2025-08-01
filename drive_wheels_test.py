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
import time
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    a=2
    # กำหนดความเร็วของล้อแมคคานัม (หน่วย: RPM - รอบต่อนาที)
    speed = 23.87
    # กำหนดเวลาหยุดระหว่างการเคลื่อนไหว (หน่วย: วินาที)
    # slp = 4.55
    slp = 15.87
    while a > 0:
    # เดินหน้า 1 วินาที
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(slp)

        # หมุน 180 องศา (ล้อซ้ายหมุนไปข้างหน้า, ล้อขวาหมุนถอยหลัง)
        print("หมุน 180 องศา...")
        turn_speed = 20  # ความเร็วในการหมุน
        turn_time = 6.0  # เวลาในการหมุน (ปรับตามการทดสอบ)
        
        # w1=หน้าซ้าย, w2=หน้าขวา, w3=หลังขวา, w4=หลังซ้าย
        ep_chassis.drive_wheels(w1=turn_speed, w2=-turn_speed, w3=-turn_speed, w4=turn_speed)
        time.sleep(turn_time)
        a=a-1
    if a == 0:
        # หยุด
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)



    ep_robot.close()
