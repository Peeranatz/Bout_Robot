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

import time
import robomaster
from robomaster import robot

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal

    pitch_val = 0      # กำหนด pitch ให้คงที่ (ตรงที่สุด)
    yaw_left = -90     # องศาซ้ายสุด
    yaw_right = 90     # องศาขวาสุด
    step = 1         # หมุนทีละ 1 องศา

    # เริ่มต้นที่ตรงกลาง
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()
    time.sleep(0.5)

    # หมุนไปทางซ้ายทีละ step จนถึงซ้ายสุด
    for yaw in range(0, yaw_left - 1, -step):
        ep_gimbal.moveto(pitch=pitch_val, yaw=yaw).wait_for_completed()
        

    # หมุนกลับมาตรงกลาง
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()
    

    # หมุนไปทางขวาทีละ step จนถึงขวาสุด
    for yaw in range(0, yaw_right + 1, step):
        ep_gimbal.moveto(pitch=pitch_val, yaw=yaw).wait_for_completed()
        

    # หมุนกลับมาตรงกลาง
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()

    ep_robot.close()