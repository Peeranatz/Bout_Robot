# -*- coding:utf-8 -*-
import time
import math
import csv
import cv2
import numpy as np
from robomaster import robot

# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def update(self, measurement):
        current_time = time.time()
        error = self.setpoint - measurement
        dt = 0.0
        if self._last_time is not None:
            dt = current_time - self._last_time
        self._last_time = current_time

        self._integral += error * dt if dt > 0 else 0
        derivative = (error - self._prev_error) / dt if dt > 0 else 0
        self._prev_error = error

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        return output


class ChickenTracker:
    def __init__(self, robot, gimbal, detector):
        self.robot = robot
        self.gimbal = gimbal
        self.detector = detector
        self.pid_yaw = PID(Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0.5)   # Center of screen X
        self.pid_pitch = PID(Kp=0.1, Ki=0.01, Kd=0.05, setpoint=0.5) # Center of screen Y

    def run(self, duration=30):
        start_time = time.time()

        # สร้างไฟล์ CSV สำหรับเก็บข้อมูล
        log_file = open('chicken_tracking.csv', 'w', newline='')
        csv_writer = csv.writer(log_file)
        csv_writer.writerow([
            'Timestamp',
            'err_total', 'err_x', 'err_y',
            'accumulate_err_x', 'accumulate_err_y', 'accumulate_err_total',
            'controller_output_x', 'controller_output_y',
            'gimbal_yaw_angle', 'gimbal_pitch_angle'
        ])

        try:
            while time.time() - start_time < duration:
                frame = self.detector.get_frame()
                detection = self.detector.detect_chicken(frame)

                timestamp = time.time() - start_time

                if detection is not None:
                    # คำนวณ error
                    error_yaw = self.pid_yaw.setpoint - detection['x']
                    error_pitch = self.pid_pitch.setpoint - detection['y']
                    error_total = (error_yaw**2 + error_pitch**2) ** 0.5

                    # PID update
                    yaw_speed = -self.pid_yaw.update(detection['x'])
                    pitch_speed = self.pid_pitch.update(detection['y'])

                    # จำกัดความเร็ว
                    max_speed = 50
                    yaw_speed = max(-max_speed, min(max_speed, yaw_speed))
                    pitch_speed = max(-max_speed, min(max_speed, pitch_speed))

                    # ส่งคำสั่งควบคุม gimbal
                    self.gimbal.drive_speed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)

                    # อ่านมุม gimbal
                    yaw_angle, pitch_angle = self.gimbal.get_angle()

                    # Accumulate error
                    acc_x = self.pid_yaw._integral
                    acc_y = self.pid_pitch._integral
                    acc_total = acc_x + acc_y

                    # บันทึกข้อมูล PID ลง CSV
                    csv_writer.writerow([
                        timestamp,
                        error_total, error_yaw, error_pitch,
                        acc_x, acc_y, acc_total,
                        yaw_speed, pitch_speed,
                        yaw_angle, pitch_angle
                    ])

                time.sleep(0.05)

        finally:
            log_file.close()
            print("การบันทึกข้อมูลเสร็จสิ้น -> chicken_tracking.csv")


# Mock Detector สำหรับทดสอบ
class MockDetector:
    def get_frame(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def detect_chicken(self, frame):
        # จำลอง detection: วัตถุเคลื่อนที่แบบ sinusoidal
        t = time.time()
        x = 0.5 + 0.1 * math.sin(t)
        y = 0.5 + 0.1 * math.cos(t)
        return {'x': x, 'y': y}


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    detector = MockDetector()
    tracker = ChickenTracker(ep_robot, ep_gimbal, detector)

    tracker.run(duration=20)

    ep_robot.close()
