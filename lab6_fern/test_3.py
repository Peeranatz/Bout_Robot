# -*-coding:utf-8-*-
import cv2
import numpy as np
import time
import os
from robomaster import robot, camera
import csv
import threading

class ChickenTrackerPID:
    def __init__(self):
        # PID Parameters
        self.pid_yaw = PID(110, 30, 8, setpoint=0.5)
        self.pid_pitch = PID(110, 30, 8, setpoint=0.5)
        
        # สร้างโฟลเดอร์สำหรับเก็บภาพ
        self.save_dir = "detected_photos"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # ตัวแปรสำหรับเก็บค่า accumulated errors
        self.accumulate_error_x = 0
        self.accumulate_error_y = 0
        self.accumulate_error_total = 0
        self.last_gimbal_yaw = 0
        self.last_gimbal_pitch = 0
            
        # ตัวแปรสำหรับ statistics
        self.detection_count = 0
        self.total_detections = 0
        self.start_time = time.time()

    def detect_yellow_object(self, image):
        # แปลงภาพเป็น HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # กำหนดช่วงสีเหลือง 3 ช่วง
        lower_yellow1 = np.array([20, 100, 100]); upper_yellow1 = np.array([30, 255, 255])
        lower_yellow2 = np.array([15, 50, 120]); upper_yellow2 = np.array([25, 255, 255])
        lower_yellow3 = np.array([25, 50, 50]); upper_yellow3 = np.array([35, 255, 200])
        
        # สร้าง mask สำหรับแต่ละช่วงสี
        mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
        mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
        mask3 = cv2.inRange(hsv, lower_yellow3, upper_yellow3)
        
        # รวม mask ทั้งสาม
        yellow_mask = cv2.bitwise_or(mask1, mask2)
        yellow_mask = cv2.bitwise_or(yellow_mask, mask3)
        
        # กำหนด ROI ตรงกลาง
        H, W = image.shape[:2]
        cx, cy = W // 2, H // 2
        roi_frac_w, roi_frac_h = 0.25, 0.30
        
        rw = int(W * roi_frac_w / 2)
        rh = int(H * roi_frac_h / 2)
        
        x1 = max(0, cx - rw)
        y1 = max(0, cy - rh)
        x2 = min(W, cx + rw)
        y2 = min(H, cy + rh)
        
        # สร้าง mask สำหรับ ROI
        roi_mask = np.zeros_like(yellow_mask)
        roi_mask[y1:y2, x1:x2] = 255
        
        # รวม mask สีเหลืองกับ ROI
        masked = cv2.bitwise_and(yellow_mask, roi_mask)
        
        # ทำ Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        masked = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel)
        masked = cv2.morphologyEx(masked, cv2.MORPH_OPEN, kernel)
        
        # หา contours
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # เตรียมข้อมูลสำหรับส่งกลับ
        detection_info = {
            'detected': False,
            'x': 0, 'y': 0,
            'width': 0, 'height': 0,
            'roi': (x1, y1, x2, y2)
        }
        
        if contours:
            # หา contour ที่ใหญ่ที่สุด
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            
            if w * h > 200:  # กรองพื้นที่ขนาดเล็ก
                detection_info['detected'] = True
                # คำนวณตำแหน่งเป็น normalized coordinates (0-1)
                detection_info['x'] = (x + w/2) / W
                detection_info['y'] = (y + h/2) / H
                detection_info['width'] = w
                detection_info['height'] = h
                
        return detection_info

    def run(self):
        # เชื่อมต่อกับ RoboMaster
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        
        ep_camera = ep_robot.camera
        ep_gimbal = ep_robot.gimbal
        
        # เริ่ม video stream
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
        
        # สร้างไฟล์ CSV สำหรับบันทึกข้อมูล PID ในโฟลเดอร์ lab6_fern
        log_filepath = os.path.join(os.path.dirname(__file__), 'chicken_tracking.csv')
        log_file = open(log_filepath, 'w', newline='')
        csv_writer = csv.writer(log_file)
        csv_writer.writerow([
            'Timestamp',
            'Error_Yaw', 'Error_Pitch', 'Error_Total',
            'P_Yaw', 'I_Yaw', 'D_Yaw',
            'Speed_Yaw',
            'P_Pitch', 'I_Pitch', 'D_Pitch',
            'Speed_Pitch',
            'Accumulate_Error_X', 'Accumulate_Error_Y', 'Accumulate_Error_Total',
            'Controller_Output_X', 'Controller_Output_Y',
            'Gimbal_Pitch_Angle', 'Gimbal_Yaw_Angle'
        ])
        
        print("[INFO] เริ่มการทำงาน - กด 's' เพื่อบันทึกภาพ, 'q' เพื่อออก")
        
        try:
            while True:
                # อ่านภาพจากกล้อง
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)
                if frame is None:
                    continue
                
                # ตรวจจับวัตถุสีเหลือง
                detection = self.detect_yellow_object(frame)
                
                # สร้างภาพสำหรับแสดงผล
                display = frame.copy()
                
                # วาด ROI
                x1, y1, x2, y2 = detection['roi']
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                if detection['detected']:
                    # คำนวณตำแหน่งจริงบนภาพ
                    h, w = frame.shape[:2]
                    center_x = int(detection['x'] * w)
                    center_y = int(detection['y'] * h)
                    box_w = detection['width']
                    box_h = detection['height']
                    
                    # วาด bounding box
                    cv2.rectangle(display, 
                                (center_x - box_w//2, center_y - box_h//2),
                                (center_x + box_w//2, center_y + box_h//2),
                                (0, 255, 0), 2)
                    
                    # ใช้ PID ควบคุม gimbal
                    timestamp = time.time()
                    
                    # คำนวณ error
                    error_yaw = self.pid_yaw.setpoint - detection['x']
                    error_pitch = self.pid_pitch.setpoint - detection['y']
                    error_total = np.sqrt(error_yaw**2 + error_pitch**2)
                    
                    # คำนวณ accumulated errors
                    self.accumulate_error_x += error_yaw
                    self.accumulate_error_y += error_pitch
                    self.accumulate_error_total += error_total
                    
                    # PID update
                    yaw_speed = -self.pid_yaw.update(detection['x'])
                    pitch_speed = self.pid_pitch.update(detection['y'])
                    
                    # จำกัดความเร็ว
                    max_speed = 50
                    yaw_speed = max(-max_speed, min(max_speed, yaw_speed))
                    pitch_speed = max(-max_speed, min(max_speed, pitch_speed))
                    
                    # ส่งคำสั่งควบคุม gimbal
                    ep_gimbal.drive_speed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)
                    
                    # อัพเดตมุม gimbal (ประมาณการณ์จากความเร็ว)
                    dt = 0.1  # ประมาณเวลาระหว่างเฟรม
                    self.last_gimbal_yaw += yaw_speed * dt
                    self.last_gimbal_pitch += pitch_speed * dt
                    
                    # คำนวณ controller outputs
                    controller_output_x = self.pid_yaw.last_p + self.pid_yaw.last_i + self.pid_yaw.last_d
                    controller_output_y = self.pid_pitch.last_p + self.pid_pitch.last_i + self.pid_pitch.last_d
                    
                    # บันทึกข้อมูล PID
                    csv_writer.writerow([
                        timestamp,
                        error_yaw, error_pitch, error_total,
                        self.pid_yaw.last_p, self.pid_yaw.last_i, self.pid_yaw.last_d,
                        yaw_speed,
                        self.pid_pitch.last_p, self.pid_pitch.last_i, self.pid_pitch.last_d,
                        pitch_speed,
                        self.accumulate_error_x, self.accumulate_error_y, self.accumulate_error_total,
                        controller_output_x, controller_output_y,
                        self.last_gimbal_pitch, self.last_gimbal_yaw
                    ])
                    
                    # แสดงข้อความบนภาพ
                    cv2.putText(display, f"Size: {box_w}x{box_h}px", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    # ถ้าไม่พบวัตถุ หยุด gimbal
                    ep_gimbal.drive_speed(yaw_speed=0, pitch_speed=0)
                    cv2.putText(display, "Searching...", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # แสดงภาพ
                cv2.imshow("Chicken Tracker", display)
                
                # รับค่าจากคีย์บอร์ด
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    # บันทึกภาพ
                    timestamp = int(time.time())
                    filename = f"detection_{timestamp}.jpg"
                    filepath = os.path.join(self.save_dir, filename)
                    cv2.imwrite(filepath, display)
                    print(f"[INFO] บันทึกภาพ {filepath}")
                elif key == ord('q'):
                    break
                
        except KeyboardInterrupt:
            print("\n[INFO] หยุดโปรแกรมโดยผู้ใช้")
        finally:
            # ปิดการเชื่อมต่อ
            ep_gimbal.drive_speed(yaw_speed=0, pitch_speed=0)
            ep_camera.stop_video_stream()
            ep_robot.close()
            cv2.destroyAllWindows()
            log_file.close()
            print("[INFO] ปิดการทำงานเรียบร้อย")

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None
        
        # เก็บค่า PID components ล่าสุด
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
    
    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
    
    def update(self, current_value):
        error = self.setpoint - current_value
        now = time.time()
        
        if self._last_time is None:
            self._last_time = now
            return 0
            
        dt = now - self._last_time
        self._last_time = now
        
        if dt == 0:
            return 0
            
        # คำนวณ PID components
        self.last_p = self.Kp * error
        self._integral += error * dt
        self.last_i = self.Ki * self._integral
        
        derivative = (error - self._prev_error) / dt
        self.last_d = self.Kd * derivative
        
        self._prev_error = error
        
        # รวมผลลัพธ์
        output = self.last_p + self.last_i + self.last_d
        return output

if __name__ == '__main__':
    tracker = ChickenTrackerPID()
    tracker.run()