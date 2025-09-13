# Real-time Distance Detection for RoboMaster
# แสดงผลที่ terminal โดยไม่ต้องรู้ระยะทางล่วงหน้า

import robomaster
from robomaster import robot
import cv2
import numpy as np
import time
import os
import sys

class TerminalDistanceDetector:
    def __init__(self):
        # ค่าคงที่จากการคำนวณ
        self.c_x = 7.5   # cm - ความกว้างจริงของสี่เหลี่ยม
        self.c_y = 7.7   # cm - ความสูงจริงของสี่เหลี่ยม
        self.b_x = 336.00  # focal length equivalent แกน x
        self.b_y = 341.56  # focal length equivalent แกน y
        
        # ตัวแปรสำหรับ statistics
        self.detection_count = 0
        self.total_detections = 0
        self.start_time = time.time()
        
    def clear_screen(self):
        """ล้างหน้าจอ terminal"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def detect_square_in_frame(self, frame):
        """ตรวจจับสี่เหลี่ยมสีดำในเฟรม"""
        try:
            # กำหนด ROI ตรงกลาง - ลดพื้นที่ให้เล็กลงเพื่อโฟกัสตรงกลาง
            roi_frac_w = 0.25  # ลดจาก 0.40 เป็น 0.25 (25% ของความกว้าง)
            roi_frac_h = 0.30  # ลดจาก 0.45 เป็น 0.30 (30% ของความสูง)
            
            H, W = frame.shape[:2]
            cx, cy = W // 2, H // 2
            
            rw = int(W * roi_frac_w / 2)
            rh = int(H * roi_frac_h / 2)
            
            x1 = max(0, cx - rw)
            y1 = max(0, cy - rh)
            x2 = min(W, cx + rw)
            y2 = min(H, cy + rh)
            
            roi = frame[y1:y2, x1:x2].copy()
            
            # ประมวลผลภาพ
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Otsu threshold
            _, mask = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            # Morphology
            kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
            kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
            mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel_open, iterations=1)
            
            # หา contours
            contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # เลือก contour ที่ดีที่สุด
            best = None
            best_score = -1
            
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                area = w * h
                
                # เพิ่มเงื่อนไขกรองที่เข้มงวดขึ้น
                if area < 300:  # เพิ่มจาก 200 เป็น 300
                    continue
                
                # คำนวณ squareness - ต้องเป็นรูปสี่เหลี่ยมจริง ๆ
                sq = min(w, h) / max(w, h)
                if sq < 0.7:  # ถ้าไม่ใกล้เคียงสี่เหลี่ยม ก็ข้าม
                    continue
                
                # คำนวณระยะห่างจากจุดกลาง ROI
                cx_cnt = x + w / 2
                cy_cnt = y + h / 2
                dx = (cx_cnt - (roi.shape[1] / 2)) / roi.shape[1]
                dy = (cy_cnt - (roi.shape[0] / 2)) / roi.shape[0]
                center_penalty = np.hypot(dx, dy)
                
                # ต้องอยู่ใกล้กลาง ROI มาก ๆ
                if center_penalty > 0.3:  # ถ้าไกลจากกลางเกินไป ก็ข้าม
                    continue
                
                # คะแนน - เน้นความเป็นสี่เหลี่ยมและตำแหน่งกลางมากขึ้น
                score = (sq * 5.0) + (np.log(area + 1) * 0.1) - (center_penalty * 10.0)
                
                if score > best_score:
                    best_score = score
                    best = (x, y, w, h)
            
            if best is not None:
                bx, by, bw, bh = best
                # แปลงพิกัดกลับไปยังภาพต้นฉบับ - นี่คือสิ่งที่หายไป!
                final_x = x1 + bx
                final_y = y1 + by
                return (bw, bh, best_score, final_x, final_y, x1, y1, x2, y2)
            
        except Exception as e:
            return None
        
        return None
    
    def calculate_distance(self, detection_result):
        """คำนวณระยะทางจากขนาด detection"""
        if detection_result is None:
            return None
        
        w, h, score, final_x, final_y, x1, y1, x2, y2 = detection_result
        
        # สมการคำนวณระยะทาง: d = (b × c) / a
        distance_x = (self.b_x * self.c_x) / w
        distance_y = (self.b_y * self.c_y) / h
        
        # ใช้ค่าเฉลี่ยของทั้งสองแกน
        avg_distance = (distance_x + distance_y) / 2
        
        return {
            'average': avg_distance,
            'x_axis': distance_x,
            'y_axis': distance_y,
            'width_px': w,
            'height_px': h,
            'confidence': score
        }
    
    def print_header(self):
        """แสดงหัวข้อของโปรแกม"""
        print("=" * 80)
        print("ROBOMASTER REAL-TIME DISTANCE DETECTOR")
        print("=" * 80)
        print("สมการที่ใช้: ระยะทาง (cm) = (b × c) / a")
        print(f"ค่าคงที่: b_x={self.b_x}, b_y={self.b_y}, c_x={self.c_x}cm, c_y={self.c_y}cm")
        print("วัตถุเป้าหมาย: สี่เหลี่ยมสีดำ ขนาด 7.5×7.7 cm")
        print("กด Ctrl+C เพื่อออกจากโปรแกรม")
        print("=" * 80)
        print()
    
    def display_results(self, result, fps):
        """แสดงผลการตรวจจับ"""
        print("\r" + " " * 80, end="")  # ล้างบรรทัด
        
        if result is None:
            print(f"\rกำลังค้นหาสี่เหลี่ยมสีดำ... | FPS: {fps:.1f} | เวลา: {time.time() - self.start_time:.1f}s", end="", flush=True)
        else:
            distance = result['average']
            width = result['width_px']
            height = result['height_px']
            confidence = result['confidence']
            
            # แสดงผลแบบเรียลไทม์
            output = f"\rระยะทาง: {distance:6.1f} cm | ขนาด: {width:3.0f}×{height:3.0f} px | คะแนน: {confidence:5.2f} | FPS: {fps:4.1f}"
            print(output, end="", flush=True)
            
            self.detection_count += 1
            
            # แสดงรายละเอียดทุก ๆ 30 การตรวจจับ
            if self.detection_count % 30 == 0:
                print()  # ขึ้นบรรทัดใหม่
                print(f"รายละเอียด: X-axis={result['x_axis']:.1f}cm, Y-axis={result['y_axis']:.1f}cm")
                print(f"สถิติ: ตรวจจับได้ {self.detection_count} ครั้งจากทั้งหมด {self.total_detections} เฟรม")
    
    def run_detection(self):
        """เริ่มการตรวจจับแบบเรียลไทม์"""
        self.clear_screen()
        self.print_header()
        
        # เชื่อมต่อกับ RoboMaster
        print("กำลังเชื่อมต่อกับ RoboMaster...")
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        print("เชื่อมต่อสำเร็จ!")
        
        # เริ่ม video stream
        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=True, resolution="360p")
        print("เริ่ม video stream...")
        print()
        
        # ตัวแปรสำหรับคำนวณ FPS
        fps_counter = 0
        fps_start_time = time.time()
        current_fps = 0
        
        try:
            while True:
                # รับเฟรมจากกล้อง
                frame = ep_camera.read_cv2_image(timeout=0.5)
                
                if frame is not None:
                    self.total_detections += 1
                    
                    # ตรวจจับสี่เหลี่ยม
                    detection_result = self.detect_square_in_frame(frame)
                    
                    # คำนวณระยะทาง
                    result = self.calculate_distance(detection_result)
                    
                    # สร้างภาพแสดงผลพร้อมกรอบ
                    display_frame = frame.copy()
                    
                    if detection_result is not None:
                        w, h, score, final_x, final_y, x1, y1, x2, y2 = detection_result
                        
                        # วาดกรอบ ROI (สีแดง)
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        # วาดกรอบสี่เหลี่ยมที่ตรวจจับได้ (สีเขียว)
                        cv2.rectangle(display_frame, (final_x, final_y), 
                                    (final_x + w, final_y + h), (0, 255, 0), 3)
                        
                        if result is not None:
                            # แสดงข้อมูลระยะทางบนภาพ
                            distance = result['average']
                            text = f"Distance: {distance:.1f} cm"
                            cv2.putText(display_frame, text, (final_x, final_y - 10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # แสดงขนาดในหน่วย pixel
                            size_text = f"Size: {w}x{h} px"
                            cv2.putText(display_frame, size_text, (final_x, final_y + h + 25), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            # แสดงคะแนนความมั่นใจ
                            conf_text = f"Score: {score:.2f}"
                            cv2.putText(display_frame, conf_text, (final_x, final_y + h + 45), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        # วาดเฉพาะกรอบ ROI เมื่อไม่มีการตรวจจับ
                        H, W = frame.shape[:2]
                        cx, cy = W // 2, H // 2
                        roi_frac_w, roi_frac_h = 0.25, 0.30
                        rw = int(W * roi_frac_w / 2)
                        rh = int(H * roi_frac_h / 2)
                        x1 = max(0, cx - rw)
                        y1 = max(0, cy - rh)
                        x2 = min(W, cx + rw)
                        y2 = min(H, cy + rh)
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        # แสดงข้อความค้นหา
                        cv2.putText(display_frame, "Searching for black square...", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # แสดงภาพในหน้าต่าง
                    cv2.imshow("RoboMaster Distance Detection", display_frame)
                    cv2.waitKey(1)  # อัปเดตหน้าต่าง
                    
                    # คำนวณ FPS
                    fps_counter += 1
                    if fps_counter >= 10:
                        current_fps = fps_counter / (time.time() - fps_start_time)
                        fps_counter = 0
                        fps_start_time = time.time()
                    
                    # แสดงผลที่ terminal
                    self.display_results(result, current_fps)
                
                time.sleep(0.05)  # หน่วงเวลาเล็กน้อย
                
        except KeyboardInterrupt:
            print()
            print()
            print("หยุดการทำงานโดยผู้ใช้")
        except Exception as e:
            print()
            print(f"Error: {e}")
        finally:
            # แสดงสถิติสรุป
            print()
            print("=" * 50)
            print("สรุปผลการทำงาน:")
            runtime = time.time() - self.start_time
            success_rate = (self.detection_count / max(self.total_detections, 1)) * 100
            print(f"เวลาทำงาน: {runtime:.1f} วินาที")
            print(f"ตรวจจับสำเร็จ: {self.detection_count}/{self.total_detections} เฟรม ({success_rate:.1f}%)")
            print(f"FPS เฉลี่ย: {self.total_detections/runtime:.1f}")
            print("=" * 50)
            
            # ปิดการเชื่อมต่อ
            cv2.destroyAllWindows()  # ปิดหน้าต่างแสดงภาพ
            ep_camera.stop_video_stream()
            ep_robot.close()
            print("ปิดการเชื่อมต่อเรียบร้อย")

def main():
    """ฟังก์ชันหลัก"""
    print("เริ่มต้น Real-time Distance Detector...")
    detector = TerminalDistanceDetector()
    detector.run_detection()

if __name__ == "__main__":
    main()
