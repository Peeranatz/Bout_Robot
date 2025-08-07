# -*-coding:utf-8-*-
"""
RoboMaster EP - ยิงเป้าตามภารกิจ พร้อมแสดงสถานะและระยะทาง
เวอร์ชัน: 7.1 (Displaying Status & Estimated Distance)
"""
# ... (import statements เหมือนเดิม) ...
import time, cv2, robomaster
from robomaster import robot, gimbal, vision, blaster, led, camera

class MissionBasedShooting:
    def __init__(self):
        # <<--- เพิ่มใหม่: ค่าคงที่สำหรับปรับเทียบระยะทาง (ต้องจูนกับของจริง)
        # ค่านี้ได้จากการทดลอง: REAL_MARKER_WIDTH_M * FOCAL_LENGTH / APPARENT_WIDTH_PIXELS
        # เราจะเริ่มจากค่าสมมติไปก่อน
        self.DISTANCE_CALIBRATION_CONSTANT = 0.1 

        # ตัวแปรสถานะ
        self.is_target_in_view = False
        self.target_x = 0.5
        self.target_w = 0 # <<--- เพิ่มใหม่: เก็บความกว้างของ marker ที่เห็น
        self.target_id = ""
        # ... (ส่วนที่เหลือของ __init__ เหมือนเดิม) ...
        self.mission_sequence = ['1', '4', '5', '4', '1']
        self.mission_index = 0
        self.robot_initialized = False
        self.scan_speed = 10
        self.scan_direction = 1
        self.scan_left_limit = -80
        self.scan_right_limit = 80
        self.P_GAIN_YAW = 100
        self.current_yaw = 0

    def initialize_robot(self):
        # ... (เหมือนเดิมทุกประการ) ...
        print("🔧 เริ่มต้นการเชื่อมต่อ RoboMaster EP...")
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="ap")
        self.ep_gimbal = self.ep_robot.gimbal
        self.ep_vision = self.ep_robot.vision
        self.ep_blaster = self.ep_robot.blaster
        self.ep_led = self.ep_robot.led
        self.ep_camera = self.ep_robot.camera
        self.ep_vision.sub_detect_info(name="marker", callback=self.on_detect_marker)
        print("✅ ระบบ Vision พร้อมใช้งาน")
        self.ep_gimbal.sub_angle(freq=20, callback=self.on_gimbal_angle)
        print("✅ ระบบติดตามมุม Gimbal พร้อมใช้งาน")
        self.ep_camera.start_video_stream(display=False, resolution="720p")
        print("✅ ระบบสตรีมวิดีโอพร้อมใช้งาน")
        self.set_led_color(0, 255, 0)
        self.robot_initialized = True
        print("✅ RoboMaster EP พร้อมใช้งาน!")

    def on_detect_marker(self, marker_info):
        """Callback ที่จะทำงานเมื่อกล้องตรวจพบ Marker"""
        if len(marker_info) > 0:
            x, y, w, h, info = marker_info[0]
            self.is_target_in_view = True
            self.target_x = x
            self.target_w = w # <<--- เพิ่มใหม่: อัปเดตความกว้างของ marker
            self.target_id = info
        else:
            self.is_target_in_view = False
    
    # ... (on_gimbal_angle, set_led_color, center_on_target, firing_action เหมือนเดิม) ...
    def on_gimbal_angle(self, angle_info):
        self.current_yaw = angle_info[1]
    def set_led_color(self, r, g, b):
        self.ep_led.set_led(comp=led.COMP_ALL, r=r, g=g, b=b, effect=led.EFFECT_ON)
    def center_on_target(self):
        print(f"🎯 กำลังเล็งเป้าหมาย '{self.target_id}' ให้เข้ากลาง...")
        while self.is_target_in_view:
            error_x = 0.5 - self.target_x
            if abs(error_x) < 0.015:
                print("✅ เข้าเป้าแล้ว!")
                self.ep_gimbal.drive_speed(0, 0)
                return True
            yaw_speed = self.P_GAIN_YAW * error_x
            self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=yaw_speed)
            time.sleep(0.01)
        print("⚠️ เป้าหมายหลุดจากมุมกล้อง! กลับไปสแกนต่อ...")
        return False
    def firing_action(self):
        print("🔥 เริ่มลำดับการยิง!")
        self.set_led_color(255, 0, 0)
        time.sleep(0.5)
        print("💥 ยิง!")
        self.ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
        time.sleep(1)
        self.set_led_color(0, 255, 0)
        print("✅ ยิงเสร็จสิ้น!")


    def run_mission(self):
        if not self.robot_initialized: return
        print("\n🚀 เริ่มภารกิจยิงเป้าตามลำดับ...")
        print(f"ลำดับภารกิจ: {self.mission_sequence}")
        
        self.ep_gimbal.moveto(pitch=0, yaw=self.scan_left_limit, yaw_speed=120).wait_for_completed()
        
        while self.mission_index < len(self.mission_sequence):
            current_target_id = self.mission_sequence[self.mission_index]
            status_text = f"Hunting: '{current_target_id}' ({self.mission_index + 1}/{len(self.mission_sequence)})"

            # Loop การสแกนหาเป้าหมายปัจจุบัน
            while True:
                img = self.ep_camera.read_cv2_image(strategy="newest")
                if img is not None:
                    # <<--- เพิ่มใหม่: แสดงสถานะและระยะทางบนจอ
                    # แสดงสถานะปัจจุบัน
                    cv2.putText(img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    
                    if self.is_target_in_view:
                        # คำนวณและแสดงระยะทาง
                        estimated_distance = 0
                        if self.target_w > 0:
                            estimated_distance = self.DISTANCE_CALIBRATION_CONSTANT / self.target_w
                        
                        dist_text = f"ID: {self.target_id} | Dist: {estimated_distance:.2f}m"
                        
                        # วาด Bounding Box
                        x_pixel = int(self.target_x * img.shape[1])
                        cv2.line(img, (x_pixel, 0), (x_pixel, img.shape[0]), (0, 255, 255), 1)
                        cv2.putText(img, dist_text, (x_pixel + 5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    cv2.imshow("RoboMaster Cam", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("⚠️ ผู้ใช้สั่งหยุดโปรแกรม")
                        return

                if self.is_target_in_view and self.target_id == current_target_id:
                    self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                    aim_success = self.center_on_target()
                    
                    if aim_success:
                        self.firing_action()
                        self.mission_index += 1
                    
                    time.sleep(1.5)
                    break 
                
                # ... (ส่วนการสแกนเหมือนเดิม) ...
                current_scan_speed = self.scan_speed * self.scan_direction
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=current_scan_speed)
                if self.current_yaw >= self.scan_right_limit: self.scan_direction = -1
                elif self.current_yaw <= self.scan_left_limit: self.scan_direction = 1
        
        print("\n🎉🎉🎉 สำเร็จภารกิจ! ยิงครบทุกเป้าหมายตามลำดับแล้ว 🎉🎉🎉")

    # ... (shutdown เหมือนเดิม) ...
    def shutdown(self):
        if not self.robot_initialized: return
        print("🔌 กำลังปิดระบบ...")
        if hasattr(self, 'ep_camera'): self.ep_camera.stop_video_stream()
        if hasattr(self, 'ep_vision'): self.ep_vision.unsub_detect_info(name="marker")
        if hasattr(self, 'ep_gimbal'):
            self.ep_gimbal.unsub_angle()
            self.ep_gimbal.recenter().wait_for_completed()
        if hasattr(self, 'ep_led'): self.set_led_color(0, 0, 0)
        if hasattr(self, 'ep_robot'): self.ep_robot.close()
        print("✅ ปิดระบบเสร็จสิ้น!")

# ... (if __name__ == '__main__' เหมือนเดิม) ...
if __name__ == '__main__':
    scanner = MissionBasedShooting()
    try:
        scanner.initialize_robot()
        input("📋 วาง Marker 4, 1, 5 ให้พร้อม แล้วกด Enter เพื่อเริ่มภารกิจ...")
        scanner.run_mission()
    except KeyboardInterrupt:
        print("\n⚠️ ผู้ใช้สั่งหยุดโปรแกรม (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ เกิดข้อผิดพลาดร้ายแรง: {e}")
    finally:
        cv2.destroyAllWindows()
        scanner.shutdown()