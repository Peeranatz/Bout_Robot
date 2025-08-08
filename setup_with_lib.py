# -*- coding:utf-8 -*-
"""
RoboMaster EP - Search and Destroy Mission (ใช้ PID ควบคุม yaw+pitch ตอนเล็ง)
เวอร์ชัน: 27.2-mod-pid-yawpitch-led
"""

import time
import cv2
import robomaster
import threading
from robomaster import robot, vision, blaster, led
from queue import Empty
import colorsys  # สำหรับแปลง HSV -> RGB
import csv  # สำหรับเขียนไฟล์ CSV

# ==================== Helper Class ====================
class MarkerInfo:
    """เก็บข้อมูลของ Marker ที่ตรวจพบ"""
    def __init__(self, x, y, w, h, info):
        self.x, self.y, self.w, self.h, self.info = x, y, w, h, info

# ==================== PID Controller Class ====================
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def update(self, current_value):
        error = self.setpoint - current_value
        now = time.time()
        delta_time = 0
        if self._last_time is not None:
            delta_time = now - self._last_time
        self._last_time = now

        if delta_time == 0:
            derivative = 0
        else:
            derivative = (error - self._prev_error) / delta_time

        self._integral += error * delta_time
        self._prev_error = error

        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        return output

# ==================== Global Variables ====================
detected_markers = []
mission_sequence = ['1', '4', '5', '4', '1']
mission_index = 0
mission_stopped = False

latest_frame = None
is_video_thread_running = False

rainbow_running = True

# ==================== Callback Functions ====================
def on_detect_marker(marker_info):
    """บันทึก marker ที่ตรวจพบ (callback จาก SDK)"""
    global detected_markers
    detected_markers.clear()
    for x, y, w, h, info in marker_info:
        detected_markers.append(MarkerInfo(x, y, w, h, info))

# ==================== Thread Functions ====================
def video_stream_worker(ep_camera):
    global latest_frame, is_video_thread_running
    print("[Video] Thread started.")
    while is_video_thread_running:
        try:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.01)
            if frame is not None:
                latest_frame = frame
        except Empty:
            pass
        except Exception as e:
            print(f"[Video] Error: {e}")
    print("[Video] Thread stopped.")

def led_rainbow_effect(ep_led):
    global rainbow_running
    hue = 0.0
    while rainbow_running:
        r, g, b = colorsys.hsv_to_rgb(hue, 1, 1)
        r, g, b = int(r * 255), int(g * 255), int(b * 255)
        try:
            ep_led.set_led(comp=led.COMP_ALL, r=r, g=g, b=b, effect=led.EFFECT_ON)
        except Exception:
            pass
        hue += 0.01
        if hue > 1.0:
            hue = 0.0
        time.sleep(0.05)

# ==================== Action Functions ====================
def firing_effect(ep_blaster, ep_robot, ep_led):
    print("     ยิง! (แสง + เสียง)")

    # เปลี่ยนไฟเป็นสีแดงตอนยิง
    try:
        ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
    except Exception:
        pass

    try:
        ep_robot.play_sound(robot.SOUND_ID_SHOOT)
    except Exception:
        pass

    try:
        for brightness in range(1, 256, 30):
            ep_blaster.set_led(brightness=brightness, effect=blaster.LED_ON)
            time.sleep(0.02)
        time.sleep(0.2)
        ep_blaster.set_led(brightness=0, effect=blaster.LED_OFF)
    except Exception:
        pass

    # กลับเป็นสีน้ำเงินหลังยิง
    try:
        ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=255, effect=led.EFFECT_ON)
    except Exception:
        pass

# ==================== Main Execution ====================
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_led = ep_robot.led

    # สร้างไฟล์ CSV สำหรับบันทึกข้อมูล yaw, pitch, timestamp
    log_file = open('gimbal_log_fire_marker.csv', 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(log_file)
    csv_writer.writerow(['Timestamp', 'YawSpeed', 'PitchSpeed'])

    # ตั้งไฟเริ่มต้นเป็นสีน้ำเงิน (โหมดค้นหา)
    try:
        ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=255, effect=led.EFFECT_ON)
    except Exception:
        pass

    ep_camera.start_video_stream(display=False, resolution="360p")
    ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    is_video_thread_running = True
    video_thread = threading.Thread(target=video_stream_worker, args=(ep_camera,), daemon=True)
    video_thread.start()

    print("[Setup] รีเซ็ต Gimbal...")
    try:
        ep_gimbal.recenter(pitch_speed=50, yaw_speed=50).wait_for_completed()
    except Exception:
        pass
    time.sleep(1)

    # Settings
    SEARCH_SPEED = 50
    COOLDOWN_DURATION = 0.05
    MAX_YAW = 45
    CENTER_YAW = 0.0

    # PID parameters
    pid_yaw = PID(110, 30, 8, setpoint=0.5)
    pid_pitch = PID(110, 30, 8, setpoint=0.5)

    try:
        while mission_index < len(mission_sequence) and not mission_stopped:
            current_target_id = mission_sequence[mission_index]
            print(f"\n[Mission] ค้นหาเป้าหมาย '{current_target_id}' ({mission_index+1}/{len(mission_sequence)})")

            target_sighted = False
            found_marker = None
            direction = 1

            try:
                ep_gimbal.recenter(pitch_speed=50, yaw_speed=50).wait_for_completed()
            except Exception:
                pass
            time.sleep(0.1)

            pid_yaw.reset()
            pid_pitch.reset()

            try:
                ep_gimbal.drive_speed(yaw_speed=0, pitch_speed=0)
            except Exception:
                pass

            time.sleep(0.1)

            search_timeout = time.time() + 35
            current_yaw_position = 0.0
            started_search = False

            while time.time() < search_timeout and not mission_stopped:
                if not started_search:
                    print("[Search] เริ่มต้นค้นหาที่ yaw = 0")
                    try:
                        ep_gimbal.move(yaw=CENTER_YAW, yaw_speed=SEARCH_SPEED).wait_for_completed()
                    except Exception:
                        pass
                    started_search = True
                else:
                    target_angle = MAX_YAW * direction
                    print(f"[Search] หมุนไป yaw = {target_angle} องศา")
                    try:
                        ep_gimbal.move(yaw=target_angle, yaw_speed=SEARCH_SPEED).wait_for_completed()
                    except Exception:
                        pass
                    direction *= -1

                scan_start = time.time()
                while time.time() - scan_start < 0.5 and not mission_stopped:
                    img = latest_frame
                    if img is not None:
                        display_img = img.copy()

                        for m in detected_markers:
                            w, h = int(m.w * display_img.shape[1]), int(m.h * display_img.shape[0])
                            cx, cy = int(m.x * display_img.shape[1]), int(m.y * display_img.shape[0])
                            cv2.rectangle(display_img, (cx - w // 2, cy - h // 2),
                                          (cx + w // 2, cy + h // 2), (0, 255, 0), 2)
                            if m.info == current_target_id:
                                found_marker = m
                                target_sighted = True

                        cv2.putText(display_img, f"Searching for '{current_target_id}'...",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
                        cv2.imshow("Realtime Camera", display_img)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        mission_stopped = True
                        break

                    if target_sighted:
                        break

                if target_sighted:
                    break

            if target_sighted and not mission_stopped:
                print(f"[Mission] พบเป้าหมาย '{current_target_id}'! เริ่มเล็งด้วย PID")
                pid_yaw.reset()
                pid_pitch.reset()

                # เปลี่ยนไฟเป็นสีแดงขณะเล็ง
                try:
                    ep_led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
                except Exception:
                    pass

                aim_timeout = time.time() + 10
                while time.time() < aim_timeout and not mission_stopped:
                    img = latest_frame
                    if img is None:
                        time.sleep(0.01)
                        continue

                    target_marker = None
                    for m in detected_markers:
                        if m.info == current_target_id:
                            target_marker = m
                            break

                    if target_marker is None:
                        print("[Warning] Lost marker ระหว่างเล็ง")
                        time.sleep(0.05)
                        continue

                    yaw_output = -pid_yaw.update(target_marker.x)
                    pitch_output = pid_pitch.update(target_marker.y)

                    max_speed = 50
                    yaw_speed_cmd = max(-max_speed, min(max_speed, yaw_output))
                    pitch_speed_cmd = max(-max_speed, min(max_speed, pitch_output))

                    # บันทึกข้อมูล yaw, pitch และ timestamp ลง CSV
                    csv_writer.writerow([time.time(), yaw_speed_cmd, pitch_speed_cmd])

                    error_yaw = abs(pid_yaw.setpoint - target_marker.x)
                    error_pitch = abs(pid_pitch.setpoint - target_marker.y)
                    tolerance = 0.02

                    try:
                        ep_gimbal.drive_speed(yaw_speed=yaw_speed_cmd, pitch_speed=pitch_speed_cmd)
                    except Exception:
                        pass

                    display_img = img.copy()
                    w, h = int(target_marker.w * display_img.shape[1]), int(target_marker.h * display_img.shape[0])
                    cx, cy = int(target_marker.x * display_img.shape[1]), int(target_marker.y * display_img.shape[0])
                    cv2.rectangle(display_img, (cx - w // 2, cy - h // 2),
                                  (cx + w // 2, cy + h // 2), (0, 255, 255), 3)
                    cv2.putText(display_img, f"Aiming PID...", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                    cv2.imshow("Realtime Camera", display_img)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        mission_stopped = True
                        break

                    if error_yaw < tolerance and error_pitch < tolerance:
                        print("[Mission] เล็งตรงเป้าเรียบร้อย!")
                        break
                    time.sleep(0.02)

                try:
                    ep_gimbal.drive_speed(yaw_speed=0, pitch_speed=0)
                except Exception:
                    pass

                # ยิง
                firing_effect(ep_blaster, ep_robot, ep_led)

                try:
                    ep_gimbal.recenter(pitch_speed=50, yaw_speed=50).wait_for_completed()
                except Exception:
                    pass
                time.sleep(0.1)

                mission_index += 1
                time.sleep(COOLDOWN_DURATION)

            elif not mission_stopped:
                print(f"[Mission] ไม่พบเป้าหมาย '{current_target_id}'")
                mission_index += 1
                time.sleep(COOLDOWN_DURATION)

        if not mission_stopped:
            print("\n[Mission] ภารกิจเสร็จสิ้น!")
            try:
                ep_gimbal.recenter().wait_for_completed()
            except Exception:
                pass

    except KeyboardInterrupt:
        print("\n[User] หยุดโปรแกรม (Ctrl+C)")

    finally:
        print("\n[System] กำลังปิดระบบ...")
        is_video_thread_running = False
        rainbow_running = False

        if 'video_thread' in locals():
            video_thread.join()

        cv2.destroyAllWindows()
        try:
            ep_vision.unsub_detect_info(name="marker")
        except Exception:
            pass
        try:
            ep_camera.stop_video_stream()
        except Exception:
            pass
        try:
            ep_gimbal.drive_speed(0, 0)
        except Exception:
            pass
        try:
            ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=0)
        except Exception:
            pass
        try:
            ep_robot.close()
        except Exception:
            pass

        # ปิดไฟล์ CSV
        log_file.close()
        print("[System] ปิดระบบเรียบร้อย")
