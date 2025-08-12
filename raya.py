# -*-coding:utf-8-*-
import robomaster
from robomaster import robot
import time
import math

# -------------------- ตัวแปร Global --------------------
position = [0, 0, 0]
pid_control_active = False
start_x, start_y = None, None
previous_error, previous_time, integral_error = None, None, 0.0
current_yaw = 0.0
yaw_offset = 0.0  # ค่าชดเชยมุมเริ่มต้น
tof_readings_mm = []

# -------------------- ค่าพารามิเตอร์ --------------------
TARGET_DISTANCE = 0.6
TOLERANCE = 0.02
kp = 1.0
ki = 1
kd = 0.1
max_speed = 10
max_integral = 5.0

WALL_THRESHOLD_CM = 51
GIMBAL_YAW_SPEED = 300

# -------------------- ฟังก์ชัน ToF Sensor --------------------
def get_average_distance_cm(ep_sensor, num_samples=5, freq=10):
    """
    เก็บข้อมูลจาก ToF sensor ตามจำนวนครั้งที่กำหนด (num_samples)
    แล้วคำนวณหาค่าเฉลี่ยเป็นเซนติเมตร
    """
    global tof_readings_mm
    tof_readings_mm = []

    def sampling_callback(tof_info):
        if len(tof_readings_mm) < num_samples:
            tof_readings_mm.append(tof_info[0])

    ep_sensor.sub_distance(freq=freq, callback=sampling_callback)

    while len(tof_readings_mm) < num_samples:
        time.sleep(0.05)

    ep_sensor.unsub_distance()

    if not tof_readings_mm:
        return float('inf')

    avg_distance_mm = sum(tof_readings_mm) / len(tof_readings_mm)
    avg_distance_cm = (avg_distance_mm - 80) / 10.0
    
    return avg_distance_cm

# -------------------- Callback สำหรับตำแหน่ง --------------------
def sub_position_handler(position_info):
    """
    ใช้ PID ควบคุมการเดินตรง
    """
    global position, pid_control_active, start_x, start_y
    global previous_error, previous_time, integral_error

    position = position_info
    x, y = position[0], position[1]

    if not pid_control_active:
        return

    current_time = time.time()
    current_distance = math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)
    current_error = TARGET_DISTANCE - current_distance

    # P
    p_value = kp * current_error

    # I
    dt = current_time - previous_time if previous_time else 0
    if dt > 0:
        integral_error += current_error * dt
        integral_error = max(min(integral_error, max_integral), -max_integral)
    i_value = ki * integral_error

    # D
    d_value = 0
    if previous_error is not None and dt > 0:
        derror = current_error - previous_error
        d_value = kd * (derror / dt)

    # รวม PID
    pid_speed = max(min(p_value + i_value + d_value, max_speed), -max_speed)

    if abs(current_error) > TOLERANCE:
        ep_chassis.drive_speed(x=pid_speed, y=0, z=0)
    else:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        pid_control_active = False
        print("🎯 ถึงเป้าหมายแล้ว")

    previous_error, previous_time = current_error, current_time

# -------------------- Callback สำหรับมุม --------------------
def sub_attitude_handler(attitude_info):
    global current_yaw
    current_yaw = attitude_info[0]

# -------------------- ฟังก์ชันหมุนตามมุม --------------------
def adjust_yaw_to_target(ep_chassis, target_angle):
    """
    ใช้ P-Control หมุน (kp=1.0, ki=0, kd=0)
    """
    global current_yaw, yaw_offset
    print(f"🕹️  เริ่มปรับมุมสู่: {target_angle}°")
    turn_kp = 1.0
    angle_tolerance = 1.5
    start_time, timeout = time.time(), 5

    while time.time() - start_time < timeout:
        corrected_yaw = current_yaw - yaw_offset
        if corrected_yaw > 180:
            corrected_yaw -= 360
        elif corrected_yaw < -180:
            corrected_yaw += 360

        error = target_angle - corrected_yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < angle_tolerance:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            print(f"✅ ปรับมุมสำเร็จที่ {corrected_yaw:.2f}°")
            return

        turn_speed = max(min(error * turn_kp, 60), -60)
        ep_chassis.drive_speed(x=0, y=0, z=turn_speed)
        time.sleep(0.02)

    ep_chassis.drive_speed(x=0, y=0, z=0)
    print("⚠️ หมดเวลาในการปรับมุม")

# -------------------- ฟังก์ชันสแกนและตัดสินใจ --------------------
def scan_and_decide(ep_gimbal, ep_sensor):
    """
    สแกนระยะทางด้วย ToF sensor ที่มุม -90°, 0°, 90°
    และตัดสินใจทิศทางการเดิน
    """
    scan_angles = [-90, 0, 90]
    wall_status = {}
    
    print(f"--- Starting scan with gimbal speed: {GIMBAL_YAW_SPEED}°/s ---")
    
    for angle in scan_angles:
        print(f"\nMoving to yaw angle: {angle} degrees...")
        
        ep_gimbal.moveto(yaw=angle, pitch=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
        time.sleep(0.5)
        
        avg_dist = get_average_distance_cm(ep_sensor, num_samples=5)
        print(f"Average distance: {avg_dist:.2f} cm")
        wall_status[angle] = avg_dist

    # แสดงสถานะการสแกน
    print("\n--- Scan Complete ---")
    status_parts = []
    
    if wall_status[-90] < WALL_THRESHOLD_CM:
        status_parts.append("ซ้าย Not Ready")
    else:
        status_parts.append("ซ้าย Ready")
        
    if wall_status[0] < WALL_THRESHOLD_CM:
        status_parts.append("กลาง Not Ready")
    else:
        status_parts.append("กลาง Ready")
        
    if wall_status[90] < WALL_THRESHOLD_CM:
        status_parts.append("ขวา Not Ready")
    else:
        status_parts.append("ขวา Ready")
        
    final_status_string = " ".join(status_parts)
    print(f"///{final_status_string}")
    
    return wall_status

# -------------------- ฟังก์ชันการเดินด้วย PID --------------------
def move_forward_with_pid(ep_chassis):
    """
    เดินหน้าด้วย PID control
    """
    global pid_control_active, start_x, start_y, previous_time, previous_error, integral_error
    
    print("🚶‍♂️ เริ่มเดินตรง (PID)...")
    
    # รอให้ได้ข้อมูลตำแหน่งเริ่มต้น
    while tuple(position) == (0, 0, 0):
        time.sleep(0.01)

    start_x, start_y = position[0], position[1]
    previous_time = time.time()
    previous_error = TARGET_DISTANCE
    integral_error = 0.0
    pid_control_active = True

    while pid_control_active:
        time.sleep(0.05)
    
    # หยุดให้สนิท
    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.7)

# -------------------- Main Program --------------------
if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor

    # เริ่มต้นระบบ
    ep_chassis.sub_attitude(freq=20, callback=sub_attitude_handler)
    print("🔌 กำลังเชื่อมต่อ...")
    time.sleep(1)

    yaw_offset = current_yaw
    ep_gimbal.recenter().wait_for_completed()
    print(f"🔄 ตั้งค่า Offset: {yaw_offset:.2f}° | กล้องมองตรง")

    ep_chassis.sub_position(freq=20, callback=sub_position_handler)
    time.sleep(1)

    print("\n==================== 🚁 เริ่มการทำงานอัตโนมัติ 🚁 ====================")
    
    # สแกนสิ่งแวดล้อม
    wall_status = scan_and_decide(ep_gimbal, ep_sensor)
    
    # ตัดสินใจทิศทางการเดิน
    left_ready = wall_status[-90] >= WALL_THRESHOLD_CM
    center_ready = wall_status[0] >= WALL_THRESHOLD_CM
    right_ready = wall_status[90] >= WALL_THRESHOLD_CM
    
    desired_heading_angle = 0  # มุมปัจจุบัน
    
    # วนลูปต่อเนื่องจนกว่าจะไม่มีทางเดิน
    while True:
        if left_ready:
            print("\n🔄 ด้านซ้าย Ready - เลี้ยวซ้าย 90°")
            desired_heading_angle -= 90  # เลี้ยวซ้าย (ลบมุม)
            if desired_heading_angle < -180:
                desired_heading_angle += 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            
            # กดกิมบอลไป 0°
            ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
            time.sleep(0.5)
            
            # เดินหน้า 0.6 เมตร
            move_forward_with_pid(ep_chassis)
            
        elif center_ready:
            print("\n➡️ ด้านกลาง Ready - เดินหน้าตรง")
            # กิมบอลอยู่ที่ 0° อยู่แล้ว
            ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
            time.sleep(0.5)
            
            # เดินหน้า 0.6 เมตร
            move_forward_with_pid(ep_chassis)
            
        elif right_ready:
            print("\n↩️ ด้านขวา Ready - เลี้ยวขวา 90°")
            desired_heading_angle += 90  # เลี้ยวขวา (บวกมุม)
            if desired_heading_angle > 180:
                desired_heading_angle -= 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            
            # กดกิมบอลไป 0°
            ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
            time.sleep(0.5)
            
            # เดินหน้า 0.6 เมตร
            move_forward_with_pid(ep_chassis)
            
        else:
            print("\n🚫 ไม่มีด้านไหน Ready - เลี้ยวซ้าย 180° และเดินหน้าแล้วหยุด")
            # เลี้ยวซ้ายสองครั้ง (180°)
            desired_heading_angle -= 180  # เลี้ยวซ้าย 180°
            if desired_heading_angle < -180:
                desired_heading_angle += 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            
            # กดกิมบอลกลับ 0°
            ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
            time.sleep(0.5)
            
            # เดินหน้า 0.6 เมตร
            move_forward_with_pid(ep_chassis)
            
            # หยุดการทำงาน
            break
        
        # สแกนใหม่หลังจากการเดิน
        print("\n--- สแกนใหม่หลังการเดิน ---")
        wall_status = scan_and_decide(ep_gimbal, ep_sensor)
        left_ready = wall_status[-90] >= WALL_THRESHOLD_CM
        center_ready = wall_status[0] >= WALL_THRESHOLD_CM
        right_ready = wall_status[90] >= WALL_THRESHOLD_CM

    print("\n🏁 การทำงานเสร็จสิ้น - ไม่มีทางให้เดินแล้ว")
    
    # คืนกิมบอลกลับสู่ตำแหน่งเริ่มต้น
    ep_gimbal.recenter(yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
    
    # ปิดการเชื่อมต่อ
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_robot.close()
    print("✅ ปิดการเชื่อมต่อเรียบร้อย")