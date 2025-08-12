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

# ===== ตัวแปรระบบพิกัดและหน่วยความจำ =====
robot_position = [0, 0]  # พิกัดปัจจุบันของหุ่นยนต์ [x, y]
visited_cells = set()    # เซตของพิกัดที่เคยไปแล้ว {(x, y), ...}
path_history = []        # ลิสต์เส้นทางที่เดินมา [(x, y), ...]
current_facing = "north" # ทิศทางที่หุ่นยนต์หันหน้าอยู่ปัจจุบัน

# -------------------- ค่าพารามิเตอร์ --------------------
TARGET_DISTANCE = 0.6
TOLERANCE = 0.02
kp = 1.0
ki = 0.3
kd = 0.2
max_speed = 8
max_integral = 5.0

WALL_THRESHOLD_CM = 51
GIMBAL_YAW_SPEED = 300

# ===== ฟังก์ชันแสดงสถานะระบบ ===== 
# เพิ่มหลังบรรทัด GIMBAL_YAW_SPEED = 300

def display_system_status():
    """แสดงสถานะทั้งหมดของระบบ"""
    print("\n" + "="*60)
    print("📊 SYSTEM STATUS DISPLAY")
    print("="*60)
    
    # ตัวแปรตำแหน่งและทิศทาง
    print("🗺️  POSITION & DIRECTION:")
    print(f"   Robot Position: {robot_position}")
    print(f"   Current Facing: {current_facing}")
    print(f"   Current Yaw: {current_yaw:.2f}°")
    print(f"   Yaw Offset: {yaw_offset:.2f}°")
    corrected_yaw = current_yaw - yaw_offset
    if corrected_yaw > 180:
        corrected_yaw -= 360
    elif corrected_yaw < -180:
        corrected_yaw += 360
    print(f"   Corrected Yaw: {corrected_yaw:.2f}°")
    
    # ตัวแปรหน่วยความจำ
    print(f"\n🧠 MEMORY SYSTEM:")
    print(f"   Visited Cells: {len(visited_cells)} locations")
    print(f"   Visited Set: {sorted(list(visited_cells))}")
    print(f"   Path History: {path_history}")
    
    # ตัวแปร PID
    print(f"\n⚙️  PID CONTROL:")
    print(f"   PID Active: {pid_control_active}")
    print(f"   Start Position: ({start_x}, {start_y})")
    print(f"   Target Distance: {TARGET_DISTANCE}")
    print(f"   Tolerance: {TOLERANCE}")
    print(f"   Previous Error: {previous_error}")
    print(f"   Integral Error: {integral_error}")
    print(f"   kp={kp}, ki={ki}, kd={kd}")
    
    # ตัวแปรเซนเซอร์
    print(f"\n📡 SENSOR DATA:")
    print(f"   ToF Readings (mm): {tof_readings_mm}")
    print(f"   Wall Threshold: {WALL_THRESHOLD_CM} cm")
    
    # ตัวแปรการเคลื่อนที่
    print(f"\n🚗 MOVEMENT PARAMETERS:")
    print(f"   Max Speed: {max_speed}")
    print(f"   Max Integral: {max_integral}")
    print(f"   Gimbal Yaw Speed: {GIMBAL_YAW_SPEED}°/s")
    
    print("="*60)

def display_wall_analysis(wall_status):
    """แสดงการวิเคราะห์กำแพงแบบละเอียด"""
    print(f"\n🔍 WALL ANALYSIS DETAILED:")
    print(f"   Left (-90°): {wall_status[-90]:.2f} cm {'✅ Clear' if wall_status[-90] >= WALL_THRESHOLD_CM else '❌ Blocked'}")
    print(f"   Center (0°): {wall_status[0]:.2f} cm {'✅ Clear' if wall_status[0] >= WALL_THRESHOLD_CM else '❌ Blocked'}")
    print(f"   Right (90°): {wall_status[90]:.2f} cm {'✅ Clear' if wall_status[90] >= WALL_THRESHOLD_CM else '❌ Blocked'}")
    print(f"   Threshold: {WALL_THRESHOLD_CM} cm")

def display_movement_calculation(direction_chosen, angle_change=None):
    """แสดงการคำนวณการเคลื่อนที่"""
    print(f"\n🧮 MOVEMENT CALCULATION:")
    print(f"   Chosen Direction: {direction_chosen}")
    print(f"   Current Facing: {current_facing}")
    if angle_change:
        print(f"   Angle Change: {angle_change}°")
    
    # คำนวณตำแหน่งถัดไป
    movement_direction = get_movement_direction(current_facing)
    future_position = update_position(robot_position, movement_direction)
    print(f"   Movement Direction: {movement_direction}")
    print(f"   Future Position: {future_position}")
    print(f"   Already Visited: {is_visited(future_position)}")

# ===== ฟังก์ชันจัดการระบบพิกัด =====
def update_position(current_pos, direction):
    """
    อัปเดตพิกัดตามทิศทางการเดิน
    Args:
        current_pos: [x, y] พิกัดปัจจุบัน
        direction: 'up', 'down', 'left', 'right'
    Returns:
        [x, y] พิกัดใหม่
    """
    x, y = current_pos
    if direction == 'up':     # เดินหน้า
        return [x + 1, y]
    elif direction == 'down': # ถอยหลัง
        return [x - 1, y]
    elif direction == 'left': # สไลด์ซ้าย
        return [x, y - 1]
    elif direction == 'right': # สไลด์ขวา
        return [x, y + 1]
    return current_pos

def update_facing_direction(current_facing, turn_direction):
    """
    อัปเดตทิศทางที่หุ่นยนต์หันหน้าหลังจากหมุน
    Args:
        current_facing: 'north', 'south', 'east', 'west'
        turn_direction: 'left', 'right', 'u_turn'
    Returns:
        ทิศทางใหม่
    """
    directions = ['north', 'east', 'south', 'west']
    current_index = directions.index(current_facing)
    
    if turn_direction == 'left':
        new_index = (current_index - 1) % 4
    elif turn_direction == 'right':
        new_index = (current_index + 1) % 4
    elif turn_direction == 'u_turn':
        new_index = (current_index + 2) % 4
    else:
        return current_facing
        
    return directions[new_index]

def get_movement_direction(facing):
    """
    แปลงทิศทางการหันหน้าเป็นทิศทางการเคลื่อนที่
    """
    direction_map = {
        'north': 'up',
        'south': 'down', 
        'east': 'right',
        'west': 'left'
    }
    return direction_map.get(facing, 'up')

def calculate_required_facing(from_pos, to_pos):
    """
    คำนวณทิศทางที่ต้องหันหน้าเพื่อเดินจาก from_pos ไป to_pos
    Args:
        from_pos: [x, y] พิกัดเริ่มต้น
        to_pos: [x, y] พิกัดเป้าหมาย
    Returns:
        'north', 'south', 'east', 'west' หรือ None ถ้าไม่สามารถเดินได้
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    
    # ต้องเดินทีละ 1 ช่องเท่านั้น
    if abs(dx) + abs(dy) != 1:
        return None
        
    if dx == 1:  # เดินไป x เพิ่มขึ้น
        return 'north'
    elif dx == -1:  # เดินไป x ลดลง
        return 'south'
    elif dy == 1:  # เดินไป y เพิ่มขึ้น
        return 'east'
    elif dy == -1:  # เดินไป y ลดลง
        return 'west'
    
    return None

def calculate_turn_angle(current_facing, target_facing):
    """
    คำนวณมุมที่ต้องหมุนจากทิศทางปัจจุบันไปทิศทางเป้าหมาย
    Returns:
        (angle_change, turn_type) 
        angle_change: มุมที่ต้องเปลี่ยน (-180 ถึง 180)
        turn_type: 'none', 'left', 'right', 'u_turn'
    """
    directions = ['north', 'east', 'south', 'west']
    current_index = directions.index(current_facing)
    target_index = directions.index(target_facing)
    
    angle_diff = (target_index - current_index) % 4
    
    if angle_diff == 0:
        return 0, 'none'
    elif angle_diff == 1:
        return 90, 'right'
    elif angle_diff == 2:
        return 180, 'u_turn'
    elif angle_diff == 3:
        return -90, 'left'

def record_visit(position):
    """
    บันทึกพิกัดที่เดินถึงลงในระบบหน่วยความจำ
    """
    global visited_cells, path_history
    pos_tuple = tuple(position)
    visited_cells.add(pos_tuple)
    path_history.append(pos_tuple)
    print(f"   Visited cells: {sorted(list(visited_cells))}")
    print(f"   Path history length: {len(path_history)}")

def is_visited(position):
    """
    ตรวจสอบว่าพิกัดนี้เคยไปแล้วหรือยัง
    """
    return tuple(position) in visited_cells

def backtrack_one_step():
    """
    ย้อนกลับไปพิกัดก่อนหน้า (ถ้ามี)
    Returns:
        พิกัดเป้าหมายสำหรับการย้อนกลับ หรือ None ถ้าไม่สามารถย้อนได้
    """
    global path_history
    if len(path_history) >= 2:
        # ลบพิกัดปัจจุบันออก
        current = path_history.pop()
        # พิกัดก่อนหน้าคือเป้าหมาย
        target = path_history[-1]
        print(f"🔙 Backtrack จาก {current} กลับไป {target}")
        return list(target)
    else:
        print("⚠️ ไม่สามารถ Backtrack ได้ (ไม่มีประวัติการเดิน)")
        return None

def execute_backtrack(ep_chassis, ep_gimbal, target_position):
    """
    ดำเนินการ Backtrack ไปยังพิกัดเป้าหมาย
    Args:
        target_position: [x, y] พิกัดที่ต้องการไป
    Returns:
        True ถ้าสำเร็จ, False ถ้าไม่สำเร็จ
    """
    global robot_position, current_facing, desired_heading_angle
    
    print(f"🔄 เริ่ม Backtrack จาก {robot_position} ไป {target_position}")
    
    # คำนวณทิศทางที่ต้องหัน
    required_facing = calculate_required_facing(robot_position, target_position)
    if not required_facing:
        print("❌ ไม่สามารถคำนวณทิศทางได้ - ระยะทางไม่ถูกต้อง")
        return False
    
    print(f"🧭 ต้องหันหน้าไป: {required_facing} (ปัจจุบัน: {current_facing})")
    
    # คำนวณการหมุน
    angle_change, turn_type = calculate_turn_angle(current_facing, required_facing)
    
    if turn_type != 'none':
        print(f"🔄 ต้องหมุน: {turn_type} ({angle_change}°)")
        desired_heading_angle += angle_change
        
        # ปรับมุมให้อยู่ในช่วง -180 ถึง 180
        if desired_heading_angle > 180:
            desired_heading_angle -= 360
        elif desired_heading_angle < -180:
            desired_heading_angle += 360
            
        adjust_yaw_to_target(ep_chassis, desired_heading_angle)
        current_facing = required_facing
        print(f"✅ หมุนเสร็จ - หันหน้า: {current_facing}")
    else:
        print("➡️ ไม่ต้องหมุน - ทิศทางถูกต้องแล้ว")
    
    # กดกิมบอลไป 0° และเดินหน้า
    ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
    time.sleep(0.5)
    
    # เดินหน้า 0.6 เมตร
    print("🚶‍♂️ เดินไปยังพิกัดเป้าหมาย...")
    move_forward_with_pid(ep_chassis)
    
    # อัปเดตตำแหน่ง
    robot_position = target_position[:]
    print(f"✅ Backtrack สำเร็จ - ตำแหน่งใหม่: {robot_position}")
    
    return True

def get_available_directions(current_wall_status):
    """
    หาทิศทางที่สามารถเดินได้และยังไม่เคยไป
    Args:
        current_wall_status: {-90: distance, 0: distance, 90: distance} 
        หรือ {-90: distance, 0: distance, 90: distance, 180: distance} สำหรับครั้งแรก
    Returns:
        list ของทิศทางที่เป็นไปได้ ['center', 'left', 'right'] หรือรวม ['u_turn'] ถ้ามี
    """
    global robot_position, current_facing
    available = []
    
    # เปลี่ยนลำดับเป็น: กลาง → ซ้าย → ขวา → กลับหลัง (ถ้ามี)
    
    # ตรวจสอบทิศทางกลาง (เดินหน้า) ก่อน
    if current_wall_status[0] >= WALL_THRESHOLD_CM:
        center_movement = get_movement_direction(current_facing)
        future_pos = update_position(robot_position, center_movement)
        if not is_visited(future_pos):
            available.append('center')
    
    # ตรวจสอบทิศทางซ้าย
    if current_wall_status[-90] >= WALL_THRESHOLD_CM:
        left_facing = update_facing_direction(current_facing, 'left')
        left_movement = get_movement_direction(left_facing)
        future_pos = update_position(robot_position, left_movement)
        if not is_visited(future_pos):
            available.append('left')
    
    # ตรวจสอบทิศทางขวา
    if current_wall_status[90] >= WALL_THRESHOLD_CM:
        right_facing = update_facing_direction(current_facing, 'right')
        right_movement = get_movement_direction(right_facing)
        future_pos = update_position(robot_position, right_movement)
        if not is_visited(future_pos):
            available.append('right')
    
    # ตรวจสอบทิศทางข้างหลัง (ถ้ามีข้อมูลจากการสแกน 4 ทาง)
    if 180 in current_wall_status and current_wall_status[180] >= WALL_THRESHOLD_CM:
        uturn_facing = update_facing_direction(current_facing, 'u_turn')
        uturn_movement = get_movement_direction(uturn_facing)
        future_pos = update_position(robot_position, uturn_movement)
        if not is_visited(future_pos):
            available.append('u_turn')
    
    return available

def get_average_distance_cm(ep_sensor, num_samples=5, freq=10):
    """
    เก็บข้อมูลจาก ToF sensor ตามจำนวนครั้งที่กำหนด (num_samples)
    แล้วคำนวณหาค่าเฉลี่ยเป็นเซนติเมตร
    """
    global tof_readings_mm
    tof_readings_mm = []

    # เพิ่มตรงนี้ - หลังจาก tof_readings_mm = []
    print(f"📡 เก็บข้อมูลจาก ToF sensor: {num_samples} samples at {freq} Hz")

    def sampling_callback(tof_info):
        if len(tof_readings_mm) < num_samples:
            tof_readings_mm.append(tof_info[0])

    ep_sensor.sub_distance(freq=freq, callback=sampling_callback)

    while len(tof_readings_mm) < num_samples:
        time.sleep(0.05)

    ep_sensor.unsub_distance()

    if not tof_readings_mm:
        print("   ❌ No ToF readings collected!")
        return float('inf')

    avg_distance_mm = sum(tof_readings_mm) / len(tof_readings_mm)
    avg_distance_cm = (avg_distance_mm - 80) / 10.0

    print(f"   Raw readings: {tof_readings_mm}")
    print(f"   Average (mm): {avg_distance_mm:.2f}")
    print(f"   Average (cm): {avg_distance_cm:.2f}")
    
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
def scan_and_decide(ep_gimbal, ep_sensor, ep_chassis, is_first_scan=False):
    """
    สแกนระยะทางด้วย ToF sensor 
    - ครั้งแรก: สแกน 4 ทาง (-90°, 0°, 90°, 180°)
    - ครั้งถัดไป: สแกน 3 ทาง (-90°, 0°, 90°)
    และตัดสินใจทิศทางการเดิน พร้อมล็อคล้อระหว่างสแกน
    """
    if is_first_scan:
        scan_angles = [-90, 0, 90, 180]  # สแกน 4 ทางครั้งแรก
    else:
        scan_angles = [-90, 0, 90]       # สแกน 3 ทางปกติ
        
    wall_status = {}
    
    print(f"--- Starting scan with gimbal speed: {GIMBAL_YAW_SPEED}°/s ---")
    
    # ===== ล็อคตัวหุ่นยนต์ให้อยู่กับที่ขณะสแกน (ล็อคล้อ 100%) =====
    ep_chassis.drive_speed(x=0, y=0, z=0)
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)  # ล็อคความเร็วล้อทุกล้อ
    time.sleep(0.2)
    
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
    
    if wall_status[0] < WALL_THRESHOLD_CM:
        status_parts.append("กลาง Not Ready")
    else:
        status_parts.append("กลาง Ready")
        
    if wall_status[-90] < WALL_THRESHOLD_CM:
        status_parts.append("ซ้าย Not Ready")
    else:
        status_parts.append("ซ้าย Ready")
        
    if wall_status[90] < WALL_THRESHOLD_CM:
        status_parts.append("ขวา Not Ready")
    else:
        status_parts.append("ขวา Ready")

    # เพิ่มสถานะข้างหลัง (เฉพาะครั้งแรก)
    if is_first_scan and 180 in wall_status:
        if wall_status[180] < WALL_THRESHOLD_CM:
            status_parts.append("ข้างหลัง Not Ready")
        else:
            status_parts.append("ข้างหลัง Ready")
        
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
    
    # เซ็ตไฟเป็นสีเหลืองตอนเชื่อมต่อ และคงไว้ตลอด
    ep_led = ep_robot.led
    ep_led.set_led(comp="all", r=255, g=255, b=0, effect="on")
    time.sleep(1)

    yaw_offset = current_yaw
    ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
    print(f"🔄 ตั้งค่า Offset: {yaw_offset:.2f}° | กิมบอลตั้งไว้ที่ 0° (กลางหุ่น)")
    
    # ตั้งค่าระบบพิกัดเริ่มต้น
    robot_position = [0, 0]
    visited_cells = set()
    path_history = []
    current_facing = "north"
    
    print(f"🗺️  ตั้งค่าระบบพิกัดเริ่มต้น: {robot_position} | ทิศทาง: {current_facing}")
    record_visit(robot_position)  # บันทึกจุดเริ่มต้น

    ep_chassis.sub_position(freq=20, callback=sub_position_handler)
    time.sleep(1)

    print("\n==================== 🚁 เริ่มการทำงานอัตโนมัติ 🚁 ====================")
    
    # ===== Main Loop: สำรวจเขาวงกต =====
print("\n==================== 🚁 เริ่มการสำรวจเขาวงกต 🚁 ====================")
    
desired_heading_angle = 0  # มุมปัจจุบัน
is_first_scan = True       # เพิ่มตัวแปรสำหรับครั้งแรก

# วนลูปต่อเนื่องจนกว่าจะไม่มีทางเดิน
while True:
    print(f"\n📍 ตำแหน่งปัจจุบัน: {robot_position} | ทิศทาง: {current_facing}")
    
    # สแกนสิ่งแวดล้อม (ครั้งแรกสแกน 4 ทาง)
    wall_status = scan_and_decide(ep_gimbal, ep_sensor, ep_chassis, is_first_scan)
    
    # เปลี่ยนเป็น False หลังจากสแกนครั้งแรกแล้ว
    if is_first_scan:
        is_first_scan = False
    
    # หาทิศทางที่เป็นไปได้ (ไม่ชนกำแพงและไม่เคยไป)
    available_directions = get_available_directions(wall_status)
    print(f"🧭 ทิศทางที่เป็นไปได้: {available_directions}")
    
    # ตัดสินใจทิศทางตามลำดับความสำคัญใหม่: กลาง > ซ้าย > ขวา > ข้างหลัง (เฉพาะครั้งแรก)
    chosen_direction = None
    if 'center' in available_directions:
        chosen_direction = 'center'
    elif 'left' in available_directions:
        chosen_direction = 'left'
    elif 'right' in available_directions:
        chosen_direction = 'right'
    elif 'back' in available_directions:  # เพิ่มตัวเลือกข้างหลัง
        chosen_direction = 'back'
    
    if chosen_direction:
        # มีทางให้เดิน
        if chosen_direction == 'center':
            print("\n➡️ เลือกเดินหน้าตรง")
            # ไม่ต้องหมุน ทิศทางคงเดิม
            
        elif chosen_direction == 'left':
            print("\n🔄 เลือกเดินซ้าย - เลี้ยวซ้าย 90°")
            desired_heading_angle -= 90  # เลี้ยวซ้าย (ลบมุม)
            if desired_heading_angle < -180:
                desired_heading_angle += 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            current_facing = update_facing_direction(current_facing, 'left')
            
        elif chosen_direction == 'right':
            print("\n↩️ เลือกเดินขวา - เลี้ยวขวา 90°")
            desired_heading_angle += 90  # เลี้ยวขวา (บวกมุม)
            if desired_heading_angle > 180:
                desired_heading_angle -= 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            current_facing = update_facing_direction(current_facing, 'right')
            
        elif chosen_direction == 'u_turn':  # เปลี่ยนจาก 'back' เป็น 'u_turn'
            print("\n🔄 เลือกเดินข้างหลัง - หมุนกลับ 180°")
            desired_heading_angle += 180  # หมุนกลับ 180°
            if desired_heading_angle > 180:
                desired_heading_angle -= 360
            elif desired_heading_angle < -180:
                desired_heading_angle += 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            current_facing = update_facing_direction(current_facing, 'u_turn')
            
        elif chosen_direction == 'back':
            print("\n🔄 เลือกเดินข้างหลัง - หมุนกลับ 180°")
            desired_heading_angle += 180  # หมุนกลับ 180°
            if desired_heading_angle > 180:
                desired_heading_angle -= 360
            elif desired_heading_angle < -180:
                desired_heading_angle += 360
            adjust_yaw_to_target(ep_chassis, desired_heading_angle)
            current_facing = update_facing_direction(current_facing, 'back')
        
        # กดกิมบอลไป 0° และเดินหน้า
        ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()
        time.sleep(0.5)
        
        # เดินหน้า 0.6 เมตร
        move_forward_with_pid(ep_chassis)
        
        # อัปเดตพิกัดและบันทึกการเยี่ยมชม
        movement_direction = get_movement_direction(current_facing)
        robot_position = update_position(robot_position, movement_direction)
        record_visit(robot_position)
        
    # ไม่ควรไปถึงจุดนี้ เพราะเราตรวจสอบ available_directions แล้ว
    # แต่เก็บไว้เผื่อมีกรณีพิเศษ
    else:
        print("\n⚠️ ไม่พบทิศทางที่เลือก - สิ้นสุดการทำงาน")
        break

print(f"\n🏁 การทำงานเสร็จสิ้น | เยี่ยมชมทั้งหมด: {len(visited_cells)} ช่อง")
print(f"📈 เส้นทางที่เดินมา: {path_history}")

# คืนกิมบอลกลับสู่ตำแหน่งเริ่มต้น
ep_gimbal.recenter(yaw_speed=GIMBAL_YAW_SPEED).wait_for_completed()

# ปิดการเชื่อมต่อ
ep_chassis.unsub_position()
ep_chassis.unsub_attitude()
ep_robot.close()
print("✅ ปิดการเชื่อมต่อเรียบร้อย")