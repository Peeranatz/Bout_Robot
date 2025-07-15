import time
from robomaster import robot

# ตัวแปร global สำหรับเก็บค่ามุมล่าสุด
latest_angle = [0, 0]
# ตัวแปร global สำหรับเก็บค่าระยะ TOF ล่าสุด (TOF1)
latest_distance = [0]

def angle_callback(angle_info):
    global latest_angle
    # angle_info = (pitch, yaw, pitch_ground, yaw_ground)
    latest_angle[0] = angle_info[0]
    latest_angle[1] = angle_info[1]

def distance_callback(distance_info):
    global latest_distance
    # distance_info = [tof1, tof2, tof3, tof4]
    latest_distance[0] = distance_info[0]

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor  # ใช้สำหรับอ่าน TOF

    pitch_val = 0
    yaw_left = -90
    yaw_right = 90
    step = 1

    data = []

    # subscribe ข้อมูลมุม gimbal
    ep_gimbal.sub_angle(freq=10, callback=angle_callback)
    # subscribe ข้อมูลระยะ TOF
    ep_sensor.sub_distance(freq=10, callback=distance_callback)
    time.sleep(0.5)

    # เริ่มต้นที่ตรงกลาง
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()
    time.sleep(0.5)

    # หมุนไปทางซ้ายทีละ step จนถึงซ้ายสุด (เก็บค่าทุก step)
    for yaw in range(0, yaw_left - 1, -step):
        ep_gimbal.moveto(pitch=pitch_val, yaw=yaw).wait_for_completed()
        time.sleep(0.1)
        pitch_angle, yaw_angle = latest_angle[0], latest_angle[1]
        distance = latest_distance[0]
        data.append((pitch_angle, yaw_angle, distance))
        print(f"Pitch: {pitch_angle}, Yaw: {yaw_angle}, Distance: {distance + 79}")

    # หมุนกลับมาตรงกลาง (ไม่เก็บค่า)
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()
    time.sleep(0.5)

    # หมุนไปทางขวาทีละ step จนถึงขวาสุด (เก็บค่าทุก step)
    for yaw in range(0, yaw_right + 1, step):
        ep_gimbal.moveto(pitch=pitch_val, yaw=yaw).wait_for_completed()
        time.sleep(0.1)
        pitch_angle, yaw_angle = latest_angle[0], latest_angle[1]
        distance = latest_distance[0]
        data.append((pitch_angle, yaw_angle, distance))
        print(f"Pitch: {pitch_angle}, Yaw: {yaw_angle}, Distance: {distance + 79}")

    # หมุนกลับมาตรงกลาง (ไม่เก็บค่า)
    ep_gimbal.moveto(pitch=pitch_val, yaw=0).wait_for_completed()

    # ยกเลิก subscribe angle และ distance
    ep_gimbal.unsub_angle()
    ep_sensor.unsub_distance()
    ep_robot.close()

    # บันทึกข้อมูลลงไฟล์ (csv)
    with open("gimbal_tof_angle_data.csv", "w") as f:
        f.write("pitch_angle,yaw_angle,distance\n")
        for pitch_angle, yaw_angle, distance in data:
            f.write(f"{pitch_angle},{yaw_angle},{distance}\n")