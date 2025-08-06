import robomaster
import time
import math
from robomaster import robot

# เพิ่มตัวแปรสำหรับเซ็นเซอร์
current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
start_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
target_distance = 2.0  # เป้าหมาย 2 เมตร

# เพิ่มตัวแปรสำหรับ gimbal
current_chassis_yaw = 0.0  # มุม yaw ปัจจุบันของหุ่นยนต์

def position_callback(position_info):
    """รับข้อมูลตำแหน่งจากเซ็นเซอร์"""
    global current_position, current_chassis_yaw
    current_position['x'] = position_info[0]
    current_position['y'] = position_info[1] 
    current_position['z'] = position_info[2]
    
    # อัพเดทมุม yaw ของหุ่นยนต์
    current_chassis_yaw = position_info[2]
    
    # คำนวณระยะทางที่เดินมาแล้ว
    distance = math.sqrt((current_position['x'] - start_position['x'])**2 + 
                        (current_position['y'] - start_position['y'])**2)
    print(f"📏 ระยะทาง: {distance:.3f}m, Yaw: {current_chassis_yaw:.1f}°")

def setup_gimbal(ep_gimbal):
    """ตั้งค่าเริ่มต้นสำหรับ gimbal"""
    print("🎯 กำลังตั้งค่า gimbal...")
    # ตั้งค่า gimbal ให้หันตรงไปข้างหน้า (yaw=0, pitch=0)
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=50).wait_for_completed()
    print("✅ gimbal พร้อมใช้งาน - หันหน้าตรง")

def update_gimbal_direction(ep_gimbal):
    """อัพเดททิศทาง gimbal ให้ตามหุ่นยนต์"""
    # ให้ gimbal หันตามมุม yaw ของหุ่นยนต์
    # ใช้ recenter เพื่อให้ gimbal หันตามทิศทางหุ่นยนต์
    ep_gimbal.recenter(pitch_speed=100, yaw_speed=100).wait_for_completed()

def reset_gimbal(ep_gimbal):
    """รีเซ็ต gimbal กลับสู่ตำแหน่งเริ่มต้น"""
    print("🔄 กำลังรีเซ็ต gimbal กลับตำแหน่งเริ่มต้น...")
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=50, yaw_speed=50).wait_for_completed()
    print("✅ gimbal กลับสู่ตำแหน่งเริ่มต้นแล้ว")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal  # เพิ่มการควบคุม gimbal
    
    try:
        # ตั้งค่าเริ่มต้น gimbal
        setup_gimbal(ep_gimbal)
        
        ep_chassis.sub_position(freq=10, callback=position_callback)
        time.sleep(0.5)  # รอให้เซ็นเซอร์เริ่มทำงาน
        
        # บันทึกตำแหน่งเริ่มต้น
        start_position.update(current_position)
        print(f"📍 ตำแหน่งเริ่มต้น: x={start_position['x']:.3f}, y={start_position['y']:.3f}")
        
        a = 2
        
        while a > 0:
            print(f"🚶 รอบที่ {3-a}: เริ่มเดินหน้า 2 เมตร...")
            
            # เดินหน้าด้วยการตรวจสอบระยะทาง
            ep_chassis.drive_wheels(w1=23.87, w2=23.87, w3=23.87, w4=23.87)
            
            # รอจนกว่าจะเดินได้ 2 เมตร
            while True:
                distance = math.sqrt((current_position['x'] - start_position['x'])**2 + 
                                   (current_position['y'] - start_position['y'])**2)
                if distance >= target_distance:
                    break
                time.sleep(0.1)  # ตรวจสอบทุก 100ms
            
            # หยุดเมื่อเดินครบระยะ
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            print(f"⏸️ รอบที่ {3-a}: เดิน {distance:.3f} เมตรเสร็จแล้ว - หยุดพัก 3 วินาที")
            time.sleep(3.0)

            # หมุน 180 องศา
            print(f"🔄 รอบที่ {3-a}: เริ่มหมุน 180 องศา...")
            ep_chassis.drive_wheels(w1=20, w2=-20, w3=-20, w4=20)
            time.sleep(6.0)
            
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            print(f"✅ รอบที่ {3-a}: หมุนเสร็จแล้ว")
            
            # อัพเดท gimbal ให้หันตามทิศทางใหม่ของหุ่นยนต์
            update_gimbal_direction(ep_gimbal)
            time.sleep(1.0)
            
            # รีเซ็ตตำแหน่งเริ่มต้นสำหรับรอบถัดไป
            start_position.update(current_position)
            
            a = a - 1
        
        print("🎉 ภารกิจเสร็จสมบูรณ์!")
        
        # รีเซ็ต gimbal กลับสู่ตำแหน่งเริ่มต้น
        reset_gimbal(ep_gimbal)
        
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาด: {e}")
        
    finally:
        # หยุดการเคลื่อนที่
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        
        # รีเซ็ต gimbal ก่อนปิดโปรแกรม
        try:
            reset_gimbal(ep_gimbal)
        except:
            pass
        
        # ยกเลิกการติดตามเซ็นเซอร์
        ep_chassis.unsub_position()
        ep_robot.close()