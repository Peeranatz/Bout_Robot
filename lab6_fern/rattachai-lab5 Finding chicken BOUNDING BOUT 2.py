# -*-coding:utf-8-*-
import cv2
import numpy as np
import time
from robomaster import robot, camera

# -----------------------------
# ฟังก์ชันตรวจจับไก่ (รวมทุกขั้นตอน) บนภาพ BGR
# -----------------------------
def detect_chicken_final(image):
    # แปลงภาพเป็น HSV เพื่อการแยกสีเหลืองง่ายขึ้น
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # -----------------------------
    # กำหนดช่วงสีเหลือง 3 ช่วง
    # -----------------------------
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

    # -----------------------------
    # สร้าง ROI (วงรี) ตรงกลางภาพเพื่อโฟกัสบริเวณกลาง
    # -----------------------------
    h, w = image.shape[:2]
    center_x, center_y = w // 2, h // 2
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    axes_x, axes_y = w // 6, h // 6
    cv2.ellipse(roi_mask, (center_x, center_y), (axes_x, axes_y),
                0, 0, 360, 255, -1)  # วงรีสีขาวบน mask

    # รวม mask สีเหลืองกับ ROI
    combined_mask = cv2.bitwise_and(yellow_mask, roi_mask)

    # -----------------------------
    # ทำ Morphological operations เพื่อลบ noise และเติมช่องว่าง
    # -----------------------------
    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

    # -----------------------------
    # หา contours ของวัตถุ
    # -----------------------------
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # -----------------------------
    # เตรียมภาพผลลัพธ์
    # -----------------------------
    result = image.copy()
    bbox = None

    if contours:
        # หา contour ที่ใหญ่ที่สุด
        largest = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(largest)
        # ตรวจสอบว่าพื้นที่พอเหมาะ (> 200 pixels) ก่อนวาด
        if bw * bh > 200:
            bbox = (x, y, bw, bh)
            # วาด bounding box สีเขียว
            cv2.rectangle(result, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            # แสดงขนาด bounding box บนภาพ
            cv2.putText(result, f"BBox: {bw}x{bh}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return result, bbox


# -----------------------------
# ส่วน main สำหรับรันบน RoboMaster
# -----------------------------
if __name__ == '__main__':
    # เชื่อมต่อ RoboMaster
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")  # ใช้การเชื่อมต่อ AP (Wi-Fi)

    # เริ่มสตรีมวิดีโอ
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    print("[INFO] Press 's' to save image, 'q' to quit")

    while True:
        # อ่านภาพจากกล้อง
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)
        if frame is None:
            continue

        # ตรวจจับไก่บนภาพ
        processed, bbox = detect_chicken_final(frame)

        # แสดงภาพผลลัพธ์
        cv2.imshow("RoboMaster Detection", processed)

        # รอคำสั่งคีย์
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            # กด 's' เพื่อบันทึกภาพ
            timestamp = int(time.time())
            filename = f"detect_capture_{timestamp}.jpg"
            cv2.imwrite(filename, processed)
            print(f"[INFO] Saved {filename}")
        elif key == ord('q'):
            # กด 'q' เพื่อออก
            break

    # ปิดสตรีมและตัดการเชื่อมต่อ
    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows()
