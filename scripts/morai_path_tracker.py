#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import time
import math

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

from morai_msgs.msg import CtrlCmd, CollisionData, ObjectStatusList


# ==========================
# 파라미터(필요시 조정)
# ==========================
TARGET_VELOCITY_KMH = 15.0      # 기본 주행 속도 (km/h)
STOP_HOLD_SEC        = 2.0       # 정지선에서 정차 유지 시간
CTRL_RATE_HZ         = 20

# (신규) 차선 폭 추정을 위한 픽셀 값 (실험을 통해 튜닝 필요)
LANE_WIDTH_PX = 380


# 차선/정지선 ROI (이미지 해상도에 맞춰 필요시 조정)
LANE_ROI_Y1_RATIO    = 0.60      # 하단 40% 중 윗경계 (예: 0.60*h ~ 0.78*h 사용)
LANE_ROI_Y2_RATIO    = 0.78
STOP_ROI_Y1_RATIO    = 0.70
STOP_ROI_Y2_RATIO    = 0.85

# 신호등 ROI (정지선 근처에서 화면 상단 중앙 영역)
TL_ROI_X1_RATIO      = 0.40
TL_ROI_X2_RATIO      = 0.60
TL_ROI_Y1_RATIO      = 0.05
TL_ROI_Y2_RATIO      = 0.35

# HSV 색상 범위 (필요 시 튜닝)
# 빨강은 2 구간으로 잡음
HSV_RED1_LO = (0,   100, 100)
HSV_RED1_HI = (10,  255, 255)
HSV_RED2_LO = (170, 100, 100)
HSV_RED2_HI = (180, 255, 255)

HSV_GREEN_LO = (35,  80,  80)
HSV_GREEN_HI = (85,  255, 255)

HSV_YELLOW_LO = (15,  80,  80)
HSV_YELLOW_HI = (35,  255, 255)

# 픽셀 카운트 임계치 (ROI 크기에 따라 조정)
TL_DETECT_THRESHOLD  = 250       # 신호등 색 검출 최소 픽셀 수
STOPLINE_THRESHOLD   = 2500      # 정지선 흰색 픽셀 수


# ==========================
# 상태 정의
# ==========================
STATE_TRAFFIC_SIGN = 0
STATE_LANE_DRIVE = 1
STATE_FINISH = 2

class PathTracker:
    def __init__(self):
        self.bridge = CvBridge() # [수정] CvBridge 초기화
        self.image = None

        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)
        # [추가] 디버깅용 이미지 토픽 Publisher
        self.roi_img_pub = rospy.Publisher("/debug/roi_visualization", Image, queue_size=1)
        self.lane_img_pub = rospy.Publisher("/debug/lane_detection", Image, queue_size=1)

        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.camera_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.collision_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)

        self.state = STATE_TRAFFIC_SIGN
        self.last_stop_time = 0.0
        self.target_velocity = TARGET_VELOCITY_KMH
        self.steering_rad = 0.3
        self.rate = rospy.Rate(CTRL_RATE_HZ)
        rospy.loginfo("[PathTracker] Initialized.")

    def camera_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def collision_callback(self, msg: CollisionData):
        if len(msg.collision_object) > 0:
            rospy.logwarn_throttle(2.0, "[CollisionData] collision_object exists.")

    def object_callback(self, msg: ObjectStatusList):
        rospy.loginfo_throttle(5.0, f"[Object_topic] NPCs: {msg.num_of_npcs}")

    def drive(self, steering_rad: float, target_velocity: float):
        rospy.loginfo_throttle(1.0, f"[Drive] Steering: {steering_rad:.2f} rad, Velocity: {target_velocity:.2f} km/h")
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = self.target_velocity
        cmd.steering = float(steering_rad)
        self.ctrl_pub.publish(cmd)

    def stop_car(self):
        self.drive(0.0, 0.0)

    def cut_roi(self, frame, x1r, y1r, x2r, y2r):
        h, w = frame.shape[:2]
        x1, x2 = int(w * x1r), int(w * x2r)
        y1, y2 = int(h * y1r), int(h * y2r)
        if x2 <= x1 or y2 <= y1:
            return None, None
        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)

    def check_stopline(self, frame) -> bool:
        roi, _ = self.cut_roi(frame, 0.0, STOP_ROI_Y1_RATIO, 1.0, STOP_ROI_Y2_RATIO)
        if roi is None: return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 180), (180, 60, 255))
        count = cv2.countNonZero(mask)
        rospy.loginfo_throttle(1.0, f"[Stopline] white pixels: {count}")
        return count > STOPLINE_THRESHOLD

    def detect_traffic_light(self, frame):
        roi, _ = self.cut_roi(frame, TL_ROI_X1_RATIO, TL_ROI_Y1_RATIO, TL_ROI_X2_RATIO, TL_ROI_Y2_RATIO)
        if roi is None: return "UNKNOWN"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_r1 = cv2.inRange(hsv, HSV_RED1_LO, HSV_RED1_HI)
        mask_r2 = cv2.inRange(hsv, HSV_RED2_LO, HSV_RED2_HI)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_green = cv2.inRange(hsv, HSV_GREEN_LO, HSV_GREEN_HI)
        mask_yellow = cv2.inRange(hsv, HSV_YELLOW_LO, HSV_YELLOW_HI)
        r_cnt, g_cnt, y_cnt = cv2.countNonZero(mask_red), cv2.countNonZero(mask_green), cv2.countNonZero(mask_yellow)
        rospy.loginfo_throttle(1.0, f"[TL] R:{r_cnt} G:{g_cnt} Y:{y_cnt}")
        if r_cnt > TL_DETECT_THRESHOLD and r_cnt > g_cnt and r_cnt > y_cnt: return "RED"
        if g_cnt > TL_DETECT_THRESHOLD and g_cnt > r_cnt and g_cnt > y_cnt: return "GREEN"
        if y_cnt > TL_DETECT_THRESHOLD and y_cnt > r_cnt and y_cnt > g_cnt: return "YELLOW"
        return "UNKNOWN"

    def lane_midpoint(self, frame):
        roi, _ = self.cut_roi(frame, 0.0, LANE_ROI_Y1_RATIO, 1.0, LANE_ROI_Y2_RATIO)
        if roi is None: return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 60, 75)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)
        
        debug_roi = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(debug_roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # [수정] 차선 인식 과정을 토픽으로 발행
        lane_img_msg = self.bridge.cv2_to_imgmsg(debug_roi, "bgr8")
        self.lane_img_pub.publish(lane_img_msg)

        if lines is None: return None

        h, w = roi.shape[:2]
        cx_roi = w // 2
        left, right = [], []
        for line in lines:
            x1,y1,x2,y2 = line[0]
            dx, slope = (x2 - x1) + 1e-6, (y2 - y1) / ((x2 - x1) + 1e-6)
            if slope < 0 and x1 < cx_roi and x2 < cx_roi: left.append(line)
            elif slope > 0 and x1 > cx_roi and x2 > cx_roi: right.append(line)

        if len(left) > 0 and len(right) > 0:
            lx = np.mean([ (l[0][0] + l[0][2]) * 0.5 for l in left ])
            rx = np.mean([ (r[0][0] + r[0][2]) * 0.5 for r in right ])
            mid_roi = (lx + rx) * 0.5
        elif len(left) > 0:
            lx = np.mean([ (l[0][0] + l[0][2]) * 0.5 for l in left ])
            mid_roi = lx + (LANE_WIDTH_PX / 2)
        elif len(right) > 0:
            rx = np.mean([ (r[0][0] + r[0][2]) * 0.5 for r in right ])
            mid_roi = rx - (LANE_WIDTH_PX / 2)
        else:
            return None
        return mid_roi, cx_roi

    def steering_from_error(self, err_px):
        Kp = 0.005
        steer = -Kp * err_px
        return float(np.clip(steer, -0.5, 0.5))

    def run(self):
        rospy.loginfo("[PathTracker] Waiting for camera...")
        rospy.wait_for_message("/image_jpeg/compressed", CompressedImage)
        rospy.loginfo("[PathTracker] Camera Ready. Entering main loop.")

        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()
                continue

            frame = self.image.copy()
            debug_frame = frame.copy()

            _, lane_coords = self.cut_roi(debug_frame, 0.0, LANE_ROI_Y1_RATIO, 1.0, LANE_ROI_Y2_RATIO)
            _, stop_coords = self.cut_roi(debug_frame, 0.0, STOP_ROI_Y1_RATIO, 1.0, STOP_ROI_Y2_RATIO)
            _, tl_coords = self.cut_roi(debug_frame, TL_ROI_X1_RATIO, TL_ROI_Y1_RATIO, TL_ROI_X2_RATIO, TL_ROI_Y2_RATIO)

            if lane_coords: cv2.rectangle(debug_frame, (lane_coords[0], lane_coords[1]), (lane_coords[2], lane_coords[3]), (0, 255, 0), 2)
            if stop_coords: cv2.rectangle(debug_frame, (stop_coords[0], stop_coords[1]), (stop_coords[2], stop_coords[3]), (0, 255, 255), 2)
            if tl_coords: cv2.rectangle(debug_frame, (tl_coords[0], tl_coords[1]), (tl_coords[2], tl_coords[3]), (255, 0, 255), 2)
            
            # [수정] ROI 시각화 이미지를 토픽으로 발행
            roi_img_msg = self.bridge.cv2_to_imgmsg(debug_frame, "bgr8")
            self.roi_img_pub.publish(roi_img_msg)

            # --- 이하 FSM 로직은 이전과 동일 ---
            if self.state == STATE_TRAFFIC_SIGN:
                if self.check_stopline(frame):
                    self.stop_car()
                    self.last_stop_time = time.time()
                    start_wait = time.time()
                    while not rospy.is_shutdown():
                        if self.image is None: continue
                        if self.detect_traffic_light(self.image.copy()) == "GREEN": break
                        self.stop_car()
                        if time.time() - start_wait > 15.0:
                            rospy.logwarn("[TRAFFIC_SIGN] Timeout.")
                            break
                        self.rate.sleep()
                    self.state = STATE_LANE_DRIVE
                    self.drive(0.0, max(5.0, TARGET_VELOCITY_KMH * 0.5))
                    self.rate.sleep()
                else:
                    mid = self.lane_midpoint(frame)
                    if mid: self.drive(self.steering_from_error(mid[0] - mid[1]), max(5.0, TARGET_VELOCITY_KMH * 0.5))
                    else: self.drive(0.0, 5.0)
            elif self.state == STATE_LANE_DRIVE:
                if self.check_stopline(frame) and (time.time() - self.last_stop_time > 5.0):
                    self.state = STATE_TRAFFIC_SIGN
                    continue
                mid = self.lane_midpoint(frame)
                if mid: self.drive(self.steering_from_error(mid[0] - mid[1]), self.target_velocity)
                else: self.drive(0.0, max(8.0, TARGET_VELOCITY_KMH * 0.4))
            elif self.state == STATE_FINISH:
                self.stop_car()
                break
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("morai_path_tracker")
    node = PathTracker()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.stop_car()
