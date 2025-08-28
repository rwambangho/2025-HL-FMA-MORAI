#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
my_driver2.py
ROS 기반 자율주행 테스트 노드:
- 카메라(CompressedImage)로 차선/정지선/신호등 검출
- GPS로 속도 추정 (pyproj 사용 시 UTM 변환)
- IMU에서 yaw 추출
- Stanley 조향 제어 + 단순 P 속도 제어
"""

import math
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Imu
from morai_msgs.msg import CtrlCmd, GPSMessage

try:
    from pyproj import Proj
    _HAS_PYPROJ = True
except Exception:
    _HAS_PYPROJ = False

# OpenCV 내부 스레드 수를 1로 제한(임베디드/저사양 환경에서 안정성 향상)
cv2.setNumThreads(1)

# -----------------------
# 유틸 함수
# -----------------------
def clamp(x, a, b):
    """값 x를 [a, b] 범위로 제한."""
    return a if x < a else (b if x > b else x)

def decode_compressed(msg: CompressedImage):
    """ROS CompressedImage를 OpenCV BGR 이미지로 디코드."""
    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img
    except Exception as e:
        rospy.logwarn_throttle(1.0, "imdecode failed: %s", e)
        return None

# -----------------------
# GPS 기반 속도 추정기
# -----------------------
class SimpleSpeedEstimator:
    """연속된 GPS 좌표(UTM) 거리/시간으로 속도 추정."""
    def __init__(self, zone=52):
        self.last_xy = None
        self.last_t = None
        self.vx = 0.0
        self.proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False) if _HAS_PYPROJ else None

    def update(self, gps: GPSMessage):
        """GPSMessage 수신 시 속도 갱신."""
        if self.proj is None:
            return self.vx
        x, y = self.proj(gps.longitude, gps.latitude)
        t = rospy.get_time()
        if self.last_xy is not None and self.last_t is not None:
            dt = max(1e-3, t - self.last_t)
            ds = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.vx = ds / dt
        self.last_xy = (x, y)
        self.last_t = t
        return self.vx

# -----------------------
# 신호등 상태 enum
# -----------------------
class TL:
    NONE = 0
    GREEN = 1
    GREEN_LEFT = 2
    YELLOW = 3
    RED = 4

# -----------------------
# 메인 노드
# -----------------------
class MyDriver2:
    """카메라/IMU/GPS를 구독하여 차선 추종 및 신호대응 제어 명령을 퍼블리시."""
    def __init__(self):
        # --- 파라미터 로드 ---
        self.cam_topic = rospy.get_param('~topics/cam', '/image_jpeg/compressed')
        imu_topic = rospy.get_param('~topics/imu', '/imu')
        gps_topic = rospy.get_param('~topics/gps', '/gps')
        cmd_topic = rospy.get_param('~topics/ctrl_cmd', '/ctrl_cmd')

        # 차량/제어 파라미터
        self.L = rospy.get_param('~vehicle/wheelbase', 2.6)
        self.a_max = rospy.get_param('~vehicle/max_accel', 0.8)
        self.b_max = rospy.get_param('~vehicle/max_brake', 0.6)
        self.v_max = rospy.get_param('~vehicle/max_speed', 8.0)
        self.v_min = rospy.get_param('~vehicle/min_speed', 1.5)

        self.k_steer = rospy.get_param('~ctrl/k_steer', 0.7)
        self.k_soft = rospy.get_param('~ctrl/k_soft', 1.0)
        self.kp_speed = rospy.get_param('~ctrl/kp_speed', 0.4)
        self.curve_k = rospy.get_param('~ctrl/curve_k', 5.0)
        self.curve_min_v = rospy.get_param('~ctrl/curve_min_v', 2.0)

        # 이미지 ROI 비율(하단 영역에서 차선 검출)
        self.h_bottom = float(rospy.get_param('~roi/h_bottom', 1.0))
        self.h_top = float(rospy.get_param('~roi/h_top', 0.6))
        if not (0.0 <= self.h_top < self.h_bottom <= 1.0):
            rospy.logwarn("ROI invalid (h_top=%s, h_bottom=%s). Reset to 0.55~1.0", self.h_top, self.h_bottom)
            self.h_top, self.h_bottom = 0.55, 1.0

        # HSV 마스크 범위
        def arr(name, default):
            return np.array(rospy.get_param(name, default), dtype=np.uint8)
        self.hsv_white_low  = arr('~hsv/white_low',  [0, 0, 200])
        self.hsv_white_high = arr('~hsv/white_high', [180, 30, 255])
        self.hsv_yel_low    = arr('~hsv/yel_low',    [20, 100, 100])
        self.hsv_yel_high   = arr('~hsv/yel_high',   [30, 255, 255])
        self.hsv_red1_low   = arr('~hsv/red1_low',   [0, 80, 80])
        self.hsv_red1_high  = arr('~hsv/red1_high',  [10, 255, 255])
        self.hsv_red2_low   = arr('~hsv/red2_low',   [170, 80, 80])
        self.hsv_red2_high  = arr('~hsv/red2_high',  [180, 255, 255])
        self.hsv_grn_low    = arr('~hsv/green_low',  [40, 80, 80])
        self.hsv_grn_high   = arr('~hsv/green_high', [90, 255, 255])
        self.hsv_ylw_low    = arr('~hsv/yellow_low', [15, 120, 120])
        self.hsv_ylw_high   = arr('~hsv/yellow_high',[35, 255, 255])

        # --- 상태 변수 ---
        self.cam_img = None
        self.last_cam = rospy.Time(0)
        self.vx = 0.0
        self.last_imu = rospy.Time(0)
        self.last_gps = rospy.Time(0)
        self.imu_yaw = 0.0

        # 차선이 안 보일 때 마지막 유효 편차/각도 사용
        self.last_valid_e = 0.0
        self.last_valid_psi = 0.0

        self.speed_est = SimpleSpeedEstimator(zone=52)

        # --- Pub/Sub 설정 ---
        self.pub_cmd = rospy.Publisher(cmd_topic, CtrlCmd, queue_size=10)
        rospy.Subscriber(self.cam_topic, CompressedImage, self.cb_cam, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber(imu_topic, Imu, self.cb_imu, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber(gps_topic, GPSMessage, self.cb_gps, queue_size=10, tcp_nodelay=True)

        rospy.loginfo("my_driver2 ready: cam=%s, imu=%s, gps=%s -> cmd=%s",
                      self.cam_topic, imu_topic, gps_topic, cmd_topic)

        # 카메라 타임아웃(초) — 너무 작으면 빈번히 timeout 발생할 수 있음
        self.timeout_sec = rospy.get_param('~topics/timeout_sec', 0.5)

    # -----------------------
    # 콜백
    # -----------------------
    def cb_cam(self, msg: CompressedImage):
        """카메라 프레임 수신 시 디코딩 및 최신 시각 갱신."""
        img = decode_compressed(msg)
        if img is None:
            rospy.logwarn_throttle(1.0, "cam decode None")
            return
        self.cam_img = img
        self.last_cam = rospy.Time.now()

    def cb_imu(self, msg: Imu):
        """IMU의 사원수에서 yaw(방위각) 계산."""
        q = msg.orientation
        ysqr = q.y * q.y
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (ysqr + q.z * q.z)
        self.imu_yaw = math.atan2(t3, t4)
        self.last_imu = rospy.Time.now()

    def cb_gps(self, msg: GPSMessage):
        """GPS 수신 시 속도 추정 업데이트."""
        self.vx = self.speed_est.update(msg)
        self.last_gps = rospy.Time.now()

    # -----------------------
    # 비전 유틸(차선/정지선/신호등)
    # -----------------------
    def lane_mask(self, img):
        """하단 ROI에서 흰/노란 차선 마스크 추출."""
        h, w = img.shape[:2]
        y0 = int(self.h_top * h)
        y1 = int(self.h_bottom * h)
        if y1 <= y0:
            y0 = int(0.55 * h)
            y1 = h
        roi = img[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_w = cv2.inRange(hsv, self.hsv_white_low, self.hsv_white_high)
        mask_y = cv2.inRange(hsv, self.hsv_yel_low, self.hsv_yel_high)
        mask = cv2.bitwise_or(mask_w, mask_y)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        return mask, y0

    def detect_stopline(self, img):
        """정지선 검출: ROI 내 수평 직선(HoughLinesP) 확인."""
        mask, _ = self.lane_mask(img)
        edges = cv2.Canny(mask, 80, 160)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80, minLineLength=60, maxLineGap=20)
        if lines is None:
            return False
        h = mask.shape[0]
        y_th = int(0.85 * h)
        for x1, y1, x2, y2 in lines[:, 0, :]:
            if abs(y2 - y1) < 8 and min(y1, y2) > y_th:
                return True
        return False

    def detect_tl(self, img):
        """신호등 색 검출(빨/노/초, 좌회전 추정)."""
        h, w = img.shape[:2]
        roi = img[0:int(0.35 * h), int(0.35 * w):int(0.65 * w)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red = cv2.bitwise_or(cv2.inRange(hsv, self.hsv_red1_low, self.hsv_red1_high),
                             cv2.inRange(hsv, self.hsv_red2_low, self.hsv_red2_high))
        ylw = cv2.inRange(hsv, self.hsv_ylw_low, self.hsv_ylw_high)
        grn = cv2.inRange(hsv, self.hsv_grn_low, self.hsv_grn_high)
        if np.sum(red) > 1200:
            return TL.RED
        if np.sum(ylw) > 1200:
            return TL.YELLOW
        if np.sum(grn) > 1200:
            h2, w2 = grn.shape
            if np.sum(grn[:, :w2 // 2]) > 1.5 * np.sum(grn[:, w2 // 2:]):
                return TL.GREEN_LEFT
            return TL.GREEN
        return TL.NONE

    def fit_lane_center(self, img):
        """
        차선 중심선 추정:
        - HoughLinesP로 좌/우 차선 후보
        - 평균 직선 피팅(cv2.fitLine)
        - 이미지 중심 대비 가로 오프셋(e), 헤딩 에러(psi), 곡률 대용량(kappa) 계산
        - 차선 미검출 시 마지막 유효(e, psi) 반환(부드러운 복구)
        """
        mask, _ = self.lane_mask(img)
        edges = cv2.Canny(mask, 60, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60, minLineLength=40, maxLineGap=30)
        h, w = mask.shape[:2]

        # 차선이 전혀 감지되지 않으면 마지막 유효값 사용
        if lines is None:
            return self.last_valid_e, self.last_valid_psi, 0.0

        lefts, rights = [], []
        for x1, y1, x2, y2 in lines[:, 0, :]:
            dx = float(x2 - x1)
            slope = 1e6 if abs(dx) < 1e-6 else (y2 - y1) / dx
            if slope < -0.3:
                lefts.append((x1, y1, x2, y2))
            elif slope > 0.3:
                rights.append((x1, y1, x2, y2))

        def avg_line(lines):
            """여러 선분을 하나의 평균 직선으로 피팅."""
            xs, ys = [], []
            for x1, y1, x2, y2 in lines:
                xs += [x1, x2]
                ys += [y1, y2]
            if len(xs) < 2:
                return None
            [vx, vy, x0, y0] = cv2.fitLine(np.array(list(zip(xs, ys)), np.int32), cv2.DIST_L2, 0, 0.01, 0.01)
            vx = float(vx); vy = float(vy); x0 = float(x0); y0 = float(y0)
            yA = int(0.9 * h); yB = int(0.6 * h)
            xA = int(x0) if abs(vx) < 1e-6 else int(x0 + (yA - y0) * vx / vy)
            xB = int(x0) if abs(vx) < 1e-6 else int(x0 + (yB - y0) * vx / vy)
            return (xA, yA, xB, yB)

        L = avg_line(lefts) if lefts else None
        R = avg_line(rights) if rights else None

        # 최종 중심선 두 점 (xA,yA)~(xB,yB) 계산
        if L and R:
            xA = (L[0] + R[0]) // 2; yA = L[1]
            xB = (L[2] + R[2]) // 2; yB = L[3]
        elif L:
            lane_px = int(0.4 * w)
            xA, yA, xB, yB = L[0] + lane_px, L[1], L[2] + lane_px, L[3]
        elif R:
            lane_px = int(0.4 * w)
            xA, yA, xB, yB = R[0] - lane_px, R[1], R[2] - lane_px, R[3]
        else:
            return self.last_valid_e, self.last_valid_psi, 0.0

        # 오프셋/헤딩/곡률 계산
        img_cx = w // 2
        lane_mid_x = xA
        offset_px = img_cx - lane_mid_x
        m_per_px = 3.7 / max(100.0, abs((L[0] - R[0])) if (L and R) else 350.0)
        offset_m = offset_px * m_per_px
        heading = math.atan2((yB - yA + 1e-6), (xB - xA + 1e-6))
        heading_err = -(heading - math.pi / 2.0)
        curvature = abs(heading_err) / max(1.0, self.vx + 1e-3)

        # 마지막 유효값 갱신
        self.last_valid_e = offset_m
        self.last_valid_psi = heading_err
        return offset_m, heading_err, curvature

    # -----------------------
    # 제어 도우미
    # -----------------------
    def stanley(self, e, psi):
        """Stanley 제어기: 편차 e, 각도오차 psi로 조향각 산출."""
        v = max(0.0, self.vx)
        return clamp(psi + math.atan2(self.k_steer * e, v + self.k_soft), -0.6, 0.6)

    def v_from_curvature(self, kappa):
        """곡률에 따라 목표 속도 제한."""
        return clamp(self.v_max / (1.0 + self.curve_k * kappa), self.curve_min_v, self.v_max)

    def speed_p(self, v_t):
        """단순 P 속도 제어기: 목표속도 대비 현재속도 오차로 가감속 결정."""
        u = self.kp_speed * (v_t - self.vx)
        return (clamp(u, 0.0, self.a_max), 0.0) if u >= 0 else (0.0, clamp(-u, 0.0, self.b_max))

    # -----------------------
    # 주기 실행
    # -----------------------
    def step(self):
        """센서 입력 확인 후 CtrlCmd 퍼블리시."""
        now = rospy.Time.now()
        timeout = rospy.Duration(self.timeout_sec)
        have_cam = (now - self.last_cam) < timeout and (self.cam_img is not None)

        cmd = CtrlCmd()
        cmd.longlCmdType = 1  # Morai: 1=가감속/브레이크/조향 직접제어

        # 카메라 타임아웃 시 안전 감속/유지
        if not have_cam:
            cmd.accel = 0.0
            cmd.brake = 0.3 if self.vx > self.v_min else 0.0
            cmd.steering = 0.0
            self.pub_cmd.publish(cmd)
            rospy.logwarn_throttle(1.0, 'my_driver2: camera timeout (%s)',
                                   str((now - self.last_cam).to_sec()))
            return

        # 인지
        e, psi, kappa = self.fit_lane_center(self.cam_img)
        stopline = self.detect_stopline(self.cam_img)
        tl = self.detect_tl(self.cam_img)

        # 신호등 대응 + 추종 제어
        if tl in (TL.RED, TL.YELLOW):
            v_t = 0.0 if stopline else max(2.0, self.v_min)
            accel, brake = self.speed_p(v_t)
            cmd.accel, cmd.brake = accel, max(brake, 0.4 if v_t == 0.0 else brake)
            cmd.steering = self.stanley(e, psi)
        else:
            v_t = self.v_from_curvature(kappa)
            cmd.accel, cmd.brake = self.speed_p(v_t)
            cmd.steering = self.stanley(e, psi)

        # 모라이 필드(사용 안 함) 초기화
        cmd.velocity = 0.0
        cmd.acceleration = 0.0
        self.pub_cmd.publish(cmd)

# -----------------------
# 엔트리포인트
# -----------------------
def main():
    rospy.init_node('my_driver2', anonymous=True)
    node = MyDriver2()
    # 35Hz 루프 (너무 높으면 CPU 부담/타임아웃 민감도↑)
    rate = rospy.Rate(35)
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
