#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Imu
from morai_msgs.msg import CtrlCmd, GPSMessage

# --------- 1D Kalman Filter for lane center smoothing ----------
class KalmanFilter1D:
    def __init__(self, process_noise=1e-2, measurement_noise=5.0, error_estimate=1.0):
        self.x = 0.0   # state (lane center x in px)
        self.P = error_estimate
        self.Q = process_noise
        self.R = measurement_noise

    def update(self, z):
        # Predict
        self.P = self.P + self.Q
        # Kalman gain
        K = self.P / (self.P + self.R)
        # Correct
        self.x = self.x + K * (z - self.x)
        self.P = (1.0 - K) * self.P
        return self.x

class TestDrive:
    def __init__(self):
        # --- IO ---
        self.pub_cmd = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cb_cam, queue_size=1)
        rospy.Subscriber("/imu", Imu, self.cb_imu, queue_size=10)
        rospy.Subscriber("/gps", GPSMessage, self.cb_gps, queue_size=10)

        # --- Vehicle Params ---
        self.v_target   = 20.0    # m/s
        self.steer_gain = 0.003   # steering proportional gain

        # --- Vision params (user-edited values kept) ---
        self.roi_top_ratio = 0.5          # use bottom half
        self.blur_ksize     = (5, 5)      # Gaussian blur
        self.canny_th1_th2  = (60, 75)    # Canny thresholds  (kept)
        self.hough_params   = dict(rho=1, theta=np.pi/180, threshold=50,
                                   minLineLength=15, maxLineGap=40)  # kept

        # HSV masks (white & yellow lanes)
        self.white_low  = np.array([0,   0, 200], np.uint8)
        self.white_high = np.array([180, 40, 255], np.uint8)
        self.yellow_low  = np.array([15, 100,  100], np.uint8)
        self.yellow_high = np.array([35, 255, 255], np.uint8)

        # --- State ---
        self.img = None
        self.kf  = KalmanFilter1D(process_noise=1e-2, measurement_noise=5.0, error_estimate=1.0)
        self.last_steer = 0.0
        self.steer_alpha = 0.7  # low-pass for steering output

        rospy.loginfo("✅ TestDrive Node Ready (Single Cam + HSV + Kalman + your thresholds)")

    # -------------------- Callbacks --------------------
    def cb_cam(self, msg):
        self.img = self.decode(msg)
        self.process_lane()

    def cb_imu(self, msg):  # reserved
        pass

    def cb_gps(self, msg):  # reserved
        pass

    @staticmethod
    def decode(msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # -------------------- Core pipeline --------------------
    def process_lane(self):
        if self.img is None:
            return

        h, w = self.img.shape[:2]
        y0 = int(h * self.roi_top_ratio)
        roi = self.img[int(h*0.6):h, :] 

        # HSV → white/yellow mask
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_white  = cv2.inRange(hsv, self.white_low,  self.white_high)
        mask_yellow = cv2.inRange(hsv, self.yellow_low, self.yellow_high)
        mask = cv2.bitwise_or(mask_white, mask_yellow)

        # Blur (kept) + Canny (kept)
        blur = cv2.GaussianBlur(mask, self.blur_ksize, 0)
        edges = cv2.Canny(blur, *self.canny_th1_th2)

        # Hough (kept minLineLength=20)
        lines = cv2.HoughLinesP(edges, **self.hough_params)
        steer_cmd = 0.0
        steer_sign = rospy.get_param('~ctrl/steer_sign', 0)  # << A: 조향 부호 파라미터 (-1 권장)

        if lines is not None:
            h_roi, w_roi = roi.shape[:2]
            x_mid = w // 2

            left_segs, right_segs = [], []
            for l in lines[:,0]:
                x1, y1, x2, y2 = l[0]
                # 그려보기(디버그)
                cv2.line(roi, (x1, y1), (x2, y2), (60, 200, 60), 2)

                dx = (x2 - x1)
                dy = (y2 - y1)
                if abs(dx) < 3:  # 세로에 가까운 선은 기울기 무한대 → 방향 판단 어려움
                    continue
                slope = dy / float(dx + 1e-6)

                # 기울기로 좌/우 분리 (카메라 좌표: 아래로 y+, 오른쪽으로 x+ 가정)
                if slope < -0.3:
                    left_segs.append((x1, y1, x2, y2))
                elif slope > 0.3:
                    right_segs.append((x1, y1, x2, y2))

            def fit_rep(lines_list, color):
                if not lines_list:
                    return None
                pts = []
                for (x1, y1, x2, y2) in lines_list:
                    pts.append([x1, y1]); pts.append([x2, y2])
                pts = np.array(pts, dtype=np.int32)
                [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)
                yA = int(0.9*h_roi); yB = int(0.6*h_roi)
                # ROI 좌표계이지만 x_mid는 전체 프레임 기준 → x는 그대로 써도 중심 비교엔 무방
                xA = int(x0 + (yA - y0) * (vx / (vy + 1e-6)))
                xB = int(x0 + (yB - y0) * (vx / (vy + 1e-6)))
                # 대표선 그리기
                cv2.line(roi, (xA, yA), (xB, yB), color, 3)
                return (xA, yA, xB, yB)

            L = fit_rep(left_segs,  (0, 255, 255))  # 노랑
            R = fit_rep(right_segs, (0, 150, 255))  # 주황

            # B: 좌/우 기반 중심 계산
            if L and R:
                lane_center = (L[0] + R[0]) // 2
            elif L and not R:
                # 좌측만 보이면 차폭(px)만큼 오른쪽으로 보정
                lane_px = int(w * 0.22)  # 필요시 0.18~0.26 조정
                lane_center = L[0] + lane_px
            elif R and not L:
                lane_px = int(w * 0.22)
                lane_center = R[0] - lane_px
            else:
                lane_center = None  # 둘 다 없음 → 조향 유지/감속 등 보수적으로

            if lane_center is not None:
                # 칼만 필터 보정
                lane_center_filt = int(self.kf.update(lane_center))

                # 중앙 오차(+면 화면 왼쪽에 중심) → 조향 부호 적용
                err_px = (x_mid - lane_center_filt)
                raw_steer = steer_sign * err_px * self.steer_gain

                # 부드럽게 (저역통과) 
                steer_cmd = self.steer_alpha * self.last_steer + (1 - self.steer_alpha) * raw_steer
                self.last_steer = steer_cmd

                # 디버그 오버레이
                cy = roi.shape[0] // 2
                cv2.circle(roi, (lane_center_filt, cy), 6, (0, 0, 255), -1)      # 추정 중심(빨강)
                cv2.line(roi, (x_mid, cy), (lane_center_filt, cy), (255, 0, 0), 2)  # 중심-오차(파랑)
                cv2.line(roi, (x_mid, 0), (x_mid, h_roi), (255, 0, 0), 1)

                rospy.loginfo_throttle(1.0, f"[lane] err_px={err_px:.1f}, steer={steer_cmd:.4f}, sign={steer_sign}")
            else:
                # 차선 사라지면 마지막 조향 천천히 감쇠
                self.last_steer *= 0.9
                steer_cmd = self.last_steer

        # publish command
        cmd = CtrlCmd()
        cmd.longlCmdType = 2   # velocity mode
        cmd.velocity = self.v_target
        cmd.steering = float(steer_cmd)
        self.pub_cmd.publish(cmd)

        # debug view
        cv2.imshow("Lane (ROI)", roi)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("test_drive")
    TestDrive()
    rospy.spin()
