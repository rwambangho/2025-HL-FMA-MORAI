#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GetTrafficLightStatus
from morai_msgs.srv import MoraiEventCmdSrv
from morai_msgs.msg import EventInfo
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2


class MissionDrive:
    def __init__(self):
        rospy.init_node("mission_drive", anonymous=True)

        # --- Publishers ---
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # --- Subscribers ---
        rospy.Subscriber("/lane_path", Path, self.lane_path_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/cluster_points", PointCloud2, self.cluster_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_cb)

        # --- Control Command ---
        self.ctrl_cmd = CtrlCmd()
        self.ctrl_cmd.longlCmdType = 1  # accel/brake 제어 모드

        # --- State Variables ---
        self.lane_path = None
        self.local_path = None
        self.vehicle_pos = Point()
        self.vehicle_yaw = 0.0
        self.current_vel = 0.0
        self.clusters = []
        self.traffic_status = -1

        # --- Parameters ---
        self.vehicle_length = 2.6
        self.lfd_gain = 0.8
        self.min_lfd = 5
        self.max_lfd = 30
        self.max_speed = 40 / 3.6  # 40 km/h 제한
        self.base_margin = 3.0     # 제동 마진
        self.max_decel = 4.0       # 최대 제동능력 (m/s^2)
        self.lane_half_width = 2.0 # 차선 중심에서 좌우 범위
        self.clear_count = 0       # 장애물 클리어 프레임 카운트
        self.clear_threshold = 5   # N프레임 연속 clear 시 재출발

        # --- 초기 Service Call (기어 D, 자동 제어 모드) ---
        self.set_event_cmd()

        rospy.loginfo("✅ MissionDrive Node Started")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    # ----------------- 초기 Service Call -----------------
    def set_event_cmd(self):
        rospy.wait_for_service("/Service_MoraiEventCmd")
        try:
            event_client = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
            req = EventInfo()
            req.option = 3  # ctrl_mode + gear
            req.ctrl_mode = 3  # automode
            req.gear = 4       # D gear
            resp = event_client(req)
            rospy.loginfo("✅ Event Command Sent: ctrl_mode=3 (automode), gear=4 (D)")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call MoraiEventCmdSrv: {e}")

    # ----------------- Callbacks -----------------
    def lane_path_callback(self, msg):
        self.lane_path = msg

    def local_path_callback(self, msg):
        self.local_path = msg

    def odom_callback(self, msg):
        self.vehicle_pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.vehicle_yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

    def status_callback(self, msg):
        self.current_vel = msg.velocity.x

    def cluster_callback(self, msg):
        self.clusters = [(p[0], p[1], p[2]) for p in pc2.read_points(msg, skip_nans=True)]

    def traffic_cb(self, msg):
        self.traffic_status = msg.trafficLightStatus

    # ----------------- Main Logic -----------------
    def run(self):
        
        if not self.local_path or len(self.local_path.poses) == 0:
        rospy.logwarn_throttle(1.0, "[MissionDrive] No local path available")
        return

        # (1) 기본 경로 추종 조향각 (local_path 기준)
        steering = self.calc_pure_pursuit(self.local_path)

        # (2) lane_path가 있을 경우, 차선 중심 보정
        if self.lane_path and len(self.lane_path.poses) > 0:
            # 현재 차량 위치 기준으로 lane_path의 중심선과 local_path의 차이를 측정
            offset = self.compute_lane_offset(self.local_path, self.lane_path)

        # 보정 임계값 초과 시, 조향각에 가중치 적용
        if abs(offset) > 0.5:  # 예: 0.5m 이상
            steering += 0.1 * offset  # 조정 계수
            rospy.loginfo_throttle(1.0, f"[Lane Assist] Offset {offset:.2f} → Steering adjusted")

        self.ctrl_cmd.steering = steering

        # 기본 목표 속도
        target_speed = self.max_speed

        # (1) 전방 커브 구간 감지 → 감속
        curvature = self.estimate_path_curvature(path)
        if curvature > self.curvature_threshold:
            target_speed = min(target_speed, 10.0)  # 커브 시 속도 감속
            rospy.loginfo("🚧 Curve detected: Slowing down")

        # (2) 전방 신호등 거리 계산 → 감속
        if self.traffic_status in [1, 4]:  # 적색 or 황색
            dist_to_signal = self.estimate_distance_to_signal(path)
            if dist_to_signal < 30.0:  # 신호등까지 30m 이내
                target_speed = min(target_speed, 10.0)
                rospy.loginfo(f"🚦 Signal ahead ({dist_to_signal:.1f}m): Slowing down")

        # # (3) 장애물 감지 → 정지
        # if self.check_obstacle_ahead(path):
        #     target_speed = 0.0
        #     rospy.logwarn("🚧 Obstacle ahead: Stopping")

        # 최종 발행
        self.ctrl_pub.publish(self.ctrl_cmd)

    def compute_lane_offset(self, local_path, lane_path):
    """
    local_path와 lane_path의 시작점 차이를 이용해 lateral offset 계산
    """
    try:
        lx = local_path.poses[0].pose.position.x
        ly = local_path.poses[0].pose.position.y
        cx = lane_path.poses[0].pose.position.x
        cy = lane_path.poses[0].pose.position.y

        return cy - ly  # lateral offset
    except:
        return 0.0

def estimate_path_curvature(self, path, lookahead=10):
    """
    앞쪽 N개의 포인트를 기반으로 경로의 곡률을 추정
    """
    if len(path.poses) < lookahead:
        return 0.0

    angles = []
    for i in range(1, lookahead):
        p1 = path.poses[i-1].pose.position
        p2 = path.poses[i].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        angle = math.atan2(dy, dx)
        angles.append(angle)

    deltas = [abs(angles[i+1] - angles[i]) for i in range(len(angles)-1)]
    return sum(deltas) / len(deltas)  # 평균 조향각 변화량

def estimate_distance_to_signal(self, path):
    """
    현재 차량 위치 기준으로 경로 상 신호등까지의 거리 추정
    (조건: 신호등 좌표 또는 위치 인덱스가 명확히 지정되어 있어야 함)
    """
    if not path or not path.poses:
        return float('inf')

    # 예시: 신호등 위치 (수동 설정 or 별도 노드로 계산)
    signal_x, signal_y = self.traffic_light_pos.x, self.traffic_light_pos.y

    # 현재 위치로부터의 거리 계산
    dx = signal_x - self.vehicle_pos.x
    dy = signal_y - self.vehicle_pos.y
    return math.hypot(dx, dy)


    # ----------------- Pure Pursuit -----------------
    def calc_pure_pursuit(self, path):
        if not path.poses:
            return None

        lfd = max(self.min_lfd,
                  min(self.max_lfd, self.current_vel * self.lfd_gain + self.min_lfd))

        # 좌표변환 행렬
        trans_matrix = np.array([
            [math.cos(self.vehicle_yaw), -math.sin(self.vehicle_yaw), self.vehicle_pos.x],
            [math.sin(self.vehicle_yaw),  math.cos(self.vehicle_yaw), self.vehicle_pos.y],
            [0, 0, 1]
        ])
        det_trans_matrix = np.linalg.inv(trans_matrix)

        forward_point = None
        for pose in path.poses:
            px, py = pose.pose.position.x, pose.pose.position.y
            local = det_trans_matrix.dot([px, py, 1.0])
            if local[0] > 0:
                dist = math.sqrt(local[0]**2 + local[1]**2)
                if dist >= lfd:
                    forward_point = local
                    break

        if forward_point is None:
            return None

        theta = math.atan2(forward_point[1], forward_point[0])
        steering = math.atan2(2 * self.vehicle_length * math.sin(theta), lfd)
        return steering


    # ----------------- Obstacle Check -----------------
    def check_obstacle_ahead(self, path):
        """
        경로 전방 safe_distance 이내 + 차선폭 이내에 장애물이 있으면 True
        """
        if not self.clusters:
            self.clear_count += 1
            return False

        # 속도 기반 동적 안전거리 계산
        v = max(0.0, self.current_vel)  # m/s
        safe_distance = (v * v) / (2 * self.max_decel) + self.base_margin

        for ox, oy, _ in self.clusters:
            for pose in path.poses[:50]:  # 전방 일정 구간만 확인
                px, py = pose.pose.position.x, pose.pose.position.y
                dx = ox - px
                dy = oy - py
                dist = math.sqrt(dx*dx + dy*dy)

                # 경로 방향 벡터 기준 lateral offset 계산
                yaw = self.vehicle_yaw
                lateral = abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)

                if dist < safe_distance and lateral < self.lane_half_width:
                    self.clear_count = 0
                    return True

        self.clear_count += 1
        return False

    # ----------------- Utils -----------------
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


if __name__ == "__main__":
    try:
        MissionDrive()
    except rospy.ROSInterruptException:
        pass
