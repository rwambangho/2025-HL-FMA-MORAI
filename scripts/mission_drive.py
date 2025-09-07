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
        # 경로 선택 (lane_path 우선, 없으면 local_path)
        path = self.lane_path if self.lane_path and len(self.lane_path.poses) > 0 else self.local_path
        if not path or not path.poses:
            return

        # Pure Pursuit 조향각
        steering = self.calc_pure_pursuit(path)
        self.ctrl_cmd.steering = steering if steering is not None else 0.0

        # 목표 속도
        target_vel = self.max_speed

        # (1) 신호등 상태 반영
        if self.traffic_status == 1:   # Red
            target_vel = 0.0
            rospy.loginfo("🔴 Red Light: Stopping")
        elif self.traffic_status == 4: # Yellow
            target_vel = min(target_vel, 10/3.6)
            rospy.loginfo("🟡 Yellow Light: Slowing")
        elif self.traffic_status in (16, 32): # Green
            pass  # 그대로 진행

        # (2) 장애물 감지
        if self.check_obstacle_ahead(path):
            target_vel = 0.0
            rospy.logwarn("🚧 Obstacle ahead: Stopping")
        else:
            # N프레임 연속 clear 시에만 재출발 허용
            if self.clear_count > self.clear_threshold:
                target_vel = self.max_speed

        # (3) 속도 제어 (P제어 기반 accel/brake)
        error = target_vel - self.current_vel
        accel_cmd = 0.5 * error
        if accel_cmd > 0:
            self.ctrl_cmd.accel = min(accel_cmd, 1.0)
            self.ctrl_cmd.brake = 0.0
        else:
            self.ctrl_cmd.accel = 0.0
            self.ctrl_cmd.brake = min(-accel_cmd, 1.0)

        # 최종 발행
        self.ctrl_pub.publish(self.ctrl_cmd)

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
