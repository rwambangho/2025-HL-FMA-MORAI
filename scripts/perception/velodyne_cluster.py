#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from sklearn.cluster import DBSCAN

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/lidar3D", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.8, min_samples=4) #eps는 차량 너비(2m)의 절반 정도를 기준으로 시작 (0.8 ~ 1.0)

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        pc_xy = self.pc_np[:, :2]
        db = self.dbscan.fit_predict(pc_xy)
        n_cluster = np.max(db) + 1

        cluster_points = []
        for c in range(n_cluster):
            c_tmp = np.mean(pc_xy[db==c, :], axis=0)
            cluster_points.append([c_tmp[0], c_tmp[1], 1])  # Adding Z coordinate as 1

             # 📌 클러스터 중심 좌표 출력
            rospy.loginfo(f"[Cluster] {c}: X={c_tmp[0]:.2f}, Y={c_tmp[1]:.2f}")

        self.publish_point_cloud(cluster_points)

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points)

        # Publish PointCloud2 message
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if ( #자차 바디가 클러스터로 인식/도로 옆 벽, 난간 등이 인식/멀리 있는 큰 물체들이 전체 클러스터로 묶임 방지
                point[0] > 1.5 and  # 차량 앞 1.5m 이상
                abs(point[1]) < 3.0 and  # 좌우 3m 내
                1.5 > point[2] > -1.25 and  # 높이 필터
                dist < 50
            ):
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin() 