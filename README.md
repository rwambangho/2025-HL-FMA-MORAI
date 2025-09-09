## 대회 개요
**주행경로**

<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/b6b90f6d-a36d-4098-9637-afcc089d929d" />

**차량모델 및 센서**

- Model : 2023_Hyundai_ioniq5

<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/2358bbea-1022-4106-bd51-3b716d7c6f66" />

- Steer Angle
  - Minimum Turning Radius(m) : 5.87
  - Max Wheel Angle(deg) : 40
- Exterior Dimensions
  - Length(m) : 4.635
  - Width(m) : 1.892
  - Height(m) : 2.434
  - Wheelbase(m) : 3.000
  - Front Overhang(m) : 0.845
  - Rear Overhang(m) : 0.79
- GPS
  - 개수 : 1대
  - 네트워크 : ROS
- IMU
  - 개수 : 1대
  - 네트워크 : ROS
- 3D LIDAR
  - 개수 : 1대
  - Model : VLP16
  - Intensity type : Intensity
  - 네트워크 : UDP
- Camera
  - 개수 : 1대
  - Model : Camera
  - Ground Truth : None
  - Viz Bounding Box(2D/3D) : x
  - 해상도 : 640*480
  - 네트워크 : ROS


## 시스템 구조

<img width="593" height="571" alt="morai drawio (1)" src="https://github.com/user-attachments/assets/b6035681-28b5-46ac-8753-a3197d430ab1" />


`$rqt_graph`

<img width="1300" height="690" alt="rosgraph" src="https://github.com/user-attachments/assets/dbdffc12-f6f3-4903-a2fd-4a4145ec25ee" />

## 주요 코드
**global_path_pub**
path.txt를 기반으로 맵 내 전역경로 생성

**local_path_pub**
현재 위치 기반의 로컬경로 생성

**gps_imu_parser**
GPS+IMU 데이터를 /odom으로 변환

**velodyne_cluster**
3D LiDAR 포인트를 클러스터링하여 장애물 추정

**lane_fitting**
카메라에서 차선을 검출 후 차선 중심을 경로로 변환

**mission_drive**
pid + pure pursuit 기반 주행 및 주행 미션 요구사항 로직

## 결과

<img src="https://github.com/rwambangho/2025-HL-FMA-MORAI/blob/main/morai%20drive.gif"> </img>

## 느낀점

아쉽게도 예선전은 탈락했으나 할 수 있는데까지 완성시켜보자는 생각에 완벽하지는 않지만 조금이나마 보완해보면서 **ROS환경의 통신 메커니즘을 잘 이해할 수 있었고** 여러 센서 노드와 주행 제어 노드들이 연결된 구조에서, 정상 주행 및 장애 상황을 반복 테스트하며 전체 시스템을 디버깅 하는 과정에서 작게나마 **SIL 수준의 통합 테스트를 경험할 수 있었습니다.** 또한 모듈화된 구조 설계를 통해 모듈단위로 먼저 테스트하고 통합테스트로 이어지게 하여 기능별 문제를 빠르게 발견하고 해결할 수 있었습니다. 추후 ISO26262 또는 A-SPICE 기반 표준으로 테스트 시나리오를 개발하고 단위 테스트하는 방향으로 프로젝트를 보완해보고자 합니다.


