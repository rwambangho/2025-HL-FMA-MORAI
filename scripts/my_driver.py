#!/usr/bin/env python
# -*- coding: utf-8 -*- 10
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math
from sensor_msgs.msg import CompressedImage, Imu
from morai_msgs.msg import CtrlCmd, GPSMessage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import importlib.util

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 17  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = CtrlCmd()  # 카메라 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
stopline_num = 1 # 정지선 발견때마다 1씩 증가
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치

#=============================================
# 차선 인식 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30  # 카메라 FPS 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ROI_START_ROW = 300  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 380  # 차선을 찾을 ROT 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 40  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 

#=============================================
# 콜백함수 - USB 전방카메라 토픽을 받아서 처리하는 콜백함수.
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수.  
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수.  
# 입력으로 시간(초)를 받아 그 시간동안 속도=0 토픽을 모터로 보냄
#=============================================
def stop_car(sleep_sec):
    for i in range(sleep_sec*10): 
        drive(angle=new_angle, speed=0)
        time.sleep(0.1)
    
#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 핸들 조정함.
#=============================================
def sonic_drive():
    global new_angle, new_speed

    # 앞쪽 가까이에 장애물이 있으면 차량 멈춤
    if (0 < ultra_msg[2] < 5):
        new_angle = new_angle
        new_speed = 0
        print("Car Brake, Stop! : ", ultra_msg)

    # 왼쪽이 오른쪽보다 멀리 있으면 있으면 좌회전 주행
    elif (ultra_msg[1] > ultra_msg[3]):
        new_angle = -75
        new_speed = Fix_Speed
        print("Turn left : ", ultra_msg)
        
    # 오른쪽이 왼쪽보다 멀리 있으면 있으면 우회전 주행
    elif (ultra_msg[1] < ultra_msg[3]):
        new_angle = 75
        new_speed = Fix_Speed
        print("Turn right : ", ultra_msg)

    # 위 조건에 해당하지 않는 경우라면 (오른쪽과 왼쪽이 동일한 경우) 똑바로 직진 주행
    else:
        new_angle = 0
        new_speed = Fix_Speed
        print("Go Straight : ", ultra_msg)

    # 모터에 주행명령 토픽을 보낸다
    drive(new_angle, new_speed)

#=============================================
# 카메라 이미지를 영상처리하여 
# 정지선이 있는지 체크하고 True 또는 False 값을 반환.
#=============================================
def check_stopline():
    global stopline_num

    # 원본 영상을 화면에 표시
    #cv2.imshow("Original Image", image)
    
    # image(원본이미지)의 특정영역(ROI Area)을 잘라내기
    roi_img = CompressedImage[300:480, 0:640]
    cv2.imshow("ROI Image", roi_img)

    # HSV 포맷으로 변환하고 V채널에 대해 범위를 정해서 흑백이진화 이미지로 변환
    hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV) 
    upper_white = np.array([255, 255, 255])
    lower_white = np.array([0, 0, 180])
    binary_img = cv2.inRange(hsv_image, lower_white, upper_white)
    #cv2.imshow("Black&White Binary Image", binary_img)

    # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
    stopline_check_img = binary_img[100:120, 200:440] 
    #cv2.imshow("Stopline Check Image", stopline_check_img)
    
    # 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
    img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(img, (200,100),(440,120),Green,3)
    cv2.imshow('Stopline Check', img)
    cv2.waitKey(1)
    
    # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
    stopline_count = cv2.countNonZero(stopline_check_img)
    
    # 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
    if stopline_count > 2500:
        print("Stopline Found...! -", stopline_num)
        stopline_num = stopline_num + 1
        cv2.destroyWindow("ROI Image")
        return True
    
    else:
        return False
        
#=============================================
# 카메라 이미지를 영상처리하여 
# 신호등의 출발 신호를 체크해서 True 또는 False 값을 반환.
#=============================================
def check_traffic_sign():
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    cimg = CompressedImage.copy()
    Center_X, Center_Y = 500, 100  # ROI 영역의 중심위치 좌표 
    XX, YY = 140, 100  # 위 중심위치 좌표에서 좌우로 XX만큼씩, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄   

    # ROI 영역을 녹색 사각형으로 그려서 화면에 표시함 
    cv2.rectangle(cimg, (Center_X-XX, Center_Y-YY), (Center_X+XX, Center_Y+YY) , Green, 2)
	
	# 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
    roi_img = cimg[Center_Y-YY:Center_Y+YY, Center_X-XX:Center_X+XX]

    # 칼라 이미지를 회색 이미지로 바꿈  
    img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)

    # 노이즈 제거를 위해 블러링 처리를 함 
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Hough Circle 변환을 이용해서 이미지에서 원을 (여러개) 찾음 
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=40, param2=20, minRadius=15, maxRadius=25)

    # 디버깅을 위해서 Canny 처리를 했을때의 모습을 화면에 표시함
	# 위 HoughCircles에서 param1, param2에 사용했던 값을 아래 canny에서 똑같이 적용해야 함. 순서 조심.
    canny = cv2.Canny(blur, 20, 40)
    cv2.imshow('Area for HoughCircles', canny)

    # 이미지에서 원이 발견됐다면 아래 if 안으로 들어가서 신호등 찾는 작업을 진행함  
    if circles is not None:
    
        # 원 중심의 X좌표값으로 소팅 - 화면의 왼쪽부터 순서대로 다시 정리 
        circles = np.round(circles[0, :]).astype("int")
        circles = sorted(circles, key=lambda circle: circle[0])

        # 가장 밝은 원을 찾을 때 사용할 변수 선언 
        max_mean_value = 0
        max_mean_value_circle = None
        max_mean_value_index = None

        # 발견된 원들에 대해서 루프를 돌면서 하나씩 처리 
 	    # 원의 중심좌표, 반지름. 내부밝기 정보를 구해서 화면에 출력  
        for i, (x, y, r) in enumerate(circles):
            x1 = x - r // 2
            y1 = y - r // 2
            x2 = x + r // 2
            y2 = y + r // 2
            roi = img[y1:y2, x1:x2]
            mean_value = np.mean(roi)
            print(f"Circle {i} at ({x},{y}), radius={r}: mean value={mean_value}")
			
            # 여기에 발견된 원들 중에서 가장 밝은 원을 찾는 코드가 추가되어야 함 
            if mean_value > max_mean_value:
                max_mean_value = mean_value
                max_mean_value_circle = (x, y, r)
                max_mean_value_index = i

            # 찾은 원을 녹색으로 그리고, 원 안에 작은 빨간색 사각형(밝기 정보를 계산할 영역 표시)을 그림 
            cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Green, 2)
            cv2.rectangle(cimg, (x1+Center_X-XX, y1+Center_Y-YY), (x2+Center_X-XX, y2+Center_Y-YY), Red, 2)

        # 가장 밝은 원을 찾았으면 그 원의 정보를 화면에 출력 
        if max_mean_value_circle is not None:
            (x, y, r) = max_mean_value_circle
            print(f" --- Circle {max_mean_value_index} has the biggest mean value")

        # 신호등 찾기 결과가 그림으로 표시된 이미지를 화면에 출력
        cv2.imshow('Circles Detected', cimg)
        cv2.waitKey(1000)
    
	    # 찾은 원 중에서 오른쪽 3번째 원이 가장 밝으면 (파란색 신호등) True 리턴 
        if (i == 2) and (max_mean_value_index == 2):
            print("Traffic Sign is Blue...!")
            cv2.destroyWindow('Area for HoughCircles')
            return True
        
		# 그렇지 않으면 (파란색 신호등이 아니면) False 반환 
        else:
            print("Traffic Sign is NOT Blue...!")
            return False

    # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
    print("Can't find Traffic Sign...!")
    return False

#=============================================
# 카메라 영상 이미지에서 차선을 찾아 그 위치를 반환하는 코드
#=============================================
def lane_detect():

    global image
    prev_x_left = 0
    prev_x_right = WIDTH

    img = CompressedImage.copy() # 이미지처리를 위한 카메라 원본이미지 저장
    display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장
    
    # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
    roi_img = img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
    line_draw_img = roi_img.copy()

    #=========================================
    # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
    # 블러링 처리를 통해 노이즈를 제거한 후에 (약간 뿌옇게, 부드럽게)
    # Canny 변환을 통해 외곽선 이미지로 만들기
    #=========================================
    gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)

    # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
    all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,50,50,20)
    
    if all_lines is None:
        return False, 0, 0

    #=========================================
    # 선분들의 기울기 값을 각각 모두 구한 후에 리스트에 담음. 
    # 기울기의 절대값이 너무 작은 경우 (수평선에 가까운 경우)
    # 해당 선분을 빼고 담음. 
    #=========================================
    slopes = []
    filtered_lines = []

    for line in all_lines:
        x1, y1, x2, y2 = line[0]

        if (x2 == x1):
            slope = 1000.0
        else:
            slope = float(y2-y1) / float(x2-x1)
    
        if 0.2 < abs(slope):
            slopes.append(slope)
            filtered_lines.append(line[0])

    if len(filtered_lines) == 0:
        return False, 0, 0

    #=========================================
    # 왼쪽 차선에 해당하는 선분과 오른쪽 차선에 해당하는 선분을 구분하여 
    # 각각 별도의 리스트에 담음.
    #=========================================
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = filtered_lines[j]
        slope = slopes[j]

        x1,y1, x2,y2 = Line

        # 기울기 값이 음수이고 화면의 왼쪽에 있으면 왼쪽 차선으로 분류함
        # 기준이 되는 X좌표값 = (화면중심값 - Margin값)
        Margin = 0
        
        if (slope < 0) and (x2 < WIDTH/2-Margin):
            left_lines.append(Line.tolist())

        # 기울기 값이 양수이고 화면의 오른쪽에 있으면 오른쪽 차선으로 분류함
        # 기준이 되는 X좌표값 = (화면중심값 + Margin값)
        elif (slope > 0) and (x1 > WIDTH/2+Margin):
            right_lines.append(Line.tolist())

    # 디버깅을 위해 차선과 관련된 직선과 선분을 그리기 위한 도화지 준비
    line_draw_img = roi_img.copy()
    
    # 왼쪽 차선에 해당하는 선분은 빨간색으로 표시
    for line in left_lines:
        x1,y1, x2,y2 = line
        cv2.line(line_draw_img, (x1,y1), (x2,y2), Red, 2)

    # 오른쪽 차선에 해당하는 선분은 노란색으로 표시
    for line in right_lines:
        x1,y1, x2,y2 = line
        cv2.line(line_draw_img, (x1,y1), (x2,y2), Yellow, 2)

    #=========================================
    # 왼쪽/오른쪽 차선에 해당하는 선분들의 데이터를 적절히 처리해서 
    # 왼쪽차선의 대표직선과 오른쪽차선의 대표직선을 각각 구함.
    # 기울기와 Y절편값으로 표현되는 아래와 같은 직선의 방적식을 사용함.
    # (직선의 방정식) y = mx + b (m은 기울기, b는 Y절편)
    #=========================================

    # 왼쪽 차선을 표시하는 대표직선을 구함        
    m_left, b_left = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    # 왼쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
    size = len(left_lines)
    if size != 0:
        for line in left_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0                
            
        x_avg = x_sum / (size*2)
        y_avg = y_sum / (size*2)
        m_left = m_sum / size
        b_left = y_avg - m_left * x_avg

        if m_left != 0.0:
            #=========================================
            # (직선 #1) y = mx + b 
            # (직선 #2) y = 0
            # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
            x1 = int((0.0 - b_left) / m_left)

            #=========================================
            # (직선 #1) y = mx + b 
            # (직선 #2) y = ROI_HEIGHT
            # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
            x2 = int((ROI_HEIGHT - b_left) / m_left)

            # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
            cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), Blue, 2)

    # 오른쪽 차선을 표시하는 대표직선을 구함      
    m_right, b_right = 0.0, 0.0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

    # 오른쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
    size = len(right_lines)
    if size != 0:
        for line in right_lines:
            x1, y1, x2, y2 = line
            x_sum += x1 + x2
            y_sum += y1 + y2
            if(x2 != x1):
                m_sum += float(y2-y1)/float(x2-x1)
            else:
                m_sum += 0     
       
        x_avg = x_sum / (size*2)
        y_avg = y_sum / (size*2)
        m_right = m_sum / size
        b_right = y_avg - m_right * x_avg

        if m_right != 0.0:
            #=========================================
            # (직선 #1) y = mx + b 
            # (직선 #2) y = 0
            # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
            x1 = int((0.0 - b_right) / m_right)

            #=========================================
            # (직선 #1) y = mx + b 
            # (직선 #2) y = ROI_HEIGHT
            # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
            x2 = int((ROI_HEIGHT - b_right) / m_right)

            # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
            cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), Blue, 2)

    #=========================================
    # 차선의 위치를 찾기 위한 기준선(수평선)은 아래와 같음.
    #   (직선의 방정식) y = L_ROW 
    # 위에서 구한 2개의 대표직선, 
    #   (직선의 방정식) y = (m_left)x + (b_left)
    #   (직선의 방정식) y = (m_right)x + (b_right)
    # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
    #=========================================

    #=========================================        
    # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
    # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
    #=========================================
    if m_left == 0.0:
        x_left = prev_x_left  # 변수에 저장해 놓았던 이전 값을 가져옴

    #=========================================
    # 아래 2개 직선의 교점을 구함
    # (직선의 방정식) y = L_ROW  
    # (직선의 방정식) y = (m_left)x + (b_left)
    #=========================================
    else:
        x_left = int((L_ROW - b_left) / m_left)
                        
    #=========================================
    # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
    # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
    #=========================================
    if m_right == 0.0:
        x_right = prev_x_right  # 변수에 저장해 놓았던 이전 값을 가져옴	
	
    #=========================================
    # 아래 2개 직선의 교점을 구함
    # (직선의 방정식) y = L_ROW  
    # (직선의 방정식) y = (m_right)x + (b_right)
    #=========================================
    else:
        x_right = int((L_ROW - b_right) / m_right)
       
    #=========================================
    # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
    # 이 경우에 반대쪽 차선의 위치 정보를 이용해서 내 위치값을 정함 
    #=========================================
    if m_left == 0.0 and m_right != 0.0:
        x_left = x_right - 380

    if m_left != 0.0 and m_right == 0.0:
        x_right = x_left + 380

    # 이번에 구한 값으로 예전 값을 업데이트 함			
    prev_x_left = x_left
    prev_x_right = x_right
	
    # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치를 구함
    x_midpoint = (x_left + x_right) // 2 

    #=========================================
    # 디버깅용 이미지 그리기
    # (1) 수평선 그리기 (직선의 방정식) y = L_ROW 
    # (2) 수평선과 왼쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
    # (3) 수평선과 오른쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
    # (4) 왼쪽 교점과 오른쪽 교점의 중점 위치에 작은 파란색 사각형 그리기
    # (5) 화면의 중앙점 위치에 작은 빨간색 사각형 그리기 
    #=========================================
    cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), Yellow, 2)
    cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), Green, 4)
    cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), Green, 4)
    cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), Blue, 4)
    cv2.rectangle(line_draw_img, (View_Center-5,L_ROW-5), (View_Center+5,L_ROW+5), Red, 4)

    # 위 이미지를 디버깅용 display_img에 overwrite해서 화면에 디스플레이 함
    display_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH] = line_draw_img
    cv2.imshow("Lanes positions", display_img)
    cv2.waitKey(1)

    return True, x_left, x_right

#=============================================
# AR 패지키지가 발행하는 토픽을 받아서 
# 제일 가까이 있는 AR Tag에 적힌 ID 값을 반환함.
# 추가로 거리값과 좌우치우침값을 함께 반환함.
#=============================================
def check_AR():
    global ar_msg

    if (len(ar_msg["ID"]) == 0):
        # 아직 AR 토픽이 없거나 발견된 AR태그가 없으면 리턴
        return 99, 10.0, 0.0, 0.0  # ID값은 99, 거리값은 10.0, Z위치값은 0.0, X위치값은 0.0으로 반환 

    # 새로 도착한 AR태그에 대해서 아래 작업 수행
    z_pos = 10.0  # Z위치값을 10미터로 초기화
    x_pos = 10.0  # X위치값을 10미터로 초기화
    
    for i in range(len(ar_msg["ID"])):
        # 발견된 AR태그 모두에 대해서 조사
        if(ar_msg["DZ"][i] < z_pos):
            # 더 가까운 거리에 AR태그가 있으면 그걸 사용
            id_value = ar_msg["ID"][i]
            z_pos = ar_msg["DZ"][i]
            x_pos = ar_msg["DX"][i]

    # ID번호, 거리값(미터), 좌우치우침값(미터) 리턴
    distance = math.sqrt(z_pos**2 + x_pos**2)  # 거리값은 피타고라스 정리로 계산 
    return id_value, round(distance,2), round(z_pos,2), round(x_pos,2)
        
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image, img_ready 
    global new_angle, new_speed
    
    SENSOR_DRIVE = 1
    TRAFFIC_SIGN = 2
    LANE_DRIVE = 3
    PARKING = 6
    FINISH = 9
	
    # 처음에 어떤 미션부터 수행할 것인지 여기서 결정한다. 
    drive_mode = TRAFFIC_SIGN
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('my_driver')
    motor = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    rospy.Subscriber("/image_jpeg/compressed",CompressedImage,usbcam_callback, queue_size=1)
    
    #=========================================
    # 첫번째 토픽이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/image_jpeg/compressed", CompressedImage)
    print("Camera Ready --------------")
    
    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")
	
    # 일단 차량이 움직이지 않도록 정지 상태로 만듭니다.  
    stop_car(1)
	
    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # ======================================
        # 출발선에서 신호등을 찾습니다. 
        # 일단 정차해 있다가 파란색 불이 켜지면 출발합니다.
        # 초음파센서 미로주행 SENSOR_DRIVE 모드로 넘어갑니다.  
        # ======================================
        while drive_mode == TRAFFIC_SIGN:
		
            # 앞에 있는 신호등에 파란색 불이 켜졌는지 체크합니다.  
            result = check_traffic_sign()
			
            if (result == True):
                # 만약 신호등이 파란색 불로 바뀌었으면 미로주행 SENSOR_DRIVE 모드로 넘어갑니다.
                drive_mode = SENSOR_DRIVE  
                print ("----- Ultrasonic driving Start... -----")
                #cv2.destroyAllWindows()
                
        # ======================================
        # 초음파 센서로 주행합니다.
        # 정지선이 보이면 차선인식주행 LANE_DRIVE 모드로 넘어갑니다. 
        # ======================================
        while drive_mode == SENSOR_DRIVE:

            # 초음파세센서를 이용해서 미로주행을 합니다. 
            sonic_drive()  

            # 바닥에 정지선이 있는지 체크합니다. 
            result = check_stopline()
             
            if (result == True):
                # 만약 정지선이 발견되면 차량을 정차시키고 LANE_DRIVE 모드로 넘거갑니다.
                stop_car(1)
                drive_mode = LANE_DRIVE  
                print ("----- Lane driving Start... -----")
                #cv2.destroyAllWindows()
                
        # ======================================
        # 차선을 보고 주행합니다.
        # AR태그가 발견되면 주차모드로 넘어갑니다.	 
        # ======================================
        while drive_mode == LANE_DRIVE:
		
            # 카메라 이미지에서 차선의 위치를 알아냅니다. 
            found, x_left, x_right = lane_detect()
			
            if found:
                # 차선인식이 됐으면 차선의 위치정보를 이용해서 핸들 조향각을 결정합니다. 
                x_midpoint = (x_left + x_right) // 2 
                new_angle = (x_midpoint - View_Center) // 3
                drive(new_angle, new_speed)
				
            else:
                # 차선인식이 안 됐으면 기존 핸들 각도를 그대로 유지한채로 주행합니다. 	
                drive(new_angle, new_speed)
            
            # 전방에 AR태그가 보이는지 체크합니다.             
            ar_ID, distance, z_pos, x_pos = check_AR()

            if (distance < 1.5) and (ar_ID != 99): 
                # 1.50미터 안에 AR태그가 있고 ID값이 '99'가 아니면 PARKING 모드로 넘어갑니다.  
                stop_car(3)
                drive_mode = PARKING  
                print ("----- PARKING Start... -----")
  
        # ======================================
        # AR 표지판을 보고 주차합니다. 
        # AR 표지판 바로 앞에 가면 주행종료 모드로 변경합니다.
        # ======================================
        while drive_mode == PARKING:
       
            # 전방에 AR태그가 보이는지 체크합니다.   
            ar_ID, distance, z_pos, x_pos = check_AR()
            print("Distance=",distance,"Z_pos=",z_pos," X_pos=",x_pos)

            if (ar_ID == 99):
                # AR태그가 안 보이면 AR태그를 계속 찾습니다.   
                continue
    
            else:
                # AR태그가 안 보이면 AR태그를 계속 찾습니다.   
                if (z_pos > 0.2):
                    # Z값이 20센치 이상이이면 핸들 각도를 조종하면서 주행합니다.  
                    new_angle = int(x_pos)
					
                else:
                    # Z값이 20센치 이하이면 주차영역에 들어온 것으로 하고 차량을 세우고 종료 FINISH 모드로 넘어갑니다.     
                    new_angle = 0
                    new_speed = 0                      
                    drive_mode = FINISH  
                    print ("----- Parking completed... -----")

            drive(new_angle, new_speed)

        # ======================================
        # 주행을 끝냅니다. 
        # ======================================
        if drive_mode == FINISH:
           
            # 차량을 정지시키고 모든 작업을 끝냅니다.
            stop_car(1)  
            time.sleep(2)
            print ("----- Bye~! -----")
            return            

#=============================================
# 메인함수 호툴
# start() 함수가 실질적인 메인함수임. 
#=============================================
if __name__ == '__main__':
    start()
