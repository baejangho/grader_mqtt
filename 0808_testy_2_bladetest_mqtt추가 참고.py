from socket import *
from threading import *
from MG_UI_variable_1027 import *
import numpy as np

# import pyrealsense2 as rs
import time
import struct
import tkinter as tk
from tkinter import ttk
import csv
from datetime import datetime

# UI에 필요한 라이브러리
# import keyBoard
#import keyBoard
import json
import cv2          # pip install opencv-python 로 터미널 입력하여 설치
import math
from ultralytics import YOLO
import random
# import mti

## mqtt 통신 관련 모듈
import paho.mqtt.client as mqtt

#thread_lock = Lock()

### KU_PC IP, PORT 설정 ###
#KU_Server_IP = '192.168.219.158'        # 테스트용 노트북 IP(코오롱)
#KU_Server_IP = '192.168.0.2'            # 테스트용 노트북 IP(코오롱)
# KU_Server_IP = '192.168.0.17'          # 501
# KU_Server_IP = '192.168.0.18'          # 506
# KU_Server_IP = '192.168.1.24'          # 파주 시연용 1
# KU_Server_IP = '192.168.0.60'            # 파주 시연용 2
KU_Server_IP = '192.168.0.83'            # 파주 시연용 3
#KU_Server_IP='192.168.50.98'
#KU_Server_IP = '127.0.0.1'
KU_Server_PORT = 9999                      # KU_PC - UI 포트
KU_Server_camera_PORT = 9998               # KU_PC - UI camera 포트
      
### CRIO IP, PORT 설정 ###


#CRIO_IP = '192.168.0.19'                  # CRIO 
CRIO_IP = '127.0.0.1'                       # CRIO 고정 IP(내 컴퓨터)
#CRIO_IP = '169.254.156.106'                # 노트북 자체 IP
CRIO_PORT = 6004           
CRIO_PORT_2=7100                 

class Server_Client():
    def __init__(self):

        self.stop_event = Event()
        ### CRIO : server, KU : server, UI : client ###
        # self.thread_lock = Lock() ##주석처리함@장호
        ### CRIO IP로 설정 ### 
        self.crio_ip = CRIO_IP
        self.crio_port = CRIO_PORT
        
        self.crio_port2= CRIO_PORT_2    
        ### KU_PC IP로 설정 ###
        self.KU_serve_ip = KU_Server_IP
        self.KU_serve_port = KU_Server_PORT
        self.KU_camera_port = KU_Server_camera_PORT
        # self.bladeDone = Event() ## 주석처리함@장호
        
        self.road_color = 0
        ###  Variables  ###
        self.vel        = 0
        
        self.offset = 0
        
        self.euler_X    = 0
        self.euler_Y    = 0
        self.euler_Z    = 0
        
        self.LeftCyl    = 0
        self.RightCyl   = 0
        self.SideCyl    = 0
        self.TiltCyl    = 0
        
        self.BladeRotation = 0
        
        self.LeftCylPres1 = 0
        self.LeftCylPres2 = 0
        self.RightCylPres1 = 0
        self.RightCylPres2 = 0
        self.SideCylPres1 = 0
        self.SideCylPres2 = 0
        self.TiltCylPres1 = 0
        self.TiltCylPres2 = 0
        self.driving_gear = 0
        
        self.IMU1_Roll  = 0
        self.IMU1_Pitch = 0
        self.IMU2_Roll  = 0
        self.IMU2_Pitch = 0
        
        self.latitude   = 0
        self.longitude  = 0
        self.altitude   = 0
        self.BldSideShift = 0
        self.current_date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        
        self.verticalOffset = 0
        self.desired_height = 0

        self.logStatus = 0

        self.northing = 0
        self.easting = 0

        # self.drivingStatus = 0      # waiting status
        self.drivingStatus_txt = ''
        self.blade_onoff = 0
        self.RTK_Status = 0
        self.HeadingAngle=0
        self.calibratedRoll = 0
        self.calibratedPitch = 0
        self.steeringAngle=0
        # 블레이드 제어를 위해 처리해야 하는 것 : 목표 고도값 갱신
        self.blade_altitude = 14
        self.blade_reference_distance = 3.5 - 1.08
        
        self.controlSwitch = False
        
        self.horizontalOffset =0   # [mm]
        self.bladeTilt = 0                  # [deg]
        self.bladeRot = 0                   # [deg]
        self.bladeRoll = 0                 # [deg]
        
        self.RTK_Status = 0
        
        self.filtered_roll = self.IMU1_Roll
        self.filtered_pitch = self.IMU1_Pitch
        self.filtered_alt = self.altitude
        
        self.CMDB = call_MG_CRIO_CMDB()
        # self.CMDD = call_MG_CRIO_CMDD()
        # Generate the CSV file name with the current date
        # self.csv_file_name = f'logfile/data_logging_{self.current_date}.csv'
        # print(self.csv_file_name)        
        self.drivingStatus = 1
        self.workStart = True # 버튼으로 결정
        self.sequence = 2
        
        self.first_flag = 0
        self.second_flag = 0

        #M 시스템즈 충돌 방지
        self.radar_packetnumber_Rear=0
        self.radar_installposition_Rear=0
        self.radar_status_Rear = 0
        self.radar_reserved1_Rear=0
        self.radar_SWversion_Rear=0
        self.radar_targetX_Rear=0
        self.radar_targetY_Rear=0
        self.radar_targetPower_Rear=0
        self.radar_reserved2_Rear=0
        self.radar_faultcode_Rear=0

        self.radar_packetnumber_Right=0
        self.radar_installposition_Right=0
        self.radar_status_Right= 0
        self.radar_reserved1_Right=0
        self.radar_SWversion_Right=0
        self.radar_targetX_Right=0
        self.radar_targetY_Right=100
        self.radar_targetPower_Right=0
        self.radar_reserved2_Right=0
        self.radar_faultcode_Right=0

        self.radar_packetnumber_Left=0
        self.radar_installposition_Left=0
        self.radar_status_Left= 0
        self.radar_reserved1_Left=0
        self.radar_SWversion_Left=5
        self.radar_targetX_Left=0
        self.radar_targetY_Left=1
        self.radar_targetPower_Left=0
        self.radar_reserved2_Left=0
        self.radar_faultcode_Left=0

        self.radar_packetnumber_RightFront=0
        self.radar_installposition_RightFront=0
        self.radar_status_RightFront= 0
        self.radar_reserved1_RightFront=0
        self.radar_SWversion_RightFront=0
        self.radar_targetX_RightFront=0
        self.radar_targetY_RightFront=0
        self.radar_targetPower_RightFront=0
        self.radar_reserved2_RightFront=0
        self.radar_faultcode_RightFront=0

        time.sleep(1)
 
        ### KU_client 소켓 생성 ###
        self.KU_client_socket = socket(AF_INET, SOCK_STREAM) 
        ### KU_client_2 소켓생성
        self.KU_client_socket2 = socket(AF_INET, SOCK_STREAM)  
        ## KU_server 소켓 생성 ###
        self.KU_server_socket = socket(AF_INET, SOCK_STREAM)                 
        ### KU_camera_server 소켓 생성 ###
        self.KU_camera_server_socket = socket(AF_INET, SOCK_STREAM)   
        
        # CAMERA_BUFFER_SIZE =3*480*640  #v320, 240
        CAMERA_BUFFER_SIZE =150*1024  #v320, 240
        self.KU_camera_server_socket.setsockopt(SOL_SOCKET, SO_RCVBUF, CAMERA_BUFFER_SIZE)
        
        ## KU_PC 서버 소켓 bind ###        
        self.KU_server_socket.bind((self.KU_serve_ip,self.KU_serve_port))
        self.KU_camera_server_socket.bind((self.KU_serve_ip,self.KU_camera_port))
        
        ## blocking setting ###
        self.KU_server_socket.setblocking(True)
        self.KU_client_socket.setblocking(True)
        self.KU_client_socket2.setblocking(True)
        self.KU_camera_server_socket.setblocking(True)
        
        ## 접속 허용하는 장치의 개수 설정 : listen ###///
        self.KU_server_socket.listen(1)                  # 1개(작업자 디스플레이)의 접속만 허용/////////////////
        self.KU_camera_server_socket.listen(1)           # 1개(작업자 디스플레이 카메라)의 접속만 허용
        
        self.TCP_connect()


        
## 유의해야 하는 상황 : 실행 전 데이터 초기화 시켜야 하는 것        
        
    def TCP_connect(self):
        ### CRIO에 접속 요청 ###
        print('CRIO에 연결을 시도합니다.')
        self.KU_client_socket.connect((self.crio_ip,self.crio_port))           
        print('CRIO와 SupervisorPC가 연결되었습니다.')
        self.KU_client_socket2.connect((self.crio_ip,self.crio_port2))
        print('CRIO2와 SupervisorPC가 연결되었습니다.')
        ### UI의 접속을 허가 ###
        print('작업자 UI/UI camera의 접속을 기다립니다.')
        self.UI_client_socket, (ip, port) = self.KU_server_socket.accept()
        print('작업자 UI가 KU_server에 접속하였습니다.')
        self.UI_client_camera_socket, (ip_camera, port_camera) = self.KU_camera_server_socket.accept()                               
        print('작업자 UI camera가 KU_server에 접속하였습니다.')

        self.UI_client_socket.setblocking(True)
        self.UI_client_camera_socket.setblocking(True)

        ### Thread 1 : CRIO에서 데이터 받고 CMDA, CMDC 분류, 단위 변환
        th1 = Thread(target=self.recv_from_CRIO_th1, args=()) 

        ### Thread 2 : GPS값에 기반한 블레이드 제어 - 명령값 들어가고 나서 움직이도록 함
        th2 = Thread(target=self.blade_control_th2, args=())

        ### Thread 3 : 실시간 CMDC 정보 CRIO로 보냄
        th3 = Thread(target=self.send_drive_mesg_th3,args=())

        ### Thread 4 : UI에 Camera 정보 전달, AI 적용한 화면 송신
        th4 = Thread(target=self.realsense_th4,args=())

        ### Thread 5 : CRIO에서 UI로 정보 보냄 
        th5 = Thread(target=self.send_grader_status_th5,args=())

        ### Thread 6 : UI에서 조작 명령을 받고 목표 명령을 선언하기
        th6 = Thread(target=self.recv_bld_order_th6,args=())

        ### Thread 7 : 자체 UI로 조작, 상태값 표시, 비상 정지
        th7 = Thread(target=self.KU_control_th7,args=())

        ## Thread8 : cmdf
        th8 = Thread(target=self.recv_from_CRIO_th8, args=()) 
        
        #############추가##############
        ## Thread9 : MQTT 통신
        th9 = Thread(target=self.mqtt_communication_th9, args=()) 


        self.model = YOLO("yolov8n-seg-customv2.pt")
        # self.model = YOLO("yolov8n_seg_customv2.pt")
        self.class_names = self.model.names
        
        #print('Class Names: ', self.class_names)
        self.colors = [[0, 0, 255],[42, 42, 165]]
        # cap = cv2.VideoCapture('test.mp4')
        self.cap = cv2.VideoCapture(0)
        
        
        self.initialize_GUI()
        
        #self.initialize_GUI2()

        #th4.start()
        #th2.start()  
        th1.start()
        th3.start()
        th5.start()
        th6.start()
        th7.start()
        th8.start()
        th9.start()
        
    #  LEGB 규칙 (Local, Enclosing, Global, Built-in)에 의해 전역변수보다 로컬 변수의 속도가 더빠름
    def mqtt_communication_th9(self):
        # callback 함수 정의
        def on_connect(client, userdata, flags, rc):
            print("Connected with result code "+str(rc))
            client.subscribe(sub_topic) # 구독 신청
        def on_message(client, userdata, msg): # 구독한 메시지 수신
            print(msg.topic+" "+str(msg.payload))
        def on_publish(client, userdata, mid): # 발행한 메시지 확인
            print("Message Published...")
        
        payload = self.mg_state
        
        sub_topic = "command/workplan/grader" # 구독 topic 명
        pub_topic = "equipment/grader"  # 발행 topic 명
        withpoint_url = "withpoints.asuscomm.com"   # 브로커 주소

        mqtt_client = mqtt.Client(client_id="grader_01")    # 클라이언트 객체 생성
        mqtt_client.username_pw_set("grader", "grader") # 사용자 인증정보 설정
        mqtt_client.on_connect = on_connect # 연결 콜백 지정
        mqtt_client.on_message = on_message # 메시지 수신 콜백 지정
        mqtt_client.on_publish = on_publish # pub 수신 콜백 지정

        mqtt_client.connect(withpoint_url, 50592, 60) # 브로커에 연결

        mqtt_client.loop_start()

        while True:
            current_time = datetime.now()
            timestamp_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
            payload["Timestamp"]=timestamp_str
            mqtt_client.publish(pub_topic, str(payload), qos=0, retain=False) # 메시지 발행
            if self.stop_event.is_set():
                break
            time.sleep(1) # 1초 대기
            
    
    ### Thread1 : CRIO에서 데이터 받고 CMDA, CMDC 분류, 단위 변환
    def recv_from_CRIO_th8(self): 
        while not self.stop_event.is_set():
            bytearray_CRIO_Data2 = self.KU_client_socket2.recv(1024)
            parts2 = bytearray_CRIO_Data2.split(b'\r\n')
            for part2 in parts2:
                if len(part2)==61:
                    try:
                        MG_CRIO_Data_f = struct.unpack(
                        '>cccccBBBBBhHHHBBBBBBhHHHBBBBBBhHHHBBBBBBhHHHB'#5c5B1h3H1B5B1h3H1B5B1h3H1B5B1h3H1B
                        ,part2
                        )
                        #radar data 가공 전
                        self.radar_packetnumber_Rear=MG_CRIO_Data_f[5]
                        self.radar_installposition_Rear=MG_CRIO_Data_f[6]
                        self.radar_status_Rear = MG_CRIO_Data_f[7]
                        self.radar_reserved1_Rear=MG_CRIO_Data_f[8]
                        self.radar_SWversion_Rear=MG_CRIO_Data_f[9]
                        self.radar_targetX_Rear=MG_CRIO_Data_f[10]
                        self.radar_targetY_Rear=MG_CRIO_Data_f[11]
                        self.radar_targetPower_Rear=MG_CRIO_Data_f[12]
                        self.radar_reserved2_Rear=MG_CRIO_Data_f[13]
                        self.radar_faultcode_Rear=MG_CRIO_Data_f[14]

                        self.radar_packetnumber_Right=MG_CRIO_Data_f[15]
                        self.radar_installposition_Right=MG_CRIO_Data_f[16]
                        self.radar_status_Right= MG_CRIO_Data_f[17]
                        self.radar_reserved1_Right=MG_CRIO_Data_f[18]
                        self.radar_SWversion_Right=MG_CRIO_Data_f[19]
                        self.radar_targetX_Right=MG_CRIO_Data_f[20]
                        self.radar_targetY_Right=MG_CRIO_Data_f[21]
                        self.radar_targetPower_Right=MG_CRIO_Data_f[22]
                        self.radar_reserved2_Right=MG_CRIO_Data_f[23]
                        self.radar_faultcode_Right=MG_CRIO_Data_f[24]

                        self.radar_packetnumber_Left=MG_CRIO_Data_f[25]
                        self.radar_installposition_Left=MG_CRIO_Data_f[26]
                        self.radar_status_Left= MG_CRIO_Data_f[27]
                        self.radar_reserved1_Left=MG_CRIO_Data_f[28]
                        self.radar_SWversion_Left=MG_CRIO_Data_f[29]
                        #print(f"t_y:{self.radar_targetY_Right}")
                        self.radar_targetX_Left=MG_CRIO_Data_f[30]
                        self.radar_targetY_Left=MG_CRIO_Data_f[31]
                        self.radar_targetPower_Left=MG_CRIO_Data_f[32]
                        self.radar_reserved2_Left=MG_CRIO_Data_f[33]
                        self.radar_faultcode_Left=MG_CRIO_Data_f[34]

                        self.radar_packetnumber_RightFront=MG_CRIO_Data_f[35]
                        self.radar_installposition_RightFront=MG_CRIO_Data_f[36]
                        self.radar_status_RightFrontr= MG_CRIO_Data_f[37]
                        self.radar_reserved1_RightFront=MG_CRIO_Data_f[38]
                        self.radar_SWversion_RightFront=MG_CRIO_Data_f[39]
                        self.radar_targetX_RightFront=MG_CRIO_Data_f[40]
                        self.radar_targetY_RightFront=MG_CRIO_Data_f[41]
                        self.radar_targetPower_RightFront=MG_CRIO_Data_f[42]
                        self.radar_reserved2_RightFront=MG_CRIO_Data_f[43]
                        self.radar_faultcode_RightFront=MG_CRIO_Data_f[44]
                        #print(f"cmdf 받고있음 :{part2}")
                    except:
                        print(f"잘못된 프로토콜 전송f : {part2}")

                else:
                    
                    continue
        self.KU_client_socket2.close()  
        self.KU_server_socket.close()

    def recv_from_CRIO_th1(self):
        tim = time.time()
        DT = 0.1
        tau_alt = 0.045
        tau_roll = 0.05
        tau_pitch = 0.05
        alpha_alt = np.exp(-DT/tau_alt)
        alpha_roll = np.exp(-DT/tau_roll)
        alpha_pitch = np.exp(-DT/tau_pitch)
        filtered_roll = 0
        filtered_pitch = 0
        filtered_alt = 0
        i = 0
        while not self.stop_event.is_set():

            bytearray_CRIO_Data = self.KU_client_socket.recv(1024)      # including CR/LF
            parts = bytearray_CRIO_Data.split(b'\r\n')                  # 여러 개 들어온 데이터 슬라이싱
            
            for part in parts:
                
                if len(part)==162:

                    try:
                        
                        MG_CRIO_Data = struct.unpack(
                            '>ccccchhhhhhhhhHHHHHHHHHHHHHHHHHHhhhhhhhhhhhhhhhhBBBBhhhhBBfBddddBBHHHHHHHHH'
                            ,part
                        )
                        
                        # 전처리 구간
                        self.vel = MG_CRIO_Data[52]*0.001              # 예상 [m/s]  
                        #print(f"cmda")
                        self.LeftCyl = MG_CRIO_Data[9] *0.1            # [mm]
                        self.RightCyl = MG_CRIO_Data[10]*0.1           # [mm]
                        self.SideCyl = MG_CRIO_Data[11]*0.1            # [mm]
                        self.BldSideShift = MG_CRIO_Data[12]*0.1       # [mm]
                        self.TiltCyl    = MG_CRIO_Data[13]*0.1         # [mm]
                                    
                        self.IMU1_Roll = MG_CRIO_Data[39]*0.01 - 0.85        # [deg]         - 초기 IMU 설치 오차에 의한 offset 필요
                        self.IMU1_Pitch = MG_CRIO_Data[38]*0.01 - 0.5683       # [deg]         - 초기 IMU 설치 오차에 의한 offset 필요
                        
                        self.IMU2_Roll  = MG_CRIO_Data[47]*0.01     # [deg]
                        self.IMU2_Pitch = MG_CRIO_Data[46]*0.01     # [deg]
                        if MG_CRIO_Data[63] != 0:
                            self.HeadingAngle = MG_CRIO_Data[63]           # 미지
                        
                        self.steeringAngle = MG_CRIO_Data[53]*0.1          # 미지
                        self.BladeRotation = MG_CRIO_Data[8]*0.1       # [deg]  -> 확인 필요
                        self.driving_gear = MG_CRIO_Data[65]       # 1 : Reverse / 2 : Netural / 3 : Forward
                        self.RTK_Status = MG_CRIO_Data[59]
                        
                        # Waypoint 명령값을 2차원으로 변경하는 법 필요
                        if MG_CRIO_Data[59] == 16:
                            self.latitude = MG_CRIO_Data[60]
                            self.longitude = MG_CRIO_Data[61]
                            self.altitude = MG_CRIO_Data[62]
                            
                        # LPF 적용
                        if i == 0 and MG_CRIO_Data[59] == 16:
                            filtered_roll = self.IMU1_Roll
                            filtered_pitch = self.IMU1_Pitch
                            filtered_alt = self.altitude
                            i = 1
                        
                        filtered_roll = (1 - alpha_roll) * filtered_roll + alpha_roll * self.IMU1_Roll 
                        filtered_pitch = (1 - alpha_pitch) * filtered_pitch + alpha_pitch * self.IMU1_Pitch
                        filtered_alt = (1 - alpha_alt) * filtered_alt + alpha_alt * self.altitude
                        
                        self.filtered_roll = filtered_roll
                        self.filtered_pitch = filtered_pitch
                        self.filtered_alt = filtered_alt
                        
                        if self.logStatus == 1:
                            rows = []
                            # self.cur_time = time.time()
                            self.cur_time = time.perf_counter()
                            rows.append([(self.cur_time-self.logtime),self.LeftCyl,self.RightCyl,self.SideCyl,self.BladeRotation,self.BldSideShift,self.TiltCyl,self.IMU1_Roll,self.IMU1_Pitch,self.IMU2_Roll,self.IMU2_Pitch, self.vel, self.latitude,self.longitude,self.altitude,self.HeadingAngle,self.filtered_roll,self.filtered_pitch,self.filtered_alt,self.RTK_Status,self.driving_gear])
                            self.writer.writerows(rows)

                        ## filtered data와 raw data를 비교해볼 것 @장호
                 
                    except:
                        print(f"잘못된 프로토콜 전송a : {part}")
                    

                elif len(part)==7:   
                    try:
                        MG_CRIO_Data_C = struct.unpack(
                        '>ccccccB'
                        ,part
                        )
                        # 주행 상태를 판별하는 지표
                        self.drivingStatus = int(MG_CRIO_Data_C[6])

                    except:
                        print(f"잘못된 프로토콜 전송c : {part}")
                
 
                else:
                    
                    continue

        self.KU_client_socket.close()    
        #self.KU_client_socket2.close()  
        self.KU_server_socket.close()

          
    ### Thread 2 : GPS값에 기반한 블레이드 제어 - 명령값 들어가고 나서 움직이도록 함
    def blade_control_th2(self):
        i = 1 ##주석처리함 @장호

        while not self.stop_event.is_set():  
            print(self.RTK_Status)
            if self.RTK_Status == 16:
                # RTK_Status = self.RTK_Status
                # self.blade_altitude = self.altitude + (4 * np.sin(self.IMU1_Pitch)-2.4*(1-np.cos(self.IMU1_Roll)*np.cos(self.IMU1_Pitch)))
                
                # Kalman filter 적용 가능한 부분
                time.sleep(2)
                blade_altitude = self.filtered_alt - 3.5 * np.cos(np.round(self.filtered_roll,1)/180*np.pi)
                print(blade_altitude)   
                
                break
        blade_reference_distance_x = 4
        blade_reference_distance_z = 2.4    
        CMDB = call_MG_CRIO_CMDB()
        self.position_flag1 = False
        self.position_flag2 = False
        self.working_flag1 = True
        self.working_flag2 = False
        height = 0  
        self.height1 = 0.02
        self.height2 = 0.02

        startPoint1 = [35.9364447583333, 128.814669425]                    # 작업1 시작 위치
        startPoint2 = [35.9364998944444, 128.814675647222]                        # 작업2 시작 위치
        finalPoint1 = [35.9365040944444, 128.814624425]                    # 작업1 끝 위치
        finalPoint2 = [35.9364486166667, 128.814617555556]  
        # startPoint1 = [37.6746458805556, 126.629568394444]                    # 작업1 시작 위치
        # startPoint2 = [37.6746383444444, 126.629586986111]                        # 작업2 시작 위치
        # finalPoint1 = [37.6748419555556, 126.629701647222]                    # 작업1 끝 위치
        # finalPoint2 = [37.6748349444444, 126.629720277778]                        # 작업2 끝 위치 
        # startPoint1 = [37.6747269833333, 126.629623508333]                    # 작업1 시작 위치
        # startPoint2 = [37.6747190388889, 126.62964205]                        # 작업2 시작 위치
        # finalPoint1 = [37.6747269833333, 126.629623508333]                    # 작업1 끝 위치
        # finalPoint2 = [37.6747190388889, 126.62964205]                        # 작업2 끝 위치 
        thredhold = 4.5*1e-10                            # 영역 범위 지정
        thredhold1 = 5.5*1e-10                            # 영역 범위 지정
                
        while not self.stop_event.is_set():
            # if self.controlSwitch == True:    
            if True:   # 블레이드 자율작업과 연동 @ 장호 
                # 제어 명령 처리
                prev_time = time.time()

                ## th2 요구 변수들 설정
                altitude = self.filtered_alt
                latitude = self.latitude                    # filter 필요
                longitude = self.longitude 

                IMU1_Roll = self.filtered_roll
                IMU1_Pitch = self.filtered_pitch
                
                verticalOffset = self.verticalOffset        # [mm]
                horizontalOffset = self.horizontalOffset    # [mm]
                bladeTilt = self.bladeTilt                  # [deg]
                bladeRot = self.bladeRot                    # [deg]
                bladeRoll = self.bladeRoll                  # [deg]

                drivingStatus = self.driving_gear           # 1 : Reverse / 2 : Netural / 3 : Forward

                

                # if self.bladeDone.is_set(): # 4m, 2.4
                # if self.sequence == 2: # 후진 신호로 변경
                if drivingStatus == 1 or drivingStatus == 2: # 후진/중립 시 블레이드 초기값으로 들어올림 @장호
                    self.CMDB = call_MG_CRIO_CMDB()
                    self.KU_client_socket.sendall(self.CMDB)
                    self.road_color = 0
                    self.position_flag1 = False
                    self.position_flag2 = False
                    if (finalPoint1[0] - latitude)**2 + (finalPoint1[1]- longitude)**2 < thredhold:
                        self.working_flag1 = False
                        self.working_flag2 = True
                        if (self.height2 - self.height1 < 0.05) and i == 0:
                            self.height1 -= 0.02
                            i = 1
                            print(f'height1 : {self.height1}')
                            
                        
                    if (finalPoint2[0] - latitude)**2 + (finalPoint2[1]- longitude)**2 < thredhold:
                        self.working_flag1 = True
                        self.working_flag2 = False
                        if (self.height2 - self.height1 < 0.05) and i == 0:
                            # self.height1 -= 0.02
                            i = 1
                            print(f'height2 : {self.height2}')
                        
                    
                    print('후진 또는 중립입니다.')
                
                # if self.sequence == 0 or self.sequence == 1:
                elif drivingStatus == 3: # 전진 시 블레이드 제어
                    
                    if (((startPoint1[0] - latitude)**2 + (startPoint1[1]- longitude)**2 < thredhold1) or self.position_flag1) and self.working_flag1:
                        # 목표값 설정
                        refRot = 30/180*np.pi
                        refRoll = -1.2/180*np.pi
                        bladeTilt = 0

                        height = self.height1    
                        side = 0         
                        self.position_flag1 = True
                        self.position_flag2 = False
                        self.road_color = 2
                        print('1번 영역 작업중')
                    elif (((startPoint2[0] - latitude)**2 + (startPoint2[1]- longitude)**2 < thredhold1) or self.position_flag2) and self.working_flag2:
                        # 목표값 설정
                        refRot = 30/180*np.pi
                        refRoll = -1.2/180*np.pi
                        bladeTilt = 0
                        
                        height = self.height2  
                        side = 500            
                        self.position_flag2 = True
                        self.position_flag1 = False
                        self.road_color = 2
                        print('2번 영역 작업중')
                    else: # 그 외 영역
                        #height = 0
                        self.CMDB = call_MG_CRIO_CMDB()
                        self.KU_client_socket.sendall(self.CMDB)
                        self.road_color = 0
                        print('else')

                    if self.position_flag1 == True or self.position_flag2 == True:

                        i = 0
                        car_roll = np.round(IMU1_Roll,1)/180*np.pi
                        car_pitch = np.round(IMU1_Pitch,1)/180*np.pi
                        initial_guess = (0.0, 0.0)

                        offset_roll = math.atan((np.cos(car_pitch)*np.sin(car_roll))/np.cos(car_roll))
                        # print(f'offset : {offset_roll}')

                        verticalOffsetReq = verticalOffset*0.001       # 블레이드 높이 조절    범위 : 0~1000 mm
                        bladeTiltReq = bladeTilt                # -60 ~ 60   # 번위수정 @장호
                        bladeRotReq = bladeRot/180*np.pi + refRot 
                        bladeRollReq = bladeRoll/180*np.pi - offset_roll + refRoll  # -30 ~ 30
                        
                        # print('전처리 완료')
                        # desired_height = verticalOffsetReq + (blade_altitude - altitude  + np.sin(car_pitch) * blade_reference_distance_x + np.cos(car_roll) * np.cos(car_pitch) * blade_reference_distance_z)  
                        # desired_height = blade_altitude - altitude - 1.03 + verticalOffsetReq + self.offset
                        desired_height = (blade_altitude - altitude)/np.cos(car_roll) + blade_reference_distance_z + self.offset + 0.06 + height
                        # desired_height = -1.03 + height + self.offset


                        # 지면에 닿았을 때가 desired height가 1.08이 되어야 함
                        if desired_height <-1.3 or desired_height > - 0.7:
                            print(f'height : {desired_height}, Blade altitude : {blade_altitude}, Receiver altitude : {altitude}, Offset : {verticalOffsetReq}')
                            elp_time = time.time()-prev_time
                            time.sleep(0.3-elp_time)
                            continue
                        
                        # self.blade_altitude : 블레이드 목표 작업 높이
                        # self.altitude : 그레이더의 실시간 GNSS 고도값
                        # self.blade_reference_distance : 볼조인트와 그레이더 GNSS 사이 높이차이
                        # self.verticalOffset : 그레이더의 높이 오프셋 (높이는 쪽으로 되어있음) ## 무슨뜻인지질문 @장호

                        # Use fsolve to find the solutions for pitch and roll
                        if bladeTilt > 30: bladeTilt = 30                          # -60 ~ 60 번위수정 @장호
                        elif bladeTilt < 0: bladeTilt = 0
                        
                        tiltCylOrder = int(-1800/30*(bladeTiltReq)+200)  # 선형 변환 한것 : 더 정밀하게 하려면 다른 변환(삼각변환) 각도->길이
                        Tiltangle = (bladeTiltReq-56)*np.pi/180     
                        
                        # horizontalOffsetReq = horizontalOffset - 1000*(l_z*(np.cos(Tiltangle)*np.sin(bladeRollReq)) - np.cos(bladeRollReq)*np.sin(bladeRotReq)*np.sin(Tiltangle) + l_4z*np.sin(bladeRollReq))/(np.cos(bladeRollReq)*np.cos(bladeRotReq))   # 블레이드 옆으로 빼는거    범위 : 0~1000 mm 
                        horizontalOffsetReq = horizontalOffset
                        # 방정식들은 다 m 단위
                        solutions = fsolve(func=lambda x: [crossAngle(x[0],x[1],bladeRollReq,bladeRotReq), desiredHeight(x[0],x[1],desired_height,bladeRotReq,Tiltangle)], x0=initial_guess)
                        pitch_solution, roll_solution = solutions

                        yaw_solution = yaw
                        
                        Reul = rotz(yaw_solution)@roty(pitch_solution)@rotx(roll_solution)
                        
                        tb1 = Reul@b1
                        tb2 = Reul@b2
                        tb3 = Reul@b3

                        # Inverse kinematics - d1~d3 : 실린더 길이로 변환
                        d1 = np.sqrt(a1.T@a1 + tb1.T@tb1 - 2*a1.T@tb1) 
                        d2 = np.sqrt(a2.T@a2 + tb2.T@tb2 - 2*a2.T@tb2)
                        d3 = np.sqrt(a3.T@a3 + tb3.T@tb3 - 2*a3.T@tb3)
                        print(f'd1 : {d1}, d2 : {d2}, d3 : {d3},')
                        
                        # 실린더 변위값
                        leftCylOrder = -(d1-d1_0)*10000
                        rightCylOrder = -(d2-d2_0)*10000
                        sideCylOrder = -(d3-d3_0)*10000
                        # 임시 : 매핑한 값
                        
                        # # 과도한 블레이드 가동범위 방지
                        # if leftCylOrder < -3300 or rightCylOrder < -3300:
                        #     print('out of range')
                        #     elp_time = time.time()-prev_time
                        #     time.sleep(0.3-elp_time)
                        #     return
                        
                        if leftCylOrder > 10 or rightCylOrder > 10:
                            print('out of range')
                            elp_time = time.time()-prev_time
                            time.sleep(0.3-elp_time)
                            return  

                        bladeRotOrder = - bladeRotReq*10/np.pi*180
                        if bladeRotOrder<-400 or bladeRotOrder>400:
                            print('Too big blade rotation order')
                            return
                        
                        temp = struct.pack('>h',int(side*10))
                        CMDB[16] = temp[0]
                        CMDB[17] = temp[1]
                        # leftCylOrder
                        temp = struct.pack('>h',int(leftCylOrder))
                        CMDB[10] = temp[0]
                        CMDB[11] = temp[1]
                        # rightCylOrder
                        temp = struct.pack('>h',int(rightCylOrder))
                        CMDB[12] = temp[0]
                        CMDB[13] = temp[1]    
                        # sideCylOrder
                        temp = struct.pack('>h',int(sideCylOrder))
                        CMDB[14] = temp[0]
                        CMDB[15] = temp[1]
                        # tiltCylOrder
                        temp = struct.pack('>h',int(tiltCylOrder))
                        CMDB[18] = temp[0]
                        CMDB[19] = temp[1]
                        # bladeRotOrder
                        temp = struct.pack('>h',int(bladeRotOrder))
                        CMDB[8] = temp[0]
                        CMDB[9] = temp[1]

                        print(leftCylOrder,rightCylOrder,sideCylOrder,tiltCylOrder,bladeRotOrder,horizontalOffset)
                        self.CMDB = CMDB
                        self.KU_client_socket.sendall(self.CMDB)
                    else:
                        pass
                else:
                    self.road_color = 0
                    print('기어상태값 이상')
                # 안전장치

                elp_time = time.time()-prev_time
                # print(f'Th2 소요 시간 : {elp_time}')
                if elp_time < 1:
                    time.sleep(1-elp_time)    
                else: 
                    print('블레이드 제어 명령 코드 1초 초과')

                
                
        self.UI_client_socket.close()
        self.KU_server_socket.close()


    ### Thread 3 : 실시간 CMDC 정보 CRIO로 보냄
    def send_drive_mesg_th3(self):
        i = 0
        
        pre_DrivingStatus = 11 # 초기 driving statue
        num_driving = 0
        total_num_driving = 1 # 한번에 모든 waypoint 전송 수정@장호
        self.workEnd = True
        while not self.stop_event.is_set():
            tim = time.time()
            drivingStatus = self.drivingStatus
            
            # if self.workStart and self.controlSwitch and not self.workEnd:
            if self.workStart and not self.workEnd:
            # if True:
                if pre_DrivingStatus != drivingStatus:
                # if True:
                    if drivingStatus == 0: # WAITING
                        ### 블레이드 자세 목표값 설정(한양대 정보로 부터 계산) ###
                        self.position_flag1 = False
                        self.position_flag2 = False
                        self.working_flag1 = True
                        self.working_flag2 = False
                        self.offset -= 0.005
                        
                        # 한양대에서 파일로 넘겨주는 경우
                        with open("230802_23_Straigt_Fw.json", encoding='utf-8') as f:
                            waypoint_json = json.load(f)
                        f.close()
                        msg_CMDC2 = call_MG_CRIO_CMDC2(waypoint_json)
                        
                        
                        print('driving status :','WAITING(Waypoints 정보 수신 대기)')
                        self.drivingStatus_txt = 'Waiting'
                        msg_CMDC1 = call_MG_CRIO_CMDC1()
                        msg_CMDC1[6] = 3
                                                
                        self.KU_client_socket.sendall(msg_CMDC2)
                        print('Waypoint 전송 완료')
                        time.sleep(4)
                        self.KU_client_socket.sendall(msg_CMDC1)
                        print('CMDC1 수신')
                        time.sleep(3)

                        

                    elif drivingStatus == 1: # DISABLE
                        print('driving status :','DISABLE(초기위치에러)')
                        self.drivingStatus_txt = 'Disable'
                        
                    elif drivingStatus == 2: # READY
                        print('driving status :','READY(주행 시작 수신 대기)')
                        self.drivingStatus_txt = 'Ready'
                        msg_CMDC1 = call_MG_CRIO_CMDC1()
                        msg_CMDC1[6] = 1
                        self.KU_client_socket.sendall(msg_CMDC1)
                    
                    elif drivingStatus == 3: # START
                        self.drivingStatus_txt = 'Start'
                        print('driving status :','START(주행 시작)')

                    elif drivingStatus == 4: # RUNNING
                        self.drivingStatus_txt = 'Running'
                        print('driving status :','RUNNING(주행 중)')
                        self.blade_onoff = 1
                    
                    elif drivingStatus == 5: # EMS STOP
                        self.drivingStatus_txt = 'Ems Stop'
                        print('driving status :','EMS STOP(비상 정지)')
                        self.blade_onoff = 0
                    
                    elif drivingStatus == 6: # finish
                        num_driving += 1
                        # self.second_flag = 0
                        self.drivingStatus_txt = 'Finish'
                        print('driving status :','FINISH(주행 완료)',num_driving)
                        # if num_driving == total_num_driving: # 수정@1
                        #     self.workEnd = True
                        self.blade_onoff = 0
                    
                    elif drivingStatus == 7: # RECEIVED
                        self.drivingStatus_txt = 'Received'
                        print('driving status :','RECEIVED(waypoints 정보 수신 완료)')
                    
                    elif drivingStatus == 9: # TIME OUT1
                        self.drivingStatus_txt = 'Timeout 1'
                        print('driving status :','TIME OUT1(POC통신오류)')
                        self.blade_onoff = 0

                    elif drivingStatus == 10: # TIME OUT2
                        self.drivingStatus_txt = 'Timeout 2'
                        print('driving status :','TIME OUT2(주행경로수신 오류)')
                        self.blade_onoff = 0
                        '''
                        msg_CMDC1 = call_MG_CRIO_CMDC1
                        msg_CMDC1[6] = 3
                        self.KU_client_socket.sendall(msg_CMDC1)
                        time.sleep(2)
                        with open("hanyang_1.json", encoding='utf-8') as f:
                            waypoint_json = json.load(f)
                        msg_CMDC2 = call_MG_CRIO_CMDC2(waypoint_json)
                        self.KU_client_socket.sendall(msg_CMDC2)
                        '''

                pre_DrivingStatus = drivingStatus
            else:
                num_driving = 0
                pre_DrivingStatus = 1
            elp_tim = time.time() - tim    
            # print(f'Th3 여유 시간 : {0.5-elp_tim}')
            try:
                time.sleep(2-elp_tim)  # 2초로 변경 @장호
            except:
                print('CMDC2초 초과')
                continue
            
        self.UI_client_socket.close()
        self.KU_server_socket.close()    
        
    def realsense_th4(self):

        while True:
            
            # self.thread_lock.acquire()
            s = time.time()
            # print('read 시작 전')   
            success, img = self.cap.read()
            # print(cap)
            # print('read 시작')
            if not success:
                print('UI break 지남')
                continue
                # break

            h, w, _ = img.shape
            # results = self.model.predict(img, show=True, save=True, conf=0.5)
            # results = self.model.predict(img, save=True, conf=0.4,verbose = False)
            results = self.model.predict(img, save=True, conf=0.4, amp = False)
            for r in results:
                boxes = r.boxes  # Boxes object for bbox outputs
                masks = r.masks  # Masks object for segment masks outputs
                # probs = r.probs  # Class probabilities for classification outputs
            # cpu 문제 없으면 gpu로 변경하여 사용
            if masks is not None:
                masks = masks.data.cpu()
                for seg, box in zip(masks.data.cpu().numpy(), boxes):
                # for seg, box in zip(masks.numpy(), boxes):

                    seg = cv2.resize(seg, (w, h))
                    img = overlay(img, seg, self.colors[int(box.cls)], 0.4)
                    
                    xmin = int(box.data[0][0])
                    ymin = int(box.data[0][1])
                    xmax = int(box.data[0][2])
                    ymax = int(box.data[0][3])
                    
                    plot_one_box([xmin, ymin, xmax, ymax], img, self.colors[int(box.cls)], f'{self.class_names[int(box.cls)]} {float(box.conf):.3}')

            cv2.imshow('img', img)
            
            mat = cv2.resize(img, (320, 240))
            
            mat_test = mat.tobytes()

            self.UI_client_camera_socket.send(mat_test)

            # cv2.waitKey(3)      # miliseconds
            f = time.time()
            # print(f'YOLO 소요 시간 : {f-s}')
            # self.thread_lock.release()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # print(f'Th4 소요 시간 : {f-s}')
            
            if f-s < 1:
                time.sleep(1-(f-s))
                # print(f'Th4 여유 시간 : {0.5-(f-s)}')
                
            else:
                print("UI 송신 시간초과")
                
        self.UI_client_camera_socket.close()



    ### Thread 5 : CRIO에서 UI로 정보 보냄     
    def send_grader_status_th5(self):

        '''
        UI에 보내기 전 추가 전처리가 필요한 변수에 대한 논의 필요
        후처리된 영상 데이터도 보내도록 변경 필요
        '''

        self.mg_state = MG_State

        while not self.stop_event.is_set():
            tim = time.time()
            latitude = self.latitude
            longitude = self.longitude
            altitude = self.altitude
            #print("flag0")
            IMU1_Roll = round(self.IMU1_Roll,1)
            IMU1_Pitch = round(self.IMU1_Pitch)
            HeadingAngle = round(self.HeadingAngle,0)
            
            IMU2_Roll = round(self.IMU2_Roll,1)
            IMU2_Pitch = round(self.IMU2_Pitch,1)
            #print("flag1")
            BladeRotation = round(self.BladeRotation,1)
            verticalOffset = round(self.verticalOffset,1)
            
            # 데이터 JSON 형태로 변형 후 UI로 데이터 전달
            self.mg_state["grd_bdy_lttd"] = latitude
            self.mg_state["grd_bdy_lgtd"] = longitude
            self.mg_state["grd_bdy_altd"] = altitude
            
            self.mg_state["grd_bdy_x_pose"] = IMU1_Roll 
            self.mg_state["grd_bdy_y_pose"] = IMU1_Pitch
            self.mg_state["grd_bdy_z_pose"] = HeadingAngle+120
            self.mg_state["grd_bld_x_pose"] =  IMU2_Roll        # blade roll
            self.mg_state["grd_bld_y_pose"] = IMU2_Pitch         # blade cutting angle
            
            self.mg_state["grd_bld_z_pose"] = BladeRotation
            self.mg_state["grd_bld_z_offset"] = verticalOffset
            
            
            self.mg_state["grd_running_state"] = self.road_color
            
            # print(self.mg_state)
            
            self.str_MG_State = json.dumps(self.mg_state)
            
            # UI에 전달하는 시간 주기는 0.5초
            try:
                self.UI_client_socket.sendall(bytes(self.str_MG_State+'||',encoding="utf-8"))
            except:
                print('작디와 접속이 끊어졌습니다.')

            # UI 테스트용
            # self.BladeRotation = 50*math.sin(i/1/math.pi)
            # 전처리 데이터 UI에 보낼 수 있도록 형태 바꿈 - MG_State

            elapsed_time = time.time() - tim
            if elapsed_time < 0.5:
                time.sleep(0.5-elapsed_time)              # 데이터 주기 0.1초 (100 ms)
                # print(f'Th5 소요 시간 : {elapsed_time}')
                
            else:
                print("UI로 정보 송신 시간초과")
        
        self.UI_client_socket.close()
        self.KU_server_socket.close()

    def recv_bld_order_th6(self):

        while not self.stop_event.is_set():
            
            prev_time = time.time()
            print("Waiting for UI message")
            try:
                byte_UI_CMD = self.UI_client_socket.recv(1024)
            except:
                print('작디와 연결이 끊어졌습니다.') 
                continue
            
            
            
            if not byte_UI_CMD:
                print("No data received")
                continue
                
            print("Received data:", byte_UI_CMD)
            print("UI message received")

            str_UI_CMD = byte_UI_CMD.decode('utf-8')
            str_UI_CMD = str_UI_CMD.replace('||','')
            
            try:
                self.dict_UI_CMD = json.loads(str_UI_CMD)
                
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
                print("Data content:", str_UI_CMD)
                continue
            
            self.dict_UI_CMD = json.loads(str_UI_CMD)
                        
            if self.dict_UI_CMD['ProtocolType'] == 'B':
        
                MG_Response['grd_req_msg_type'] = 'B'
                
                self.controlSwitch = bool(self.dict_UI_CMD["grd_mc_ctrl_on_rqst"])
                self.verticalOffset = float(self.dict_UI_CMD["grd_set_request"]["bld_vtcal_offset"])        # mm
                self.horizontalOffset = float(self.dict_UI_CMD["grd_set_request"]["bld_hrztal_offset"])     # mm 
                self.bladeTilt = float(self.dict_UI_CMD["grd_set_request"]["bld_pitch"])                 # deg        
                self.bladeRot = float(self.dict_UI_CMD["grd_set_request"]["bld_yaw"])                    # deg
                self.bladeRoll = float(self.dict_UI_CMD["grd_set_request"]["bld_roll"])                  # deg
                
                self.UI_client_socket.sendall(json.dumps(MG_Response).encode('utf-8'))
                
                
            # elif self.dict_UI_CMD['ProtocolType'] == 'T':
            #     print('waypoint go')
            #     # CMDC 코드 설계 필요 - 질문드려야 함
            #     MG_Response['grd_req_msg_type'] = 'T'

            #     self.UI_client_socket.sendall(MG_Response)

            else:
                print('Unknown message received')
                break

        ### Thread 7 : 자체 UI로 조작 - 상태값 표시, 비상 정지
    def KU_control_th7(self):
                
        while not self.stop_event.is_set():
            
            prev_time = time.time()
            # 블레이드 정보 표시
            self.UI_LeftCyl.set(self.LeftCyl)
            self.UI_RightCyl.set(self.RightCyl)
            self.UI_SideCyl.set(self.SideCyl)
            self.UI_TiltCyl.set(self.TiltCyl)       
            self.UI_BladeRotation.set(self.BladeRotation)          
            self.UI_BldSideShift.set(self.BldSideShift)
            self.UI_IMU1_Roll.set(self.IMU1_Roll)
            self.UI_IMU1_Pitch.set(self.IMU1_Pitch)
            self.UI_IMU2_Roll.set(self.IMU2_Roll)   
            self.UI_IMU2_Pitch.set(self.IMU2_Pitch)
            self.UI_driving_gear.set(self.driving_gear)
            
            # 주행 정보 표시
            self.UI_vel.set(self.vel)
            self.UI_HeadingAngle.set(self.HeadingAngle)
            self.UI_steeringAngle.set(self.steeringAngle)
            self.UI_latitude.set(self.latitude)
            self.UI_longitude.set(self.longitude)
            self.UI_altitude.set(self.altitude)
            
            self.UI_drivingStatus.set(self.drivingStatus)
        
            
            self.UI_drivingStatus_txt.set(self.drivingStatus_txt)
            #radar
            self.UI_radar_packetnumber_Rear.set(self.radar_packetnumber_Rear)
            self.UI_radar_installposition_Rear.set(self.radar_installposition_Rear)
            self.UI_radar_status_Rear.set(self.radar_status_Rear)
            self.UI_radar_reserved1_Rear.set(self.radar_reserved1_Rear)
            self.UI_radar_SWversion_Rear.set(self.radar_SWversion_Rear)
            self.UI_radar_targetX_Rear.set(self.radar_targetX_Rear)
            self.UI_radar_targetY_Rear.set(self.radar_targetY_Rear)
            self.UI_radar_targetPower_Rear.set(self.radar_targetPower_Rear)
            self.UI_radar_reserved2_Rear.set(self.radar_reserved2_Rear)
            self.UI_radar_faultcode_Rear.set(self.radar_faultcode_Rear)

            self.UI_radar_packetnumber_Right.set(self.radar_packetnumber_Right)
            self.UI_radar_installposition_Right.set(self.radar_installposition_Right)
            self.UI_radar_status_Right.set(self.radar_status_Right)
            self.UI_radar_reserved1_Right.set(self.radar_reserved1_Right)
            self.UI_radar_SWversion_Right.set(self.radar_SWversion_Right)
            self.UI_radar_targetX_Right.set(self.radar_targetX_Right)
            self.UI_radar_targetY_Right.set(self.radar_targetY_Right)
            self.UI_radar_targetPower_Right.set(self.radar_targetPower_Right)
            self.UI_radar_reserved2_Right.set(self.radar_reserved2_Right)
            self.UI_radar_faultcode_Right.set(self.radar_faultcode_Right)

            self.UI_radar_packetnumber_Left.set(self.radar_packetnumber_Left)
            self.UI_radar_installposition_Left.set(self.radar_installposition_Left)
            self.UI_radar_status_Left.set(self.radar_status_Left)
            self.UI_radar_reserved1_Left.set(self.radar_reserved1_Left)
            self.UI_radar_SWversion_Left.set(self.radar_SWversion_Left)
            self.UI_radar_targetX_Left.set(self.radar_targetX_Left)
            self.UI_radar_targetY_Left.set(self.radar_targetY_Left)
            self.UI_radar_targetPower_Left.set(self.radar_targetPower_Left)
            self.UI_radar_reserved2_Left.set(self.radar_reserved2_Left)
            self.UI_radar_faultcode_Left.set(self.radar_faultcode_Left)

            self.UI_radar_packetnumber_RightFront.set(self.radar_packetnumber_RightFront)
            self.UI_radar_installposition_RightFront.set(self.radar_installposition_RightFront)
            self.UI_radar_status_RightFront.set(self.radar_status_RightFront)
            self.UI_radar_reserved1_RightFront.set(self.radar_reserved1_RightFront)
            self.UI_radar_SWversion_RightFront.set(self.radar_SWversion_RightFront)
            self.UI_radar_targetX_RightFront.set(self.radar_targetX_RightFront)
            self.UI_radar_targetY_RightFront.set(self.radar_targetY_RightFront)
            self.UI_radar_targetPower_RightFront.set(self.radar_targetPower_RightFront)
            self.UI_radar_reserved2_RightFront.set(self.radar_reserved2_RightFront)
            self.UI_radar_faultcode_RightFront.set(self.radar_faultcode_RightFront)

            try:
                elp_tim = time.time() - prev_time 
                # print(f'Th7 여유 시간 : {elp_tim}')
                time.sleep(1 - elp_tim)
                
            except: 
                # print('Camera time is too fast')
                continue
        # self.UI_client_socket.close()
        self.KU_server_socket.close()


  
    def initialize_GUI(self):

               
        self.SV_GUI = tk.Tk()
        self.SV_GUI.geometry('1300x900')
        self.SV_GUI.title("KU_Auto_MotorGrader")

        self.driving_onoff = tk.IntVar()
        self.driving_onoff.set(1)

        self.logOnOff = tk.IntVar()
        self.logOnOff.set(1)
        # 블레이드 정보
        self.UI_LeftCyl = tk.DoubleVar()
        self.UI_LeftCyl.set(self.LeftCyl)

        self.UI_RightCyl = tk.DoubleVar()
        self.UI_RightCyl.set(self.RightCyl)

        self.UI_SideCyl = tk.DoubleVar()
        self.UI_SideCyl.set(self.SideCyl)

        self.UI_TiltCyl = tk.DoubleVar()
        self.UI_TiltCyl.set(self.TiltCyl)       

        self.UI_BladeRotation = tk.DoubleVar()
        self.UI_BladeRotation.set(self.BladeRotation)          

        self.UI_BldSideShift = tk.DoubleVar()
        self.UI_BldSideShift.set(self.BldSideShift)
        
        self.UI_IMU1_Roll = tk.DoubleVar()
        self.UI_IMU1_Roll.set(self.IMU1_Roll)

        self.UI_IMU1_Pitch = tk.DoubleVar()
        self.UI_IMU1_Pitch.set(self.IMU1_Pitch)
        
        self.UI_IMU2_Roll = tk.DoubleVar()
        self.UI_IMU2_Roll.set(self.IMU2_Roll)

        self.UI_IMU2_Pitch = tk.DoubleVar()
        self.UI_IMU2_Pitch.set(self.IMU2_Pitch)

        self.UI_driving_gear = tk.DoubleVar()
        self.UI_driving_gear.set(self.driving_gear)
 
        # 주행 정보
        self.UI_vel = tk.DoubleVar()
        self.UI_vel.set(self.vel)

        self.UI_HeadingAngle = tk.DoubleVar()
        self.UI_HeadingAngle.set(self.HeadingAngle)
        
        self.UI_steeringAngle = tk.DoubleVar()
        self.UI_steeringAngle.set(self.steeringAngle)

        self.UI_latitude = tk.DoubleVar()
        self.UI_latitude.set(self.latitude)

        self.UI_longitude = tk.DoubleVar()
        self.UI_longitude.set(self.longitude)
        
        self.UI_altitude = tk.DoubleVar()
        self.UI_altitude.set(self.altitude)

        self.UI_drivingStatus = tk.DoubleVar()
        self.UI_drivingStatus.set(self.drivingStatus)
        
        self.UI_drivingStatus_txt = tk.StringVar()
        self.UI_drivingStatus_txt.set(self.drivingStatus_txt)

        self.UI_radar_packetnumber_Rear = tk.DoubleVar()
        self.UI_radar_installposition_Rear = tk.DoubleVar()
        self.UI_radar_status_Rear = tk.DoubleVar()
        self.UI_radar_reserved1_Rear = tk.DoubleVar()
        self.UI_radar_SWversion_Rear = tk.DoubleVar()
        self.UI_radar_targetX_Rear = tk.DoubleVar()
        self.UI_radar_targetY_Rear = tk.DoubleVar()
        self.UI_radar_targetPower_Rear = tk.DoubleVar()
        self.UI_radar_reserved2_Rear = tk.DoubleVar()
        self.UI_radar_faultcode_Rear = tk.DoubleVar()

        self.UI_radar_packetnumber_Right = tk.DoubleVar()
        self.UI_radar_installposition_Right = tk.DoubleVar()
        self.UI_radar_status_Right = tk.DoubleVar()
        self.UI_radar_reserved1_Right = tk.DoubleVar()
        self.UI_radar_SWversion_Right = tk.DoubleVar()
        self.UI_radar_targetX_Right = tk.DoubleVar()
        self.UI_radar_targetY_Right = tk.DoubleVar()
        self.UI_radar_targetPower_Right = tk.DoubleVar()
        self.UI_radar_reserved2_Right = tk.DoubleVar()
        self.UI_radar_faultcode_Right = tk.DoubleVar()

        self.UI_radar_packetnumber_Left = tk.DoubleVar()
        self.UI_radar_installposition_Left = tk.DoubleVar()
        self.UI_radar_status_Left = tk.DoubleVar()
        self.UI_radar_reserved1_Left = tk.DoubleVar()
        self.UI_radar_SWversion_Left = tk.DoubleVar()
        self.UI_radar_SWversion_Left.set(self.radar_SWversion_Left)
        self.UI_radar_targetX_Left = tk.DoubleVar()
        self.UI_radar_targetY_Left = tk.DoubleVar()
        self.UI_radar_targetPower_Left = tk.DoubleVar()
        self.UI_radar_reserved2_Left = tk.DoubleVar()
        self.UI_radar_faultcode_Left = tk.DoubleVar()

        self.UI_radar_packetnumber_RightFront = tk.DoubleVar()
        self.UI_radar_installposition_RightFront = tk.DoubleVar()
        self.UI_radar_status_RightFront = tk.DoubleVar()
        self.UI_radar_reserved1_RightFront = tk.DoubleVar()
        self.UI_radar_SWversion_RightFront = tk.DoubleVar()
        self.UI_radar_targetX_RightFront = tk.DoubleVar()
        self.UI_radar_targetY_RightFront = tk.DoubleVar()
        self.UI_radar_targetPower_RightFront = tk.DoubleVar()
        self.UI_radar_reserved2_RightFront = tk.DoubleVar()
        self.UI_radar_faultcode_RightFront = tk.DoubleVar()


        # print(f"b_y:{self.UI_BladeRotation}")
        # print(f"sw2:{self.UI_radar_SWversion_Left}")
        # # Setting UI data
        # self.UI_radar_packetnumber_Rear.set(self.radar_packetnumber_Rear)
        # self.UI_radar_installposition_Rear.set(self.radar_installposition_Rear)
        # self.UI_radar_status_Rear.set(self.radar_status_Rear)
        # self.UI_radar_reserved1_Rear.set(self.radar_reserved1_Rear)
        # self.UI_radar_SWversion_Rear.set(self.radar_SWversion_Rear)
        # self.UI_radar_targetX_Rear.set(self.radar_targetX_Rear)
        # self.UI_radar_targetY_Rear.set(self.radar_targetY_Rear)
        # self.UI_radar_targetPower_Rear.set(self.radar_targetPower_Rear)
        # self.UI_radar_reserved2_Rear.set(self.radar_reserved2_Rear)
        # self.UI_radar_faultcode_Rear.set(self.radar_faultcode_Rear)

        # self.UI_radar_packetnumber_Right.set(self.radar_packetnumber_Right)
        # self.UI_radar_installposition_Right.set(self.radar_installposition_Right)
        # self.UI_radar_status_Right.set(self.radar_status_Right)
        # self.UI_radar_reserved1_Right.set(self.radar_reserved1_Right)
        # self.UI_radar_SWversion_Right.set(self.radar_SWversion_Right)
        # self.UI_radar_targetX_Right.set(self.radar_targetX_Right)
        # self.UI_radar_targetY_Right.set(self.radar_targetY_Right)
        # self.UI_radar_targetPower_Right.set(self.radar_targetPower_Right)
        # self.UI_radar_reserved2_Right.set(self.radar_reserved2_Right)
        # self.UI_radar_faultcode_Right.set(self.radar_faultcode_Right)

        # self.UI_radar_packetnumber_Left.set(self.radar_packetnumber_Left)
        # self.UI_radar_installposition_Left.set(self.radar_installposition_Left)
        # self.UI_radar_status_Left.set(self.radar_status_Left)
        # self.UI_radar_reserved1_Left.set(self.radar_reserved1_Left)
        # self.UI_radar_SWversion_Left.set(self.radar_SWversion_Left)
        # self.UI_radar_targetX_Left.set(self.radar_targetX_Left)
        # self.UI_radar_targetY_Left.set(self.radar_targetY_Left)
        # self.UI_radar_targetPower_Left.set(self.radar_targetPower_Left)
        # self.UI_radar_reserved2_Left.set(self.radar_reserved2_Left)
        # self.UI_radar_faultcode_Left.set(self.radar_faultcode_Left)

        # self.UI_radar_packetnumber_RightFront.set(self.radar_packetnumber_RightFront)
        # self.UI_radar_installposition_RightFront.set(self.radar_installposition_RightFront)
        # self.UI_radar_status_RightFront.set(self.radar_status_RightFront)
        # self.UI_radar_reserved1_RightFront.set(self.radar_reserved1_RightFront)
        # self.UI_radar_SWversion_RightFront.set(self.radar_SWversion_RightFront)
        # self.UI_radar_targetX_RightFront.set(self.radar_targetX_RightFront)
        # self.UI_radar_targetY_RightFront.set(self.radar_targetY_RightFront)
        # self.UI_radar_targetPower_RightFront.set(self.radar_targetPower_RightFront)
        # self.UI_radar_reserved2_RightFront.set(self.radar_reserved2_RightFront)
        # self.UI_radar_faultcode_RightFront.set(self.radar_faultcode_RightFront)

        Fr = []
        #print(Fr) 
        Fr.append(ttk.LabelFrame(self.SV_GUI,width=100, height=1000, text='KU_UI_TEST',relief=tk.SOLID,labelanchor='n'))
        Fr[0].grid(column=0, row=0, padx=10, pady=10, sticky='n')   
        Fr.append(ttk.LabelFrame(self.SV_GUI, width=600, height=1000, text='MG_Data_Blade',relief=tk.SOLID,labelanchor='n'))         # 데이터
        Fr[1].grid(column=0, row=1, padx=10, pady=10, sticky='w') # padx / pady 외부여백
        Fr.append(ttk.LabelFrame(self.SV_GUI, width=600, height=1000, text='MG_Data_Drive',relief=tk.SOLID,labelanchor='n'))         # 데이터
        Fr[2].grid(column=0, row=2, padx=10, pady=10, sticky='w') 

        self.stop_Btn = ttk.Button(Fr[0],text='프로그램 종료',command=self.stop_program)
        self.stop_Btn.grid(column=0, row=0, sticky='n', padx=10, pady=5)  
          
        # Blade variable
        self.Rcv1 = ttk.Label(Fr[1],text = 'Left Cylinder',width = 30)
        self.Rcv1.grid(column=0, row=0, sticky='w', padx=10, pady=5)    
        self.RcvMsg1 = ttk.Label(Fr[1], textvariable = self.UI_LeftCyl, wraplength=500,width = 10)
        self.RcvMsg1.grid(column=1, row=0, sticky='e', padx=20, pady=5)

        self.Rcv2 = ttk.Label(Fr[1],text = 'Right Cylinder',width = 30)
        self.Rcv2.grid(column=0, row=1, sticky='w', padx=10, pady=5)    
        self.RcvMsg2 = ttk.Label(Fr[1], textvariable = self.UI_RightCyl, wraplength=500,width = 10)
        self.RcvMsg2.grid(column=1, row=1, sticky='e', padx=20, pady=5)

        self.Rcv3 = ttk.Label(Fr[1],text = 'Sideshift Cylinder',width = 30)
        self.Rcv3.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.RcvMsg3 = ttk.Label(Fr[1], textvariable = self.UI_SideCyl, wraplength=500,width = 10)
        self.RcvMsg3.grid(column=1, row=2, sticky='e', padx=20, pady=5)

        self.Rcv4 = ttk.Label(Fr[1],text = 'Tilt Cylinder',width = 30)
        self.Rcv4.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        self.RcvMsg4 = ttk.Label(Fr[1], textvariable = self.UI_TiltCyl, wraplength=500,width = 10)
        self.RcvMsg4.grid(column=1, row=3, sticky='e', padx=20, pady=5)

        self.Rcv5 = ttk.Label(Fr[1],text = 'Blade Rotation',width = 30)
        self.Rcv5.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        self.RcvMsg5 = ttk.Label(Fr[1], textvariable = self.UI_BladeRotation, wraplength=500,width = 10)
        self.RcvMsg5.grid(column=1, row=4, sticky='e', padx=20, pady=5)

        self.Rcv6 = ttk.Label(Fr[1],text = 'Blade Side Shift',width = 30)
        self.Rcv6.grid(column=0, row=5, sticky='w', padx=10, pady=5)    
        self.RcvMsg6 = ttk.Label(Fr[1], textvariable = self.UI_BldSideShift, wraplength=500,width = 10)
        self.RcvMsg6.grid(column=1, row=5, sticky='e', padx=20, pady=5)

        self.Rcv7 = ttk.Label(Fr[1],text = 'IMU1 Roll',width = 30)
        self.Rcv7.grid(column=0, row=6, sticky='w', padx=10, pady=5)    
        self.RcvMsg7 = ttk.Label(Fr[1], textvariable = self.UI_IMU1_Roll, wraplength=500,width = 10)
        self.RcvMsg7.grid(column=1, row=6, sticky='e', padx=20, pady=5)

        self.Rcv8 = ttk.Label(Fr[1],text = 'IMU1 Pitch',width = 30)
        self.Rcv8.grid(column=0, row=7, sticky='w', padx=10, pady=5)    
        self.RcvMsg8 = ttk.Label(Fr[1], textvariable = self.UI_IMU1_Pitch, wraplength=500,width = 10)
        self.RcvMsg8.grid(column=1, row=7, sticky='e', padx=20, pady=5)
        
        self.Rcv9 = ttk.Label(Fr[1],text = 'IMU2 Roll',width = 30)
        self.Rcv9.grid(column=0, row=8, sticky='w', padx=10, pady=5)    
        self.RcvMsg9 = ttk.Label(Fr[1], textvariable = self.UI_IMU2_Roll, wraplength=500,width = 10)
        self.RcvMsg9.grid(column=1, row=8, sticky='e', padx=20, pady=5)

        self.Rcv10 = ttk.Label(Fr[1],text = 'IMU2 Pitch',width = 30)
        self.Rcv10.grid(column=0, row=9, sticky='w', padx=10, pady=5)    
        self.RcvMsg10 = ttk.Label(Fr[1], textvariable = self.UI_IMU2_Pitch, wraplength=500,width = 10)
        self.RcvMsg10.grid(column=1, row=9, sticky='e', padx=20, pady=5)

        # Driving variable
        self.Rcv15 = ttk.Label(Fr[2],text = 'Velocity',width = 30)
        self.Rcv15.grid(column=0, row=0, sticky='w', padx=10, pady=5)    
        self.RcvMsg15 = ttk.Label(Fr[2], textvariable = self.UI_vel, wraplength=500,width = 10)
        self.RcvMsg15.grid(column=1, row=0, sticky='e', padx=20, pady=5)

        self.Rcv16 = ttk.Label(Fr[2],text = 'Heading Angle',width = 30)
        self.Rcv16.grid(column=0, row=1, sticky='w', padx=10, pady=5)    
        self.RcvMsg16 = ttk.Label(Fr[2], textvariable = self.UI_HeadingAngle, wraplength=500,width = 10)
        self.RcvMsg16.grid(column=1, row=1, sticky='e', padx=20, pady=5)
        
        self.Rcv17 = ttk.Label(Fr[2],text = 'Steering Angle',width = 30)
        self.Rcv17.grid(column=0, row=2, sticky='w', padx=10, pady=5)    
        self.RcvMsg17 = ttk.Label(Fr[2], textvariable = self.UI_steeringAngle, wraplength=500,width = 10)
        self.RcvMsg17.grid(column=1, row=2, sticky='e', padx=20, pady=5)

        self.Rcv18 = ttk.Label(Fr[2],text = 'Latitude',width = 30)
        self.Rcv18.grid(column=0, row=3, sticky='w', padx=10, pady=5)    
        self.RcvMsg18 = ttk.Label(Fr[2],textvariable = self.UI_latitude, wraplength=500,width = 10)
        self.RcvMsg18.grid(column=1, row=3, sticky='e', padx=20, pady=5)  
                
        self.Rcv19 = ttk.Label(Fr[2],text = 'Longitude',width = 30)
        self.Rcv19.grid(column=0, row=4, sticky='w', padx=10, pady=5)    
        self.RcvMsg19 = ttk.Label(Fr[2],textvariable = self.UI_longitude, wraplength=500,width = 10)
        self.RcvMsg19.grid(column=1, row=4, sticky='e', padx=20, pady=5)   

        self.Rcv20 = ttk.Label(Fr[2],text = 'Altitude',width = 30)
        self.Rcv20.grid(column=0, row=5, sticky='w', padx=10, pady=5)    
        self.RcvMsg20 = ttk.Label(Fr[2],textvariable = self.UI_altitude, wraplength=500,width = 10)
        self.RcvMsg20.grid(column=1, row=5, sticky='e', padx=20, pady=5)   
        
        self.Rcv21 = ttk.Label(Fr[2],text = 'Driving Status',width = 30)
        self.Rcv21.grid(column=0, row=6, sticky='w', padx=10, pady=5)    
        self.RcvMsg21 = ttk.Label(Fr[2],textvariable = self.UI_drivingStatus, wraplength=500,width = 10)
        self.RcvMsg21.grid(column=1, row=6, sticky='e', padx=20, pady=5)  
        self.RcvMsg22 = ttk.Label(Fr[2],textvariable = self.UI_drivingStatus_txt, wraplength=500,width = 10)
        self.RcvMsg22.grid(column=2, row=6, sticky='e', padx=20, pady=5) 
      
        self.Rcv23 = ttk.Label(Fr[2],text = 'Driving gear',width = 30)
        self.Rcv23.grid(column=0, row=7, sticky='w', padx=10, pady=5)    
        self.RcvMsg23 = ttk.Label(Fr[2],textvariable = self.UI_driving_gear, wraplength=500,width = 10)
        self.RcvMsg23.grid(column=1, row=7, sticky='e', padx=20, pady=5)    
        
        self.Rcv24 = ttk.Label(Fr[2],text = 'test',width = 30)
        self.Rcv24.grid(column=0, row=8, sticky='w', padx=10, pady=5)    
        # self.RcvMsg24 = ttk.Label(Fr[2],textvariable = self.UI_radar_SWversion_Left, wraplength=500,width = 10)
        # self.RcvMsg24.grid(column=1, row=8, sticky='e', padx=20, pady=5)
        
        
        # UI Test
        self.CMDC1_btn1 = ttk.Button(Fr[0],text='SEND (3)',command=self.CMDC1_send)
        self.CMDC1_btn1.grid(column=0, row=1, sticky='n', padx=10, pady=5)   
        
        self.CMDC1_btn2 = ttk.Button(Fr[0],text='START (1)',command=self.CMDC1_start)
        self.CMDC1_btn2.grid(column=1, row=1, sticky='n', padx=10, pady=5)  
        
        self.CMDC1_btn3 = ttk.Button(Fr[0],text='DEFAULT (0)',command=self.CMDC1_default)
        self.CMDC1_btn3.grid(column=2, row=1, sticky='n', padx=10, pady=5)  

        self.CMDC1_btn4 = ttk.Button(Fr[0],text='EMS STOP (2)',command=self.CMDC1_EMS)
        self.CMDC1_btn4.grid(column=3, row=1, sticky='n', padx=10, pady=5)  
        
        self.CMDC2_btn1 = ttk.Button(Fr[0],text='Waypoint(JH ver)',command=self.waypoint_send_1)
        self.CMDC2_btn1.grid(column=0, row=2, sticky='n', padx=10, pady=5)          

        self.CMDC2_btn2 = ttk.Button(Fr[0],text='Waypoint(JS ver)',command=self.waypoint_send_2)
        self.CMDC2_btn2.grid(column=1, row=2, sticky='n', padx=10, pady=5)          
  
        self.blade_btn1 = ttk.Button(Fr[0],text='작업 #1',command=self.blade1)
        self.blade_btn1.grid(column=0, row=3, sticky='n', padx=10, pady=5)   
        self.blade_btn1 = ttk.Button(Fr[0],text='블레이드 올리기',command=self.blade_lift)
        self.blade_btn1.grid(column=1, row=3, sticky='n', padx=10, pady=5)   
        self.blade_btn1 = ttk.Button(Fr[0],text='작업 #2',command=self.blade2)
        self.blade_btn1.grid(column=2, row=3, sticky='n', padx=10, pady=5)   
        self.blade_btn1 = ttk.Button(Fr[0],text='up',command=self.up)
        self.blade_btn1.grid(column=3, row=3, sticky='n', padx=10, pady=5)   
        self.blade_btn1 = ttk.Button(Fr[0],text='down',command=self.down)
        self.blade_btn1.grid(column=4, row=3, sticky='n', padx=10, pady=5)   

     
        self.btnDriving1 = ttk.Radiobutton(Fr[0],text = 'driving_onoff on', value = 0, variable = self.driving_onoff, command=self.drivingFn)
        self.btnDriving1.grid(column=0, row=4, sticky='w', padx=10, pady=5) 
           
        self.btnDriving2 = ttk.Radiobutton(Fr[0],text = 'driving_onoff off', value = 1, variable = self.driving_onoff, command=self.drivingFn)
        self.btnDriving2.grid(column=1, row=4, sticky='w', padx=10, pady=5)  


        self.btnLogging1 = ttk.Radiobutton(Fr[0],text = 'logging on', value = 0, variable = self.logOnOff, command=self.loggingFn)
        self.btnLogging1.grid(column=0, row=5, sticky='w', padx=10, pady=5) 
           
        self.btnLogging2 = ttk.Radiobutton(Fr[0],text = 'logging off', value = 1, variable = self.logOnOff, command=self.loggingFn)
        self.btnLogging2.grid(column=1, row=5, sticky='w', padx=10, pady=5)  
        

        self.RcvMsg24 = ttk.Label(Fr[2],textvariable = self.UI_radar_targetY_Right, wraplength=500,width = 10)
        self.RcvMsg24.grid(column=2, row=2, sticky='e', padx=20, pady=5)
        # ##################################################################
        rear_frame = tk.LabelFrame(self.SV_GUI, text="Rear Radar")
        right_frame = tk.LabelFrame(self.SV_GUI, text="Right Radar")
        left_frame = tk.LabelFrame(self.SV_GUI, text="Left Radar")
        right_front_frame = tk.LabelFrame(self.SV_GUI, text="Right Front Radar")

        rear_frame.grid(row=1, column=1, padx=10, pady=10, sticky='nw')
        right_frame.grid(row=2, column=1, padx=10, pady=10, sticky='nw')
        left_frame.grid(row=1, column=2, padx=10, pady=10, sticky='nw')
        right_front_frame.grid(row=2, column=2, padx=10, pady=10, sticky='nw')

        ###Function to add labels to frames
        def add_labels(frame, data):
            row = 0
            for label_text, variable in data:
                tk.Label(frame, text=label_text).grid(row=row, column=0, sticky='w')
                #tk.Label(frame, textvariable=variable).grid(row=row, column=1, sticky='w')
                row += 1
        def update_labels(frame, data):
            print("update")
            row = 0
            for label_text, variable in data:
                #tk.Label(frame, text=label_text).grid(row=row, column=0, sticky='w')
                tk.Label(frame, textvariable=variable).grid(row=row, column=1, sticky='w')
                row += 1

        # Data for each frame
        rear_data = [
            ("Packet Number", self.UI_radar_packetnumber_Rear),
            ("Install Position", self.UI_radar_installposition_Rear),
            ("Status", self.UI_radar_status_Rear),
            ("Reserved1", self.UI_radar_reserved1_Rear),
            ("SW Version", self.UI_radar_SWversion_Rear),
            ("Target X", self.UI_radar_targetX_Rear),
            ("Target Y", self.UI_radar_targetY_Rear),
            ("Target Power", self.UI_radar_targetPower_Rear),
            ("Reserved2", self.UI_radar_reserved2_Rear),
            ("Fault Code", self.UI_radar_faultcode_Rear),
        ]

        right_data = [
            ("Packet Number", self.UI_radar_packetnumber_Right),
            ("Install Position", self.UI_radar_installposition_Right),
            ("Status", self.UI_radar_status_Right),
            ("Reserved1", self.UI_radar_reserved1_Right),
            ("SW Version", self.UI_radar_SWversion_Right),
            ("Target X", self.UI_radar_targetX_Right),
            ("Target Y", self.UI_radar_targetY_Right),
            ("Target Power", self.UI_radar_targetPower_Right),
            ("Reserved2", self.UI_radar_reserved2_Right),
            ("Fault Code", self.UI_radar_faultcode_Right),
        ]

        left_data = [
            ("Packet Number", self.UI_radar_packetnumber_Left),
            ("Install Position", self.UI_radar_installposition_Left),
            ("Status", self.UI_radar_status_Left),
            ("Reserved1", self.UI_radar_reserved1_Left),
            ("SW Version", self.UI_radar_SWversion_Left),
            ("Target X", self.UI_radar_targetX_Left),
            ("Target Y", self.UI_radar_targetY_Left),
            ("Target Power", self.UI_radar_targetPower_Left),
            ("Reserved2", self.UI_radar_reserved2_Left),
            ("Fault Code", self.UI_radar_faultcode_Left),
        ]

        right_front_data = [
            ("Packet Number", self.UI_radar_packetnumber_RightFront),
            ("Install Position", self.UI_radar_installposition_RightFront),
            ("Status", self.UI_radar_status_RightFront),
            ("Reserved1", self.UI_radar_reserved1_RightFront),
            ("SW Version", self.UI_radar_SWversion_RightFront),
            ("Target X", self.UI_radar_targetX_RightFront),
            ("Target Y", self.UI_radar_targetY_RightFront),
            ("Target Power", self.UI_radar_targetPower_RightFront),
            ("Reserved2", self.UI_radar_reserved2_RightFront),
            ("Fault Code", self.UI_radar_faultcode_RightFront),
        ]

        # Add labels to each frame
        add_labels(rear_frame, rear_data)
        print(f"flag:{add_labels}")
        add_labels(right_frame, right_data)
        add_labels(left_frame, left_data)
        add_labels(right_front_frame, right_front_data)
        
        update_labels(rear_frame, rear_data)
        print(f"up_flag:{update_labels}")
        update_labels(right_frame, right_data)
        update_labels(left_frame, left_data)
        update_labels(right_front_frame, right_front_data)
        self.updbtn1 = ttk.Button(Fr[0],text='update',command=update_labels(right_frame, right_data))
        self.updbtn1.grid(column=5, row=4, sticky='n', padx=10, pady=5)   

        #self.Rcv24 = ttk.Label(rear_frame,text = 'test',width = 30)
        #self.Rcv24.grid(column=0, row=8, sticky='w', padx=10, pady=5)    

        
    

    
 
        

    def up(self):
        self.offset += 0.01

    def down(self):
        self.offset -= 0.01

    def blade1(self):
        self.sequence = 0
        self.road_color = 2

    def blade2(self):
        self.sequence = 1
        self.road_color = 2

    def blade_lift(self):
        self.sequence = 2    
        self.road_color = 1

    def stop_program(self):
        self.stop_event.set()

    def CMDC1_send(self):
        msg_CMDC1 = call_MG_CRIO_CMDC1()
        temp = struct.pack('>B',3)
        msg_CMDC1[6] = temp[0]
        print(msg_CMDC1)
        self.KU_client_socket.sendall(msg_CMDC1)
        print('CMDC1 수신')

    def CMDC1_start(self):
        msg_CMDC1 = call_MG_CRIO_CMDC1()
        temp = struct.pack('>B',1)
        msg_CMDC1[6] = temp[0]
        print(msg_CMDC1)
        self.KU_client_socket.sendall(msg_CMDC1)
        print('CMDC1 수신')

    def CMDC1_default(self):
        msg_CMDC1 = call_MG_CRIO_CMDC1()
        temp = struct.pack('>B',0)
        msg_CMDC1[6] = temp[0]
        print(msg_CMDC1)
        self.KU_client_socket.sendall(msg_CMDC1)
        print('CMDC1 수신')

    def CMDC1_EMS(self):
        msg_CMDC1 = call_MG_CRIO_CMDC1()
        temp = struct.pack('>B',2)
        msg_CMDC1[6] = temp[0]
        print(msg_CMDC1)
        self.KU_client_socket.sendall(msg_CMDC1)
        print('CMDC1 수신')

    def waypoint_send_1(self):
        with open("1_240802_14_Straigt_Fw_vel4.json", encoding='utf-8') as f:
            waypoint_json = json.load(f)
        f.close()
        msg_CMDC2 = call_MG_CRIO_CMDC2(waypoint_json)
        self.KU_client_socket.sendall(msg_CMDC2)
        time.sleep(2)
        
    def waypoint_send_2(self):
        with open("1_240802_14_Straigt_Fw_vel4", encoding='utf-8') as f:
            waypoint_json = json.load(f)
        f.close()
        msg_CMDC2 = call_MG_CRIO_CMDC2(waypoint_json)
        self.KU_client_socket.sendall(msg_CMDC2)
        time.sleep(2)


    def loggingFn(self):
        if self.logOnOff.get() == 0:       # logging on
            # self.CMDB[5] = 1
            # self.SVP_client_socket.sendall(self.CMDB)            
            self.current_date = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            # self.logtime = time.time()
            self.logtime = time.perf_counter()
            self.csv_file_name = f'logfile/data_logging_{self.current_date}.csv'
            print(self.csv_file_name)
            self.file = open(self.csv_file_name,'w',newline='')
            self.writer = csv.writer(self.file,delimiter=',')  
            self.rows = []
            self.rows.append(['time','self.LeftCyl','self.RightCyl','self.SideCyl','self.BladeRotation','self.BldSideShift','self.TiltCyl','self.IMU1_Roll','self.IMU1_Pitch','self.IMU2_Roll','self.IMU2_Pitch', 'self.vel', 'self.latitude','self.longitude','self.altitude','self.HeadingAngle','self.filtered_roll','self.filtered_pitch','self.filtered_alt','self.RTK_Status','self.driving_gear'])
            self.writer.writerows(self.rows)
            print('Logging set')
            
            self.logStatus = 1
                   
        else:
            self.file.close()
            self.logStatus = 0

    def drivingFn(self):
        if self.driving_onoff.get() == 0:
            self.workEnd = False
            self.workStart = True
            print('driving on')
            
        else:
            self.workEnd = True
            self.workStart = False
            print('driving off')
            

# # 외부에서 새로 추가한 코드

#     def response_UI(self, dict_UI_CMD, C_socket):

#         # UI 이벤트 시작
#         print('UI 이벤트 실행')
#         dict_MG_Resposse = send_MG_Reponse(dict_UI_CMD)
#         str_MG_Response = json.dumps(dict_MG_Resposse)
        
        
#         ###고려대에서 UI의 cmd를 받고 그에 대한 응답을 UI에 전달###
#         C_socket.sendall(bytes(str_MG_Response+'||',encoding="utf-8"))
#         #print('SVPC에서 UI에 MG_Response를 전달했습니다.')

#         #print(self.dict_UI_CMD) # 자체 UI에 표시
#         #self.UI_SV_Data.set(self.dict_UI_CMD)
#         self.UI_SV_Data.set(f'MC Controll on/off : {self.dict_UI_CMD["grd_mc_ctrl_on_rqst"]}\n'
#                             f'desired blade roll : {self.dict_UI_CMD["grd_set_request"]["bld_roll"]}\n'
#                             f'desired blade pitch : {self.dict_UI_CMD["grd_set_request"]["bld_pitch"]}\n'
#                             f'desired blade yaw : {self.dict_UI_CMD["grd_set_request"]["bld_yaw"]}')

    
if __name__ == "__main__":
    # Server_Client().initialize_GUI()
    Server_Client()
    tk.mainloop()
    client = Server_Client()
    #client.SV_GUI2.mainloop()