# paho.mqtt.client 모듈을 이용한 MQTT 메세지 송수신

import paho.mqtt.client as mqtt
import time
from datetime import datetime
import json

sub_topic = "command/workplan/grader"
pub_topic = "equipment/grader"
withpoint_url = "withpoints.asuscomm.com"   # 브로커 주소

current_time = datetime.now()
timestamp_str = current_time.strftime("%Y-%m-%d %H:%M:%S")

payload ={
  "grd_mac_addr": "AB:BC:CD:DE:EF:F0",
  "grd_protocol_id": "A",
  "grd_type": 1,
  "grd_id": "test",
  "Timestamp": timestamp_str,
  "grd_bdy_lttd": 35.943114,
  "grd_bdy_lgtd": 126.577257,
  "grd_bdy_altd": 67.596,
  "grd_bdy_x_pose": 50.0,
  "grd_bdy_y_pose": 50.0,
  "grd_bdy_z_pose": 50.0,
  "grd_bdy_velocity": 10.0,
  "grd_bld_x_pose": 0.0,
  "grd_bld_y_pose": 0.0,
  "grd_bld_z_pose": 0.0,
  "grd_bld_z_offset": 0.0,
  "grd_running_state": 1,
  "grd_running_mode": 0,
  "grd_reverse_drv": bool(0),
  "grd_rdr_detect":0
} 

# callback 함수 정의
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(sub_topic) # 구독 신청
def on_message(client, userdata, msg): # 구독한 메시지 수신
    # print(msg.topic+" "+str(msg.payload))
    pass
def on_publish(client, userdata, mid): # 발행한 메시지 확인
    print("Message Published...")
def on_disconnect(client, userdata, rc): # 연결이 끊어졌을 때 호출
    print("Disconnected with result code "+str(rc))
    while rc != 0:
        try:
            print("Attempting to reconnect...")
            rc = client.reconnect()
        except Exception as e:
            print(f"Reconnect failed: {e}")
            time.sleep(5) # 5초 대기 후 재시도

mqtt_client = mqtt.Client(client_id="grader_01")    # 클라이언트 객체 생성
mqtt_client.username_pw_set("grader", "grader") # 사용자 인증정보 설정
mqtt_client.on_connect = on_connect # 연결 콜백 지정
mqtt_client.on_message = on_message # 메시지 수신 콜백 지정
mqtt_client.on_publish = on_publish # pub 수신 콜백 지정
mqtt_client.on_disconnect = on_disconnect # 연결 끊김 콜백 지정


mqtt_client.connect(withpoint_url, 50592, 60) # 브로커에 연결

mqtt_client.loop_start()

while True:
    current_time = datetime.now()
    timestamp_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
    payload["Timestamp"]=timestamp_str
    print(payload)
    mqtt_client.publish(pub_topic, json.dumps(payload), qos=0, retain=False) # 메시지 발행
    time.sleep(1) # 1초 대기



